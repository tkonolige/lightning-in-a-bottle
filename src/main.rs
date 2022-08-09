#![no_std]
#![no_main]

use core::ops::Deref;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use hal::gpio::{Output, Pin, PushPull};
use hal::prelude::*;
use nrf52832_hal as hal;
use panic_probe as _;
use smart_leds::{brightness, gamma, RGB8};

const LEDS_PER_STRIP: usize = 600;
const BOX_LENGTH: usize = 300;
// 600 leds * 12 bytes/led + 25 byte for reset rounded up to the nearest multiple of 256 bytes
const LED_BYTES: usize = (LEDS_PER_STRIP * 12 + 50 + 255) / 256 * 256;

#[repr(C, align(8))]
pub struct LEDStrip<Timer, SPIM, PPI> {
    control_bits: &'static mut [u8; LED_BYTES],
    counter: Timer,
    spi: SPIM,
    ppi: PPI,
    pin: Pin<Output<PushPull>>,
    sck_pin: Pin<Output<PushPull>>,
}
impl<Timer, SPIM, PPI> LEDStrip<Timer, SPIM, PPI>
where
    Timer: Deref<Target = hal::pac::timer0::RegisterBlock>,
    SPIM: Deref<Target = hal::pac::spim0::RegisterBlock>,
    PPI: hal::ppi::ConfigurablePpi,
{
    fn new(
        counter: Timer,
        spi: SPIM,
        ppi: PPI,
        pin: Pin<Output<PushPull>>,
        sck_pin: Pin<Output<PushPull>>,
        control_bits: &'static mut [u8; LED_BYTES],
    ) -> LEDStrip<Timer, SPIM, PPI> {
        LEDStrip {
            control_bits,
            counter,
            spi,
            ppi,
            pin,
            sck_pin,
        }
    }

    fn configure(&mut self) {
        // output on pin 13
        self.spi.psel.mosi.write(|x| {
            unsafe { x.bits(self.pin.psel_bits()) };
            x.connect().connected()
        });
        // sck must be connected
        self.spi.psel.sck.write(|x| {
            unsafe { x.bits(self.sck_pin.psel_bits()) };
            x.connect().connected()
        });
        // dont need the other outputs
        self.spi.psel.miso.write(|x| {
            unsafe { x.bits(11) };
            x.connect().disconnected()
        });
        self.spi.enable.write(|x| x.enable().enabled());
        self.spi.frequency.write(|x| x.frequency().m4());
        self.spi
            .txd
            .ptr
            .write(|x| unsafe { x.ptr().bits(self.control_bits.as_ptr() as u32) });
        // read 256 bytes at a time
        self.spi
            .txd
            .maxcnt
            .write(|x| unsafe { x.maxcnt().bits(0xff) });
        // advance pointer 256 bytes each time
        self.spi.txd.list.write(|x| x.list().array_list());
        // receive nothing
        self.spi.rxd.maxcnt.write(|x| unsafe { x.maxcnt().bits(0) });
        // start next transfer as soon as a transfer is finished
        self.spi.shorts.write(|x| x.end_start().enabled());

        self.counter.mode.write(|x| x.mode().counter());
        self.counter.bitmode.write(|x| x.bitmode()._32bit());
        self.counter.tasks_clear.write(|x| unsafe { x.bits(1) });
        // count up to the total number of sends
        self.counter.cc[0].write(|x| unsafe { x.bits((self.control_bits.len() / 256 - 1) as u32) });
        self.counter.intenset.write(|x| x.compare0().set());
        self.counter.tasks_start.write(|x| unsafe { x.bits(1) });
        self.ppi.set_task_endpoint(&self.counter.tasks_count);
        self.ppi.set_event_endpoint(&self.spi.events_end);
        self.ppi.enable();

        // enable spi
        self.spi
            .config
            .write(|x| x.order().msb_first().cpol().active_low().cpha().trailing());
        self.spi.orc.write(|x| unsafe { x.orc().bits(0) });
        compiler_fence(SeqCst);
    }

    fn set_data<T>(&mut self, br: u8, iterator: T)
    where
        T: Iterator<Item = RGB8>,
    {
        // power draw is max 20mA per color
        // from https://www.temposlighting.com/guides/power-any-ws2812b-setup
        // mA  brightness
        // 10  50
        // 18  100
        // 28  150
        // 48  200
        // 50  250
        let pixel_draw = [10, 10, 18, 28, 48, 50, 50, 50];
        let mut total_draw_mA: usize = 0;
        let patterns = [0b1000_1000, 0b1000_1110, 0b1110_1000, 0b1110_1110];
        // let patterns = [0b1110_1110, 0b1110_1110, 0b1110_1110, 0b1110_1110];
        for (i, rgb) in gamma(brightness(iterator, br)).enumerate() {
            let r = rgb.r as usize;
            let r_l = r / 50 * 50;
            let r_h = (r + 49) / 50 * 50;
            let g = rgb.g as usize;
            let g_l = g / 50 * 50;
            let g_h = (g + 49) / 50 * 50;
            let b = rgb.b as usize;
            let b_l = b / 50 * 50;
            let b_h = (b + 49) / 50 * 50;
            let draw_r = (pixel_draw[r_l / 50] * (r - r_l) + pixel_draw[r_h / 50] * (r_h - r)) / 50;
            let draw_g = (pixel_draw[g_l / 50] * (g - g_l) + pixel_draw[g_h / 50] * (g_h - g)) / 50;
            let draw_b = (pixel_draw[b_l / 50] * (b - b_l) + pixel_draw[b_h / 50] * (b_h - b)) / 50;
            total_draw_mA += (draw_r + draw_g + draw_b) / 3;
            assert!(i < LEDS_PER_STRIP, "Max {LEDS_PER_STRIP} leds");
            self.control_bits[i * 12 + 0] = patterns[((rgb.g & 0b1100_0000) >> 6) as usize];
            self.control_bits[i * 12 + 1] = patterns[((rgb.g & 0b0011_0000) >> 4) as usize];
            self.control_bits[i * 12 + 2] = patterns[((rgb.g & 0b0000_1100) >> 2) as usize];
            self.control_bits[i * 12 + 3] = patterns[((rgb.g & 0b0000_0011) >> 0) as usize];
            self.control_bits[i * 12 + 4] = patterns[((rgb.r & 0b1100_0000) >> 6) as usize];
            self.control_bits[i * 12 + 5] = patterns[((rgb.r & 0b0011_0000) >> 4) as usize];
            self.control_bits[i * 12 + 6] = patterns[((rgb.r & 0b0000_1100) >> 2) as usize];
            self.control_bits[i * 12 + 7] = patterns[((rgb.r & 0b0000_0011) >> 0) as usize];
            self.control_bits[i * 12 + 8] = patterns[((rgb.b & 0b1100_0000) >> 6) as usize];
            self.control_bits[i * 12 + 9] = patterns[((rgb.b & 0b0011_0000) >> 4) as usize];
            self.control_bits[i * 12 + 10] = patterns[((rgb.b & 0b0000_1100) >> 2) as usize];
            self.control_bits[i * 12 + 11] = patterns[((rgb.b & 0b0000_0011) >> 0) as usize];
        }
        assert!(total_draw_mA < 10_000, "total draw (mA): {total_draw_mA}");
    }

    fn start(&mut self) {
        compiler_fence(SeqCst);
        // while self.spi.events_end.read().bits() == 0 {}
        // self.spi.events_end.write(|x| unsafe{ x.bits(0)});
        self.spi
            .txd
            .ptr
            .write(|x| unsafe { x.ptr().bits(self.control_bits.as_ptr() as u32) });
        // self.spi.events_end.write(|x| unsafe{ x.bits(0)});
        self.spi.shorts.write(|x| x.end_start().enabled());
        self.counter.tasks_clear.write(|x| unsafe { x.bits(1) });
        self.spi.tasks_start.write(|x| unsafe { x.bits(1) });
        compiler_fence(SeqCst);
    }

    fn interrupt(&mut self) {
        self.counter.events_compare[0].write(|x| unsafe { x.bits(0) });
        self.counter.tasks_clear.write(|x| unsafe { x.bits(1) });
        // self.spi.tasks_stop.write(|x| unsafe{x.bits(1)});
        self.spi.shorts.write(|x| x.end_start().disabled());
        compiler_fence(SeqCst);
    }
}

fn xorshift(seed: u32, inp: u32) -> (u32, u32) {
    let mut x: u64 = (seed as u64) << 32 | inp as u64;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    x *= 0x2545F4914F6CDD1D;
    ((x >> 32) as u32, (x & 0xffffffff) as u32)
}

#[rtic::app(device = crate::lightstrip::hal::pac, peripherals=true)]
mod lightstrip {
    // use apa102_spi as apa102;
    use core::ops::Deref;
    use defmt_rtt as _; // for logging
    use hal::gpio::Level;
    use hal::pac::{SPIM0, SPIM1, SPIM2, TIMER0, TIMER1, TIMER2, TIMER3};
    use hal::ppi::{Ppi0, Ppi1, Ppi2};
    use nrf52832_hal as hal;
    use smart_leds::{gamma, hsv::hsv2rgb, hsv::Hsv};

    use crate::*;

    #[derive(Eq, PartialEq, Copy, Clone, Debug)]
    pub enum Effect {
        Rainbow,
        Lightning,
        Breathing,
        Off,
    }

    static mut LED_BITS1: [u8; LED_BYTES] = [0; LED_BYTES];
    static mut LED_BITS2: [u8; LED_BYTES] = [0; LED_BYTES];
    static mut LED_BITS3: [u8; LED_BYTES] = [0; LED_BYTES];

    #[shared]
    struct Shared {
        leds: crate::LEDStrip<TIMER2, SPIM0, Ppi0>,
        leds2: crate::LEDStrip<TIMER0, SPIM1, Ppi1>,
        leds3: crate::LEDStrip<TIMER1, SPIM2, Ppi2>,
        j: usize,
        effect: Effect,
        brightness: u8,
    }
    #[local]
    struct Local {
        timer: hal::Timer<TIMER3, hal::timer::Periodic>,
        // This is state used locally at every tick of the timer. You can access them via
        // `ctx.local.variable` within the fimer function.
        // If you need some state, add it here.
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let p0 = hal::gpio::p0::Parts::new(ctx.device.P0);
        let _clocks = hal::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();
        let mut status_led = p0.p0_17.into_push_pull_output(Level::Low).degrade();
        status_led.set_high().unwrap();

        let ppi_channels = hal::ppi::Parts::new(ctx.device.PPI);
        let mosi = p0.p0_12.into_push_pull_output(Level::Low).degrade();
        let sck_1 = p0.p0_11.into_push_pull_output(Level::Low).degrade();
        let mut leds = crate::LEDStrip::new(
            ctx.device.TIMER2,
            ctx.device.SPIM0,
            ppi_channels.ppi0,
            mosi,
            sck_1,
            unsafe { &mut LED_BITS1 },
        );
        leds.configure();
        leds.start();

        let miso = p0.p0_13.into_push_pull_output(Level::Low).degrade();
        let sck_2 = p0.p0_10.into_push_pull_output(Level::Low).degrade();
        let mut leds2 = crate::LEDStrip::new(
            ctx.device.TIMER0,
            ctx.device.SPIM1,
            ppi_channels.ppi1,
            miso,
            sck_2,
            unsafe { &mut LED_BITS2 },
        );
        leds2.configure();
        leds2.start();

        let p15 = p0.p0_14.into_push_pull_output(Level::Low).degrade();
        let sck_3 = p0.p0_09.into_push_pull_output(Level::Low).degrade();
        let mut leds3 = crate::LEDStrip::new(
            ctx.device.TIMER1,
            ctx.device.SPIM2,
            ppi_channels.ppi2,
            p15,
            sck_3,
            unsafe { &mut LED_BITS3 },
        );
        leds3.configure();
        leds3.start();

        let mut timer = hal::Timer::periodic(ctx.device.TIMER3);
        timer.enable_interrupt();
        let update_delay: u32 = 10_000;
        timer.start(update_delay); // 100 times a second

        (
            Shared {
                leds,
                leds2,
                leds3,
                j: 0,
                effect: Effect::Breathing,
                brightness: 255,
            },
            Local { timer },
            init::Monotonics(),
        )
    }

    // This is called 100 times a second
    #[task(binds=TIMER3, local=[timer], shared=[j], priority=3)]
    fn timer3(mut ctx: timer3::Context) {
        // this resets the timer so it doesn't fire until the timeout is hit again
        ctx.local
            .timer
            .event_compare_cc0()
            .write(|x| unsafe { x.bits(0) });
        ctx.shared.j.lock(|j| *j += 1);
    }

    fn colors<T, S, P>(
        leds: &mut crate::LEDStrip<T, S, P>,
        effect: Effect,
        brightness: u8,
        j: usize,
    ) where
        T: Deref<Target = hal::pac::timer0::RegisterBlock>,
        S: Deref<Target = hal::pac::spim0::RegisterBlock>,
        P: hal::ppi::ConfigurablePpi,
    {
        // This code computes the actual color. It returns an iterator of length equal to
        // the number of leds
        match effect {
            Effect::Rainbow => {
                leds.set_data(brightness, {
                    (0..LEDS_PER_STRIP).map(|i| {
                        hsv2rgb(Hsv {
                            hue: ((i % BOX_LENGTH * 3 + (j) / 3) % 256) as u8,
                            sat: 150,
                            // the led strip can draw a LOT of power if this value is set high.
                            // Probably safest to leave it at 150 for now
                            val: 150,
                        })
                    })
                });
            }
            Effect::Lightning => {
                let flash_len = 200;
                let whiteout_len = 10;
                let loc: isize = ((j * 2) % (BOX_LENGTH + flash_len + whiteout_len))
                    .try_into()
                    .unwrap();
                leds.set_data(brightness, {
                    (0..LEDS_PER_STRIP).map(|i| {
                        let x = i % BOX_LENGTH;
                        let y = i / BOX_LENGTH;
                        let (_, r) = xorshift(0xdeadbeef, x as u32);
                        let is_lit = (r >> 11) % 2 == y as u32;
                        if loc < BOX_LENGTH as isize {
                            if (x as isize) < loc && is_lit {
                                let dist = 10 - (loc - x as isize).max(0).min(10);
                                let val = (if i == loc as usize {
                                    255
                                } else {
                                    150 * dist / 10
                                })
                                .try_into()
                                .unwrap();
                                hsv2rgb(Hsv {
                                    hue: 0,
                                    sat: 10,
                                    val,
                                })
                            } else {
                                RGB8 { r: 0, g: 0, b: 0 }
                            }
                        } else if (loc as usize) < BOX_LENGTH + whiteout_len {
                            hsv2rgb(Hsv {
                                hue: 0,
                                sat: 10,
                                val: 200 - (i - BOX_LENGTH) as u8,
                            })
                        } else {
                            if is_lit {
                                hsv2rgb(Hsv {
                                    hue: 0,
                                    sat: 10,
                                    val: (150 - (loc as usize - BOX_LENGTH - whiteout_len) / 3)
                                        .try_into()
                                        .unwrap(),
                                })
                            } else {
                                RGB8 { r: 0, g: 0, b: 0 }
                            }
                        }
                    })
                });
            }
            Effect::Breathing => {
                // https://avital.ca/notes/a-closer-look-at-apples-breathing-light
                let x = (j % 2000) as f64 / (2000. / 6.);
                let max_brightness = 150.;
                let val = ((libm::exp(-(x - 4.) * (x - 4.) / 2.) * 0.5 + 0.5) * max_brightness)
                    .min(max_brightness) as u8;
                leds.set_data(
                    brightness,
                    (0..LEDS_PER_STRIP).map(|_| {
                        hsv2rgb(Hsv {
                            hue: 0,
                            sat: 100,
                            val,
                        })
                    }),
                );
            }
            Effect::Off => {
                leds.set_data(0, {
                    (0..LEDS_PER_STRIP).map(|_| RGB8 { r: 0, g: 0, b: 0 })
                });
            }
        }
    }

    #[task(binds=TIMER2, shared=[leds, j, effect, brightness])]
    fn timer2(ctx: timer2::Context) {
        (
            ctx.shared.leds,
            ctx.shared.effect,
            ctx.shared.j,
            ctx.shared.brightness,
        )
            .lock(|leds, effect, j, brightness| {
                leds.interrupt();
                colors(leds, *effect, *brightness, *j);
                leds.start();
            });
    }

    #[task(binds=TIMER0, shared=[leds2, j, effect, brightness], priority=2)]
    fn timer0(ctx: timer0::Context) {
        (
            ctx.shared.leds2,
            ctx.shared.effect,
            ctx.shared.j,
            ctx.shared.brightness,
        )
            .lock(|leds, effect, j, brightness| {
                leds.interrupt();
                colors(leds, *effect, *brightness, *j);
                leds.start();
            });
    }

    #[task(binds=TIMER1, shared=[leds3, j, effect, brightness], priority=2)]
    fn timer1(ctx: timer1::Context) {
        (
            ctx.shared.leds3,
            ctx.shared.effect,
            ctx.shared.j,
            ctx.shared.brightness,
        )
            .lock(|leds, effect, j, brightness| {
                leds.interrupt();
                colors(leds, *effect, *brightness, *j);
                leds.start();
            });
    }
}
