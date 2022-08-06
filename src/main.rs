#![no_std]
#![no_main]

use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use hal::gpio::{Output, Pin, PushPull};
use hal::prelude::*;
use nrf52832_hal as hal;
use panic_probe as _;
use smart_leds::{ RGB8};

#[repr(C, align(8))]
pub struct LEDStrip {
    control_bits: [u8; 6144], // 500 leds * 12 bytes/led + 20 byte for reset rounded up to the nearest multiple of 256 bytes
    counter: hal::pac::TIMER2,
    spi: hal::pac::SPIM0,
    ppi: hal::ppi::Ppi0,
    pin: Pin<Output<PushPull>>,
}
impl LEDStrip {
    fn new(
        counter: hal::pac::TIMER2,
        spi: hal::pac::SPIM0,
        ppi: hal::ppi::Ppi0,
        pin: Pin<Output<PushPull>>,
    ) -> LEDStrip {
        LEDStrip {
            control_bits: [0; 6144],
            counter,
            spi,
            ppi,
            pin,
        }
    }

    fn configure(&mut self) {
        // output on pin 13
        self.spi.psel.mosi.write(|x| {
            unsafe { x.bits(self.pin.psel_bits()) };
            x.connect().connected()
        });
        // dont need the other outputs
        self.spi.psel.sck.write(|x| {
            unsafe { x.pin().bits(12) };
            x.connect().connected()
        });
        self.spi.psel.miso.write(|x| {
            unsafe { x.pin().bits(11) };
            x.connect().connected()
        });
        // self.spi.psel.sck.write(|x|x.connect().disconnected());
        // self.spi.psel.miso.write(|x|x.connect().disconnected());
        self.spi.enable.write(|x| x.enable().enabled());
        self.spi.frequency.write(|x| x.frequency().m4());
        self.spi
            .txd
            .ptr
            .write(|x| unsafe { x.ptr().bits(self.control_bits.as_ptr() as u32) });
        // read 256 bytes at a time
        self.spi.txd.maxcnt.write(|x| unsafe { x.maxcnt().bits(0xff) });
        // advance pointer 256 bytes each time
        self.spi.txd.list.write(|x| x.list().array_list());
        // receive nothing
        self.spi.rxd.maxcnt.write(|x| unsafe { x.maxcnt().bits(0) });
        // start next transfer as soon as a transfer is finished
        self.spi.shorts.write(|x| x.end_start().enabled());
        // self.spi
        //     .intenset
        //     .write(|x| x.endtx().set().endrx().set().end().set());

        self.counter.mode.write(|x| x.mode().counter());
        self.counter.bitmode.write(|x| x.bitmode()._32bit());
        self.counter.tasks_clear.write(|x| unsafe { x.bits(1) });
        // count up to the total number of sends
        self.counter.cc[0].write(|x| unsafe { x.bits((self.control_bits.len() / 256 - 1) as u32) });
        self.counter.intenset.write(|x| x.compare0().set());
        self.counter.tasks_start.write(|x| unsafe {x.bits(1)});
        self.ppi.set_task_endpoint(&self.counter.tasks_count);
        self.ppi.set_event_endpoint(&self.spi.events_end);
        self.ppi.enable();

        // enable spi
        self.spi
            .config
            .write(|x| x.order().msb_first().cpol().active_low().cpha().leading());
        self.spi.orc.write(|x| unsafe { x.orc().bits(0) });
        compiler_fence(SeqCst);
    }

    fn set_data<T>(&mut self, iterator: T)
    where
        T: Iterator<Item = RGB8>,
    {
        let patterns = [0b1000_1000, 0b1000_1110, 0b1110_1000, 0b1110_1110];
        // let patterns = [0b1110_1110, 0b1110_1110, 0b1110_1110, 0b1110_1110];
        for (i, rgb) in iterator.enumerate() {
            assert!(i < 500, "Max 500 leds");
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

#[rtic::app(device = crate::lightstrip::hal::pac, peripherals=true)]
mod lightstrip {
    // use apa102_spi as apa102;
    use defmt_rtt as _; // for logging
    use hal::gpio::Level;
    use hal::pac::TIMER1;
    use hal::prelude::*;
    use nrf52832_hal as hal;
    use smart_leds::{gamma, hsv::hsv2rgb, hsv::Hsv, SmartLedsWrite, RGB8};
    use ws2812_spi::prerendered as ws2812;
    #[shared]
    struct Shared {
        leds: crate::LEDStrip,
    }
    #[local]
    struct Local {
        timer: hal::Timer<TIMER1, hal::timer::Periodic>,
        // This is state used locally at every tick of the timer. You can access them via
        // `ctx.local.variable` within the fimer function.
        // If you need some state, add it here.
        // leds: apa102::Apa102<hal::spi::Spi<hal::pac::SPI0>>,
        // leds: ws2812::Ws2812<'static, hal::spi::Spi<hal::pac::SPI0>>,
        j: usize,
    }

    // must be # leds * 12 + 40
    // static mut SPI_DATA: [u8; 300 * 12 + 40] = [0; 300 * 12 + 40];

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let p0 = hal::gpio::p0::Parts::new(ctx.device.P0);
        let _clocks = hal::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();
        let sck = p0.p0_12.into_push_pull_output(Level::Low).degrade();
        let mosi = p0.p0_13.into_push_pull_output(Level::Low).degrade();
        let miso = p0.p0_11.into_pulldown_input().degrade();
        let mut status_led = p0.p0_17.into_push_pull_output(Level::Low).degrade();
        status_led.set_high().unwrap();

        let ppi_channels = hal::ppi::Parts::new(ctx.device.PPI);
        let mut leds =
            crate::LEDStrip::new(ctx.device.TIMER2, ctx.device.SPIM0, ppi_channels.ppi0, mosi);
        leds.configure();
        leds.set_data(gamma(
            // This code computes the actual color. It returns an iterator of length equal to
            // the number of leds
            (0..500).map(|i| {
                hsv2rgb(Hsv {
                    hue: 0,
                    sat: 0,
                    // the led strip can draw a LOT of power if this value is set high.
                    // Probably safest to leave it at 150 for now
                    val: 100,
                })
            }),
        ));
        leds.start();

        let mut timer = hal::Timer::periodic(ctx.device.TIMER1);
        timer.enable_interrupt();
        let update_delay: u32 = 10_000;
        timer.start(update_delay); // 100 times a second

        (Shared { leds }, Local { timer, j: 0 }, init::Monotonics())
    }

    // This is called 100 times a second
    #[task(binds=TIMER1, local=[timer], shared=[leds])]
    fn timer1(mut ctx: timer1::Context) {
        // this resets the timer so it doesn't fire until the timeout is hit again
        ctx.local
            .timer
            .event_compare_cc0()
            .write(|x| unsafe { x.bits(0) });

        ctx.shared.leds.lock(|leds| {
            // leds.configure();
            // leds.set_data(gamma(
            //     // This code computes the actual color. It returns an iterator of length equal to
            //     // the number of leds
            //     (0..500).map(|i| {
            //         hsv2rgb(Hsv {
            //             hue: ((i * 3 + *ctx.local.j / 3) % 256) as u8,
            //             sat: 150,
            //             // the led strip can draw a LOT of power if this value is set high.
            //             // Probably safest to leave it at 150 for now
            //             val: 100,
            //         })
            //     }),
            // ));
            // leds.start();
        });
        // *ctx.local.j += 1;
    }

    #[task(binds=TIMER2, shared=[leds], local=[j])]
    fn timer2(mut ctx: timer2::Context) {
        ctx.shared.leds.lock(|leds| {
            leds.interrupt();
            leds.set_data(gamma(
                // This code computes the actual color. It returns an iterator of length equal to
                // the number of leds
                (0..500).map(|i| {
                    hsv2rgb(Hsv {
                        hue: ((i * 3 + *ctx.local.j / 3) % 256) as u8,
                        sat: 150,
                        // the led strip can draw a LOT of power if this value is set high.
                        // Probably safest to leave it at 150 for now
                        val: 100,
                    })
                }),
            ));
            leds.start();
        });
        *ctx.local.j += 1;
    }
}
