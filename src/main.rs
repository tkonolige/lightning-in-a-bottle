#![no_std]
#![no_main]

use core::ops::Deref;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use hal::gpio::{Output, Pin, PushPull};
use hal::prelude::*;
use nrf52832_hal as hal;
use panic_probe as _;
use smart_leds::RGB8;

#[repr(C, align(8))]
pub struct LEDStrip<Timer, SPIM, PPI> {
    control_bits: &'static mut [u8; 6144], // 500 leds * 12 bytes/led + 25 byte for reset rounded up to the nearest multiple of 256 bytes
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
        control_bits: &'static mut [u8; 6144],
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
    use hal::pac::{SPIM0, TIMER1, TIMER2, TIMER0, SPIM1, SPIM2, TIMER3};
    use hal::timer::Instance;
    use hal::ppi::{Ppi0, Ppi1, Ppi2};
    use hal::prelude::*;
    use nrf52832_hal as hal;
    use smart_leds::{gamma, hsv::hsv2rgb, hsv::Hsv, SmartLedsWrite, RGB8};

    static mut led_bits: [u8; 6144] = [0; 6144];
    static mut led_bits2: [u8; 6144] = [0; 6144];
    static mut led_bits3: [u8; 6144] = [0; 6144];

    #[shared]
    struct Shared {
        leds: crate::LEDStrip<TIMER2, SPIM0, Ppi0>,
        leds2: crate::LEDStrip<TIMER0, SPIM1, Ppi1>,
        leds3: crate::LEDStrip<TIMER1, SPIM2, Ppi2>,
        j: usize,
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
        let mosi = p0.p0_13.into_push_pull_output(Level::Low).degrade();
        let sck_1 = p0.p0_11.into_push_pull_output(Level::Low).degrade();
        let mut leds =
            crate::LEDStrip::new(ctx.device.TIMER2, ctx.device.SPIM0, ppi_channels.ppi0, mosi, sck_1, unsafe {&mut led_bits});
        leds.configure();
        leds.start();

        let miso = p0.p0_14.into_push_pull_output(Level::Low).degrade();
        let sck_2 = p0.p0_10.into_push_pull_output(Level::Low).degrade();
        let mut leds2 =
            crate::LEDStrip::new(ctx.device.TIMER0, ctx.device.SPIM1, ppi_channels.ppi1, miso, sck_2, unsafe{&mut led_bits2});
        leds2.configure();
        leds2.start();

        let p15 = p0.p0_15.into_push_pull_output(Level::Low).degrade();
        let sck_3 = p0.p0_09.into_push_pull_output(Level::Low).degrade();
        let mut leds3 =
            crate::LEDStrip::new(ctx.device.TIMER1, ctx.device.SPIM2, ppi_channels.ppi2, p15, sck_3, unsafe{&mut led_bits2});
        leds3.configure();
        leds3.start();

        let mut timer = hal::Timer::periodic(ctx.device.TIMER3);
        timer.enable_interrupt();
        let update_delay: u32 = 10_000;
        timer.start(update_delay); // 100 times a second

        (Shared { leds, leds2, leds3, j: 0  }, Local { timer, }, init::Monotonics())
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

    #[task(binds=TIMER2, shared=[leds, j])]
    fn timer2(ctx: timer2::Context) {
        (ctx.shared.leds, ctx.shared.j).lock(|leds, j| {
            leds.interrupt();
            leds.set_data(gamma(
                // This code computes the actual color. It returns an iterator of length equal to
                // the number of leds
                (0..500).map(|i| {
                    hsv2rgb(Hsv {
                        hue: ((i * 3 + *j / 3) % 256) as u8,
                        sat: 150,
                        // the led strip can draw a LOT of power if this value is set high.
                        // Probably safest to leave it at 150 for now
                        val: 150,
                    })
                }),
            ));
            leds.start();
        });
    }

    #[task(binds=TIMER0, shared=[leds2, j], priority=2)]
    fn timer0(ctx: timer0::Context) {
        (ctx.shared.leds2, ctx.shared.j).lock(|leds, j| {
            leds.interrupt();
            leds.set_data(gamma(
                // This code computes the actual color. It returns an iterator of length equal to
                // the number of leds
                (0..500).map(|i| {
                    hsv2rgb(Hsv {
                        hue: ((i * 3 + *j / 3) % 256) as u8,
                        sat: 150,
                        // the led strip can draw a LOT of power if this value is set high.
                        // Probably safest to leave it at 150 for now
                        val: 150,
                    })
                }),
            ));
            leds.start();
        });
    }

    #[task(binds=TIMER1, shared=[leds3, j], priority=2)]
    fn timer1(ctx: timer1::Context) {
        (ctx.shared.leds3, ctx.shared.j).lock(|leds, j| {
            leds.interrupt();
            leds.set_data(gamma(
                // This code computes the actual color. It returns an iterator of length equal to
                // the number of leds
                (0..500).map(|i| {
                    hsv2rgb(Hsv {
                        hue: ((i * 3 + *j / 3) % 256) as u8,
                        sat: 150,
                        // the led strip can draw a LOT of power if this value is set high.
                        // Probably safest to leave it at 150 for now
                        val: 150,
                    })
                }),
            ));
            leds.start();
        });
    }
}
