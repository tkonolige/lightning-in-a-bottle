#![no_std]
#![no_main]

use panic_probe as _;

#[rtic::app(device = crate::lightstrip::hal::pac, peripherals=true)]
mod lightstrip {
    use apa102_spi as apa102;
    use defmt_rtt as _; // for logging
    use hal::gpio::Level;
    use hal::pac::TIMER1;
    use hal::prelude::*;
    use nrf52832_hal as hal;
    use smart_leds::{gamma, hsv::hsv2rgb, hsv::Hsv, SmartLedsWrite, RGB8};
    #[shared]
    struct Shared {}
    #[local]
    struct Local {
        timer: hal::Timer<TIMER1, hal::timer::Periodic>,
        // This is state used locally at every tick of the timer. You can access them via
        // `ctx.local.variable` within the fimer function.
        // If you need some state, add it here.
        leds: apa102::Apa102<hal::spi::Spi<hal::pac::SPI0>>,
        data: [RGB8; 150], // # TODO: change depending on length of strip
        j: usize,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let p0 = hal::gpio::p0::Parts::new(ctx.device.P0);
        let sck = p0.p0_12.into_push_pull_output(Level::Low).degrade();
        let mosi = p0.p0_13.into_push_pull_output(Level::Low).degrade();

        let spi = hal::spi::Spi::new(
            ctx.device.SPI0,
            hal::spi::Pins {
                sck,
                mosi: Some(mosi),
                miso: None,
            },
            hal::spi::Frequency::M4,
            apa102::MODE,
        );
        let mut leds = apa102::Apa102::new(spi);
        let mut data = [RGB8::default(); 150];

        let mut timer = hal::Timer::periodic(ctx.device.TIMER1);
        timer.enable_interrupt();
        let update_delay: u32 = 10_000;
        timer.start(update_delay); // 100 times a second

        let j = 0;
        for i in 0..150 {
            data[i] = hsv2rgb(Hsv {
                hue: ((i * 3 + j) % 256) as u8,
                sat: 255,
                val: 100,
            });
        }
        leds.write(gamma(data.iter().cloned())).unwrap();

        (
            Shared {},
            Local {
                leds,
                timer,
                data,
                j,
            },
            init::Monotonics(),
        )
    }

    // This is called 100 times a second
    #[task(binds=TIMER1, local=[leds, timer, data, j])]
    fn timer1(ctx: timer1::Context) {
        ctx.local
            .timer
            .event_compare_cc0()
            .write(|x| unsafe { x.bits(0) });
        for i in 0..150 {
            ctx.local.data[i] = hsv2rgb(Hsv {
                hue: ((i * 3 + *ctx.local.j / 3) % 256) as u8,
                sat: 100,
                val: 255,
            });
        }
        ctx.local
            .leds
            .write(gamma(ctx.local.data.iter().cloned()))
            .unwrap();
        *ctx.local.j += 1;
    }
}
