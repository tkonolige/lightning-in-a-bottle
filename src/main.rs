#![no_std]
#![no_main]

use panic_probe as _;

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
    struct Shared {}
    #[local]
    struct Local {
        timer: hal::Timer<TIMER1, hal::timer::Periodic>,
        // This is state used locally at every tick of the timer. You can access them via
        // `ctx.local.variable` within the fimer function.
        // If you need some state, add it here.
        // leds: apa102::Apa102<hal::spi::Spi<hal::pac::SPI0>>,
        leds: ws2812::Ws2812<'static, hal::spi::Spi<hal::pac::SPI0>>,
        j: usize,
    }

    // must be # leds * 12 + 40
    static mut SPI_DATA: [u8; 300 * 12 + 40] = [0; 300 * 12 + 40];

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let p0 = hal::gpio::p0::Parts::new(ctx.device.P0);
        let sck = p0.p0_12.into_push_pull_output(Level::Low).degrade();
        let mosi = p0.p0_13.into_push_pull_output(Level::Low).degrade();
        let mut status_led = p0.p0_17.into_push_pull_output(Level::Low).degrade();
        status_led.set_high().unwrap();

        let spi = hal::spi::Spi::new(
            ctx.device.SPI0,
            hal::spi::Pins {
                sck,
                mosi: Some(mosi),
                miso: None,
            },
            hal::spi::Frequency::M4,
            ws2812::MODE,
        );
        let leds = ws2812::Ws2812::new(spi, unsafe { &mut SPI_DATA });

        let mut timer = hal::Timer::periodic(ctx.device.TIMER1);
        timer.enable_interrupt();
        let update_delay: u32 = 10_000;
        timer.start(update_delay); // 100 times a second

        (
            Shared {},
            Local {
                leds,
                timer,
                j: 0,
            },
            init::Monotonics(),
        )
    }

    // This is called 100 times a second
    #[task(binds=TIMER1, local=[leds, timer, j])]
    fn timer1(ctx: timer1::Context) {
        // this resets the timer so it doesn't fire until the timeout is hit again
        ctx.local
            .timer
            .event_compare_cc0()
            .write(|x| unsafe { x.bits(0) });

        ctx.local
            .leds
            .write(gamma(
                // This code computes the actual color. It returns an iterator of length equal to
                // the number of leds
                (0..300).map(|i| {
                    hsv2rgb(Hsv {
                        hue: ((i * 3 + *ctx.local.j / 3) % 256) as u8,
                        sat: 150,
                        // the led strip can draw a LOT of power if this value is set high.
                        // Probably safest to leave it at 150 for now
                        val: 150,
                    })
                }),
            ))
            .unwrap();
        *ctx.local.j += 1;
    }
}
