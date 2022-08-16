#![no_std]
#![no_main]

mod radio;
use panic_probe as _;

#[rtic::app(device = crate::remote::hal::pac, peripherals=true)]
mod remote {
    // use apa102_spi as apa102;
    use core::sync::atomic::{compiler_fence, Ordering};
    use defmt_rtt as _; // for logging
    use hal::gpio::{Input, Level, Output, Pin, PullUp, PushPull};
    use hal::pac::{GPIOTE, P0, POWER, TIMER2, TIMER3};
    use hal::prelude::*;
    use hal::timer::Instance;
    use nrf52832_hal as hal;
    use radio::{Command, Message, Radio, RadioState, MESSAGE_LENGTH};

    use crate::*;

    static mut RECV_BUF: [u8; MESSAGE_LENGTH] = [0; MESSAGE_LENGTH];
    static mut SEND_BUF: [u8; MESSAGE_LENGTH] = [0; MESSAGE_LENGTH];
    const ACK_WAIT_TIME: u32 = 1000; // 100 microsecond wait

    const SYSTEMOFF_TIME: u32 = 10000;

    #[shared]
    struct Shared {
        radio: Radio,
        ack_timer: TIMER2,
        ack_number: u8,
        ack_failures: u8,
        waiting_to_send: bool,
        waiting_for_ack: bool,
        systemoff_timer: TIMER3,
        gpiote: GPIOTE,
        msg: Message,
        status_led: Pin<Output<PushPull>>,
    }
    #[local]
    struct Local {
        pin_anim: Pin<Input<PullUp>>,
        pin_br_up: Pin<Input<PullUp>>,
        pin_br_down: Pin<Input<PullUp>>,
        pin_off: Pin<Input<PullUp>>,
        power: POWER,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let p0 = hal::gpio::p0::Parts::new(ctx.device.P0);
        let _clocks = hal::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();

        let mut radio = Radio::new(ctx.device.RADIO, 1, 0xf);

        // setup timer to check if ack is received
        let ack_timer = ctx.device.TIMER2;
        ack_timer.set_shorts_oneshot();
        ack_timer.set_oneshot();
        ack_timer.tasks_clear.write(|w| unsafe { w.bits(1) });
        ack_timer.cc[0].write(|w| unsafe { w.cc().bits(ACK_WAIT_TIME) });
        ack_timer.intenset.write(|w| w.compare0().set());

        // 16MHz / (2 ^ 15) = 488.28125Hz
        let systemoff_timer = ctx.device.TIMER3;
        systemoff_timer.set_shorts_oneshot();
        systemoff_timer
            .prescaler
            .write(|w| unsafe { w.prescaler().bits(0b1111) });
        systemoff_timer.bitmode.write(|w| w.bitmode()._32bit());
        systemoff_timer.tasks_clear.write(|w| unsafe { w.bits(1) });
        systemoff_timer.cc[0].write(|w| unsafe { w.cc().bits(SYSTEMOFF_TIME) });
        systemoff_timer.events_compare[0].reset();
        systemoff_timer.intenset.write(|w| w.compare0().set());
        systemoff_timer.tasks_start.write(|w| unsafe { w.bits(1) });

        let gpiote = ctx.device.GPIOTE;
        gpiote.intenclr.write(|w| w.port().set_bit());
        gpiote.events_port.reset();
        // enable interrupt for sense event
        gpiote.intenset.write(|w| w.port().set_bit());
        // make sure no latch is set
        let port = P0::ptr();
        let latch = unsafe { &(*port).latch };
        latch.write(|w| unsafe { w.bits(0) });

        let pin_anim = p0.p0_04.into_pullup_input().degrade();
        let pin_br_up = p0.p0_05.into_pullup_input().degrade();
        let pin_br_down = p0.p0_30.into_pullup_input().degrade();
        let pin_off = p0.p0_02.into_pullup_input().degrade();
        compiler_fence(Ordering::SeqCst);
        unsafe {
            (*(port)).pin_cnf[pin_anim.pin() as usize].write(|w| {
                w.dir()
                    .input()
                    .input()
                    .connect()
                    .pull()
                    .pullup()
                    .sense()
                    .low()
            });
            (*(port)).pin_cnf[pin_br_up.pin() as usize].write(|w| {
                w.dir()
                    .input()
                    .input()
                    .connect()
                    .pull()
                    .pullup()
                    .sense()
                    .low()
            });
            (*(port)).pin_cnf[pin_br_down.pin() as usize].write(|w| {
                w.dir()
                    .input()
                    .input()
                    .connect()
                    .pull()
                    .pullup()
                    .sense()
                    .low()
            });
            (*(port)).pin_cnf[pin_off.pin() as usize].write(|w| {
                w.dir()
                    .input()
                    .input()
                    .connect()
                    .pull()
                    .pullup()
                    .sense()
                    .low()
            });
        }

        let status_led = p0.p0_11.into_push_pull_output(Level::Low).degrade();

        (
            Shared {
                radio,
                ack_timer,
                ack_number: 0,
                ack_failures: 0,
                waiting_for_ack: false,
                waiting_to_send: false,
                systemoff_timer,
                gpiote,
                msg: Message {
                    cmd: Command::TurnOff,
                    ack_number: 0,
                },
                status_led,
            },
            Local {
                pin_anim,
                pin_br_up,
                pin_br_down,
                pin_off,
                power: ctx.device.POWER,
            },
            init::Monotonics(),
        )
    }

    fn send(radio: &mut Radio, ack_timer: &mut TIMER2, waiting_for_ack: &mut bool) {
        compiler_fence(Ordering::SeqCst);
        *waiting_for_ack = true;
        ack_timer.events_compare[0].reset();
        compiler_fence(Ordering::SeqCst);
        // defmt::info!("sending");
        unsafe {
            radio.send(&mut SEND_BUF);
        }
        compiler_fence(Ordering::SeqCst);
    }

    #[task(binds=GPIOTE, shared=[radio, systemoff_timer, gpiote, ack_number, status_led, ack_timer, waiting_for_ack, msg], local=[pin_anim, pin_br_up, pin_br_down, pin_off])]
    fn keypress(mut ctx: keypress::Context) {
        let port = P0::ptr();
        let latch = unsafe { &(*port).latch };
        latch.write(|w| unsafe { w.bits(0) });
        ctx.shared.gpiote.lock(|gpiote| gpiote.events_port.reset());
        // kick systemoff timer
        ctx.shared.systemoff_timer.lock(|systemoff_timer| {
            systemoff_timer.tasks_clear.write(|w| unsafe { w.bits(1) });
        });

        let cmd = if ctx.local.pin_anim.is_low().unwrap() {
            Command::NextAnimation
        } else if ctx.local.pin_br_up.is_low().unwrap() {
            Command::BrightnessUp
        } else if ctx.local.pin_br_down.is_low().unwrap() {
        ctx.shared.status_led.lock(|led| led.set_high().unwrap());
            Command::BrightnessDown
        } else if ctx.local.pin_off.is_low().unwrap() {
            Command::TurnOff
        } else {
            return;
        };
        let message = Message {
            cmd,
            ack_number: ctx.shared.ack_number.lock(|ack_number| {
                let a = *ack_number;
                *ack_number += 1;
                a
            }),
        };

        // send message
        unsafe {
            SEND_BUF = message.serialize();
        }
        (
            ctx.shared.radio,
            ctx.shared.ack_timer,
            ctx.shared.waiting_for_ack,
            ctx.shared.msg,
        )
            .lock(|radio, ack_timer, waiting_for_ack, msg| {
                *msg = message;
                send(radio, ack_timer, waiting_for_ack)
            });
    }

    #[task(
        binds = RADIO,
        shared = [
            radio,
            waiting_to_send,
            ack_timer,
            ack_failures,
            ack_number,
            waiting_for_ack,
            msg,
            status_led,
        ],
        priority = 3
    )]
    fn radio_task(mut ctx: radio_task::Context) {
        ctx.shared.status_led.lock(|led| led.set_low());
        (
            ctx.shared.radio,
            ctx.shared.waiting_to_send,
            ctx.shared.ack_timer,
            ctx.shared.ack_failures,
            ctx.shared.waiting_for_ack,
            ctx.shared.ack_number,
            ctx.shared.msg,
        )
            // TODO: Radio send/recv should not fire until radio is already disabled. ie set
            // disabled interrupt and then dispatch to the correct task
            .lock(
                |mut radio,
                 waiting_to_send,
                 mut ack_timer,
                 ack_failures,
                 waiting_for_ack,
                 ack_number,
                 msg| {
                    if radio.state() == &RadioState::Sending {
                        radio.send_complete();
                        // defmt::info!("Send complete, receiving");

                        if *waiting_to_send {
                            unsafe {
                                SEND_BUF = msg.serialize();
                            }
                            send(radio, ack_timer, waiting_for_ack);
                            *waiting_to_send = false;
                        } else if *waiting_for_ack {
                            // set timer to check for ack timeout, could probably hook this up so
                            // the radio triggers the timer itself
                            ack_timer.events_compare[0].reset();
                            ack_timer.tasks_start.write(|w| unsafe { w.bits(1) });
                            // Wait for ack
                            unsafe {
                                radio.recv(&mut RECV_BUF);
                            }
                            *waiting_for_ack = false;
                            compiler_fence(Ordering::SeqCst);
                        }
                    } else if radio.state() == &RadioState::Receiving {
                        let (success, addr) = radio.read_packet();
                        // defmt::info!("recv from {}", addr);
                        if !success || addr != 0 {
                            return;
                        }

                        // Reset failure count because we got a message back
                        *ack_failures = 0;
                        // Stop ack timer
                        // defmt::info!("canceling timer from radio");
                        ack_timer.tasks_stop.write(|w| unsafe { w.bits(1) });
                        ack_timer.events_compare[0].reset();
                        compiler_fence(Ordering::SeqCst);

                        let msg_ack_number = unsafe { RECV_BUF[0] };

                        // Check if the matrix we received is the same as the current state, if not
                        // send current state. We also send if there are changes that have not been
                        // sent yet.
                        if msg_ack_number != *ack_number || *waiting_to_send {
                            compiler_fence(Ordering::SeqCst);
                            send(&mut radio, &mut ack_timer, waiting_for_ack);
                            *waiting_to_send = false;
                        }
                    }
                },
            );
    }

    #[task(binds = TIMER3,
           shared = [
            systemoff_timer,
            gpiote,
           ],
           local = [
            power,
           ],
           priority=3
           )]
    fn systemoff(mut ctx: systemoff::Context) {
        ctx.shared.systemoff_timer.lock(|systemoff_timer| {
            systemoff_timer.events_compare[0].reset();
            systemoff_timer.intenclr.write(|w| w.compare0().clear());
            systemoff_timer.tasks_stop.write(|w| unsafe { w.bits(1) });
            systemoff_timer
                .tasks_shutdown
                .write(|w| unsafe { w.bits(1) });
        });

        // reset latched state so detect isn't immediate retriggered
        let port = P0::ptr();
        let latch = unsafe { &(*port).latch };
        latch.write(|w| unsafe { w.bits(0) });

        ctx.shared.gpiote.lock(|gpiote| {
            gpiote.events_port.reset();
            // enable interrupt for sense event
            gpiote.intenset.write(|w| w.port().set_bit());
        });

        unsafe {
            (*hal::pac::CLOCK::ptr())
                .tasks_hfclkstop
                .write(|w| w.bits(1));
        }
        ctx.local.power.systemoff.write(|w| w.systemoff().set_bit());

        // ctx.local.led_yellow.set_low().unwrap();

        // defmt::info!("Going into systemoff");
    }
    #[idle()]
    fn idle(cx: idle::Context) -> ! {
        loop {
            rtic::export::wfi()
        }
    }
}
