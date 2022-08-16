use core::sync::atomic::{compiler_fence, Ordering};

use nrf52832_hal as hal;

// TODO: better CRC?
const CRC_INIT: u32 = 0x0000_FFFF;
const CRC_POLY: u32 = 0x0001_1021;
pub const MESSAGE_LENGTH: usize = 2;

#[derive(Debug, PartialEq, Eq)]
pub enum Command {
    NextAnimation,
    BrightnessUp,
    BrightnessDown,
    TurnOff,
}

impl Command {
    pub fn to_int(&self) -> u8 {
        match self {
            Self::NextAnimation => 0,
            Self::BrightnessUp => 1,
            Self::BrightnessDown => 2,
            Self::TurnOff => 3,
        }
    }
}

impl TryFrom<u8> for Command {
    type Error = ();

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(Self::NextAnimation),
            1 => Ok(Self::BrightnessUp),
            2 => Ok(Self::BrightnessDown),
            3 => Ok(Self::TurnOff),
            _ => Err(()),
        }
    }
}

pub struct Message {
    pub ack_number: u8,
    pub cmd: Command,
}

impl Message {
    pub fn serialize(&self) -> [u8; MESSAGE_LENGTH] {
        let mut buf: [u8; MESSAGE_LENGTH] = [0; MESSAGE_LENGTH];
        buf[0] = self.ack_number;
        buf[1] = self.cmd.to_int();
        buf
    }

    pub fn from(msg: &[u8; MESSAGE_LENGTH]) -> Self {
        Self {
            ack_number: msg[0],
            cmd: msg[1].try_into().unwrap(),
        }
    }
}

#[derive(PartialEq, Eq, defmt::Format)]
pub enum RadioState {
    Sending,
    Receiving,
    Off,
}

pub struct Radio {
    radio: hal::pac::RADIO,
    addr: u8,
    recv_from: u8,
    state: RadioState,
}

impl Radio {
    pub fn new(radio: hal::pac::RADIO, addr: u8, recv_from: u8) -> Self {
        // Disables all interrupts, Nordic's code writes to all bits, seems to be okay
        radio.intenclr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
        // use proprietary radio protocol
        radio.mode.write(|w| w.mode().nrf_2mbit());
        // set power level
        radio.txpower.write(|w| w.txpower().pos4d_bm());

        // TODO, these numbers seem to work well
        let base0 = 0xe7e7e7e7;
        let base1 = 0x43434343;
        let prefix0 = 0x23c343e7;
        let prefix1 = 0x13e363a3;

        radio
            .shorts
            .write(|w| w.ready_start().enabled().end_disable().enabled());

        // Enable fast ramp-up
        radio.modecnf0.modify(|_, w| w.ru().fast());

        unsafe {
            radio
                .pcnf0
                .write(|w| w.lflen().bits(0).s1len().bits(0).s0len().bit(false)); // packet size is statically known

            radio.pcnf1.write(|w| {
                w.maxlen()
                    .bits(MESSAGE_LENGTH as u8 + 4)
                    .statlen()
                    .bits(MESSAGE_LENGTH as u8) // number of bytes on air, static size
                    // 4-Byte Base Address + 1-Byte Address Prefix
                    .balen()
                    .bits(4)
                    // Nordic's code doesn't use whitening, maybe enable in the future ?
                    // .whiteen()
                    // .set_bit()
                    .endian()
                    .big()
            });

            radio
                .crcinit
                .write(|w| w.crcinit().bits(CRC_INIT & 0x00FF_FFFF));

            radio
                .crcpoly
                .write(|w| w.crcpoly().bits(CRC_POLY & 0x00FF_FFFF));

            radio.crccnf.write(|w| w.len().two());

            radio.base0.write(|w| w.bits(base0));
            radio.base1.write(|w| w.bits(base1));

            radio.prefix0.write(|w| w.bits(prefix0));
            radio.prefix1.write(|w| w.bits(prefix1));

            let frequency = 2;
            radio.frequency.write(|w| w.frequency().bits(frequency));
        }

        Self {
            radio,
            addr,
            recv_from,
            state: RadioState::Off,
        }
    }

    pub fn send(&mut self, buf: &mut [u8; MESSAGE_LENGTH]) {
        self.send_from(self.addr, buf);
    }

    pub fn send_from(&mut self, from: u8, buf: &mut [u8; MESSAGE_LENGTH]) {
        // Essential! Wait till radio is disabled
        // while self.radio.events_disabled.read().bits() != 0 {}
        // enable "disabled" interrupt
        self.radio.intenset.write(|w| w.disabled().set_bit());
        unsafe {
            self.radio.txaddress.write(|w| w.txaddress().bits(from)); // TODO: address, clean up
                                                                      // self.radio.rxaddresses.write(|w| w.bits(1)); // TODO: address

            self.radio
                .packetptr
                .write(|w| w.bits(buf.as_mut_ptr() as u32));

            // unsafe {
            //     let p = core::slice::from_raw_parts(buf.as_mut_ptr(), 4);
            //     defmt::info!("sending {:#?}", p);
            // }
            self.radio.events_address.write(|w| w.bits(0)); // TODO: address
            self.radio.events_disabled.reset();
            self.radio.events_ready.reset();
            self.radio.events_end.reset();
            self.radio.events_payload.write(|w| w.bits(0)); // do we need this ? Probably not
                                                            // TODO: figure out how to enable this
                                                            // self.radio.shorts.modify(|_, w|
                                                            //     w.disabled_rxen().enabled() // immediately go to receive
                                                            // );

            // "Preceding reads and writes cannot be moved past subsequent writes."
            compiler_fence(Ordering::Release);

            // GO!
            self.state = RadioState::Sending;
            self.radio.tasks_txen.write(|w| w.bits(1));

            compiler_fence(Ordering::SeqCst);
            // defmt::info!("Sending");

            // busy wait for send
            // while self.radio.events_disabled.read().bits() == 0 {}

            // "Subsequent reads and writes cannot be moved ahead of preceding reads."
            compiler_fence(Ordering::Acquire);
            // defmt::info!("Sent");
        }
    }

    pub fn send_complete(&mut self) {
        // TODO: send should really immediately go to receiving
        // defmt::info!("Disbaled? {} ready {} end {}", self.radio.events_disabled.read().bits(), self.radio.events_ready.read().bits(), self.radio.events_end.read().bits());
        self.radio.intenclr.write(|w| w.disabled().set_bit());
        self.radio.events_disabled.reset();
        self.radio.events_ready.reset();
        self.radio.events_end.reset();
        self.state = RadioState::Off;
        compiler_fence(Ordering::SeqCst);
    }

    pub fn recv(&mut self, buf: &mut [u8; MESSAGE_LENGTH]) {
        while self.radio.events_disabled.read().bits() != 0 {}
        // enable end of receive interrupt
        self.radio.intenset.write(|w| w.disabled().set_bit());
        // self.radio
        //     .rxaddresses
        //     .write(|w| w.addr0().set_bit().addr1().set_bit()); // receive on addr0
        //                                                        // .write(|w| unsafe{w.bits(3)}); // receive on addr0
        self.radio
            .rxaddresses
            .write(|w| unsafe { w.bits(self.recv_from as u32) });

        unsafe {
            self.radio
                .packetptr
                .write(|w| w.bits((buf as *mut u8) as u32));
            self.radio.events_address.write(|w| w.bits(0));
            self.radio.events_disabled.reset();
            self.radio.events_ready.reset();
            self.radio.events_end.reset();
            self.radio.events_payload.write(|w| w.bits(0)); // TODO: do we need this ? Probably not

            // "Preceding reads and writes cannot be moved past subsequent writes."
            compiler_fence(Ordering::Release);

            // GO!
            self.state = RadioState::Receiving;
            self.radio.tasks_rxen.write(|w| w.bits(1));
        }
        compiler_fence(Ordering::SeqCst);
    }

    pub fn state(&self) -> &RadioState {
        &self.state
    }

    pub fn read_packet(&mut self) -> (bool, u8) {
        let crc_error = self.radio.crcstatus.read().crcstatus().is_crcerror();

        let addr = self.radio.rxmatch.read().bits();

        // self.radio.shorts.modify(|_, w|
        //     w.disabled_rxen().disabled()
        // );

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::Acquire);
        // clear all events
        self.radio.events_ready.reset();
        self.radio.events_end.reset();
        self.radio.events_disabled.reset();
        self.state = RadioState::Off;
        compiler_fence(Ordering::SeqCst);
        if crc_error {
            (false, 0)
        } else {
            (true, addr as u8)
        }
    }

    pub fn stop(&mut self) {
        // defmt::info!("STOPPPING {}", self.radio.state.read().state().is_rx());
        if self.radio.state.read().state().is_rx() {
            // defmt::info!("for real");
            self.radio.tasks_stop.write(|w| unsafe { w.bits(1) });
            // busy wait for the radio to stop
            self.state = RadioState::Off;
            compiler_fence(Ordering::SeqCst);
        }
    }
}
