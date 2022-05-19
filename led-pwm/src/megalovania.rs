#![no_std]
#![no_main]

//! Use the 16-bit timer of the AVR to play the first four measures
//! of the Megalovania chiptune using a buzzer connected to pin d9
//! (the OC1A pin).
//!
//! I created this example with the hope that it can be incorporated as an example in
//! https://github.com/Rahix/avr-hal/tree/main/examples/arduino-uno
//! and this example is therefore licensed under whatever terms are necessary to accomplish this.
//! - thoth950@gmail.com

use crate::timer_common::WaveformGenerationMode;
use arduino_hal::hal::port::PB1;
use arduino_hal::pac::TC1;
use arduino_hal::port::Pin;
use arduino_hal::{default_serial, delay_ms};
use avr_device::atmega328p::tc1::tccr1a::COM1A_A;
use avr_device::atmega328p::tc1::tccr1b::CS1_A;
use avr_hal_generic::clock::Clock;
use avr_hal_generic::port::mode::{Io, Output};
use avr_hal_generic::port::PinOps;
use avr_hal_generic::void;
use panic_halt as _;
use ufmt::{uWrite, uwriteln};
use void::ResultVoidExt;

mod timer_common;

#[arduino_hal::entry]
fn main() -> ! {
    rust_arduino_runtime::arduino_main_init();

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = default_serial!(dp, pins, 115200);

    let _ = uwriteln!(&mut serial, "do you wanna have a bad time?");

    megalovania_first_four_measures_loop(pins.d9.into_output(), pins.d13, dp.TC1, &mut serial);
}

fn megalovania_first_four_measures_loop<I: Io, P2: PinOps, W: uWrite<Error = void::Void>>(
    d9: Pin<Output, PB1>, // on the Arduino Uno, this is the OCR1 pin
    led: Pin<I, P2>,
    timer1: TC1,
    serial: &mut W,
) -> ! {
    let _oc1a = d9;

    let mut led = led.into_output();

    timer_common::rig_timer_1a(
        &timer1,
        serial,
        COM1A_A::MATCH_CLEAR,
        WaveformGenerationMode::PWMPhaseCorrectICR1, // use the OCR1 value to control the frequency of the waveform
        CS1_A::DIRECT,
    )
    .void_unwrap();

    let seq1 = pattern1(D4_FREQ);
    let seq2 = pattern1(C4_FREQ);
    let seq3 = pattern1(B3F_FREQ);
    let seq4 = pattern1(C4_FREQ);
    loop {
        play_sequence(&seq1, &timer1);
        play_sequence(&seq2, &timer1);
        play_sequence(&seq3, &timer1);
        play_sequence(&seq4, &timer1);

        led.toggle();
    }
}

fn play_sequence<'a, I>(seq: I, timer1: &TC1)
where
    I: IntoIterator<Item = &'a (f32, u16)>,
{
    for &(freq, duration) in seq.into_iter() {
        if freq > 0.0 {
            play_note(freq, duration, timer1);
        } else {
            // rest
            delay_ms(duration);
        }
    }
}

const B3F_FREQ: f32 = 233.08;
const C4_FREQ: f32 = 261.63;
const D4_FREQ: f32 = 293.66;
const F4_FREQ: f32 = 349.23;
const G4_FREQ: f32 = 392.0;
const G4S_FREQ: f32 = 415.30;
const A4_FREQ: f32 = 440.0;
const D5_FREQ: f32 = D4_FREQ * 2.;

const SIXTEENTH: u16 = 1000 * 60 / 120 / 4;

fn pattern1(opening: f32) -> [(f32, u16); 13] {
    [
        (opening, SIXTEENTH),
        (opening, SIXTEENTH),
        (D5_FREQ, SIXTEENTH * 2),
        (A4_FREQ, SIXTEENTH * 2),
        (0.0, SIXTEENTH),
        (G4S_FREQ, SIXTEENTH),
        (0.0, SIXTEENTH),
        (G4_FREQ, SIXTEENTH),
        (0.0, SIXTEENTH),
        //(F4_FREQ, EIGTH),
        (F4_FREQ, SIXTEENTH * 2),
        (D4_FREQ, SIXTEENTH),
        (F4_FREQ, SIXTEENTH),
        (G4_FREQ, SIXTEENTH),
    ]
}

fn play_note(freq: f32, millis: u16, timer1: &TC1) {
    const CLOCK: f32 = arduino_hal::DefaultClock::FREQ as f32;
    // how many ticks in a square wave period
    let ticks: u16 = (CLOCK / freq) as u16;

    // set the ICR1 to control the period (and frequency) of the PWM waveform
    timer1.icr1.write(|w| unsafe { w.bits(ticks) });
    // set the OCR1A to half the ICR1, to make a symmetric square wave
    timer1.ocr1a.write(|w| unsafe { w.bits(ticks / 2) });

    // play the note for 7/10 of the duration
    let on = millis * 7 / 10;
    arduino_hal::delay_ms(on);

    // set OCR1a so high that it never triggers and the wave stops oscillating
    timer1.ocr1a.write(|w| unsafe { w.bits(0xffff) });

    // rest for the remainder of the duration
    arduino_hal::delay_ms(millis - on);
}

/*

cargo build --release &&
elf=`echo target/avr-atmega328p/release/megalovania.elf` &&
avrdude -C /etc/avrdude.conf -v -p atmega328p -c arduino -P /dev/ttyACM0  -D -Uflash:w:$elf:e

 */
