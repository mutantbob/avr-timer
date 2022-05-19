#![no_std]
#![no_main]

use arduino_hal::pac::tc1::tccr1a::COM1A_A;
use arduino_hal::pac::tc1::tccr1b::CS1_A;
use arduino_hal::pac::TC1;
use arduino_hal::port::Pin;
use arduino_hal::{default_serial, delay_ms};
use avr_hal_generic::port::mode::Io;
use avr_hal_generic::port::PinOps;
use avr_hal_generic::void;
use panic_halt as _;
use ufmt::{uWrite, uwrite, uwriteln};
use void::Void;

#[arduino_hal::entry]
fn main() -> ! {
    rust_arduino_runtime::arduino_main_init();

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = default_serial!(dp, pins, 115200);

    let _ = uwriteln!(&mut serial, "hello from Arduino");

    tone_generator(pins.d9, pins.d13, dp.TC1, &mut serial);
}

fn debug_dump(serial: &mut dyn uWrite<Error = Void>, timer1: &TC1) -> Result<(), Void> {
    /*  use ufmt::UnstableDoAsFormatter;
    uwrite!(serial, "{:x} ", 0x42);
    uwrite!(serial, "{:x} ", 0x1c2);
    serial.do_as_formatter(|serial| {
        ufmt::uDisplayHex::fmt_hex(
            &42u16, //timer1.tifr1.read().bits()
            serial,
            ufmt::HexOptions {
                upper_case: false,
                pad_char: b' ',
                pad_length: 0,
                ox_prefix: false,
            },
        )
    });*/
    uwrite!(
        serial,
        "tccr1a = {}; tccr1b = {};",
        timer1.tccr1a.read().bits(),
        timer1.tccr1b.read().bits()
    )?;
    uwrite!(
        serial,
        " tifr1 = {};"
        timer1.tifr1.read().bits(),
    )?;
    uwrite!(
        serial,
        " timsk1 = {};"
        timer1.timsk1.read().bits(),
    )?;
    uwrite!(
        serial,
        " icr1 = {};"
        timer1.icr1.read().bits(),
    )?;
    uwrite!(
        serial,
        " ocr1a = {};"
        timer1.ocr1a.read().bits(),
    )?;
    serial.write_str("\n")
}

fn tone_generator<I: Io, P1: PinOps, P2: PinOps>(
    d9: Pin<I, P1>,
    led: Pin<I, P2>,
    timer1: TC1,
    serial: &mut dyn uWrite<Error = void::Void>,
) -> ! {
    let _oc1a = d9.into_output();

    if false {
        // PWM: phase correct, 8-bit
        timer1.tccr1a.write(|w| {
            w.wgm1().bits(2).com1a().variant(COM1A_A::MATCH_CLEAR)
            // .com1b()
            // .variant(COM1A_A::MATCH_CLEAR)
        });

        timer1
            .tccr1b
            .write(|w| w.wgm1().bits(2).cs1().variant(CS1_A::DIRECT));
    } else {
        // PWM: phase correct, 8-bit
        timer1.tccr1a.modify(|_, w| w.wgm1().bits(2));
        timer1.tccr1b.modify(|_, w| w.wgm1().bits(2));

        timer1
            .tccr1a
            .modify(|_, w| w.com1a().variant(COM1A_A::MATCH_CLEAR));

        // no prescaler
        timer1.tccr1b.modify(|_, w| w.cs1().variant(CS1_A::DIRECT));
    }
    let _ = uwriteln!(serial, "tccr1a={}", timer1.tccr1a.read().bits());

    unsafe {
        avr_device::interrupt::enable();
    }

    // timer1.timsk1.write(|w| w.toie1().set_bit()); // since we haven't defined an interrupt handler, this triggers a crash

    let mut led = led.into_output();

    timer1.ocr1a.write(|w| unsafe { w.bits(0x4000) });
    let _ = debug_dump(serial, &timer1);

    // const CLOCK: f32 = 16_000_000.0;

    let seq1 = pattern1(D4_FREQ);
    let seq2 = pattern1(C4_FREQ);
    let seq3 = pattern1(B3F_FREQ);
    let seq4 = pattern1(C4_FREQ);
    // let mut iter = seq.iter().cycle();
    loop {
        play_sequence(&seq1, &timer1);
        play_sequence(&seq2, &timer1);
        play_sequence(&seq3, &timer1);
        play_sequence(&seq4, &timer1);

        // delay_ms(500);

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
    const CLOCK: f32 = 16_000_000.0;
    let ticks: u16 = (CLOCK / freq) as u16;
    timer1.icr1.write(|w| unsafe { w.bits(ticks) }); // is this necessary?
    timer1.ocr1a.write(|w| unsafe { w.bits(ticks / 2) });

    let on = millis * 7 / 10;
    arduino_hal::delay_ms(on);

    timer1.ocr1a.write(|w| unsafe { w.bits(0xffff) });

    arduino_hal::delay_ms(millis - on);
}

/*

cargo build --release &&
elf=`echo target/avr-atmega328p/release/megalovania.elf` &&
avrdude -C /etc/avrdude.conf -v -p atmega328p -c arduino -P /dev/ttyACM0  -D -Uflash:w:$elf:e

 */
