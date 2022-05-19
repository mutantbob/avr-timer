#![no_std]
#![no_main]

//! Connect a current-limiting resistor and LED in series between
//! OC1A (= PB1 = pin 9 on the Uno) and ground.
//! This app will control the LED brightness using the PWM output of Timer 1A.
//!
//! I created this example with the hope that it can be incorporated as an example in
//! https://github.com/Rahix/avr-hal/tree/main/examples/arduino-uno
//! and this example is therefore licensed under whatever terms are necessary to accomplish this.
//! - thoth950@gmail.com

use crate::timer_common::{debug_dump, WaveformGenerationMode};
use arduino_hal::default_serial;
use arduino_hal::hal::port::{PB1, PB2};
use arduino_hal::pac::tc1::tccr1a::COM1A_A;
use arduino_hal::pac::tc1::tccr1b::CS1_A;
use arduino_hal::pac::TC1;
use arduino_hal::port::Pin;
use avr_hal_generic::port::mode::Output;
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

    let _ = uwriteln!(&mut serial, "hello from Arduino");

    pulse_width_modulation_1a(
        pins.d9.into_output(),
        pins.d10.into_output(),
        pins.d13.into_output(),
        dp.TC1,
        &mut serial,
    )
}

pub fn pulse_width_modulation_1a<P3: PinOps, W: uWrite<Error = void::Void>>(
    _oc1a: Pin<Output, PB1>, // OC1A is pin 9 on an Uno
    _oc1b: Pin<Output, PB2>, // OC1B is pin 10 on an Uno
    led: Pin<Output, P3>,
    timer1: TC1,
    serial: &mut W,
) -> ! {
    const TIMER_MAX: u16 = 0x3ff; // 10-bit max

    timer_common::rig_timer_1a(
        &timer1,
        serial,
        COM1A_A::MATCH_CLEAR,
        WaveformGenerationMode::PWMPhaseCorrect10Bit,
        CS1_A::DIRECT,
    )
    .void_unwrap();

    // timer1.timsk1.write(|w| w.toie1().set_bit()); // since we haven't defined an interrupt handler, this triggers a crash

    let mut led = led.into_output();

    const PATTERN: [u16; 4] = [TIMER_MAX / 8, TIMER_MAX / 4, TIMER_MAX / 2, TIMER_MAX];
    let mut iter = PATTERN.iter().cycle();

    loop {
        let duty_cycle = *iter.next().unwrap();
        timer1.ocr1a.write(|w| unsafe { w.bits(duty_cycle) });
        let _ = uwriteln!(serial, "duty cycle {}/{}", duty_cycle, TIMER_MAX);

        arduino_hal::delay_ms(500);

        /* let n = timer1.tcnt1.read().bits();
        let _ = uwriteln!(serial, "tcnt={}", n); */

        led.toggle();
    }
}

/*

cargo build --release &&
elf=`echo target/avr-atmega328p/release/led-pwm.elf` &&
avrdude -C /etc/avrdude.conf -v -p atmega328p -c arduino -P /dev/ttyACM0  -D -Uflash:w:$elf:e

 */
