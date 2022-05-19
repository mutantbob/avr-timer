#![no_std]
#![no_main]

//! This is a rust variation on
//! https://lastminuteengineers.com/l293d-dc-motor-arduino-tutorial/
//! It demonstrates a technique for controlling a motor's speed using PWM.
//! Since motors draw significant current, the example uses an L293D motor
//! driver IC to map from logic signals to current suitable for energizing
//! the motor's coils.  That current is provided by a power supply independent
//! from the AVR microcontroller.
//!
//! I created this example with the hope that it can be incorporated as an example in
//! https://github.com/Rahix/avr-hal/tree/main/examples/arduino-uno
//! and this example is therefore licensed under whatever terms are necessary to accomplish this.
//! - thoth950@gmail.com

use crate::timer_common::WaveformGenerationMode;
use arduino_hal::hal::port::PB1;
use arduino_hal::pac::tc1::tccr1a::COM1A_A;
use arduino_hal::pac::tc1::tccr1b::CS1_A;
use arduino_hal::pac::TC1;
use arduino_hal::port::Pin;
use arduino_hal::{default_serial, delay_ms};
use avr_hal_generic::port::mode::Output;
use avr_hal_generic::port::PinOps;
use avr_hal_generic::void;
use embedded_hal::digital::v2::OutputPin;
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

    let enable = pins.d9;
    let dir1 = pins.d8;
    let dir2 = pins.d7;

    motor_demo(
        enable.into_output(),
        dir1.into_output(),
        dir2.into_output(),
        pins.d13.into_output(),
        dp.TC1,
        &mut serial,
    )
}

struct MotorWithPWM<P1: PinOps, P2: PinOps> {
    _enable: Pin<Output, PB1>, // OC1A is pin 9 on an Uno
    dir1: Pin<Output, P1>,
    dir2: Pin<Output, P2>,
    timer1: TC1,
    timer_max: u16,
}

impl<P1: PinOps, P2: PinOps> MotorWithPWM<P1, P2> {
    pub fn new<W: uWrite<Error = void::Void>>(
        enable: Pin<Output, PB1>, // OC1A is pin 9 on an Uno
        dir1: Pin<Output, P1>,
        dir2: Pin<Output, P2>,
        timer1: TC1,
        serial: &mut W,
    ) -> Self {
        const TIMER_MAX: u16 = 0x3ff; // 10-bit max

        timer_common::rig_timer_1a(
            &timer1,
            serial,
            COM1A_A::MATCH_CLEAR,
            WaveformGenerationMode::PWMPhaseCorrect10Bit,
            CS1_A::PRESCALE_1024,
        )
        .void_unwrap();

        MotorWithPWM {
            _enable: enable,
            dir1,
            dir2,
            timer1,
            timer_max: TIMER_MAX,
        }
    }

    /// `duty_cycle` should be in the range [0..1], but motors tend not to spin
    /// when provided a low duty cycle. (0.25 is the minimum for the motor I tested with)
    pub fn configure_motor(&mut self, direction: MotorDirection, duty_cycle: f32) {
        let ticks = duty_cycle * self.timer_max as f32;
        self.timer1.ocr1a.write(|w| unsafe { w.bits(ticks as u16) });

        let _ = self
            .dir1
            .set_state((direction == MotorDirection::Forward).into());
        let _ = self
            .dir2
            .set_state((direction == MotorDirection::Reverse).into());
    }
}

pub fn motor_demo<P1: PinOps, P2: PinOps, P3: PinOps, W: uWrite<Error = void::Void>>(
    enable: Pin<Output, PB1>, // OC1A is pin 9 on an Uno
    dir1: Pin<Output, P1>,
    dir2: Pin<Output, P2>,
    led: Pin<Output, P3>,
    timer1: TC1,
    serial: &mut W,
) -> ! {
    let mut motor = MotorWithPWM::new(enable, dir1, dir2, timer1, serial);

    let mut led = led.into_output();

    const PATTERN: [(MotorDirection, f32); 6] = [
        (MotorDirection::Forward, 1.0),
        (MotorDirection::Forward, 0.5),
        (MotorDirection::Unpowered, 0.0),
        (MotorDirection::Reverse, 1.0),
        (MotorDirection::Reverse, 0.5),
        (MotorDirection::Unpowered, 0.0),
    ];
    let mut iter = PATTERN.iter().cycle();

    delay_ms(2000);

    uwriteln!(serial, "go time").void_unwrap();

    loop {
        let &(a, b) = iter.next().unwrap();
        motor.configure_motor(a, b);

        delay_ms(2000);

        led.toggle();
    }
}

#[derive(PartialEq, Copy, Clone)]
enum MotorDirection {
    Forward,
    Unpowered,
    Reverse,
}

/*

cargo build --release &&
elf=`echo target/avr-atmega328p/release/motor-pwm.elf` &&
avrdude -C /etc/avrdude.conf -v -p atmega328p -c arduino -P /dev/ttyACM0  -D -Uflash:w:$elf:e

 */
