#![no_std]
#![no_main]

/// Connect a current-limiting resistor and LED in series between
/// OC1A (= PB1 = pin 9 on the Uno) and ground.
/// This app will control the LED brightness using the PWM output of Timer 1A.
///
use crate::timer_common::{debug_dump, WaveformGenerationMode};
use arduino_hal::default_serial;
use arduino_hal::pac::tc0::tccr0a::COM0A_A;
use arduino_hal::pac::tc0::tccr0a::WGM0_A::PWM_PHASE;
use arduino_hal::pac::tc0::tccr0b::CS0_A;
use arduino_hal::pac::tc1::tccr1a::COM1A_A;
use arduino_hal::pac::tc1::tccr1b::CS1_A;
use arduino_hal::pac::{TC0, TC1};
use arduino_hal::port::Pin;
use avr_hal_generic::port::mode::Io;
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

    match 1 {
        1 => pulse_width_modulation_1a(pins.d9, pins.d10, pins.d13, dp.TC1, &mut serial),
        99 => {
            let mut led = pins.d9.into_output();
            led.set_high();
            loop {
                avr_device::asm::sleep()
            }
        }
        _ => pulse_width_modulation_0a(dp.TC0, pins.d6, pins.d5, pins.d13, &mut serial),
    }
}

pub fn pulse_width_modulation_0a<
    I: Io,
    P1: PinOps,
    P2: PinOps,
    P3: PinOps,
    W: uWrite<Error = void::Void>,
>(
    timer0: TC0,
    d6: Pin<I, P1>,
    d5: Pin<I, P2>,
    led: Pin<I, P3>,
    serial: &mut W,
) -> ! {
    let _oc0a = d6.into_output();

    let _oc0b = d5.into_output();

    // PWM: phase correct
    timer0.tccr0a.write(|w| w.wgm0().variant(PWM_PHASE));
    timer0.tccr0b.write(|w| w.wgm02().bit(false));

    timer0
        .tccr0a
        .write(|w| w.com0a().variant(COM0A_A::MATCH_CLEAR));

    timer0
        .tccr0b
        .write(|w| w.foc0a().bit(false).cs0().variant(CS0_A::PRESCALE_256));

    let mut led = led.into_output();

    let _ = uwriteln!(serial, "timer0;");

    let mut duty_cycle = 0x80;
    loop {
        timer0.ocr0a.write(|w| unsafe { w.bits(duty_cycle) });

        arduino_hal::delay_ms(500);

        led.toggle();

        duty_cycle += 0x40;
    }
}

pub fn pulse_width_modulation_1a<
    I: Io,
    P1: PinOps,
    P2: PinOps,
    P3: PinOps,
    W: uWrite<Error = void::Void>,
>(
    d9: Pin<I, P1>,
    d10: Pin<I, P2>,
    led: Pin<I, P3>,
    timer1: TC1,
    serial: &mut W,
) -> ! {
    let _oc1a = d9.into_output();

    let _oc1b = d10.into_output();

    timer_common::rig_timer_1a(
        &timer1,
        serial,
        COM1A_A::MATCH_CLEAR,
        WaveformGenerationMode::PWMPhaseCorrectICR1,
        CS1_A::DIRECT,
    )
    .void_unwrap();

    timer1.icr1.write(|w| unsafe { w.bits(0xffff) });

    // timer1.timsk1.write(|w| w.toie1().set_bit()); // since we haven't defined an interrupt handler, this triggers a crash

    let mut led = led.into_output();

    let states: [u16; 4] = [0x2000, 0x4000, 0x8000, 0xffff]; // duty cycle: gets shifted up to a 16-bit value
    let mut iter = states.iter().cycle();

    let _ = debug_dump(serial, &timer1);

    loop {
        let duty_cycle = *iter.next().unwrap();
        timer1.ocr1a.write(|w| unsafe { w.bits(duty_cycle) });
        timer1.ocr1b.write(|w| unsafe { w.bits(duty_cycle >> 1) });

        arduino_hal::delay_ms(500);

        let n = timer1.tcnt1.read().bits();
        let _ = uwriteln!(serial, "tcnt={}", n);

        led.toggle();
    }
}

/*

cargo build --release &&
elf=`echo target/avr-atmega328p/release/led-pwm.elf` &&
avrdude -C /etc/avrdude.conf -v -p atmega328p -c arduino -P /dev/ttyACM0  -D -Uflash:w:$elf:e

 */
