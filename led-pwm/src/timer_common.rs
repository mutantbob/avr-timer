use arduino_hal::pac::tc1::tccr1a::COM1A_A;
use arduino_hal::pac::tc1::tccr1b::CS1_A;
use arduino_hal::pac::TC1;
use panic_halt as _;
use ufmt::{uWrite, uwrite};

pub fn debug_dump<W: uWrite>(serial: &mut W, timer1: &TC1) -> Result<(), <W as uWrite>::Error> {
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

/// https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf
/// table 16-4 Waveform Generation Mode Bit Description
#[derive(Copy, Clone)]
pub enum WaveformGenerationMode {
    PWMPhaseCorrect10Bit,
    PWMPhaseCorrectICR1,
}

impl WaveformGenerationMode {
    pub const fn low_bits(&self) -> u8 {
        match self {
            WaveformGenerationMode::PWMPhaseCorrect10Bit => 3,
            WaveformGenerationMode::PWMPhaseCorrectICR1 => 2,
        }
    }
    pub const fn high_bits(&self) -> u8 {
        match self {
            WaveformGenerationMode::PWMPhaseCorrect10Bit => 0,
            WaveformGenerationMode::PWMPhaseCorrectICR1 => 2,
        }
    }
}

/// rig the wgm1, com1a, and cs1 for timer 1.
pub fn rig_timer_1a<W: uWrite>(
    timer1: &TC1,
    serial: &mut W,
    compare_output_mode: COM1A_A,
    wgm: WaveformGenerationMode,
    clock_select: CS1_A,
) -> Result<(), <W as uWrite>::Error> {
    timer1.tccr1a.modify(|_, w| w.wgm1().bits(wgm.low_bits()));
    timer1.tccr1b.modify(|_, w| w.wgm1().bits(wgm.high_bits()));

    timer1
        .tccr1a
        .modify(|_, w| w.com1a().variant(compare_output_mode));

    timer1.tccr1b.modify(|_, w| w.cs1().variant(clock_select));

    debug_dump(serial, &timer1)
}
