//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::gpio::{Pin, PinId, PinMode, ValidPinMode, PushPullOutput},
};
use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

const MORSE_TIME_BASE_MS: u32 = 100;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

fn blink_morse<I: PinId>(msg: &str, pin: &mut Pin<I, PushPullOutput>, delay: &mut Delay) {
    let morse_msg = morse_nostd::encode::encode(msg).unwrap();

    for ch in morse_msg.chars() {
	match ch {
	    '.' => {
		pin.set_high().unwrap();
		delay.delay_ms(MORSE_TIME_BASE_MS);
		pin.set_low().unwrap();
	    }
	    '_' => {
		pin.set_high().unwrap();
		delay.delay_ms(3 * MORSE_TIME_BASE_MS);
		pin.set_low().unwrap();
	    }
	    ' ' => {
		delay.delay_ms(2 * MORSE_TIME_BASE_MS);
	    }
	    '/' => {
		delay.delay_ms(6 * MORSE_TIME_BASE_MS);
	    }
	    _other => {}
	}
	delay.delay_ms(MORSE_TIME_BASE_MS);
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");

    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        blink_morse("drozdziak1", &mut led_pin, &mut delay);
        delay.delay_ms(MORSE_TIME_BASE_MS * 7);
    }
}

// End of file
