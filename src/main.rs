#![no_std]
#![no_main]

use core::convert::Infallible;

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{digital::v2::{InputPin, ToggleableOutputPin, OutputPin}, timer::CountDown, PwmPin};
use fugit::ExtU32;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    multicore::{Multicore, Stack},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

const DUTY_CYCLE_RAISED_ARMS: u16 = 4915; // in theory, position "0"
const DUTY_CYCLE_LOWERED_ARMS: u16 = 3930; // in theory this should be around "-45"

const AUDIO: &[u8] = include_bytes!("train_crossing.wav");
const AUDIO_SAMPLE_LENGTH: u32 = 41667; // 31250 ns = 32kHz, 41666.666666666664 = 24kHz

/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static values is
/// reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything seperately and
/// modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte alignment, which allows
/// the stack guard to take up the least amount of usable RAM.
static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task(sys_freq: u32, out_pin: &mut dyn ToggleableOutputPin<Error = Infallible>) -> ! {
    let led_pin = out_pin;
    loop {
        cortex_m::asm::delay(sys_freq/2);
        led_pin.toggle().unwrap();
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

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

    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(sys_freq, &mut pins.gpio16.into_push_pull_output());
    });

    let mut status_led = pins.led.into_push_pull_output();
    status_led.set_high().unwrap();
    let mut led_crossing_0_0 = pins.gpio19.into_push_pull_output();
    let mut led_crossing_0_1 = pins.gpio20.into_push_pull_output();
    let boot_mode_in_pin = pins.gpio27.into_floating_input();
    let crossing_mode_in_pin = pins.gpio18.into_floating_input();

    // if 'crossing' is triggered then activate crossing mode (alternating leds, lower cross-guards, play crossing sound)
    // keep crossing mode active for 5 secs after 'crossing' no longer detected
    // 'crossing' will be detected by IR led and detector being interrupted
    // LED two different pairs, each pair will alternate between each other being on/off for 1 sec
    let mut is_crossing_mode = false;
    let timer = bsp::hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down_crossing_mode = timer.count_down();
    let mut count_down_led_blink = timer.count_down();
    let mut count_down_audio = timer.count_down();

    // Init PWMs
    let mut pwm_slices = bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // BEGIN servo setup -- crossing arms
    // Servo example: https://github.com/rp-rs/rp-hal-boards/blob/a17d43210996d733e74109e3736c00abb7ac364c/boards/rp-pico/examples/pico_pwm_servo.rs

    // Configure PWM0
    let pwm = &mut pwm_slices.pwm0;
    pwm.set_ph_correct();
    pwm.set_div_int(20u8); // 50 hz
    pwm.enable();

    // Output channel A on PWM0 to the GPIO0 pin and channel B on PWM0 to the GPIO1 pin
    let channel_a = &mut pwm.channel_a;
    channel_a.output_to(pins.gpio0);
    let channel_b = &mut pwm.channel_b;
    channel_b.output_to(pins.gpio1);
    // Start with arms raised
    channel_a.set_duty(DUTY_CYCLE_RAISED_ARMS);
    channel_b.set_duty(DUTY_CYCLE_RAISED_ARMS);
    // END servo setup -- crossing arms

    // BEGIN audio setup -- crossing sound
    // Audio example: https://github.com/rp-rs/rp-hal/pull/356

    // Configure PWM1
    let pwm_audio = &mut pwm_slices.pwm1;
    pwm_audio.default_config();
    pwm_audio.set_top(256); // as unsigned 8-bit WAV
    pwm_audio.set_div_int(1);
    pwm_audio.enable();
    let channel_audio = &mut pwm_audio.channel_a;
    channel_audio.output_to(pins.gpio2);
    count_down_audio.start(AUDIO_SAMPLE_LENGTH.nanos());
    let mut idx = 0x2C; // skip the header
    // END audio setup -- crossing sound

    loop {
        if boot_mode_in_pin.is_high().unwrap() {
            let gpio_activity_pin_mask = 0;
            let disable_interface_mask = 0;
            info!("going into bootloader mode.");
            bsp::hal::rom_data::reset_to_usb_boot(gpio_activity_pin_mask, disable_interface_mask);
        }

        let mut crossing_detected = false;
        // TODO: validate that this is how the IR detector works
        if crossing_mode_in_pin.is_high().unwrap() {
            if !is_crossing_mode {
                // this is the first time we detected crossing mode, init some things
                info!("Crossing mode started");
                led_crossing_0_0.set_high().unwrap();
                led_crossing_0_1.set_low().unwrap();
                // (re)start timer for blinking
                count_down_led_blink.start(1.secs());
                // lower crossing arms
                channel_a.set_duty(DUTY_CYCLE_LOWERED_ARMS);
                channel_b.set_duty(DUTY_CYCLE_LOWERED_ARMS);

                status_led.set_low().unwrap();
            }

            crossing_detected = true;
            is_crossing_mode = true;
        }

        match is_crossing_mode {
            true => {
                if count_down_led_blink.wait().is_ok() {
                    led_crossing_0_0.toggle().unwrap();
                    led_crossing_0_1.toggle().unwrap();
                }
                if count_down_audio.wait().is_ok() {
                    channel_audio.set_duty(AUDIO[idx] as u16);
                    idx = idx + 1;
                    if idx >= AUDIO.len() {
                        // loop back around and skip the header
                        idx = 0x2C;
                    }
                    count_down_audio.start(AUDIO_SAMPLE_LENGTH.nanos());
                }
                if crossing_detected {
                    // reset timer
                    count_down_crossing_mode.start(5.secs());
                } else {
                    // if enough time has passed, clear crossing mode
                    if count_down_crossing_mode.wait().is_ok() {
                        info!("Crossing mode ended");
                        is_crossing_mode = false;
                        led_crossing_0_0.set_low().unwrap();
                        led_crossing_0_1.set_low().unwrap();
                        // raise crossing arms
                        channel_a.set_duty(DUTY_CYCLE_RAISED_ARMS);
                        channel_b.set_duty(DUTY_CYCLE_RAISED_ARMS);

                        status_led.set_high().unwrap();
                    }
                }
            },
            false => {
                // noop for now
                delay.delay_ms(50);
            }
        }
    }
}
