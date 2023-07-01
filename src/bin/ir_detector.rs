#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{digital::v2::{InputPin, OutputPin}, PwmPin};
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};


#[entry]
fn main() -> ! {
    info!("Program start");
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

    let sys_freq = clocks.system_clock.freq().to_Hz();

    let mut status_led = pins.led.into_push_pull_output();
    status_led.set_high().unwrap();
    let detector_in_pin = pins.gpio18.into_floating_input();
    let boot_mode_in_pin = pins.gpio27.into_floating_input();

    // Init PWMs
    let mut pwm_slices = bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM2 --  38 khz
    let pwm_ir_led = &mut pwm_slices.pwm2;
    pwm_ir_led.default_config();
    pwm_ir_led.set_ph_correct();
    pwm_ir_led.set_div_int(1);
    let top: u16 = (sys_freq / 38_000 / 2).try_into().unwrap();
    let duty = top / 2;
    pwm_ir_led.set_top(top);
    pwm_ir_led.enable();
    // Output channel A on PWM2 to the GPIO4 pin
    let channel_ir_led = &mut pwm_ir_led.channel_a;
    channel_ir_led.output_to(pins.gpio4);
    // Set the duty cycle to 50%
    channel_ir_led.set_duty(duty);

    loop {
        if boot_mode_in_pin.is_high().unwrap() {
            let gpio_activity_pin_mask = 0;
            let disable_interface_mask = 0;
            info!("going into bootloader mode.");
            bsp::hal::rom_data::reset_to_usb_boot(gpio_activity_pin_mask, disable_interface_mask);
        }

        if detector_in_pin.is_high().unwrap() {
            status_led.set_low().unwrap();
        } else {
            status_led.set_high().unwrap();
        }

        delay.delay_ms(50);
    }
}
