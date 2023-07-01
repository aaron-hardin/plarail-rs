# Plarail

Embedded Rust projects for Plarail sets

## Crossing Guard

Detects trains for a railroad crossing. When a train is detected, flashes leds, lowers cross guard, and plays alarm sound.

One thing that might stand out as odd compared to other example code is the handling of CountDown<_> timers.
```rust
// Most example code
let _ = nb::block!(count_down.wait());

// As used here
if count_down.wait().is_ok() {
	// do something
}
```
The count down is checked to see if we need to handle it but we do not want to block because there are multiple operations going on at the same time (led, servo, and audio) and we need to check each one. As a result, the 'do something' should be relatively short so it doesn't throw off the flow.

Components:
* [Raspberry Pi Pico H](https://www.raspberrypi.com/products/raspberry-pi-pico/)
* 2 x [Micro servos](https://www.adafruit.com/product/4326)
* [Audio amplifier](https://www.adafruit.com/product/2130)
* 4Ω 3W Speaker
* 4 x Red LEDs
* 2 x IR Detector as below
* Various resistors for the LEDs

## IR Detector

Not particularly interesting, this is just a test for the IR Detector circuit used in the crossing guard.

Components:
* LED: [TSAL4400](https://www.vishay.com/docs/81006/tsal4400.pdf)
* Detector: [TSSP93038](https://www.vishay.com/docs/82866/tssp930.pdf)
  * Could probably find an easier to use detector, this one requires the LED to flash at 38kHz