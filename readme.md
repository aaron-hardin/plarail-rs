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

## IR Detector

Not particularly interesting, this is just a test for the IR Detector circuit used in the crossing guard.
