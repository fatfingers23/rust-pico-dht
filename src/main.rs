//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use hal::gpio::dynpin::DynPin;

use panic_probe as _;

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
use rp_pico::hal;

use dht_sensor::{dht11, dht22, DhtError, DhtReading};
use embedded_hal::blocking::delay;
use rp_pico::hal::gpio::Error;

/// A wrapper for DynPin, implementing both InputPin and OutputPin, to simulate
/// an open-drain pin as needed by the wire protocol the DHT11 sensor speaks.
/// https://how2electronics.com/interfacing-dht11-temperature-humidity-sensor-with-raspberry-pi-pico/

struct InOutPin {
    inner: DynPin,
}

impl InOutPin {
    fn new(inner: DynPin) -> Self {
        Self { inner }
    }
}

impl InputPin for InOutPin {
    type Error = hal::gpio::Error;
    fn is_high(&self) -> Result<bool, <Self as embedded_hal::digital::v2::InputPin>::Error> {
        self.inner.is_high()
    }
    fn is_low(&self) -> Result<bool, <Self as embedded_hal::digital::v2::InputPin>::Error> {
        self.inner.is_low()
    }
}

impl OutputPin for InOutPin {
    type Error = hal::gpio::Error;
    fn set_low(&mut self) -> Result<(), <Self as embedded_hal::digital::v2::OutputPin>::Error> {
        // To actively pull the pin low, it must also be configured as a (readable) output pin
        self.inner.into_readable_output();
        // In theory, we should set the pin to low first, to make sure we never actively
        // pull it up. But if we try it on the input pin, we get Err(Gpio(InvalidPinType)).
        self.inner.set_low()?;
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), <Self as embedded_hal::digital::v2::OutputPin>::Error> {
        // To set the open-drain pin to high, just disable the output driver by changing the
        // pin to input mode with pull-up. That way, the DHT11 can still pull the data line down
        // to send its response.
        self.inner.into_pull_up_input();
        Ok(())
    }
}

#[entry]

fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;
    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
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

    let mut led_pin = pins.gpio2.into_push_pull_output();
    // let mut gpio_28 = pins.gpio28.into_push_pull_output();
    let mut dht_pint = InOutPin::new(pins.gpio28.into());
    dht_pint.set_high().ok().unwrap();

    // The DHT11 datasheet suggests 1 second
    info!("Waiting on the sensor...");
    delay.delay_ms(1000);
    // delay.delay_ms(100*/0);

    loop {
        // gpio_28.set_high().unwrap();
        // info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        // info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);

        match dht22::Reading::read(&mut delay, &mut dht_pint) {
            Ok(dht22::Reading {
                temperature,
                relative_humidity,
            }) => info!("{}°, {}% RH", temperature, relative_humidity),
            Err(e) => match e {
                DhtError::ChecksumMismatch => info!("Checksum mismatch"),
                DhtError::PinError(e) => error!("Pin Error"),
                _ => info!("Timeout"),
            },
        }
    }
}

// End of file
