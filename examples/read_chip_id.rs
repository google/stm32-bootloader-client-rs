// Copyright 2022 The stm32-bootloader-client-rs Authors.
// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

use anyhow::Result;
use stm32_bootloader_client::Stm32;

fn main() {
    if let Err(error) = run() {
        println!("Error: {}", error);
    }
}

fn run() -> Result<()> {
    let mut config = mcp2221::Config::default();
    config.i2c_speed_hz = 400_000;
    let mut dev = mcp2221::Handle::open_first(&config)?;

    // Set GPIO pin 0 high. This is useful if your I2C bus goes through a level
    // shifter and you need to enable that level shifter in order to use the I2C
    // bus.
    let mut gpio_config = mcp2221::GpioConfig::default();
    gpio_config.set_direction(0, mcp2221::Direction::Output);
    gpio_config.set_value(0, true);
    dev.configure_gpio(&gpio_config)?;

    dev.check_bus()?;

    // This is the address for I2C1 on the STM32G0 series. See AN2606 for
    // the list of addresses for other parts.
    let config = stm32_bootloader_client::Config::i2c_address(0x51);
    let mut stm32 = Stm32::new(dev, config);
    let chip_id = stm32.get_chip_id()?;
    println!("Found chip ID: 0x{chip_id:x}");

    Ok(())
}
