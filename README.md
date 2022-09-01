# stm32-bootloader-client

This is a Rust library for communicating with the STM32 system bootloader. This
can be used, for example, to flash firmware onto an STM32 using an I2C
connection. Support for flashing over UART and SPI is not implemented, but PRs
are welcome.

### Example

See [examples/read_chip_id.rs](examples/read_chip_id.rs).

### Contributing

See [docs/contributing.md](docs/contributing.md).

### Disclaimer

This is not an officially supported Google product.

### License

This project is licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or
   https://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   https://opensource.org/licenses/MIT)

at your option.
