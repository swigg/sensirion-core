# sensirion-core

This Rust library offers a standardized set of functionalities for developing device drivers for Sensirion I<sup>2</sup>C devices. Built on top of [sensirion-i2c-rs](https://github.com/Sensirion/sensirion-i2c-rs), it simplifies implementation by providing a higher-level interface. By defining specific traits, developers can minimize boilerplate code while enhancing usability.

> [!NOTE]  
> Currently, this crate relies on `alloc` and requires an allocator. A `no-alloc` version is in development to eliminate this dependency.

## Examples Drivers

To see how other device drivers utilize this crate, refer to the following examples:  

- [sensirion-sgp4x](https://github.com/swigg/sensirion-sgp4x)  
- [sensirion-sht3x](https://github.com/swigg/sensirion-sht3x)

## Example Code

This crate provides two traits that can be implemented:

- For blocking functionality, implement `sensirion_core::blocking::SensirionI2c<I2C, D>`.
- For asynchronous functionality, implement `sensirion_core::asynchronous::SensirionI2c<I2C, D>`.

Both traits are straightforward, requiring only the implementation of getters for three fields.

```rust,no_run
// Example basic device
pub struct MyDevice<I2C, D> {
    i2c: I2C,
    address: SevenBitAddress,
    delay: D
}

impl<I2C, D> SensirionI2c<I2C, D> for MyDevice<I2C, D> {
    ///
    fn i2c(&mut self) -> &mut I2C {
        &mut self.i2c
    }

    ///
    fn address(&mut self) -> SevenBitAddress {
        &mut self.address
    }

    ///
    fn delay(&mut self) -> &mut D {
        &mut self.delay
    }
}
```