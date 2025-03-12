# sensirion-core

This Rust library offers a standardized set of functionalities for developing device drivers for Sensirion I<sup>2</sup>C devices. Built on top of [sensirion-i2c-rs](https://github.com/Sensirion/sensirion-i2c-rs), it simplifies implementation by providing a higher-level interface. By defining specific traits, developers can minimize boilerplate code while enhancing usability.

## Examples

To see how other device drivers utilize this crate, refer to the following examples:  

- [sensirion-sgp4x](https://github.com/swigg/sensirion-sgp4x)  
- [sensirion-sht3x](https://github.com/swigg/sensirion-sht3x)