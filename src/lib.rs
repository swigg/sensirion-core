#![no_std]

extern crate alloc;

/// Provides blocking implementation
pub mod blocking;

/// Provides asynchronous implementation with feature `embedded-hal-async`
#[cfg(feature = "embedded-hal-async")]
pub mod asynchronous;

mod error;
pub use error::Error;
