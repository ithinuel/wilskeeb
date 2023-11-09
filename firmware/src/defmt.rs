#[macro_export]
macro_rules! trace {
    ($($_:tt)*) => {{}};
}
#[macro_export]
macro_rules! debug {
    ($($_:tt)*) => {{}};
}
#[macro_export]
macro_rules! info {
    ($($_:tt)*) => {{}};
}
#[macro_export]
macro_rules! error {
    ($($_:tt)*) => {{}};
}
#[macro_export]
macro_rules! timestamp {
    ($($_:tt)*) => {};
}

// macros are exported at the root of the crate so pull them back here
#[allow(unused_imports)]
pub use super::{debug, error, info, timestamp, trace};

