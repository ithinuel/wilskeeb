Build with `panic_immediate_abort`:

```sh
cargo build --release -Z build-std=core,panic_abort -Z build-std-features=panic_immediate_abort \
    --features rp2040-hal/disable-intrinsic
```
