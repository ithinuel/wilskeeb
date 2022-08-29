# Building for smallest binary

`panic_fmt` is the biggest non-essential entry on the budget.  
It can be removed by rebuilding the core crates with `panic_immediate_abort`:

```sh
# formats panics although it's never used
cargo build --release

# build for smallest possible
cargo build --release -Z build-std=core,panic_abort,compiler_builtins \
                      -Z build-std-features=panic_immediate_abort,compiler-builtins-mangled-names \
            --features instrinsics

# disable rp2040 instrinsics
cargo build --release -Z build-std=core,panic_abort \
                      -Z build-std-features=panic_immediate_abort \
            --features rp2040-hal/disable-intrinsic
```

## Some experimental results

The reference build:

```text
cargo size --release
   text    data     bss     dec     hex filename
  44656       0     136   44792    aef8 wilskeeb
```

Building with `panic_immediate_abort`:

```sh
$ cargo size --release -Z build-std=core,panic_abort \
                       -Z build-std-features=panic_immediate_abort \
             --features rp2040-hal/disable-intrinsic
   text    data     bss     dec    hex  filename
  45644       0     136   45780    b2d4 wilskeeb
```

This result may be expected as the rp2040 allows to reuse the builtins from the bootrom. However…

```sh
$ cargo size --release -Z build-std=core,panic_abort,compiler_builtins \
                       -Z build-std-features=panic_immediate_abort,compiler-builtins-mangled-names
   text    data     bss     dec     hex filename
  44656       0     136   44792    aef8 wilskeeb
```

… is not expected to even build so getting a result here is surprising. Using `cargo build …`

```sh
$ cargo build --release -Z build-std=core,panic_abort,compiler_builtins \
                        -Z build-std-features=panic_immediate_abort,compiler-builtins-mangled-names
```

… fails as expected, `cargo size` does funky things and so does…

```sh
$ cargo build --release -Z build-std=core,panic_abort,compiler_builtins \
                        -Z build-std-features=panic_immediate_abort,compiler-builtins-mangled-names \
              --features intrinsics
```

### Back to the basics

```sh
cargo clean
cargo build --release
arm-none-eabi-size -t target/thumbv6m-none-eabi/release/wilskeeb
   text    data     bss     dec     hex filename
  44656       0     136   44792    aef8 target/thumbv6m-none-eabi/release/wilskeeb
  44656       0     136   44792    aef8 (TOTALS)

cargo clean
cargo build --release -Z build-std=core,panic_abort -Z build-std-features=panic_immediate_abort \
            --features rp2040-hal/disable-intrinsics
arm-none-eabi-size -t target/thumbv6m-none-eabi/release/wilskeeb
   text    data     bss     dec     hex filename
  35784       0     136   35920    8c50 target/thumbv6m-none-eabi/release/wilskeeb
  35784       0     136   35920    8c50 (TOTALS)

cargo clean
cargo build --release -Z build-std=core,panic_abort,compiler_builtins \
                      -Z build-std-features=panic_immediate_abort,compiler-builtins-mangled-names \
            --features intrinsics
arm-none-eabi-size -t target/thumbv6m-none-eabi/release/wilskeeb
   text    data     bss     dec     hex filename
  35348       0     136   35484    8a9c target/thumbv6m-none-eabi/release/wilskeeb
  35348       0     136   35484    8a9c (TOTALS)
```
