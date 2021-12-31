# box2d-rs

A native port of [Box2D](https://github.com/erincatto/box2d) to Rust.

Minimum Rust version: 1.47.0

Ported Box2D version: 2.4.1 [![commits](https://img.shields.io/badge/dynamic/json?style=flat-square&label=Commits%20waiting&logo=git&query=%24.total_commits&logoColor=ffffff&labelColor=212121&color=0093ed&url=https%3A%2F%2Fapi.github.com%2Frepos%2Ferincatto%2Fbox2d%2Fcompare%2Fv2.4.1...main)](https://github.com/erincatto/box2d/compare/v2.4.1...main)

[![Latest release on crates.io](https://img.shields.io/crates/v/box2d-rs.svg)](https://crates.io/crates/box2d-rs)
[![Documentation on docs.rs](https://docs.rs/box2d-rs/badge.svg)](https://docs.rs/box2d-rs)
[![Build status](https://github.com/HumMan/box2d-rs//workflows/Rust/badge.svg)](https://github.com/HumMan/box2d-rs/actions/workflows/rust.yml?query=branch%3Amaster)

When porting, I pursued the following goals
- Keep file system structure as is for simpler further updates
- Keep all identifiers names(except case convertion) and code order(except some specific cases)
- Store all new not original code in separate files

### TODO

- [ ] convert comments from doxygen to rustdoc
- [ ] replace imgui unsafe blocks by safe
- [ ] fix some comments
- [ ] fix deprecation warning in glium modifiers

## Known differences

- dump() function replaced by [serde](https://github.com/serde-rs/serde) library (optional dependency)
- comments converted from doxygen to rustdoc
- two additional buttons in the testbed's UI for serialize/deserialize (available only with `serde_support` feature). Serialized data available in `serialize_test` directory
- query/raycast callback class replaced by closure

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
box2d-rs = "0.0.2"
```

If you want serialize/deserialize world with serde:

```toml
[dependencies]
box2d-rs = { version = "0.0.2", features = ["serde_support"] }
```

## Compiling and running the [testbed](https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_testbed.html) from source
Build and run testbed in debug mode:

```bash
# At the reposity root
cargo build --examples --tests
cargo run --example testbed
```

In debug mode with `serde_support`:
```bash
cargo build --examples --tests --features serde_support
cargo run --example testbed --features serde_support
```

Release version with `serde_support`:
```bash
cargo build --examples --tests --release --features serde_support
cargo run --example testbed --release --features serde_support
```

Run tests
```bash
cargo test --features serde_support
```

## Crate features

Optionally, the following dependencies can be enabled:

-   `serde_support` enables serialize/deserialize of world via the `serde` crate

## Documentation
- You can look at the [testbed](https://github.com/HumMan/box2d-rs/tree/master/examples/testbed/tests) for examples.
- [Rust generated docs](https://docs.rs/box2d-rs/0.0.1/box2d_rs/)
- [Manual](https://box2d.org/documentation/)
- [reddit](https://www.reddit.com/r/box2d/)
- [Discord](https://discord.gg/NKYgCBP)
