# box2d-rs

A native port of [Box2D](https://github.com/erincatto/box2d) to Rust.

Minimum Rust version: 1.47.0

Ported Box2D version: 2.4.0

[![Latest release on crates.io](https://img.shields.io/crates/v/box2d-rs.svg)](https://crates.io/crates/box2d-rs)
[![Documentation on docs.rs](https://docs.rs/box2d-rs/badge.svg)](https://docs.rs/box2d-rs)
[![Build status](https://github.com/HumMan/box2d-rs//workflows/Rust/badge.svg)](https://github.com/HumMan/box2d-rs/actions/workflows/rust.yml?query=branch%3Amaster)

When porting, I pursued the following goals
- Keep file system structure as is for simpler further updates
- Keep all identifiers names(except case convertion) and code order(except some specific cases)
- Store all new not original code in separate files

### TODO

- [ ] convert comments from doxygen to rustdoc
- [X] replace dump() by [serde](https://github.com/serde-rs/serde)
- [X] serialize/deserialize test in testbed
- [ ] replace imgui unsafe blocks by safe
- [ ] fix some comments
- [ ] fix deprecation warning in glium modifiers
- [ ] do launch_bomb method
- [ ] rename "this" to "_self"
- [ ] replace b2_not_used by underscore syntax

This repo is a work in progress.

## Known differences

- dump() function replaced by [serde](https://github.com/serde-rs/serde) library
- comments converted from doxygen to rustdoc

## Compiling and running the testbed
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
