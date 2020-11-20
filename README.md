# box2d-rs

A native port of [Box2D](https://github.com/erincatto/box2d) to Rust.

Minimum Rust version: 1.47.0

Ported Box2D version: 2.4.0

When porting, I pursued the following goals
- Keep file system structure as is for simpler further updates
- Keep all identifiers names(except case convertion) and code order(except some specific cases)
- Store all new not original code in separate files

### TODO

- [ ] convert comments from doxygen to rustdoc
- [ ] replace dump() by serde feature
- [ ] serialize/deserialize test
- [ ] replace imgui unsafe blocks by safe
- [ ] fix some comments
- [ ] fix deprecation warning in glium modifiers
- [ ] do launch_bomb method
- [ ] rename "this" to "_self"
- [ ] replace b2_not_used by underscore syntax

This repo is a work in progress.

## Compiling and running the testbed
```bash
# At the reposity root
cargo build --examples --tests'
cargo run --example testbed
```
