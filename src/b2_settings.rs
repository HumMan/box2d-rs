/// Settings that can be overriden for your application

// Tunable Constants

/// You can use this to change the length scale used by your game.
/// For example for inches you could use 39.4.
pub const B2_LENGTH_UNITS_PER_METER: f32 = 1.0;

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
pub const B2_MAX_POLYGON_VERTICES: usize = 8;