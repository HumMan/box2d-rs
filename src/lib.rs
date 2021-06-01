#![crate_name = "box2d_rs"]
#![crate_type = "lib"]

pub mod b2_settings;
pub mod b2_draw;
pub mod b2_timer;

pub mod shapes;

pub mod b2_broad_phase;
pub mod b2_dynamic_tree;

pub mod b2_body;
pub mod b2_contact;
pub mod b2_fixture;
pub mod b2_time_step;
pub mod b2_world;
pub mod b2_world_callbacks;

pub mod joints;

pub mod b2_math;
pub mod b2_collision;
pub mod b2_distance;
pub mod b2_time_of_impact;
pub mod b2_shape;
pub mod b2_joint;
pub mod b2_growable_stack;
pub mod b2_contact_manager;
pub mod b2_rope;

mod double_linked_list;
mod linked_list;

#[cfg(feature="serde_support")]
pub mod serialize_b2_fixture;
#[cfg(feature="serde_support")]
pub mod serialize_b2_joint;
#[cfg(feature="serde_support")]
pub mod serialize_b2_body;
#[cfg(feature="serde_support")]
pub mod serialize_b2_world;

mod private;