pub mod b2_chain_shape;
pub mod b2_circle_shape;
pub mod b2_edge_shape;
pub mod b2_polygon_shape;
pub mod to_derived_shape;

#[cfg(feature="serde_support")]
pub mod serialize_b2_polygon_shape;