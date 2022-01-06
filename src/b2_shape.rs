use crate::b2_collision::*;
use crate::b2_math::{B2Transform, B2vec2};
use crate::shapes::b2rs_to_derived_shape::*;
use std::rc::Rc;
use std::cell::RefCell;

#[cfg(feature="serde_support")]
use serde::{Serialize, Deserialize};

/// This holds the mass data computed for a shape.
#[derive(Default, Clone, Copy, Debug)]
pub struct B2massData {
	/// The mass of the shape, usually in kilograms.
	pub mass: f32,

	/// The position of the shape's centroid relative to the shape's origin.
	pub center: B2vec2,

	/// The rotational inertia of the shape about the local origin.
	pub i: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub enum B2ShapeType {
	ECircle = 0,
	EEdge = 1,
	EPolygon = 2,
	EChain = 3,
	ETypeCount = 4,
}

impl Default for B2ShapeType {
	fn default() -> Self {
		B2ShapeType::ECircle
	}
}

/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in B2world are created automatically when a B2fixture
/// is created. Shapes may encapsulate a one or more child shapes.
#[derive(Default, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct B2Shape {
	pub m_type: B2ShapeType,

	/// Radius of a shape. For polygonal shapes this must be B2_POLYGON_RADIUS. There is no support for
	/// making rounded polygons.
	pub m_radius: f32,
}

impl B2Shape {
	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// 
	/// @return the shape type.
	pub fn get_type(self) -> B2ShapeType {
		return self.m_type;
	}
}

pub type ShapeDefPtr = Rc<RefCell<dyn B2shapeDynTrait>>;
pub(crate) type ShapePtr = Rc<dyn B2shapeDynTrait>;

pub trait B2shapeDynTrait: ToDerivedShape {
	fn get_base(&self) -> &B2Shape;
	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// 
	/// @return the shape type.
	fn get_type(&self) -> B2ShapeType;
	/// Clone the concrete shape using the provided allocator.
	fn clone_box(&self) -> Box<dyn B2shapeDynTrait>;
	fn clone_rc(&self) -> ShapePtr;

	/// Get the number of child primitives.
	fn get_child_count(&self) -> usize;

	/// Test a point for containment in this shape. This only works for convex shapes.
	/// * `xf` - the shape world transform.
	/// * `p` - a point in world coordinates.
	fn test_point(&self, xf: B2Transform, p: B2vec2) -> bool;

	/// Cast a ray against a child shape.
	/// * `output` - the ray-cast results.
	/// * `input` - the ray-cast input parameters.
	/// * `transform` - the transform to be applied to the shape.
	/// * `child_index` - the child shape index
	fn ray_cast(
		&self,
		output: &mut B2rayCastOutput,
		input: &B2rayCastInput,
		transform: B2Transform,
		child_index: usize,
	) -> bool;

	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// * `aabb` - returns the axis aligned box.
	/// * `xf` - the world transform of the shape.
	/// * `child_index` - the child shape
	fn compute_aabb(&self, aabb: &mut B2AABB, xf: B2Transform, child_index: usize);

	/// Compute the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// * `mass_data` - returns the mass data for this shape.
	/// * `density` - the density in kilograms per meter squared.
	fn compute_mass(&self, mass_data: &mut B2massData, density: f32);
}
