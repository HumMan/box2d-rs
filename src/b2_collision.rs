use crate::b2_math::*;
use crate::b2_common::B2_MAX_MANIFOLD_POINTS;
use crate::b2_shape::*;
use crate::private::collision as private;
use crate::shapes::b2_circle_shape::*;
use crate::shapes::b2_edge_shape::*;
use crate::shapes::b2_polygon_shape::*;

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

pub const B2_NULL_FEATURE: u8 = std::u8::MAX;

pub enum B2contactFeatureType {
    EVertex = 0,
    EFace = 1,
}

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
#[derive(Clone, Default, Copy, Debug, PartialEq)]
pub struct B2contactFeature {
    /// Feature index on shape_a
    pub index_a: u8,
    /// Feature index on shape_b
    pub index_b: u8,
    /// The feature type on shape_a
    pub type_a: u8,
    /// The feature type on shape_b
    pub type_b: u8,
}

/// Contact ids to facilitate warm starting.
#[derive(Clone, Default, Copy, Debug, PartialEq)]
pub struct B2contactId {
    pub cf: B2contactFeature,
}

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circle_b
/// -e_faceA: the local center of cirlceB or the clip point of polygon_b
/// -e_faceB: the clip point of polygon_a
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
#[derive(Clone, Copy, Debug, Default)]
pub struct B2manifoldPoint {
    ///< usage depends on manifold type
    pub local_point: B2vec2,
    ///< the non-penetration impulse
    pub normal_impulse: f32,
    ///< the friction impulse
    pub tangent_impulse: f32,
    ///< uniquely identifies a contact point between two shapes
    pub id: B2contactId,
}

#[derive(Clone, Copy, Debug)]
pub enum B2manifoldType {
    ECircles,
    EFaceA,
    EFaceB,
}

impl Default for B2manifoldType {
    fn default() -> Self {
        return B2manifoldType::ECircles;
    }
}

impl Default for B2manifold {
    fn default() -> Self {
        return B2manifold {
            points: [B2manifoldPoint::default();B2_MAX_MANIFOLD_POINTS],
            local_normal: B2vec2::default(),
            local_point: B2vec2::default(),
            manifold_type: B2manifoldType::ECircles,
            point_count: 0,
        };
    }
}

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circle_a
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygon_a
/// -e_faceB: the normal on polygon_b
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
#[derive(Clone, Copy, Debug)]
pub struct B2manifold {
    /// the points of contact
    pub points: [B2manifoldPoint; B2_MAX_MANIFOLD_POINTS],
    ///< not use for B2manifoldType::e_points
    pub local_normal: B2vec2,
    ///< usage depends on manifold type
    pub local_point: B2vec2,
    pub manifold_type: B2manifoldType,
    ///< the number of manifold points
    pub point_count: usize,
}

/// This is used to compute the current state of a contact manifold.
#[derive(Default, Clone, Copy, Debug)]
pub struct B2worldManifold {
    ///< world vector pointing from A to b
    pub normal: B2vec2,
    ///< world contact point (point of intersection)
    pub points: [B2vec2; B2_MAX_MANIFOLD_POINTS],
    ///< a negative value indicates overlap, in meters
    pub separations: [f32; B2_MAX_MANIFOLD_POINTS],
}

impl B2worldManifold {
    /// evaluate the manifold with supplied transforms. This assumes
    /// modest motion from the original state. This does not change the
    /// point count, impulses, etc. The radii must come from the shapes
    /// that generated the manifold.
    pub fn initialize(
        &mut self,
        manifold: &B2manifold,
        xf_a: B2Transform,
        radius_a: f32,
        xf_b: B2Transform,
        radius_b: f32,
    ) {
        private::b2_collision::b2_world_manifold_initialize(
            self, manifold, xf_a, radius_a, xf_b, radius_b,
        );
    }
}

/// This is used for determining the state of contact points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum B2pointState {
    ///< point does not exist
    B2NullState,
    ///< point was added in the update
    B2AddState,
    ///< point persisted across the update
    B2PersistState,
    ///< point was removed in the update
    B2RemoveState,
}

impl Default for B2pointState {
    fn default() -> Self {
        B2pointState::B2NullState
    }
}

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
pub fn b2_get_point_states(
    state1: &mut [B2pointState; B2_MAX_MANIFOLD_POINTS],
    state2: &mut [B2pointState; B2_MAX_MANIFOLD_POINTS],
    manifold1: &B2manifold,
    manifold2: &B2manifold,
) {
    private::b2_collision::b2_get_point_states(state1, state2, manifold1, manifold2);
}

/// Used for computing contact manifolds.
#[derive(Clone, Default, Copy, Debug)]
pub struct B2clipVertex {
    pub v: B2vec2,
    pub id: B2contactId,
}

/// Ray-cast input data. The ray extends from p1 to p1 + max_fraction * (p2 - p1).
#[derive(Clone, Copy, Debug)]
pub struct B2rayCastInput {
    pub p1: B2vec2,
    pub p2: B2vec2,
    pub max_fraction: f32,
}

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
#[derive(Default, Clone, Copy, Debug)]
pub struct B2rayCastOutput {
    pub normal: B2vec2,
    pub fraction: f32,
}

/// An axis aligned bounding box.
#[derive(Default, Clone, Copy, Debug)]
pub struct B2AABB {
    ///< the lower vertex
    pub lower_bound: B2vec2,
    ///< the upper vertex
    pub upper_bound: B2vec2,
}

impl B2AABB {
    /// Verify that the bounds are sorted.
    pub fn is_valid(self) -> bool {
        return b2_aabb_is_valid(self);
    }

    /// Get the center of the AABB.
    pub fn get_center(self) -> B2vec2 {
        return 0.5 * (self.lower_bound + self.upper_bound);
    }

    /// Get the extents of the AABB (half-widths).
    pub fn get_extents(self) -> B2vec2 {
        return 0.5 * (self.upper_bound - self.lower_bound);
    }

    /// Get the perimeter length
    pub fn get_perimeter(self) -> f32 {
        let wx = self.upper_bound.x - self.lower_bound.x;
        let wy = self.upper_bound.y - self.lower_bound.y;
        return 2.0 * (wx + wy);
    }

    /// Combine an AABB into this one.
    pub fn combine(&mut self, aabb: B2AABB) {
        self.lower_bound = b2_min_vec2(self.lower_bound, aabb.lower_bound);
        self.upper_bound = b2_max_vec2(self.upper_bound, aabb.upper_bound);
    }

    /// Combine two AABBs into this one.
    pub fn combine_two(&mut self, aabb1: B2AABB, aabb2: B2AABB) {
        self.lower_bound = b2_min_vec2(aabb1.lower_bound, aabb2.lower_bound);
        self.upper_bound = b2_max_vec2(aabb1.upper_bound, aabb2.upper_bound);
    }

    /// Does this aabb contain the provided AABB.
    pub fn contains(self, aabb: &B2AABB) -> bool {
        let mut result = true;
        result = result && self.lower_bound.x <= aabb.lower_bound.x;
        result = result && self.lower_bound.y <= aabb.lower_bound.y;
        result = result && aabb.upper_bound.x <= self.upper_bound.x;
        result = result && aabb.upper_bound.y <= self.upper_bound.y;
        return result;
    }

    pub fn ray_cast(self, output: &mut B2rayCastOutput, input: &B2rayCastInput) -> bool {
        return private::b2_collision::b2_aabb_ray_cast(self, output, input);
    }
}

/// Compute the collision manifold between two circles.
pub fn b2_collide_circles(
    manifold: &mut B2manifold,
    circle_a: &B2circleShape,
    xf_a: &B2Transform,
    circle_b: &B2circleShape,
    xf_b: &B2Transform,
) {
    private::b2_collide_circle::b2_collide_circles(manifold, circle_a, xf_a, circle_b, xf_b);
}

/// Compute the collision manifold between a polygon and a circle.
pub fn b2_collide_polygon_and_circle(
    manifold: &mut B2manifold,
    polygon_a: &B2polygonShape,
    xf_a: &B2Transform,
    circle_b: &B2circleShape,
    xf_b: &B2Transform,
) {
    private::b2_collide_circle::b2_collide_polygon_and_circle(
        manifold, polygon_a, xf_a, circle_b, xf_b,
    );
}

/// Compute the collision manifold between two polygons.
pub fn b2_collide_polygons(
    manifold: &mut B2manifold,
    polygon_a: &B2polygonShape,
    xf_a: &B2Transform,
    polygon_b: &B2polygonShape,
    xf_b: &B2Transform,
) {
    private::b2_collide_polygon::b2_collide_polygons(
        manifold, *polygon_a, *xf_a, *polygon_b, *xf_b,
    );
}

/// Compute the collision manifold between an edge and a circle.
pub fn b2_collide_edge_and_circle(
    manifold: &mut B2manifold,
    edge_a: &B2edgeShape,
    xf_a: &B2Transform,
    circle_b: &B2circleShape,
    xf_b: &B2Transform,
) {
    private::b2_collide_edge::b2_collide_edge_and_circle(manifold, edge_a, xf_a, circle_b, xf_b);
}

/// Compute the collision manifold between an edge and a polygon.
pub fn b2_collide_edge_and_polygon(
    manifold: &mut B2manifold,
    edge_a: &B2edgeShape,
    xf_a: &B2Transform,
    polygon_b: &B2polygonShape,
    xf_b: &B2Transform,
) {
    private::b2_collide_edge::b2_collide_edge_and_polygon(manifold, edge_a, xf_a, polygon_b, xf_b);
}

// /// Clipping for contact manifolds.
pub fn b2_clip_segment_to_line(
    v_out: &mut [B2clipVertex; 2],
    v_in: [B2clipVertex; 2],
    normal: B2vec2,
    offset: f32,
    vertex_index_a: usize,
) -> usize {
    return private::b2_collision::b2_clip_segment_to_line(
        v_out,
        v_in,
        normal,
        offset,
        vertex_index_a,
    );
}

/// Determine if two generic shapes overlap.
pub fn b2_test_overlap_shapes(
    shape_a: ShapePtr,
    index_a: usize,
    shape_b: ShapePtr,
    index_b: usize,
    xf_a: B2Transform,
    xf_b: B2Transform,
) -> bool {
    return private::b2_collision::b2_test_overlap(shape_a, index_a, shape_b, index_b, xf_a, xf_b);
}

// ---------------- Inline Functions ------------------------------------------

pub fn b2_aabb_is_valid(this: B2AABB) -> bool {
    let d: B2vec2 = this.upper_bound - this.lower_bound;
    let mut valid: bool = d.x >= 0.0 && d.y >= 0.0;
    valid = valid && this.lower_bound.is_valid() && this.upper_bound.is_valid();
    return valid;
}

pub fn b2_test_overlap(a: B2AABB, b: B2AABB) -> bool {
    let d1 = b.lower_bound - a.upper_bound;
    let d2 = a.lower_bound - b.upper_bound;

    if d1.x > 0.0 || d1.y > 0.0 {
        return false;
    }

    if d2.x > 0.0 || d2.y > 0.0 {
        return false;
    }

    return true;
}
