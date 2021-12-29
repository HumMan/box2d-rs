use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2_shape::*;
use crate::private::collision::b2_distance as private;

use std::sync::atomic::AtomicUsize;

pub static B2_GJK_CALLS: AtomicUsize = AtomicUsize::new(0);
pub static B2_GJK_ITERS: AtomicUsize = AtomicUsize::new(0);
pub static B2_GJK_MAX_ITERS: AtomicUsize = AtomicUsize::new(0);

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
#[derive(Default, Clone, Debug)]
pub struct B2distanceProxy {
    pub m_buffer: [B2vec2; 2],
    //TODO_humman тут была константная ссылка на шейп, а стала копия
    pub m_vertices: Vec<B2vec2>,
    pub m_radius: f32,
}

impl B2distanceProxy {
    /// initialize the proxy using the given shape. The shape
    /// must remain in scope while the proxy is in use.
    pub fn set_shape(&mut self, shape: ShapePtr, index: usize) {
        private::set_shape(self, shape, index);
    }

    /// initialize the proxy using a vertex cloud and radius. The vertices
    /// must remain in scope while the proxy is in use.
    pub fn set_vertices(&mut self, vertices: &[B2vec2], radius: f32) {
        private::set_vertices(self, vertices, radius);
    }

    /// Get the supporting vertex index in the given direction.
    pub fn get_support(&self, d: B2vec2) -> usize {
        return inline::get_support(self, d);
    }

    /// Get the supporting vertex in the given direction.
    pub fn get_support_vertex(&self, d: B2vec2) -> B2vec2 {
        return inline::get_support_vertex(self, d);
    }

    /// Get the vertex count.
    pub fn get_vertex_count(&self) -> usize {
        return inline::get_vertex_count(self);
    }

    /// Get a vertex by index. Used by b2Distance.
    pub fn get_vertex(&self, index: usize) -> B2vec2 {
        return inline::get_vertex(self, index);
    }
}

/// Used to warm start b2Distance.
/// Set count to zero on first call.
#[derive(Default, Clone, Copy, Debug)]
pub struct B2simplexCache {
    ///< length or area
    pub metric: f32,
    pub count: u16,
    ///< vertices on shape A
    pub index_a: [u8; 3],
    ///< vertices on shape b
    pub index_b: [u8; 3],
}

/// Input for b2Distance.
/// You have to option to use the shape radii
/// in the computation. Even
#[derive(Default, Clone, Debug)]
pub struct B2distanceInput {
    pub proxy_a: B2distanceProxy,
    pub proxy_b: B2distanceProxy,
    pub transform_a: B2Transform,
    pub transform_b: B2Transform,
    pub use_radii: bool,
}

/// Output for b2Distance.
#[derive(Default, Clone, Copy, Debug)]
pub struct B2distanceOutput {
    ///< closest point on shape_a
    pub point_a: B2vec2,
    ///< closest point on shape_b
    pub point_b: B2vec2,
    pub distance: f32,
    ///< number of GJK iterations used
    pub iterations: i32,
}

/// Compute the closest points between two shapes. Supports any combination of:
/// B2circleShape, B2polygonShape, B2edgeShape. The simplex cache is input/output.
/// On the first call set B2simplexCache.count to zero.
pub fn b2_distance_fn(
    output: &mut B2distanceOutput,
    cache: &mut B2simplexCache,
    input: &B2distanceInput,
) {
    private::b2_distance_fn(output, cache, input);
}

/// Input parameters for b2ShapeCast
#[derive(Default, Clone, Debug)]
pub struct B2shapeCastInput {
    pub proxy_a: B2distanceProxy,
    pub proxy_b: B2distanceProxy,
    pub transform_a: B2Transform,
    pub transform_b: B2Transform,
    pub translation_b: B2vec2,
}

/// Output results for b2ShapeCast
#[derive(Default, Clone, Copy, Debug)]
pub struct B2shapeCastOutput {
    pub point: B2vec2,
    pub normal: B2vec2,
    pub lambda: f32,
    pub iterations: i32,
}

/// Perform a linear shape cast of shape b moving and shape A fixed. Determines the hit point, normal, and translation fraction.
/// @returns true if hit, false if there is no hit or an initial overlap
pub fn b2_shape_cast(output: &mut B2shapeCastOutput, input: B2shapeCastInput) -> bool {
    return private::b2_shape_cast(output, input);
}

mod inline {
    use super::*;

    pub fn get_vertex_count(this: &B2distanceProxy) -> usize {
        return this.m_vertices.len();
    }

    pub fn get_vertex(this: &B2distanceProxy, index: usize) -> B2vec2 {
        b2_assert(index < this.m_vertices.len());
        return this.m_vertices[index];
    }

    pub fn get_support(this: &B2distanceProxy, d: B2vec2) -> usize {
        let mut best_index: usize = 0;
        let mut best_value: f32 = b2_dot(this.m_vertices[0], d);
        for i in 1..this.m_vertices.len() {
            let value: f32 = b2_dot(this.m_vertices[i], d);
            if value > best_value {
                best_index = i;
                best_value = value;
            }
        }

        return best_index;
    }

    pub fn get_support_vertex(this: &B2distanceProxy, d: B2vec2) -> B2vec2 {
        let mut best_index: usize = 0;
        let mut best_value: f32 = b2_dot(this.m_vertices[0], d);
        for i in 1..this.m_vertices.len() {
            let value: f32 = b2_dot(this.m_vertices[i], d);
            if value > best_value {
                best_index = i;
                best_value = value;
            }
        }

        return this.m_vertices[best_index];
    }
}
