
use crate::b2_distance::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2_shape::*;

use crate::shapes::b2rs_to_derived_shape::*;

use std::sync::atomic::Ordering;

// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.

pub fn set_shape(
	self_: &mut B2distanceProxy,
	shape: ShapePtr,
	index: usize,
) {
	match shape.as_derived() {
		ShapeAsDerived::AsCircle(circle) => {
			self_.m_vertices = vec![circle.m_p];
			self_.m_radius = circle.base.m_radius;
		}
		ShapeAsDerived::AsPolygon(polygon) => {
			self_.m_vertices = polygon.m_vertices[..polygon.m_count].to_vec();
			self_.m_radius = polygon.base.m_radius;
		}
		ShapeAsDerived::AsChain(chain) => {
			b2_assert(index < chain.m_vertices.len());

			self_.m_buffer[0] = chain.m_vertices[index];
			if index + 1 < chain.m_vertices.len() {
				self_.m_buffer[1] = chain.m_vertices[index + 1];
			} else {
				self_.m_buffer[1] = chain.m_vertices[0];
			}

			self_.m_vertices = self_.m_buffer.to_vec();
			self_.m_radius = chain.base.m_radius;
		}
		ShapeAsDerived::AsEdge(edge)  => {
			self_.m_vertices = [edge.m_vertex1, edge.m_vertex2].to_vec();
			self_.m_radius = edge.base.m_radius;
		}
	}
}

pub fn set_vertices(self_: &mut B2distanceProxy, vertices: &[B2vec2], radius: f32) {
	self_.m_vertices = Vec::from(vertices);
	self_.m_radius = radius;
}

#[derive(Default, Clone, Copy, Debug)]
struct B2simplexVertex {
	// support point in proxy_a
	 w_a: B2vec2,
	// support point in proxy_b
	w_b: B2vec2,
	// w_b -  w_a
	w: B2vec2,
	// barycentric coordinate for closest point
	a: f32,
	//  w_a index
	index_a: usize,
	// w_b index
	index_b: usize,
}

#[derive(Default, Clone, Copy, Debug)]
struct B2simplex {
	m_v: [B2simplexVertex; 3],
	m_count: usize,
}
impl B2simplex {
	fn read_cache(
		&mut self,
		cache: &B2simplexCache,
		proxy_a: &B2distanceProxy,
		transform_a: B2Transform,
		proxy_b: &B2distanceProxy,
		transform_b: B2Transform,
	) {
		b2_assert(cache.count <= 3);

		// Copy data from cache.
		self.m_count = cache.count as usize;
		//b2SimplexVertex* vertices = &self_.m_v[0];
		for i in 0..self.m_count {
			let v = &mut self.m_v[i];
			v.index_a = cache.index_a[i] as usize;
			v.index_b = cache.index_b[i] as usize;
			let w_a_local: B2vec2 = proxy_a.get_vertex(v.index_a);
			let w_b_local: B2vec2 = proxy_b.get_vertex(v.index_b);
			v. w_a = b2_mul_transform_by_vec2(transform_a, w_a_local);
			v.w_b = b2_mul_transform_by_vec2(transform_b, w_b_local);
			v.w = v.w_b - v. w_a;
			v.a = 0.0;
		}

		// Compute the new simplex metric, if it is substantially different than
		// old metric then flush the simplex.
		if self.m_count > 1 {
			let metric1: f32 = cache.metric;
			let metric2: f32 = self.get_metric();
			if metric2 < 0.5 * metric1 || 2.0 * metric1 < metric2 || metric2 < B2_EPSILON {
				// reset the simplex.
				self.m_count = 0;
			}
		}

		// If the cache is empty or invalid ...
		if self.m_count == 0 {
			let v = &mut self.m_v[0];
			v.index_a = 0;
			v.index_b = 0;
			let w_a_local: B2vec2 = proxy_a.get_vertex(0);
			let w_b_local: B2vec2 = proxy_b.get_vertex(0);
			v. w_a = b2_mul_transform_by_vec2(transform_a, w_a_local);
			v.w_b = b2_mul_transform_by_vec2(transform_b, w_b_local);
			v.w = v.w_b - v. w_a;
			v.a = 1.0;
			self.m_count = 1;
		}
	}

	fn write_cache(&self, cache: &mut B2simplexCache) {
		cache.metric = self.get_metric();
		cache.count = self.m_count as u16;

		for i in 0..self.m_count {
			cache.index_a[i] = self.m_v[i].index_a as u8;
			cache.index_b[i] = self.m_v[i].index_b as u8;
		}
	}

	fn get_search_direction(&self) -> B2vec2 {
		match self.m_count {
			1 => {
				return -self.m_v[0].w;
			}

			2 => {
				let e12: B2vec2 = self.m_v[1].w - self.m_v[0].w;
				let sgn: f32 = b2_cross(e12, -self.m_v[0].w);
				if sgn > 0.0 {
					// Origin is left of e12.
					return b2_cross_scalar_by_vec(1.0, e12);
				} else {
					// Origin is right of e12.
					return b2_cross_vec_by_scalar(e12, 1.0);
				}
			}

			_ => {
				b2_assert(false);
				return B2vec2::zero();
			}
		}
	}

	fn get_closest_point(&self) -> B2vec2 {
		match self.m_count {
			0 => {
				b2_assert(false);
				return B2vec2::zero();
			}

			1 => {
				return self.m_v[0].w;
			}

			2 => {
				return self.m_v[0].a * self.m_v[0].w + self.m_v[1].a * self.m_v[1].w;
			}
			3 => {
				return B2vec2::zero();
			}

			_ => {
				b2_assert(false);
				return B2vec2::zero();
			}
		}
	}

	fn get_witness_points(&self, p_a: &mut B2vec2, p_b: &mut B2vec2) {
		match self.m_count {
			0 => {
				b2_assert(false);
			}

			1 => {
				*p_a = self.m_v[0]. w_a;
				*p_b = self.m_v[0].w_b;
			}

			2 => {
				*p_a = self.m_v[0].a * self.m_v[0]. w_a + self.m_v[1].a * self.m_v[1]. w_a;
				*p_b = self.m_v[0].a * self.m_v[0].w_b + self.m_v[1].a * self.m_v[1].w_b;
			}

			3 => {
				*p_a = self.m_v[0].a * self.m_v[0]. w_a
					+ self.m_v[1].a * self.m_v[1]. w_a
					+ self.m_v[2].a * self.m_v[2]. w_a;
				*p_b = *p_a;
			}

			_ => {
				b2_assert(false);
			}
		}
	}

	fn get_metric(&self) -> f32 {
		match self.m_count {
			0 => {
				b2_assert(false);
				return 0.0;
			}
			1 => {
				return 0.0;
			}
			2 => {
				return b2_distance_vec2(self.m_v[0].w, self.m_v[1].w);
			}
			3 => {
				return b2_cross(self.m_v[1].w - self.m_v[0].w, self.m_v[2].w - self.m_v[0].w);
			}
			_ => {
				b2_assert(false);
				return 0.0;
			}
		}
	}

	fn solve2(&mut self) {
		b2_simplex_solve2(self);
	}
	fn solve3(&mut self) {
		b2_simplex_solve3(self);
	}
}

// solve a line segment using barycentric coordinates.
//
// p = a1 * w1 + a2 * w2
// a1 + a2 = 1
//
// The vector from the origin to the closest point on the line is
// perpendicular to the line.
// e12 = w2 - w1
// dot(p, e) = 0
// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
//
// 2-by-2 linear system
// [1      1     ][a1] = [1]
// [w1.e12 w2.e12][a2] = [0]
//
// Define
// d12_1 =  dot(w2, e12)
// d12_2 = -dot(w1, e12)
// d12 = d12_1 + d12_2
//
// Solution
// a1 = d12_1 / d12
// a2 = d12_2 / d12
fn b2_simplex_solve2(self_: &mut B2simplex) {
	let w1: B2vec2 = self_.m_v[0].w;
	let w2: B2vec2 = self_.m_v[1].w;
	let e12: B2vec2 = w2 - w1;

	// w1 region
	let d12_2: f32 = -b2_dot(w1, e12);
	if d12_2 <= 0.0 {
		// a2 <= 0, so we clamp it to 0
		self_.m_v[0].a = 1.0;
		self_.m_count = 1;
		return;
	}

	// w2 region
	let d12_1: f32 = b2_dot(w2, e12);
	if d12_1 <= 0.0 {
		// a1 <= 0, so we clamp it to 0
		self_.m_v[1].a = 1.0;
		self_.m_count = 1;
		self_.m_v[0] = self_.m_v[1];
		return;
	}

	// Must be in e12 region.
	let inv_d12: f32 = 1.0 / (d12_1 + d12_2);
	self_.m_v[0].a = d12_1 * inv_d12;
	self_.m_v[1].a = d12_2 * inv_d12;
	self_.m_count = 2;
}

// Possible regions:
// - points[2]
// - edge points[0]-points[2]
// - edge points[1]-points[2]
// - inside the triangle
fn b2_simplex_solve3(self_: &mut B2simplex) {
	let w1: B2vec2 = self_.m_v[0].w;
	let w2: B2vec2 = self_.m_v[1].w;
	let w3: B2vec2 = self_.m_v[2].w;

	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	let e12: B2vec2 = w2 - w1;
	let w1e12: f32 = b2_dot(w1, e12);
	let w2e12: f32 = b2_dot(w2, e12);
	let d12_1: f32 = w2e12;
	let d12_2: f32 = -w1e12;

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	let e13: B2vec2 = w3 - w1;
	let w1e13: f32 = b2_dot(w1, e13);
	let w3e13: f32 = b2_dot(w3, e13);
	let d13_1: f32 = w3e13;
	let d13_2: f32 = -w1e13;

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	let e23: B2vec2 = w3 - w2;
	let w2e23: f32 = b2_dot(w2, e23);
	let w3e23: f32 = b2_dot(w3, e23);
	let d23_1: f32 = w3e23;
	let d23_2: f32 = -w2e23;
	// Triangle123
	let n123: f32 = b2_cross(e12, e13);

	let d123_1: f32 = n123 * b2_cross(w2, w3);
	let d123_2: f32 = n123 * b2_cross(w3, w1);
	let d123_3: f32 = n123 * b2_cross(w1, w2);

	// w1 region
	if d12_2 <= 0.0 && d13_2 <= 0.0 {
		self_.m_v[0].a = 1.0;
		self_.m_count = 1;
		return;
	}

	// e12
	if d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0 {
		let inv_d12: f32 = 1.0 / (d12_1 + d12_2);
		self_.m_v[0].a = d12_1 * inv_d12;
		self_.m_v[1].a = d12_2 * inv_d12;
		self_.m_count = 2;
		return;
	}

	// e13
	if d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0 {
		let inv_d13: f32 = 1.0 / (d13_1 + d13_2);
		self_.m_v[0].a = d13_1 * inv_d13;
		self_.m_v[2].a = d13_2 * inv_d13;
		self_.m_count = 2;
		self_.m_v[1] = self_.m_v[2];
		return;
	}

	// w2 region
	if d12_1 <= 0.0 && d23_2 <= 0.0 {
		self_.m_v[1].a = 1.0;
		self_.m_count = 1;
		self_.m_v[0] = self_.m_v[1];
		return;
	}

	// w3 region
	if d13_1 <= 0.0 && d23_1 <= 0.0 {
		self_.m_v[2].a = 1.0;
		self_.m_count = 1;
		self_.m_v[0] = self_.m_v[2];
		return;
	}

	// e23
	if d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0 {
		let inv_d23: f32 = 1.0 / (d23_1 + d23_2);
		self_.m_v[1].a = d23_1 * inv_d23;
		self_.m_v[2].a = d23_2 * inv_d23;
		self_.m_count = 2;
		self_.m_v[0] = self_.m_v[2];
		return;
	}

	// Must be in triangle123
	let inv_d123: f32 = 1.0 / (d123_1 + d123_2 + d123_3);
	self_.m_v[0].a = d123_1 * inv_d123;
	self_.m_v[1].a = d123_2 * inv_d123;
	self_.m_v[2].a = d123_3 * inv_d123;
	self_.m_count = 3;
}

pub fn b2_distance_fn(
	output: &mut B2distanceOutput,
	cache: &mut B2simplexCache,
	input: &B2distanceInput,
) {
	B2_GJK_CALLS.fetch_add(1, Ordering::SeqCst);

	let proxy_a: &B2distanceProxy = &input.proxy_a;
	let proxy_b: &B2distanceProxy = &input.proxy_b;

	let transform_a: B2Transform = input.transform_a;
	let transform_b: B2Transform = input.transform_b;

	// initialize the simplex.
	let mut simplex = B2simplex::default();
	simplex.read_cache(cache, proxy_a, transform_a, proxy_b, transform_b);

	// Get simplex vertices as an array.
	const K_MAX_ITERS: usize = 20;

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	let mut save_a = <[usize; 3]>::default();
	let mut save_b = <[usize; 3]>::default();
	let mut save_count: usize;

	// Main iteration loop.
	let mut iter: usize = 0;
	while iter < K_MAX_ITERS {
		// Copy simplex so we can identify duplicates.
		save_count = simplex.m_count;
		for i in 0..save_count {
			save_a[i] = simplex.m_v[i].index_a;
			save_b[i] = simplex.m_v[i].index_b;
		}

		match simplex.m_count {
			1 => {
			}
			2 => {
				simplex.solve2();
			}
			3 => {
				simplex.solve3();
			}
			_ => {
				b2_assert(false);
			}
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if simplex.m_count == 3 {
			break;
		}

		// Get search direction.
		let d: B2vec2 = simplex.get_search_direction();

		// Ensure the search direction is numerically fit.
		if d.length_squared() < B2_EPSILON * B2_EPSILON {
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		let vertex = &mut simplex.m_v[simplex.m_count];
		vertex.index_a = proxy_a.get_support(b2_mul_t_rot_by_vec2(transform_a.q, -d));
		vertex. w_a = b2_mul_transform_by_vec2(transform_a, proxy_a.get_vertex(vertex.index_a));
		vertex.index_b = proxy_b.get_support(b2_mul_t_rot_by_vec2(transform_b.q, d));
		vertex.w_b = b2_mul_transform_by_vec2(transform_b, proxy_b.get_vertex(vertex.index_b));
		vertex.w = vertex.w_b - vertex. w_a;

		// Iteration count is equated to the number of support point calls.
		iter += 1;
		B2_GJK_ITERS.fetch_add(1, Ordering::SeqCst);

		// Check for duplicate support points. This is the main termination criteria.
		let mut duplicate: bool = false;
		for i in 0..save_count {
			if vertex.index_a == save_a[i] && vertex.index_b == save_b[i] {
				duplicate = true;
				break;
			}
		}

		// If we found a duplicate support point we must exit to afn cycling.
		if duplicate {
			break;
		}

		// New vertex is ok and needed.
		simplex.m_count += 1;
	}
	
		B2_GJK_MAX_ITERS.fetch_max(iter, Ordering::SeqCst);

	// Prepare output.
	simplex.get_witness_points(&mut output.point_a, &mut output.point_b);
	output.distance = b2_distance_vec2(output.point_a, output.point_b);
	output.iterations = iter as i32;

	// Cache the simplex.
	simplex.write_cache(cache);

	// Apply radii if requested.
	if input.use_radii {
		let r_a: f32 = proxy_a.m_radius;
		let r_b: f32 = proxy_b.m_radius;

		if output.distance > r_a + r_b && output.distance > B2_EPSILON {
			// Shapes are still no overlapped.
			// Move the witness points to the outer surface.
			output.distance -= r_a + r_b;
			let mut normal: B2vec2 = output.point_b - output.point_a;
			normal.normalize();
			output.point_a += r_a * normal;
			output.point_b -= r_b * normal;
		} else {
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			let p: B2vec2 = 0.5 * (output.point_a + output.point_b);
			output.point_a = p;
			output.point_b = p;
			output.distance = 0.0;
		}
	}
}

// GJK-raycast
// Algorithm by Gino van den Bergen.
// "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
pub fn b2_shape_cast(output: &mut B2shapeCastOutput, input: B2shapeCastInput) -> bool {
	output.iterations = 0;
	output.lambda = 1.0;
	output.normal.set_zero();
	output.point.set_zero();

	let proxy_a: &B2distanceProxy = &input.proxy_a;
	let proxy_b: &B2distanceProxy = &input.proxy_b;

	let radius_a: f32 = b2_max(proxy_a.m_radius, B2_POLYGON_RADIUS);
	let radius_b: f32 = b2_max(proxy_b.m_radius, B2_POLYGON_RADIUS);
	let radius: f32 = radius_a + radius_b;

	let xf_a: B2Transform = input.transform_a;
	let xf_b: B2Transform = input.transform_b;

	let r: B2vec2 = input.translation_b;
	let mut n = B2vec2::zero();
	let mut lambda: f32 = 0.0;

	// Initial simplex
	let mut simplex = B2simplex::default();
	simplex.m_count = 0;

	// Get simplex vertices as an array.
	//b2SimplexVertex* vertices = &simplex.m_v[0];

	// Get support point in -r direction
	let mut index_a: usize = proxy_a.get_support(b2_mul_t_rot_by_vec2(xf_a.q, -r));
	let mut w_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, proxy_a.get_vertex(index_a));
	let mut index_b: usize = proxy_b.get_support(b2_mul_t_rot_by_vec2(xf_b.q, r));
	let mut w_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, proxy_b.get_vertex(index_b));
	let mut v: B2vec2 = w_a - w_b;

	// Sigma is the target distance between polygons
	let sigma: f32 = b2_max(B2_POLYGON_RADIUS, radius - B2_POLYGON_RADIUS);
	const TOLERANCE: f32 = 0.5 * B2_LINEAR_SLOP;

	// Main iteration loop.
	const K_MAX_ITERS: i32 = 20;
	let mut iter: i32 = 0;
	while iter < K_MAX_ITERS && v.length() - sigma > TOLERANCE {
		b2_assert(simplex.m_count < 3);

		output.iterations += 1;

		// Support in direction -v (A - b)
		index_a = proxy_a.get_support(b2_mul_t_rot_by_vec2(xf_a.q, -v));
		w_a = b2_mul_transform_by_vec2(xf_a, proxy_a.get_vertex(index_a));
		index_b = proxy_b.get_support(b2_mul_t_rot_by_vec2(xf_b.q, v));
		w_b = b2_mul_transform_by_vec2(xf_b, proxy_b.get_vertex(index_b));
		let p: B2vec2 = w_a - w_b;

		// -v is a normal at p
		v.normalize();

		// Intersect ray with plane
		let vp: f32 = b2_dot(v, p);
		let vr: f32 = b2_dot(v, r);
		if vp - sigma > lambda * vr {
			if vr <= 0.0 {
				return false;
			}

			lambda = (vp - sigma) / vr;
			if lambda > 1.0 {
				return false;
			}

			n = -v;
			simplex.m_count = 0;
		}

		// Reverse simplex since it works with b - A.
		// Shift by lambda * r because we want the closest point to the current clip point.
		// Note that the support point p is not shifted because we want the plane equation
		// to be formed in unshifted space.
		let vertex = &mut simplex.m_v[simplex.m_count];
		vertex.index_a = index_b;
		vertex. w_a = w_b + lambda * r;
		vertex.index_b = index_a;
		vertex.w_b = w_a;
		vertex.w = vertex.w_b - vertex. w_a;
		vertex.a = 1.0;
		simplex.m_count += 1;

		match simplex.m_count {
			1 => {
			}
			2 => {
				simplex.solve2();
			}
			3 => {
				simplex.solve3();
			}
			_ => {
				b2_assert(false);
			}
		}
		// If we have 3 points, then the origin is in the corresponding triangle.
		if simplex.m_count == 3 {
			// Overlap
			return false;
		}

		// Get search direction.
		v = simplex.get_closest_point();

		// Iteration count is equated to the number of support point calls.
		iter += 1;
	}

	if iter==0
	{
		// Initial overlap
		return false;
	}

	// Prepare output.
	let mut point_a = B2vec2::zero();
	let mut point_b = B2vec2::zero();
	simplex.get_witness_points(&mut point_b, &mut point_a);

	if v.length_squared() > 0.0 {
		n = -v;
		n.normalize();
	}

	output.point = point_a + radius_a * n;
	output.normal = n;
	output.lambda = lambda;
	output.iterations = iter;
	return true;
}
