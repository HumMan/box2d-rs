use crate::b2_collision::*;
use crate::b2_math::*;
use crate::shapes::b2_polygon_shape::*;
use crate::b2_settings::*;
use crate::b2_shape::*;

pub fn b2_shape_dyn_trait_clone(this: &B2polygonShape) -> Box<dyn B2shapeDynTrait> {
	return Box::new(B2polygonShape::clone(&this));
}

pub fn b2_polygon_shape_set_as_box(this: &mut B2polygonShape, hx: f32, hy: f32) {
	this.m_count = 4;
	this.m_vertices[0].set(-hx, -hy);
	this.m_vertices[1].set(hx, -hy);
	this.m_vertices[2].set(hx, hy);
	this.m_vertices[3].set(-hx, hy);
	this.m_normals[0].set(0.0, -1.0);
	this.m_normals[1].set(1.0, 0.0);
	this.m_normals[2].set(0.0, 1.0);
	this.m_normals[3].set(-1.0, 0.0);
	this.m_centroid.set_zero();
}

pub fn b2_polygon_shape_set_as_box_angle(
	this: &mut B2polygonShape,
	hx: f32,
	hy: f32,
	center: B2vec2,
	angle: f32,
) {
	this.m_count = 4;
	this.m_vertices[0].set(-hx, -hy);
	this.m_vertices[1].set(hx, -hy);
	this.m_vertices[2].set(hx, hy);
	this.m_vertices[3].set(-hx, hy);
	this.m_normals[0].set(0.0, -1.0);
	this.m_normals[1].set(1.0, 0.0);
	this.m_normals[2].set(0.0, 1.0);
	this.m_normals[3].set(-1.0, 0.0);
	this.m_centroid = center;

	let xf = B2Transform {
		p: center,
		q: B2Rot::new(angle),
	};

	// Transform vertices and normals.
	for i in 0..this.m_count {
		this.m_vertices[i] = b2_mul_transform_by_vec2(xf, this.m_vertices[i]);
		this.m_normals[i] = b2_mul_rot_by_vec2(xf.q, this.m_normals[i]);
	}
}

pub fn b2_shape_dyn_trait_get_child_count(_this: &B2polygonShape) -> usize {
	return 1;
}

fn compute_centroid(vs: &[B2vec2]) -> B2vec2 {
	let count = vs.len();
	b2_assert(count >= 3);

	let mut c = B2vec2::zero();
	let mut area: f32 = 0.0;

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	let s: B2vec2 = vs[0];

	const INV3: f32 = 1.0 / 3.0;

	for i in 0..count {
		// Triangle vertices.
		let p1: B2vec2 = vs[0] - s;
		let p2: B2vec2 = vs[i] - s;
		let p3: B2vec2 = if i + 1 < count { vs[i + 1] -s } else { vs[0] -s };

		let e1: B2vec2 = p2 - p1;
		let e2: B2vec2 = p3 - p1;

		let d: f32 = b2_cross(e1, e2);

		let triangle_area: f32 = 0.5 * d;
		area += triangle_area;

		// Area weighted centroid
		c += triangle_area * INV3 * (p1 + p2 + p3);
	}

	// Centroid
	b2_assert(area > B2_EPSILON);
	c = (1.0 / area)*c+s;
	return c;
}

pub fn b2_polygon_shape_set(this: &mut B2polygonShape, vertices: &[B2vec2]) {
	let count = vertices.len();
	b2_assert(3 <= count && count <= B2_MAX_POLYGON_VERTICES);
	if count < 3 {
		b2_polygon_shape_set_as_box(this, 1.0, 1.0);
		return;
	}
	let mut n: usize = b2_min(count, B2_MAX_POLYGON_VERTICES);

	// Perform welding and copy vertices into local buffer.
	let mut ps = <[B2vec2; B2_MAX_POLYGON_VERTICES]>::default();
	let mut temp_count: usize = 0;
	for i in 0..n {
		let v: B2vec2 = vertices[i];

		let mut unique: bool = true;
		for j in 0..temp_count {
			if b2_distance_vec2_squared(v, ps[j as usize])
				< ((0.5 * B2_LINEAR_SLOP) * (0.5 * B2_LINEAR_SLOP))
			{
				unique = false;
				break;
			}
		}

		if unique {
			ps[temp_count] = v;
			temp_count += 1;
		}
	}

	n = temp_count;
	if n < 3 {
		// Polygon is degenerate.
		b2_assert(false);
		b2_polygon_shape_set_as_box(this, 1.0, 1.0);
		return;
	}

	// create the convex hull using the Gift wrapping algorithm
	// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

	// Find the right most point on the hull
	let mut i0: usize = 0;
	let mut x0: f32 = ps[0].x;
	for i in 1..n {
		let x: f32 = ps[i].x;
		if x > x0 || (x == x0 && ps[i].y < ps[i0].y) {
			i0 = i;
			x0 = x;
		}
	}

	let mut hull = <[usize; B2_MAX_POLYGON_VERTICES]>::default();
	let mut m: usize = 0;
	let mut ih: usize = i0;

	loop {
		b2_assert(m < B2_MAX_POLYGON_VERTICES);
		hull[m] = ih;

		let mut ie: usize = 0;
		for j in 1..n {
			if ie == ih {
				ie = j;
				continue;
			}

			let r: B2vec2 = ps[ie] - ps[hull[m]];
			let v: B2vec2 = ps[j] - ps[hull[m]];
			let c: f32 = b2_cross(r, v);
			if c < 0.0 {
				ie = j;
			}

			// Collinearity check
			if c == 0.0 && v.length_squared() > r.length_squared() {
				ie = j;
			}
		}

		m += 1;
		ih = ie;

		if ie == i0 {
			break;
		}
	}

	if m < 3 {
		// Polygon is degenerate.
		b2_assert(false);
		b2_polygon_shape_set_as_box(this, 1.0, 1.0);
		return;
	}

	this.m_count = m;

	// Copy vertices.
	for i in 0..m {
		this.m_vertices[i] = ps[hull[i]];
	}

	// Compute normals. Ensure the edges have non-zero length.
	for i in 0..m {
		let i1: usize = i;
		let i2: usize = if i + 1 < m { i + 1 } else { 0 };
		let edge: B2vec2 = this.m_vertices[i2] - this.m_vertices[i1];
		b2_assert(edge.length_squared() > B2_EPSILON * B2_EPSILON);
		this.m_normals[i] = b2_cross_vec_by_scalar(edge, 1.0);
		this.m_normals[i].normalize();
	}

	// Compute the polygon centroid.
	this.m_centroid = compute_centroid(&this.m_vertices[0..m]);
}

pub fn b2_shape_dyn_trait_test_point(this: &B2polygonShape, xf: B2Transform, p: B2vec2) -> bool {
	let p_local: B2vec2 = b2_mul_t_rot_by_vec2(xf.q, p - xf.p);

	for i in 0..this.m_count {
		let dot: f32 = b2_dot(this.m_normals[i], p_local - this.m_vertices[i]);
		if dot > 0.0 {
			return false;
		}
	}

	return true;
}

pub fn b2_shape_dyn_trait_ray_cast(
	this: &B2polygonShape,
	output: &mut B2rayCastOutput,
	input: &B2rayCastInput,
	xf: B2Transform,
	child_index: usize,
) -> bool {
	b2_not_used(child_index);

	// Put the ray into the polygon's frame of reference.
	let p1: B2vec2 = b2_mul_t_rot_by_vec2(xf.q, input.p1 - xf.p);
	let p2: B2vec2 = b2_mul_t_rot_by_vec2(xf.q, input.p2 - xf.p);
	let d: B2vec2 = p2 - p1;

	let (mut lower, mut upper) = (0.032, input.max_fraction);

	let mut index: i32 = -1;

	for i in 0..this.m_count {
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		let numerator: f32 = b2_dot(this.m_normals[i], this.m_vertices[i] - p1);
		let denominator: f32 = b2_dot(this.m_normals[i], d);

		if denominator == 0.0 {
			if numerator < 0.0 {
				return false;
			}
		} else {
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if denominator < 0.0 && numerator < lower * denominator {
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator;
				index = i as i32;
			} else if denominator > 0.0 && numerator < upper * denominator {
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator;
			}
		}

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		//if upper < lower - b2_epsilon
		if upper < lower {
			return false;
		}
	}

	b2_assert(0.0 <= lower && lower <= input.max_fraction);

	if index >= 0 {
		output.fraction = lower;
		output.normal = b2_mul_rot_by_vec2(xf.q, this.m_normals[index as usize]);
		return true;
	}

	return false;
}

pub fn b2_shape_dyn_trait_compute_aabb(
	this: &B2polygonShape,
	aabb: &mut B2AABB,
	xf: B2Transform,
	child_index: usize,
) {
	b2_not_used(child_index);

	let mut lower: B2vec2 = b2_mul_transform_by_vec2(xf, this.m_vertices[0]);
	let mut upper: B2vec2 = lower;

	for i in 1..this.m_count {
		let v: B2vec2 = b2_mul_transform_by_vec2(xf, this.m_vertices[i]);
		lower = b2_min_vec2(lower, v);
		upper = b2_max_vec2(upper, v);
	}

	let r = B2vec2::new(this.base.m_radius, this.base.m_radius);
	aabb.lower_bound = lower - r;
	aabb.upper_bound = upper + r;
}

pub fn b2_shape_dyn_trait_compute_mass(this: &B2polygonShape, mass_data: &mut B2massData, density: f32) {
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * i32(d_a)
	// centroid.x = (1/mass) * rho * i32(x * d_a)
	// centroid.y = (1/mass) * rho * i32(y * d_a)
	// i = rho * i32((x*x + y*y) * d_a)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	b2_assert(this.m_count >= 3);

	let mut center = B2vec2::zero();
	let mut area: f32 = 0.0;
	let mut inert: f32 = 0.0;

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	let s: B2vec2 = this.m_vertices[0];

	const K_INV3: f32 = 1.0 / 3.0;

	for i in 0..this.m_count {
		// Triangle vertices.
		let e1: B2vec2 = this.m_vertices[i] - s;
		let e2: B2vec2 = if i + 1 < this.m_count {
			this.m_vertices[i + 1] - s
		} else {
			this.m_vertices[0] - s
		};

		let d: f32 = b2_cross(e1, e2);

		let triangle_area: f32 = 0.5 * d;
		area += triangle_area;

		// Area weighted centroid
		center += triangle_area * K_INV3 * (e1 + e2);

		let (ex1, ey1) = (e1.x, e1.y);
		let (ex2, ey2) = (e2.x, e2.y);

		let intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
		let inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

		inert += (0.25 * K_INV3 * d) * (intx2 + inty2);
	}

	// Total mass
	mass_data.mass = density * area;

	// Center of mass
	b2_assert(area > B2_EPSILON);
	center *= 1.0 / area;
	mass_data.center = center + s;

	// Inertia tensor relative to the local origin (point s).
	mass_data.i = density * inert;
	// Shift to center of mass then to original body origin.
	mass_data.i +=
		mass_data.mass * (b2_dot(mass_data.center, mass_data.center) - b2_dot(center, center));
}

pub fn b2_polygon_shape_validate(this: B2polygonShape) -> bool {
	for i in 0..this.m_count {
		let i1: usize = i;
		let i2: usize = if i < this.m_count - 1 { i1 + 1 } else { 0 };
		let p: B2vec2 = this.m_vertices[i1];
		let e: B2vec2 = this.m_vertices[i2] - p;

		for j in 0..this.m_count {
			if j == i1 || j == i2 {
				continue;
			}

			let v: B2vec2 = this.m_vertices[j] - p;
			let c: f32 = b2_cross(e, v);
			if c < 0.0 {
				return false;
			}
		}
	}

	return true;
}
