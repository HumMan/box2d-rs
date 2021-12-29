use crate::b2_collision::*;
use crate::b2_distance::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2_shape::*;

pub fn b2_world_manifold_initialize(
	this: &mut B2worldManifold,
	manifold: &B2manifold,
	xf_a: B2Transform,
	radius_a: f32,
	xf_b: B2Transform,
	radius_b: f32,
) {
	if manifold.point_count == 0 {
		return;
	}

	match manifold.manifold_type {
		B2manifoldType::ECircles => {
			this.normal.set(1.0, 0.0);
			let point_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, manifold.local_point);
			let point_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, manifold.points[0].local_point);
			if b2_distance_vec2_squared(point_a, point_b) > B2_EPSILON * B2_EPSILON {
				this.normal = point_b - point_a;
				this.normal.normalize();
			}

			let c_a: B2vec2 = point_a + radius_a * this.normal;
			let c_b: B2vec2 = point_b - radius_b * this.normal;
			this.points[0] = 0.5 * (c_a + c_b);
			this.separations[0] = b2_dot(c_b - c_a, this.normal);
		}

		B2manifoldType::EFaceA => {
			this.normal = b2_mul_rot_by_vec2(xf_a.q, manifold.local_normal);
			let plane_point: B2vec2 = b2_mul_transform_by_vec2(xf_a, manifold.local_point);

			for i in 0..manifold.point_count {
				let clip_point: B2vec2 =
					b2_mul_transform_by_vec2(xf_b, manifold.points[i].local_point);
				let c_a: B2vec2 = clip_point
					+ (radius_a - b2_dot(clip_point - plane_point, this.normal)) * this.normal;
				let c_b: B2vec2 = clip_point - radius_b * this.normal;
				this.points[i] = 0.5 * (c_a + c_b);
				this.separations[i] = b2_dot(c_b - c_a, this.normal);
			}
		}

		B2manifoldType::EFaceB => {
			this.normal = b2_mul_rot_by_vec2(xf_b.q, manifold.local_normal);
			let plane_point: B2vec2 = b2_mul_transform_by_vec2(xf_b, manifold.local_point);

			for i in 0..manifold.point_count {
				let clip_point: B2vec2 =
					b2_mul_transform_by_vec2(xf_a, manifold.points[i].local_point);
				let c_b: B2vec2 = clip_point
					+ (radius_b - b2_dot(clip_point - plane_point, this.normal)) * this.normal;
				let c_a: B2vec2 = clip_point - radius_a * this.normal;
				this.points[i] = 0.5 * (c_a + c_b);
				this.separations[i] = b2_dot(c_a - c_b, this.normal);
			}

			// Ensure normal points from A to b.
			this.normal = -this.normal;
		}
	}
}

pub fn b2_get_point_states(
	state1: &mut [B2pointState; B2_MAX_MANIFOLD_POINTS],
	state2: &mut [B2pointState; B2_MAX_MANIFOLD_POINTS],
	manifold1: &B2manifold,
	manifold2: &B2manifold,
) {
	for i in 0..B2_MAX_MANIFOLD_POINTS {
		state1[i] = B2pointState::B2NullState;
		state2[i] = B2pointState::B2NullState;
	}

	// Detect persists and removes.
	for i in 0..manifold1.point_count {
		let id: B2contactId = manifold1.points[i].id;

		state1[i] = B2pointState::B2RemoveState;

		for j in 0..manifold2.point_count {
			
				if manifold2.points[j].id.cf == id.cf {
					state1[i] = B2pointState::B2PersistState;
					break;
				}
			
		}
	}

	// Detect persists and adds.
	for i in 0..manifold2.point_count {
		let id: B2contactId = manifold2.points[i].id;

		state2[i] = B2pointState::B2AddState;

		for j in 0..manifold1.point_count {
			if manifold1.points[j].id == id {
				state2[i] = B2pointState::B2PersistState;
				break;
			}
		}
	}
}

// From Real-time Collision Detection, p179.
pub fn b2_aabb_ray_cast(this: B2AABB, output: &mut B2rayCastOutput, input: &B2rayCastInput) -> bool {
	let mut tmin: f32 = -B2_MAX_FLOAT;
	let mut tmax: f32 = B2_MAX_FLOAT;

	let p: B2vec2 = input.p1;
	let d: B2vec2 = input.p2 - input.p1;
	let abs_d: B2vec2 = b2_abs_vec2(d);

	let mut normal = B2vec2 { x: 0.0, y: 0.0 };

	for i in 0..2 {
		if abs_d.get(i) < B2_EPSILON {
			// Parallel.
			if p.get(i) < this.lower_bound.get(i) || this.upper_bound.get(i) < p.get(i) {
				return false;
			}
		} else {
			let inv_d: f32 = 1.0 / d.get(i);
			let mut t1: f32 = (this.lower_bound.get(i) - p.get(i)) * inv_d;
			let mut t2: f32 = (this.upper_bound.get(i) - p.get(i)) * inv_d;

			// Sign of the normal vector.
			let mut s: f32 = -1.0;

			if t1 > t2 {
				b2_swap(&mut t1, &mut t2);
				s = 1.0;
			}

			// Push the min up
			if t1 > tmin {
				normal.set_zero();
				normal.set_by_index(i, s);
				tmin = t1;
			}

			// Pull the max down
			tmax = b2_min(tmax, t2);

			if tmin > tmax {
				return false;
			}
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if tmin < 0.0 || input.max_fraction < tmin {
		return false;
	}

	// Intersection.
	output.fraction = tmin;
	output.normal = normal;
	return true;
}

// Sutherland-Hodgman clipping.
pub fn b2_clip_segment_to_line(
	v_out: &mut [B2clipVertex; 2],
	v_in: [B2clipVertex; 2],
	normal: B2vec2,
	offset: f32,
	vertex_index_a: usize,
) -> usize {
	// Start with no output points
	let mut count: usize = 0;

	// Calculate the distance of end points to the line
	let distance0: f32 = b2_dot(normal, v_in[0].v) - offset;
	let distance1: f32 = b2_dot(normal, v_in[1].v) - offset;

	// If the points are behind the plane
	if distance0 <= 0.0 {
		v_out[count] = v_in[0];
		count += 1;
	}
	if distance1 <= 0.0 {
		v_out[count] = v_in[1];
		count += 1;
	}

	// If the points are on different sides of the plane
	if distance0 * distance1 < 0.0 {
		// Find intersection point of edge and plane
		let interp: f32 = distance0 / (distance0 - distance1);
		v_out[count].v = v_in[0].v + interp * (v_in[1].v - v_in[0].v);

		// VertexA is hitting edgeB.
		v_out[count].id.cf.index_a = vertex_index_a as u8;
		
			v_out[count].id.cf.index_b = v_in[0].id.cf.index_b;
		
		v_out[count].id.cf.type_a = B2contactFeatureType::EVertex as u8;
		v_out[count].id.cf.type_b = B2contactFeatureType::EFace as u8;
		count += 1;

		b2_assert(count == 2);
	}

	return count;
}

pub fn b2_test_overlap(
	shape_a: ShapePtr,
	index_a: usize,
	shape_b: ShapePtr,
	index_b: usize,
	xf_a: B2Transform,
	xf_b: B2Transform,
) -> bool {
	let mut input = B2distanceInput {
		proxy_a: B2distanceProxy::default(),
		proxy_b: B2distanceProxy::default(),
		transform_a: xf_a,
		transform_b: xf_b,
		use_radii: true,
	};

	input.proxy_a.set_shape(shape_a,index_a);
	input.proxy_b.set_shape(shape_b, index_b);

	let mut cache = B2simplexCache::default();
	cache.count = 0;

	let mut output = B2distanceOutput::default();

	b2_distance_fn(&mut output, &mut cache, &input);

	return output.distance < 10.0 * B2_EPSILON;
}
