use crate::shapes::b2_circle_shape::*;
use crate::b2_collision::*;
use crate::b2_math::*;
use crate::shapes::b2_polygon_shape::*;
use crate::b2_common::*;

pub fn b2_collide_circles(
	manifold: &mut B2manifold,
	circle_a: &B2circleShape,
	xf_a: &B2Transform,
	circle_b: &B2circleShape,
	xf_b: &B2Transform,
) {
	manifold.point_count = 0;

	let p_a: B2vec2 = b2_mul_transform_by_vec2(*xf_a, circle_a.m_p);
	let p_b: B2vec2 = b2_mul_transform_by_vec2(*xf_b, circle_b.m_p);

	let d: B2vec2 = p_b - p_a;
	let dist_sqr: f32 = b2_dot(d, d);
	let (r_a, r_b) = (circle_a.base.m_radius, circle_b.base.m_radius);
	let radius: f32 = r_a + r_b;
	if dist_sqr > radius * radius {
		return;
	}

	manifold.manifold_type = B2manifoldType::ECircles;
	manifold.local_point = circle_a.m_p;
	manifold.local_normal.set_zero();
	manifold.point_count = 1;

	manifold.points[0].local_point = circle_b.m_p;
	manifold.points[0].id.cf = Default::default();
}

pub fn b2_collide_polygon_and_circle(
	manifold: &mut B2manifold,
	polygon_a: &B2polygonShape,
	xf_a: &B2Transform,
	circle_b: &B2circleShape,
	xf_b: &B2Transform,
) {
	manifold.point_count = 0;

	// Compute circle position in the frame of the polygon.
	let c: B2vec2 = b2_mul_transform_by_vec2(*xf_b, circle_b.m_p);
	let c_local: B2vec2 = b2_mul_t_transform_by_vec2(*xf_a, c);

	// Find the min separating edge.
	let mut normal_index: usize = 0;
	let mut separation: f32 = -B2_MAX_FLOAT;
	let radius: f32 = polygon_a.base.m_radius + circle_b.base.m_radius;
	let vertex_count: usize = polygon_a.m_count;
	let vertices = &polygon_a.m_vertices;
	let normals = &polygon_a.m_normals;

	for i in 0..vertex_count {
		let s: f32 = b2_dot(normals[i], c_local - vertices[i]);

		if s > radius {
			// Early out.
			return;
		}

		if s > separation {
			separation = s;
			normal_index = i;
		}
	}

	// Vertices that subtend the incident face.
	let vert_index_1: usize = normal_index;
	let vert_index_2: usize = if vert_index_1 + 1 < vertex_count {
		vert_index_1 + 1
	} else {
		0
	};
	let v1: B2vec2 = vertices[vert_index_1];
	let v2: B2vec2 = vertices[vert_index_2];

	// If the center is inside the polygon ...
	if separation < B2_EPSILON {
		manifold.point_count = 1;
		manifold.manifold_type = B2manifoldType::EFaceA;
		manifold.local_normal = normals[normal_index];
		manifold.local_point = 0.5 * (v1 + v2);
		manifold.points[0].local_point = circle_b.m_p;
		manifold.points[0].id.cf = Default::default();
		return;
	}

	// Compute barycentric coordinates
	let u1: f32 = b2_dot(c_local - v1, v2 - v1);
	let u2: f32 = b2_dot(c_local - v2, v1 - v2);
	if u1 <= 0.0 {
		if b2_distance_vec2_squared(c_local, v1) > radius * radius {
			return;
		}

		manifold.point_count = 1;
		manifold.manifold_type = B2manifoldType::EFaceA;
		manifold.local_normal = c_local - v1;
		manifold.local_normal.normalize();
		manifold.local_point = v1;
		manifold.points[0].local_point = circle_b.m_p;
		manifold.points[0].id.cf = Default::default();
	} else if u2 <= 0.0 {
		if b2_distance_vec2_squared(c_local, v2) > radius * radius {
			return;
		}

		manifold.point_count = 1;
		manifold.manifold_type = B2manifoldType::EFaceA;
		manifold.local_normal = c_local - v2;
		manifold.local_normal.normalize();
		manifold.local_point = v2;
		manifold.points[0].local_point = circle_b.m_p;
		manifold.points[0].id.cf = Default::default();
	} else {
		let face_center: B2vec2 = 0.5 * (v1 + v2);
		let s: f32 = b2_dot(c_local - face_center, normals[vert_index_1]);
		if s > radius {
			return;
		}

		manifold.point_count = 1;
		manifold.manifold_type = B2manifoldType::EFaceA;
		manifold.local_normal = normals[vert_index_1];
		manifold.local_point = face_center;
		manifold.points[0].local_point = circle_b.m_p;
		manifold.points[0].id.cf = Default::default();
	}
}
