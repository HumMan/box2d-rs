
use crate::b2_collision::*;
use crate::b2_math::*;
use crate::shapes::b2_polygon_shape::*;
use crate::b2_common::*;

// Find the max separation between poly1 and poly2 using edge normals from poly1.
fn b2_find_max_separation(
	edge_index: &mut usize,
	poly1: B2polygonShape,
	xf1: B2Transform,
	poly2: B2polygonShape,
	xf2: B2Transform,
) -> f32 {
	let count1: usize = poly1.m_count;
	let count2: usize = poly2.m_count;
	let n1s = &poly1.m_normals;
	let v1s = &poly1.m_vertices;
	let v2s = &poly2.m_vertices;
	let xf: B2Transform = b2_mul_t_transform(xf2, xf1);

	let mut best_index: usize = 0;
	let mut max_separation: f32 = -B2_MAX_FLOAT;
	for i in 0..count1 {
		// Get poly1 normal in frame2.
		let n: B2vec2 = b2_mul_rot_by_vec2(xf.q, n1s[i]);
		let v1: B2vec2 = b2_mul_transform_by_vec2(xf, v1s[i]);

		// Find deepest point for normal i.
		let mut si: f32 = B2_MAX_FLOAT;
		for j in 0..count2 {
			let sij: f32 = b2_dot(n, v2s[j] - v1);
			if sij < si {
				si = sij;
			}
		}

		if si > max_separation {
			max_separation = si;
			best_index = i;
		}
	}

	*edge_index = best_index;
	return max_separation;
}

pub fn b2_find_incident_edge(
	c: &mut [B2clipVertex; 2],
	poly1: B2polygonShape,
	xf1: B2Transform,
	edge1: usize,
	poly2: B2polygonShape,
	xf2: B2Transform,
) {
	let normals1 = &poly1.m_normals;

	let count2: usize = poly2.m_count;
	let vertices2 = &poly2.m_vertices;
	let normals2 = &poly2.m_normals;

	b2_assert(edge1 < poly1.m_count);

	// Get the normal of the reference edge in poly2's frame.
	let normal1: B2vec2 = b2_mul_t_rot_by_vec2(xf2.q, b2_mul_rot_by_vec2(xf1.q, normals1[edge1]));

	// Find the incident edge on poly2.
	let mut index: usize = 0;
	let mut min_dot: f32 = B2_MAX_FLOAT;
	for i in 0..count2 {
		let dot: f32 = b2_dot(normal1, normals2[i]);
		if dot < min_dot {
			min_dot = dot;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	let i1: usize = index;
	let i2: usize = if i1 + 1 < count2 { i1 + 1 } else { 0 };

	c[0].v = b2_mul_transform_by_vec2(xf2, vertices2[i1]);
	c[0].id.cf.index_a = edge1 as u8;
	c[0].id.cf.index_b = i1 as u8;
	c[0].id.cf.type_a = B2contactFeatureType::EFace as u8;
	c[0].id.cf.type_b = B2contactFeatureType::EVertex as u8;

	c[1].v = b2_mul_transform_by_vec2(xf2, vertices2[i2]);
	c[1].id.cf.index_a = edge1 as u8;
	c[1].id.cf.index_b = i2 as u8;
	c[1].id.cf.type_a = B2contactFeatureType::EFace as u8;
	c[1].id.cf.type_b = B2contactFeatureType::EVertex as u8;
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on b - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
pub fn b2_collide_polygons(
	manifold: &mut B2manifold,
	poly_a: B2polygonShape,
	xf_a: B2Transform,
	poly_b: B2polygonShape,
	xf_b: B2Transform,
) {
	manifold.point_count = 0;
	let total_radius: f32 = poly_a.base.m_radius + poly_b.base.m_radius;

	let mut edge_a: usize = 0;
	let separation_a: f32 = b2_find_max_separation(&mut edge_a, poly_a, xf_a, poly_b, xf_b);
	if separation_a > total_radius {
		return;
	}

	let mut edge_b: usize = 0;
	let separation_b: f32 = b2_find_max_separation(&mut edge_b, poly_b, xf_b, poly_a, xf_a);
	if separation_b > total_radius {
		return;
	}

	let poly1: B2polygonShape; // reference polygon
	let poly2: B2polygonShape; // incident polygon
	let xf1: B2Transform;
	let xf2: B2Transform;
	let edge1: usize; // reference edge
	let flip: u8;
	const K_TOL: f32 = 0.1 * B2_LINEAR_SLOP;

	if separation_b > separation_a + K_TOL {
		poly1 = poly_b;
		poly2 = poly_a;
		xf1 = xf_b;
		xf2 = xf_a;
		edge1 = edge_b;
		manifold.manifold_type = B2manifoldType::EFaceB;
		flip = 1;
	} else {
		poly1 = poly_a;
		poly2 = poly_b;
		xf1 = xf_a;
		xf2 = xf_b;
		edge1 = edge_a;
		manifold.manifold_type = B2manifoldType::EFaceA;
		flip = 0;
	}

	let mut incident_edge = <[B2clipVertex; 2]>::default();
	b2_find_incident_edge(&mut incident_edge, poly1, xf1, edge1, poly2, xf2);

	let count1: usize = poly1.m_count;
	let vertices1 = &poly1.m_vertices;

	let iv1: usize = edge1;
	let iv2: usize = if edge1 + 1 < count1 { edge1 + 1 } else { 0 };

	let mut v11: B2vec2 = vertices1[iv1];
	let mut v12: B2vec2 = vertices1[iv2];

	let mut local_tangent: B2vec2 = v12 - v11;
	local_tangent.normalize();

	let local_normal: B2vec2 = b2_cross_vec_by_scalar(local_tangent, 1.0);
	let plane_point: B2vec2 = 0.5 * (v11 + v12);

	let tangent: B2vec2 = b2_mul_rot_by_vec2(xf1.q, local_tangent);
	let normal: B2vec2 = b2_cross_vec_by_scalar(tangent, 1.0);
	v11 = b2_mul_transform_by_vec2(xf1, v11);
	v12 = b2_mul_transform_by_vec2(xf1, v12);

	// Face offset.
	let front_offset: f32 = b2_dot(normal, v11);

	// Side offsets, extended by polytope skin thickness.
	let side_offset1: f32 = -b2_dot(tangent, v11) + total_radius;
	let side_offset2: f32 = b2_dot(tangent, v12) + total_radius;

	// Clip incident edge against extruded edge1 side edges.
	let mut clip_points1 = <[B2clipVertex; 2]>::default();
	let mut clip_points2 = <[B2clipVertex; 2]>::default();
	let mut np;

	// Clip to box side 1
	np = b2_clip_segment_to_line(&mut clip_points1, incident_edge, -tangent, side_offset1, iv1);

	if np < 2 {
		return;
	}

	// Clip to negative box side 1
	np = b2_clip_segment_to_line(&mut clip_points2, clip_points1, tangent, side_offset2, iv2);

	if np < 2 {
		return;
	}

	// Now clip_points2 contains the clipped points.
	manifold.local_normal = local_normal;
	manifold.local_point = plane_point;

	let mut point_count: usize = 0;
	for i in 0..B2_MAX_MANIFOLD_POINTS {
		let separation: f32 = b2_dot(normal, clip_points2[i].v) - front_offset;

		if separation <= total_radius {
			let cp = &mut manifold.points[point_count];
			cp.local_point = b2_mul_t_transform_by_vec2(xf2, clip_points2[i].v);
			cp.id = clip_points2[i].id;
			if flip != 0 {
				// Swap features

				let cf: B2contactFeature = cp.id.cf;
				cp.id.cf.index_a = cf.index_b;
				cp.id.cf.index_b = cf.index_a;
				cp.id.cf.type_a = cf.type_b;
				cp.id.cf.type_b = cf.type_a;
			}
			point_count += 1;
		}
	}

	manifold.point_count = point_count;
}
