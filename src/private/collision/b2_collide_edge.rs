use crate::b2_collision::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2_settings::*;
use crate::shapes::b2_circle_shape::*;
use crate::shapes::b2_edge_shape::*;
use crate::shapes::b2_polygon_shape::*;

// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
pub fn b2_collide_edge_and_circle(
	manifold: &mut B2manifold,
	edge_a: &B2edgeShape,
	xf_a: &B2Transform,
	circle_b: &B2circleShape,
	xf_b: &B2Transform,
) {
	manifold.point_count = 0;
	// Compute circle in frame of edge
	let q: B2vec2 =
		b2_mul_t_transform_by_vec2(*xf_a, b2_mul_transform_by_vec2(*xf_b, circle_b.m_p));

	let (a, b) = (edge_a.m_vertex1, edge_a.m_vertex2);
	let e: B2vec2 = b - a;

	// Normal points to the right for a CCW winding
	let mut n = B2vec2::new(e.y, -e.x);
	let offset: f32 = b2_dot(n, q - a);

	let one_sided: bool = edge_a.m_one_sided;
	if one_sided && offset < 0.0 {
		return;
	}

	// Barycentric coordinates
	let u: f32 = b2_dot(e, b - q);
	let v: f32 = b2_dot(e, q - a);

	let radius: f32 = edge_a.base.m_radius + circle_b.base.m_radius;
	let mut cf = B2contactFeature::default();
	cf.index_b = 0;
	cf.type_b = B2contactFeatureType::EVertex as u8;
	// Region A
	if v <= 0.0 {
		let p: B2vec2 = a;
		let d: B2vec2 = q - p;
		let dd: f32 = b2_dot(d, d);
		if dd > radius * radius {
			return;
		}
		// Is there an edge connected to A?
		if edge_a.m_one_sided {
			let a1: B2vec2 = edge_a.m_vertex0;
			let b1: B2vec2 = a;
			let e1: B2vec2 = b1 - a1;
			let u1: f32 = b2_dot(e1, b1 - q);
			// Is the circle in Region AB of the previous edge?
			if u1 > 0.0 {
				return;
			}
		}
		cf.index_a = 0;
		cf.type_a = B2contactFeatureType::EVertex as u8;
		manifold.point_count = 1;
		manifold.manifold_type = B2manifoldType::ECircles;
		manifold.local_normal.set_zero();
		manifold.local_point = p;
		manifold.points[0].id.cf = cf;
		manifold.points[0].local_point = circle_b.m_p;
		return;
	}
	// Region b
	if u <= 0.0 {
		let p: B2vec2 = b;
		let d: B2vec2 = q - p;
		let dd: f32 = b2_dot(d, d);
		if dd > radius * radius {
			return;
		}
		// Is there an edge connected to b?
		if edge_a.m_one_sided {
			let b2: B2vec2 = edge_a.m_vertex3;
			let a2: B2vec2 = b;
			let e2: B2vec2 = b2 - a2;
			let v2: f32 = b2_dot(e2, q - a2);
			// Is the circle in Region AB of the next edge?
			if v2 > 0.0 {
				return;
			}
		}
		cf.index_a = 1;
		cf.type_a = B2contactFeatureType::EVertex as u8;
		manifold.point_count = 1;
		manifold.manifold_type = B2manifoldType::ECircles;
		manifold.local_normal.set_zero();
		manifold.local_point = p;
		manifold.points[0].id.cf = cf;
		manifold.points[0].local_point = circle_b.m_p;
		return;
	}
	// Region AB
	let den: f32 = b2_dot(e, e);
	b2_assert(den > 0.0);
	let p: B2vec2 = (1.0 / den) * (u * a + v * b);
	let d: B2vec2 = q - p;
	let dd: f32 = b2_dot(d, d);
	if dd > radius * radius {
		return;
	}

	if offset < 0.0 {
		n.set(-n.x, -n.y);
	}
	n.normalize();
	cf.index_a = 0;
	cf.type_a = B2contactFeatureType::EFace as u8;
	manifold.point_count = 1;
	manifold.manifold_type = B2manifoldType::EFaceA;
	manifold.local_normal = n;
	manifold.local_point = a;
	manifold.points[0].id.cf = cf;
	manifold.points[0].local_point = circle_b.m_p;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum B2ePAxisType {
	EUnknown,
	EEdgeA,
	EEdgeB,
}

impl Default for B2ePAxisType {
	fn default() -> Self {
		return B2ePAxisType::EUnknown;
	}
}

// This structure is used to keep track of the best separating axis.
#[derive(Clone, Default, Copy, Debug)]
struct B2epaxis {
	normal: B2vec2,
	axis_type: B2ePAxisType,
	index: i32,
	separation: f32,
}

// This holds polygon b expressed in frame A.
#[derive(Clone, Default, Copy, Debug)]
struct B2tempPolygon {
	vertices: [B2vec2; B2_MAX_POLYGON_VERTICES],
	normals: [B2vec2; B2_MAX_POLYGON_VERTICES],
	count: usize,
}

// Reference face used for clipping
#[derive(Clone, Default, Copy, Debug)]
struct B2referenceFace {
	i1: usize,
	i2: usize,

	v1: B2vec2,
	v2: B2vec2,

	normal: B2vec2,
	side_normal1: B2vec2,
	side_offset1: f32,

	side_normal2: B2vec2,
	side_offset2: f32,
}

fn b2_compute_edge_separation(polygon_b: B2tempPolygon, v1: B2vec2, normal1: B2vec2) -> B2epaxis {
	let mut axis = B2epaxis {
		axis_type: B2ePAxisType::EEdgeA,
		index: -1,
		separation: -B2_MAX_FLOAT,
		normal: B2vec2::zero(),
	};

	let axes: [B2vec2; 2] = [normal1, -normal1];

	// Find axis with least overlap (min-max problem)
	for j in 0..2 {
		let mut sj: f32 = B2_MAX_FLOAT;

		// Find deepest polygon vertex along axis j
		for i in 0..polygon_b.count {
			let si: f32 = b2_dot(axes[j], polygon_b.vertices[i] - v1);
			if si < sj {
				sj = si;
			}
		}

		if sj > axis.separation {
			axis.index = j as i32;
			axis.separation = sj;
			axis.normal = axes[j];
		}
	}

	return axis;
}

fn b2_compute_polygon_separation(polygon_b: B2tempPolygon, v1: B2vec2, v2: B2vec2) -> B2epaxis {
	let mut axis = B2epaxis {
		axis_type: B2ePAxisType::EUnknown,
		index: -1,
		separation: -B2_MAX_FLOAT,
		normal: B2vec2::zero(),
	};

	for i in 0..polygon_b.count {
		let n: B2vec2 = -polygon_b.normals[i];

		let s1: f32 = b2_dot(n, polygon_b.vertices[i] - v1);
		let s2: f32 = b2_dot(n, polygon_b.vertices[i] - v2);
		let s: f32 = b2_min(s1, s2);

		if s > axis.separation {
			axis.axis_type = B2ePAxisType::EEdgeB;
			axis.index = i as i32;
			axis.separation = s;
			axis.normal = n;
		}
	}

	return axis;
}

pub fn b2_collide_edge_and_polygon(
	manifold: &mut B2manifold,
	edge_a: &B2edgeShape,
	xf_a: &B2Transform,
	polygon_b: &B2polygonShape,
	xf_b: &B2Transform,
) {
	manifold.point_count = 0;

	let xf: B2Transform = b2_mul_t_transform(*xf_a, *xf_b);

	let centroid_b: B2vec2 = b2_mul_transform_by_vec2(xf, polygon_b.m_centroid);

	let v1: B2vec2 = edge_a.m_vertex1;
	let v2: B2vec2 = edge_a.m_vertex2;

	let mut edge1: B2vec2 = v2 - v1;
	edge1.normalize();

	// Normal points to the right for a CCW winding
	let normal1 = B2vec2::new(edge1.y, -edge1.x);
	let offset1: f32 = b2_dot(normal1, centroid_b - v1);

	let one_sided: bool = edge_a.m_one_sided;
	if one_sided && offset1 < 0.0 {
		return;
	}

	// Get polygon_b in frameA
	let mut temp_polygon_b = B2tempPolygon::default();
	temp_polygon_b.count = polygon_b.m_count;
	for i in 0..polygon_b.m_count {
		temp_polygon_b.vertices[i] = b2_mul_transform_by_vec2(xf, polygon_b.m_vertices[i]);
		temp_polygon_b.normals[i] = b2_mul_rot_by_vec2(xf.q, polygon_b.m_normals[i]);
	}

	let radius: f32 = polygon_b.base.m_radius + edge_a.base.m_radius;

	let edge_axis: B2epaxis = b2_compute_edge_separation(temp_polygon_b, v1, normal1);
	if edge_axis.separation > radius {
		return;
	}

	let polygon_axis: B2epaxis = b2_compute_polygon_separation(temp_polygon_b, v1, v2);
	if polygon_axis.separation > radius {
		return;
	}

	// Use hysteresis for jitter reduction.
	const K_RELATIVE_TOL: f32 = 0.98;
	const K_ABSOLUTE_TOL: f32 = 0.001;

	let mut primary_axis: B2epaxis;
	if polygon_axis.separation - radius
		> K_RELATIVE_TOL * (edge_axis.separation - radius) + K_ABSOLUTE_TOL
	{
		primary_axis = polygon_axis;
	} else {
		primary_axis = edge_axis;
	}

	if one_sided {
		// Smooth collision
		// See https://box2d.org/posts/2020/06/ghost-collisions/

		let mut edge0: B2vec2 = v1 - edge_a.m_vertex0;
		edge0.normalize();
		let normal0 = B2vec2::new(edge0.y, -edge0.x);
		let convex1: bool = b2_cross(edge0, edge1) >= 0.0;

		let mut edge2: B2vec2 = edge_a.m_vertex3 - v2;
		edge2.normalize();
		let normal2 = B2vec2::new(edge2.y, -edge2.x);
		let convex2: bool = b2_cross(edge1, edge2) >= 0.0;

		const SIN_TOL: f32 = 0.1;
		let side1: bool = b2_dot(primary_axis.normal, edge1) <= 0.0;

		// Check Gauss Map
		if side1 {
			if convex1 {
				if b2_cross(primary_axis.normal, normal0) > SIN_TOL {
					// Skip region
					return;
				}

			// Admit region
			} else {
				// Snap region
				primary_axis = edge_axis;
			}
		} else {
			if convex2 {
				if b2_cross(normal2, primary_axis.normal) > SIN_TOL {
					// Skip region
					return;
				}

			// Admit region
			} else {
				// Snap region
				primary_axis = edge_axis;
			}
		}
	}

	let mut clip_points = <[B2clipVertex; 2]>::default();
	let mut rf = B2referenceFace::default();
	if primary_axis.axis_type == B2ePAxisType::EEdgeA {
		manifold.manifold_type = B2manifoldType::EFaceA;

		// Search for the polygon normal that is most anti-parallel to the edge normal.
		let mut best_index: i32 = 0;
		let mut best_value: f32 = b2_dot(primary_axis.normal, temp_polygon_b.normals[0]);
		for i in 1..temp_polygon_b.count {
			let value: f32 = b2_dot(primary_axis.normal, temp_polygon_b.normals[i]);
			if value < best_value {
				best_value = value;
				best_index = i as i32;
			}
		}

		let i1: i32 = best_index;
		let i2: i32 = if i1 + 1 < temp_polygon_b.count as i32 {
			i1 + 1
		} else {
			0
		};

		clip_points[0].v = temp_polygon_b.vertices[i1 as usize];
		clip_points[0].id.cf.index_a = 0;
		clip_points[0].id.cf.index_b = i1 as u8;
		clip_points[0].id.cf.type_a = B2contactFeatureType::EFace as u8;
		clip_points[0].id.cf.type_b = B2contactFeatureType::EVertex as u8;

		clip_points[1].v = temp_polygon_b.vertices[i2 as usize];
		clip_points[1].id.cf.index_a = 0;
		clip_points[1].id.cf.index_b = i2 as u8;
		clip_points[1].id.cf.type_a = B2contactFeatureType::EFace as u8;
		clip_points[1].id.cf.type_b = B2contactFeatureType::EVertex as u8;

		rf.i1 = 0;
		rf.i2 = 1;
		rf.v1 = v1;
		rf.v2 = v2;
		rf.normal = primary_axis.normal;
		rf.side_normal1 = -edge1;
		rf.side_normal2 = edge1;
	} else {
		manifold.manifold_type = B2manifoldType::EFaceB;

		clip_points[0].v = v2;
		clip_points[0].id.cf.index_a = 1;
		clip_points[0].id.cf.index_b = primary_axis.index as u8;
		clip_points[0].id.cf.type_a = B2contactFeatureType::EVertex as u8;
		clip_points[0].id.cf.type_b = B2contactFeatureType::EFace as u8;

		clip_points[1].v = v1;
		clip_points[1].id.cf.index_a = 0;
		clip_points[1].id.cf.index_b = primary_axis.index as u8;
		clip_points[1].id.cf.type_a = B2contactFeatureType::EVertex as u8;
		clip_points[1].id.cf.type_b = B2contactFeatureType::EFace as u8;

		rf.i1 = primary_axis.index as usize;
		rf.i2 = if rf.i1 + 1 < temp_polygon_b.count {
			rf.i1 + 1
		} else {
			0
		};
		rf.v1 = temp_polygon_b.vertices[rf.i1];
		rf.v2 = temp_polygon_b.vertices[rf.i2];
		rf.normal = temp_polygon_b.normals[rf.i1];

		// CCW winding
		rf.side_normal1.set(rf.normal.y, -rf.normal.x);
		rf.side_normal2 = -rf.side_normal1;
	}

	rf.side_offset1 = b2_dot(rf.side_normal1, rf.v1);
	rf.side_offset2 = b2_dot(rf.side_normal2, rf.v2);

	// Clip incident edge against reference face side planes
	let mut clip_points1 = <[B2clipVertex; 2]>::default();
	let mut clip_points2 = <[B2clipVertex; 2]>::default();

	// Clip to side 1
	let np: usize = b2_clip_segment_to_line(
		&mut clip_points1,
		clip_points,
		rf.side_normal1,
		rf.side_offset1,
		rf.i1,
	);

	if np < B2_MAX_MANIFOLD_POINTS {
		return;
	}

	// Clip to side 2
	let np: usize = b2_clip_segment_to_line(
		&mut clip_points2,
		clip_points1,
		rf.side_normal2,
		rf.side_offset2,
		rf.i2,
	);

	if np < B2_MAX_MANIFOLD_POINTS {
		return;
	}

	// Now clip_points2 contains the clipped points.
	if primary_axis.axis_type == B2ePAxisType::EEdgeA {
		manifold.local_normal = rf.normal;
		manifold.local_point = rf.v1;
	} else {
		manifold.local_normal = polygon_b.m_normals[rf.i1];
		manifold.local_point = polygon_b.m_vertices[rf.i1];
	}

	let mut point_count: usize = 0;
	for i in 0..B2_MAX_MANIFOLD_POINTS {
		let separation: f32;

		separation = b2_dot(rf.normal, clip_points2[i].v - rf.v1);

		if separation <= radius {
			let cp: &mut B2manifoldPoint = &mut manifold.points[point_count];

			if primary_axis.axis_type == B2ePAxisType::EEdgeA {
				cp.local_point = b2_mul_t_transform_by_vec2(xf, clip_points2[i].v);
				cp.id = clip_points2[i].id;
			} else {
				cp.local_point = clip_points2[i].v;
				cp.id.cf.type_a = clip_points2[i].id.cf.type_b;
				cp.id.cf.type_b = clip_points2[i].id.cf.type_a;
				cp.id.cf.index_a = clip_points2[i].id.cf.index_b;
				cp.id.cf.index_b = clip_points2[i].id.cf.index_a;
			}

			point_count += 1;
		}
	}

	manifold.point_count = point_count;
}
