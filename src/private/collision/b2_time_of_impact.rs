use crate::b2_distance::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2_settings::*;
use crate::b2_time_of_impact::*;
use crate::b2_timer::*;

use std::sync::atomic::Ordering;

enum B2separationFunctionType {
	EPoints,
	EFaceA,
	EFaceB,
}

//
struct B2separationFunction<'a> {
	m_proxy_a: &'a B2distanceProxy,
	m_proxy_b: &'a B2distanceProxy,
	m_sweep_a: B2Sweep,
	m_sweep_b: B2Sweep,
	m_type: B2separationFunctionType,
	m_local_point: B2vec2,
	m_axis: B2vec2,
}
impl<'a> B2separationFunction<'a> {
	// TODO_ERIN might not need to return the separation

	fn initialize(
		cache: &B2simplexCache,
		proxy_a: &'a B2distanceProxy,
		sweep_a: &B2Sweep,
		proxy_b: &'a B2distanceProxy,
		sweep_b: &B2Sweep,
		t1: f32,
		separation: &mut f32,
	) -> Self {
		let count = cache.count;
		b2_assert(0 < count && count < 3);

		let mut result = B2separationFunction {
			m_proxy_a: proxy_a,
			m_proxy_b: proxy_b,
			m_sweep_a: *sweep_a,
			m_sweep_b: *sweep_b,
			m_type: B2separationFunctionType::EPoints,
			m_local_point: B2vec2::default(),
			m_axis: B2vec2::default(),
		};

		let mut xf_a = B2Transform::default();
		let mut xf_b = B2Transform::default();
		result.m_sweep_a.get_transform(&mut xf_a, t1);
		result.m_sweep_b.get_transform(&mut xf_b, t1);

		if count == 1 {
			result.m_type = B2separationFunctionType::EPoints;
			let local_point_a: B2vec2 = result.m_proxy_a.get_vertex(cache.index_a[0] as usize);
			let local_point_b: B2vec2 = result.m_proxy_b.get_vertex(cache.index_b[0] as usize);
			let point_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, local_point_a);
			let point_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, local_point_b);
			result.m_axis = point_b - point_a;
			let s: f32 = result.m_axis.normalize();
			*separation = s;
			return result;
		} else if cache.index_a[0] == cache.index_a[1] {
			// Two points on b and one on A.
			result.m_type = B2separationFunctionType::EFaceB;
			let local_point_b1: B2vec2 = proxy_b.get_vertex(cache.index_b[0] as usize);
			let local_point_b2: B2vec2 = proxy_b.get_vertex(cache.index_b[1] as usize);

			result.m_axis = b2_cross_vec_by_scalar(local_point_b2 - local_point_b1, 1.0);
			result.m_axis.normalize();
			let normal: B2vec2 = b2_mul_rot_by_vec2(xf_b.q, result.m_axis);

			result.m_local_point = 0.5 * (local_point_b1 + local_point_b2);
			let point_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, result.m_local_point);

			let local_point_a: B2vec2 = proxy_a.get_vertex(cache.index_a[0] as usize);
			let point_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, local_point_a);

			let mut s: f32 = b2_dot(point_a - point_b, normal);
			if s < 0.0 {
				result.m_axis = -result.m_axis;
				s = -s;
			}
			*separation = s;
			return result;
		} else {
			// Two points on A and one or two points on b.
			result.m_type = B2separationFunctionType::EFaceA;
			let local_point_a1: B2vec2 = result.m_proxy_a.get_vertex(cache.index_a[0] as usize);
			let local_point_a2: B2vec2 = result.m_proxy_a.get_vertex(cache.index_a[1] as usize);
			result.m_axis = b2_cross_vec_by_scalar(local_point_a2 - local_point_a1, 1.0);
			result.m_axis.normalize();
			let normal: B2vec2 = b2_mul_rot_by_vec2(xf_a.q, result.m_axis);

			result.m_local_point = 0.5 * (local_point_a1 + local_point_a2);
			let point_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, result.m_local_point);

			let local_point_b: B2vec2 = result.m_proxy_b.get_vertex(cache.index_b[0] as usize);
			let point_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, local_point_b);

			let mut s: f32 = b2_dot(point_b - point_a, normal);
			if s < 0.0 {
				result.m_axis = -result.m_axis;
				s = -s;
			}
			*separation = s;
			return result;
		}
	}

	//
	fn find_min_separation(&self, index_a: &mut i32, index_b: &mut i32, t: f32) -> f32 {
		let mut xf_a = B2Transform::default();
		let mut xf_b = B2Transform::default();
		self.m_sweep_a.get_transform(&mut xf_a, t);
		self.m_sweep_b.get_transform(&mut xf_b, t);

		match self.m_type {
			B2separationFunctionType::EPoints => {
				let axis_a: B2vec2 = b2_mul_t_rot_by_vec2(xf_a.q, self.m_axis);
				let axis_b: B2vec2 = b2_mul_t_rot_by_vec2(xf_b.q, -self.m_axis);

				*index_a = self.m_proxy_a.get_support(axis_a) as i32;
				*index_b = self.m_proxy_b.get_support(axis_b) as i32;

				let local_point_a: B2vec2 = self.m_proxy_a.get_vertex(*index_a as usize);
				let local_point_b: B2vec2 = self.m_proxy_b.get_vertex(*index_b as usize);

				let point_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, local_point_a);
				let point_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, local_point_b);

				let separation: f32 = b2_dot(point_b - point_a, self.m_axis);
				return separation;
			}

			B2separationFunctionType::EFaceA => {
				let normal: B2vec2 = b2_mul_rot_by_vec2(xf_a.q, self.m_axis);
				let point_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, self.m_local_point);

				let axis_b: B2vec2 = b2_mul_t_rot_by_vec2(xf_b.q, -normal);
				*index_a = -1;
				*index_b = self.m_proxy_b.get_support(axis_b) as i32;

				let local_point_b: B2vec2 = self.m_proxy_b.get_vertex(*index_b as usize);
				let point_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, local_point_b);

				let separation: f32 = b2_dot(point_b - point_a, normal);
				return separation;
			}

			B2separationFunctionType::EFaceB => {
				let normal: B2vec2 = b2_mul_rot_by_vec2(xf_b.q, self.m_axis);
				let point_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, self.m_local_point);

				let axis_a: B2vec2 = b2_mul_t_rot_by_vec2(xf_a.q, -normal);

				*index_b = -1;
				*index_a = self.m_proxy_a.get_support(axis_a) as i32;

				let local_point_a: B2vec2 = self.m_proxy_a.get_vertex(*index_a as usize);
				let point_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, local_point_a);

				let separation: f32 = b2_dot(point_a - point_b, normal);
				return separation;
			}

			//unreachable
			// _ => {
			// 	b2_assert(false);
			// 	*index_a = -1;
			// 	*index_b = -1;
			// 	return 0.0;
			// }
		}
	}

	//
	fn evaluate(&self, index_a: i32, index_b: i32, t: f32) -> f32 {
		let mut xf_a = B2Transform::default();
		let mut xf_b = B2Transform::default();
		self.m_sweep_a.get_transform(&mut xf_a, t);
		self.m_sweep_b.get_transform(&mut xf_b, t);

		match self.m_type {
			B2separationFunctionType::EPoints => {
				let local_point_a: B2vec2 = self.m_proxy_a.get_vertex(index_a as usize);
				let local_point_b: B2vec2 = self.m_proxy_b.get_vertex(index_b as usize);

				let point_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, local_point_a);
				let point_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, local_point_b);
				let separation: f32 = b2_dot(point_b - point_a, self.m_axis);

				return separation;
			}

			B2separationFunctionType::EFaceA => {
				let normal: B2vec2 = b2_mul_rot_by_vec2(xf_a.q, self.m_axis);
				let point_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, self.m_local_point);

				let local_point_b: B2vec2 = self.m_proxy_b.get_vertex(index_b as usize);
				let point_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, local_point_b);

				let separation: f32 = b2_dot(point_b - point_a, normal);
				return separation;
			}

			B2separationFunctionType::EFaceB => {
				let normal: B2vec2 = b2_mul_rot_by_vec2(xf_b.q, self.m_axis);
				let point_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, self.m_local_point);

				let local_point_a: B2vec2 = self.m_proxy_a.get_vertex(index_a as usize);
				let point_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, local_point_a);

				let separation: f32 = b2_dot(point_a - point_b, normal);
				return separation;
			}

			//unreachable
			// _ => {
			// 	b2_assert(false);
			// 	return 0.0;
			// }
		}
	}
}

// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
pub fn b2_time_of_impact(output: &mut B2toioutput, input: &B2toiinput) {
	let timer = B2timer::default();

	B2_TOI_CALLS.fetch_add(1, Ordering::SeqCst);

	output.state = B2toioutputState::EUnknown;
	output.t = input.t_max;

	let proxy_a: &B2distanceProxy = &input.proxy_a;
	let proxy_b: &B2distanceProxy = &input.proxy_b;

	let mut sweep_a: B2Sweep = input.sweep_a;
	let mut sweep_b: B2Sweep = input.sweep_b;

	// Large rotations can make the root finder fail, so we normalize the
	// sweep angles.
	sweep_a.normalize();
	sweep_b.normalize();

	let t_max: f32 = input.t_max;

	let total_radius: f32 = proxy_a.m_radius + proxy_b.m_radius;
	let target: f32 = b2_max(B2_LINEAR_SLOP, total_radius - 3.0 * B2_LINEAR_SLOP);
	let tolerance: f32 = 0.25 * B2_LINEAR_SLOP;
	b2_assert(target > tolerance);

	let mut t1: f32 = 0.0;
	const K_MAX_ITERATIONS: i32 = 20; // TODO_ERIN b2Settings
	let mut iter: i32 = 0;

	// Prepare input for distance query.
	let mut cache = B2simplexCache::default();
	cache.count = 0;
	let mut distance_input = B2distanceInput::default();

	//box2d-rs: because of borrowing and lifetime problems
	distance_input.proxy_a = input.proxy_a.clone();
	distance_input.proxy_b = input.proxy_b.clone();
	distance_input.use_radii = false;

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	loop {
		let mut xf_a = B2Transform::default();
		let mut xf_b = B2Transform::default();
		sweep_a.get_transform(&mut xf_a, t1);
		sweep_b.get_transform(&mut xf_b, t1);

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		distance_input.transform_a = xf_a;
		distance_input.transform_b = xf_b;
		let mut distance_output = B2distanceOutput::default();

		b2_distance_fn(&mut distance_output, &mut cache, &distance_input);

		// If the shapes are overlapped, we give up on continuous collision.
		if distance_output.distance <= 0.0 {
			// Failure!
			output.state = B2toioutputState::EOverlapped;
			output.t = 0.0;
			break;
		}

		if distance_output.distance < target + tolerance {
			// Victory!
			output.state = B2toioutputState::ETouching;
			output.t = t1;
			break;
		}

		// initialize the separating axis.
		let mut _separation: f32 = 0.0;
		let fcn = B2separationFunction::initialize(
			&cache,
			proxy_a,
			&sweep_a,
			proxy_b,
			&sweep_b,
			t1,
			&mut _separation,
		);
		// #if 0
		// 		// dump the curve seen by the root finder
		// 		{
		// 			const i32 n = 100;
		// 			f32 dx = 1.0 / n;
		// 			f32 xs[n+1];
		// 			f32 fs[n+1];

		// 			let x: f32 = 0.0;

		// 			for (i32 i = 0; i <= n; ++i)
		// 			{
		// 				sweep_a.get_transform(&xf_a, x);
		// 				sweep_b.get_transform(&xf_b, x);
		// 				f32 f = fcn.evaluate(xf_a, xf_b) - target;

		// 				printf("%g %g\n", x, f);

		// 				xs[i] = x;
		// 				fs[i] = f;

		// 				x += dx;
		// 			}
		// 		}
		// #endif

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		let mut done: bool = false;
		let mut t2: f32 = t_max;
		let mut push_back_iter: usize = 0;
		loop {
			// Find the deepest point at t2. Store the witness point indices.
			let mut index_a: i32 = 0;
			let mut index_b: i32 = 0;
			let mut s2: f32 = fcn.find_min_separation(&mut index_a, &mut index_b, t2);

			// Is the final configuration separated?
			if s2 > target + tolerance {
				// Victory!
				output.state = B2toioutputState::ESeparated;
				output.t = t_max;
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if s2 > target - tolerance {
				// advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points.
			let mut s1: f32 = fcn.evaluate(index_a, index_b, t1);

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if s1 < target - tolerance {
				output.state = B2toioutputState::EFailed;
				output.t = t1;
				done = true;
				break;
			}

			// Check for touching
			if s1 <= target + tolerance {
				// Victory! t1 should hold the TOI (could be 0.0).
				output.state = B2toioutputState::ETouching;
				output.t = t1;
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			let mut root_iter_count: i32 = 0;
			let mut a1: f32 = t1;
			let mut a2: f32 = t2;
			loop {
				// Use a mix of the secant rule and bisection.
				let t: f32;
				if (root_iter_count & 1) == 1 {
					// Secant rule to improve convergence.
					t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
				} else {
					// Bisection to guarantee progress.
					t = 0.5 * (a1 + a2);
				}

				root_iter_count += 1;
				
				B2_TOI_ROOT_ITERS.fetch_add(1, Ordering::SeqCst);
				

				let s: f32 = fcn.evaluate(index_a, index_b, t);

				if b2_abs(s - target) < tolerance {
					// t2 holds a tentative value for t1
					t2 = t;
					break;
				}

				// Ensure we continue to bracket the root.
				if s > target {
					a1 = t;
					s1 = s;
				} else {
					a2 = t;
					s2 = s;
				}

				if root_iter_count == 50 {
					break;
				}
			}
			
			B2_TOI_MAX_ROOT_ITERS.fetch_max(root_iter_count as usize, Ordering::SeqCst);

			push_back_iter += 1;

			if push_back_iter == B2_MAX_POLYGON_VERTICES {
				break;
			}
		}

		iter += 1;

		B2_TOI_ITERS.fetch_add(1, Ordering::SeqCst);

		if done {
			break;
		}

		if iter == K_MAX_ITERATIONS {
			// Root finder got stuck. Semi-victory.
			output.state = B2toioutputState::EFailed;
			output.t = t1;
			break;
		}
	}

	B2_TOI_MAX_ITERS.fetch_max(iter as usize, Ordering::SeqCst);
	

	let time = timer.precise_time_ns();

	B2_TOI_MAX_TIME.fetch_max(time, Ordering::SeqCst);
	B2_TOI_TIME.fetch_add(time, Ordering::SeqCst);
	
}
