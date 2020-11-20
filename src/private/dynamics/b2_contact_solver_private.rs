use super::b2_contact_solver::*;
use crate::b2_contact::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_time_step::*;
use crate::b2_collision::*;

// Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned effective mass matrix.
const B2_DEBUG_SOLVER: bool = false;

const G_BLOCK_SOLVE: bool = true;

//struct B2contactPositionConstraint
//moved to header

pub(crate) fn new<D: UserDataType>(def: &B2contactSolverDef, contacts: &Vec<ContactPtr<D>>) -> B2contactSolver
{
	let mut result = B2contactSolver::default();

	let count = contacts.len();
	
	result.m_position_constraints.resize(count,Default::default());
	result.m_velocity_constraints.resize(count,Default::default());
	result.m_step = def.step;

	// initialize position independent portions of the constraints.
	for i in 0..count
	{
		let contact = contacts[i].borrow();
		let contact = contact.get_base();

		let fixture_a = &contact.m_fixture_a;
		let fixture_b = &contact.m_fixture_b;
		let shape_a = fixture_a.borrow().get_shape();
		let shape_b = fixture_b.borrow().get_shape();
		let radius_a: f32 =shape_a.get_base().m_radius;
		let radius_b: f32 =shape_b.get_base().m_radius;
		let body_a = fixture_a.borrow().get_body();
		let body_b = fixture_b.borrow().get_body();
		let body_a = body_a.borrow();
		let body_b = body_b.borrow();
		let manifold = contact.get_manifold();

		let point_count: usize = manifold.point_count;
		b2_assert(point_count > 0);

		let mut vc = &mut result.m_velocity_constraints[i];
		vc.friction = contact.m_friction;
		vc.restitution = contact.m_restitution;
		vc.tangent_speed = contact.m_tangent_speed;
		vc.index_a = body_a.m_island_index;
		vc.index_b = body_b.m_island_index;
		vc.inv_mass_a = body_a.m_inv_mass;
		vc.inv_mass_b = body_b.m_inv_mass;
		vc.inv_ia = body_a.m_inv_i;
		vc.inv_ib = body_b.m_inv_i;
		vc.contact_index = i as i32;
		vc.point_count = point_count as i32;
		vc.k.set_zero();
		vc.normal_mass.set_zero();

		let mut pc = &mut result.m_position_constraints[i];
		pc.index_a = body_a.m_island_index;
		pc.index_b = body_b.m_island_index;
		pc.inv_mass_a = body_a.m_inv_mass;
		pc.inv_mass_b = body_b.m_inv_mass;
		pc.local_center_a = body_a.m_sweep.local_center;
		pc.local_center_b = body_b.m_sweep.local_center;
		pc.inv_ia = body_a.m_inv_i;
		pc.inv_ib = body_b.m_inv_i;
		pc.local_normal = manifold.local_normal;
		pc.local_point = manifold.local_point;
		pc.point_count = point_count as i32;
		pc.radius_a = radius_a;
		pc.radius_b = radius_b;
		pc.mtype = manifold.manifold_type;

		for j in 0..point_count
		{
			let cp = manifold.points[j];
			let mut vcp = &mut vc.points[j];
	
			if result.m_step.warm_starting
			{
				vcp.normal_impulse = result.m_step.dt_ratio * cp.normal_impulse;
				vcp.tangent_impulse = result.m_step.dt_ratio * cp.tangent_impulse;
			}
			else
			{
				vcp.normal_impulse = 0.0;
				vcp.tangent_impulse = 0.0;
			}

			vcp.r_a.set_zero();
			vcp.r_b.set_zero();
			vcp.normal_mass = 0.0;
			vcp.tangent_mass = 0.0;
			vcp.velocity_bias = 0.0;

			pc.local_points[j] = cp.local_point;
		}
	}

	return result;
}

// initialize position dependent portions of the velocity constraints.
pub(crate) fn initialize_velocity_constraints<D: UserDataType>(this: &mut B2contactSolver,
	 m_positions: &[B2position], m_velocities: &[B2velocity], m_contacts: &[ContactPtr<D>])
{
	for i in 0..m_contacts.len()
	{
		let vc = &mut this.m_velocity_constraints[i];
		let pc = &mut this.m_position_constraints[i];

		let radius_a: f32 =pc.radius_a;
		let radius_b: f32 =pc.radius_b;
		let contact = m_contacts[vc.contact_index as usize].borrow();
		let manifold = contact.get_base().get_manifold();

		let index_a: i32 =vc.index_a;
		let index_b: i32 =vc.index_b;

		let m_a: f32 =vc.inv_mass_a;
		let m_b: f32 =vc.inv_mass_b;
		let i_a: f32 =vc.inv_ia;
		let i_b: f32 =vc.inv_ib;
		let local_center_a: B2vec2 =pc.local_center_a;
		let local_center_b: B2vec2 =pc.local_center_b;

		let c_a: B2vec2 =m_positions[index_a as usize].c;
		let a_a: f32 =m_positions[index_a as usize].a;
		let v_a: B2vec2 =m_velocities[index_a as usize].v;
		let w_a: f32 =m_velocities[index_a as usize].w;

		let c_b: B2vec2 =m_positions[index_b as usize].c;
		let a_b: f32 =m_positions[index_b as usize].a;
		let v_b: B2vec2 =m_velocities[index_b as usize].v;
		let w_b: f32 =m_velocities[index_b as usize].w;

		b2_assert(manifold.point_count > 0);

		let mut xf_a = B2Transform::default();
		let mut xf_b = B2Transform::default();
		xf_a.q.set(a_a);
		xf_b.q.set(a_b);
		xf_a.p = c_a - b2_mul_rot_by_vec2(xf_a.q, local_center_a);
		xf_b.p = c_b - b2_mul_rot_by_vec2(xf_b.q, local_center_b);

		let mut world_manifold = B2worldManifold::default();
		world_manifold.initialize(manifold, xf_a, radius_a, xf_b, radius_b);

		vc.normal = world_manifold.normal;

		let point_count: usize = vc.point_count as usize;
		for j in 0..point_count
		{
			let mut vcp = &mut vc.points[j];

			vcp.r_a = world_manifold.points[j] - c_a;
			vcp.r_b = world_manifold.points[j] - c_b;

			let rn_a: f32 =b2_cross(vcp.r_a, vc.normal);
			let rn_b: f32 =b2_cross(vcp.r_b, vc.normal);

			let k_normal: f32 =m_a + m_b + i_a * rn_a * rn_a + i_b * rn_b * rn_b;

			vcp.normal_mass = if k_normal > 0.0 { 1.0 / k_normal }else{ 0.0};

			let tangent: B2vec2 =b2_cross_vec_by_scalar(vc.normal, 1.0);

			let rt_a: f32 =b2_cross(vcp.r_a, tangent);
			let rt_b: f32 =b2_cross(vcp.r_b, tangent);

			let k_tangent: f32 =m_a + m_b + i_a * rt_a * rt_a + i_b * rt_b * rt_b;

			vcp.tangent_mass = if k_tangent > 0.0 { 1.0 /  k_tangent} else {0.0};

			// Setup a velocity bias for restitution.
			vcp.velocity_bias = 0.0;
			let v_rel: f32 =b2_dot(vc.normal, v_b + b2_cross_scalar_by_vec(w_b, vcp.r_b) - v_a - b2_cross_scalar_by_vec(w_a, vcp.r_a));
			if v_rel < -B2_VELOCITY_THRESHOLD
			{
				vcp.velocity_bias = -vc.restitution * v_rel;
			}
		}

		// If we have two points, then prepare the block solver.
		if vc.point_count == 2 && G_BLOCK_SOLVE
		{
			let vcp1: &B2velocityConstraintPoint = &vc.points[0];
			let vcp2: &B2velocityConstraintPoint = &vc.points[1];

			let rn1_a: f32 =b2_cross(vcp1.r_a, vc.normal);
			let rn1_b: f32 =b2_cross(vcp1.r_b, vc.normal);
			let rn2_a: f32 =b2_cross(vcp2.r_a, vc.normal);
			let rn2_b: f32 =b2_cross(vcp2.r_b, vc.normal);

			let k11: f32 =m_a + m_b + i_a * rn1_a * rn1_a + i_b * rn1_b * rn1_b;
			let k22: f32 =m_a + m_b + i_a * rn2_a * rn2_a + i_b * rn2_b * rn2_b;
			let k12: f32 =m_a + m_b + i_a * rn1_a * rn2_a + i_b * rn1_b * rn2_b;

			// Ensure a reasonable condition number.
			const K_MAX_CONDITION_NUMBER: f32 = 1000.0;
			if k11 * k11 < K_MAX_CONDITION_NUMBER * (k11 * k22 - k12 * k12)
			{
				// k is safe to invert.
				vc.k.ex.set(k11, k12);
				vc.k.ey.set(k12, k22);
				vc.normal_mass = vc.k.get_inverse();
			}
			else
			{
				// The constraints are redundant, just use one.
				// TODO_ERIN use deepest?
				vc.point_count = 1;
			}
		}
	}
}

pub(crate) fn warm_start(this: &mut B2contactSolver, m_velocities: &mut [B2velocity])
{
	// Warm start.
	for i in 0..this.m_velocity_constraints.len()
	{
		let vc = &this.m_velocity_constraints[i];

		let index_a: i32 =vc.index_a;
		let index_b: i32 =vc.index_b;
		let m_a: f32 =vc.inv_mass_a;
		let i_a: f32 =vc.inv_ia;
		let m_b: f32 =vc.inv_mass_b;
		let i_b: f32 =vc.inv_ib;
		let point_count =vc.point_count as usize;

		let mut v_a: B2vec2 =m_velocities[index_a as usize].v;
		let mut w_a: f32 =m_velocities[index_a as usize].w;
		let mut v_b: B2vec2 =m_velocities[index_b as usize].v;
		let mut w_b: f32 =m_velocities[index_b as usize].w;

		let normal: B2vec2 =vc.normal;
		let tangent: B2vec2 =b2_cross_vec_by_scalar(normal, 1.0);

		for j in 0..point_count
		{
			let vcp = &vc.points[j];
			let p: B2vec2 =vcp.normal_impulse * normal + vcp.tangent_impulse * tangent;
			w_a -= i_a * b2_cross(vcp.r_a, p);
			v_a -= m_a * p;
			w_b += i_b * b2_cross(vcp.r_b, p);
			v_b += m_b * p;
		}

		m_velocities[index_a as usize].v = v_a;
		m_velocities[index_a as usize].w = w_a;
		m_velocities[index_b as usize].v = v_b;
		m_velocities[index_b as usize].w = w_b;
	}
}

pub(crate) fn solve_velocity_constraints(this: &mut B2contactSolver, m_velocities: &mut [B2velocity])
{
	for i in 0..this.m_velocity_constraints.len()
	{
		let vc = &mut this.m_velocity_constraints[i];

		let index_a: i32 =vc.index_a;
		let index_b: i32 =vc.index_b;
		let m_a: f32 =vc.inv_mass_a;
		let i_a: f32 =vc.inv_ia;
		let m_b: f32 =vc.inv_mass_b;
		let i_b: f32 =vc.inv_ib;
		let point_count =vc.point_count as usize;

		let mut v_a: B2vec2 =m_velocities[index_a as usize].v;
		let mut w_a: f32 =m_velocities[index_a as usize].w;
		let mut v_b: B2vec2 =m_velocities[index_b as usize].v;
		let mut w_b: f32 =m_velocities[index_b as usize].w;

		let normal: B2vec2 =vc.normal;
		let tangent: B2vec2 =b2_cross_vec_by_scalar(normal, 1.0);
		let friction: f32 =vc.friction;

		b2_assert(point_count == 1 || point_count == 2);

		// solve tangent constraints first because non-penetration is more important
		// than friction.
		for j in 0..point_count
		{
			let mut vcp = &mut vc.points[j];

			// Relative velocity at contact
			let dv: B2vec2 =v_b + b2_cross_scalar_by_vec(w_b, vcp.r_b) - v_a - b2_cross_scalar_by_vec(w_a, vcp.r_a);

			// Compute tangent force
			let vt: f32 =b2_dot(dv, tangent) - vc.tangent_speed;
			let mut lambda: f32 =vcp.tangent_mass * (-vt);

			// b2_clamp the accumulated force
			let max_friction: f32 =friction * vcp.normal_impulse;
			let new_impulse: f32 =b2_clamp(vcp.tangent_impulse + lambda, -max_friction, max_friction);
			lambda = new_impulse - vcp.tangent_impulse;
			vcp.tangent_impulse = new_impulse;
			
			// Apply contact impulse
			let p: B2vec2 =lambda * tangent;

			v_a -= m_a * p;
			w_a -= i_a * b2_cross(vcp.r_a, p);

			v_b += m_b * p;
			w_b += i_b * b2_cross(vcp.r_b, p);
		}

		// solve normal constraints
		if point_count == 1 || G_BLOCK_SOLVE == false
		{
			for j in 0..point_count
			{
				let mut vcp = &mut vc.points[j];

				// Relative velocity at contact
				let dv: B2vec2 =v_b + b2_cross_scalar_by_vec(w_b, vcp.r_b) - v_a - b2_cross_scalar_by_vec(w_a, vcp.r_a);

				// Compute normal impulse
				let vn: f32 =b2_dot(dv, normal);
				let mut lambda: f32 =-vcp.normal_mass * (vn - vcp.velocity_bias);

				// b2_clamp the accumulated impulse
				let new_impulse: f32 =b2_max(vcp.normal_impulse + lambda, 0.0);
				lambda = new_impulse - vcp.normal_impulse;
				vcp.normal_impulse = new_impulse;

				// Apply contact impulse
				let p: B2vec2 =lambda * normal;
				v_a -= m_a * p;
				w_a -= i_a * b2_cross(vcp.r_a, p);

				v_b += m_b * p;
				w_b += i_b * b2_cross(vcp.r_b, p);
			}
		}
		else
		{
			// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
			// Build the mini LCP for this contact patch
			//
			// vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
			//
			// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
			// b = vn0 - velocity_bias
			//
			// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
			// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
			// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
			// solution that satisfies the problem is chosen.
			// 
			// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
			// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
			//
			// Substitute:
			// 
			// x = a + d
			// 
			// a := old total impulse
			// x := new total impulse
			// d := incremental impulse 
			//
			// For the current iteration we extend the formula for the incremental impulse
			// to compute the new total impulse:
			//
			// vn = A * d + b
			//    = A * (x - a) + b
			//    = A * x + b - A * a
			//    = A * x + b'
			// b' = b - A * a;

			let (mut cp1,mut cp2) = get_two_mut(&mut vc.points, 0, 1);

			let a = B2vec2::new(cp1.normal_impulse, cp2.normal_impulse);
			b2_assert(a.x >= 0.0 && a.y >= 0.0);

			// Relative velocity at contact
			let mut dv1: B2vec2 =v_b + b2_cross_scalar_by_vec(w_b, cp1.r_b) - v_a - b2_cross_scalar_by_vec(w_a, cp1.r_a);
			let mut dv2: B2vec2 =v_b + b2_cross_scalar_by_vec(w_b, cp2.r_b) - v_a - b2_cross_scalar_by_vec(w_a, cp2.r_a);

			// Compute normal velocity
			let mut vn1: f32 =b2_dot(dv1, normal);
			let mut vn2: f32 =b2_dot(dv2, normal);

			let mut b = B2vec2{
				x : vn1 - cp1.velocity_bias,
				y : vn2 - cp2.velocity_bias
			};

			// Compute b'
			b -= b2_mul(vc.k, a);

			const K_ERROR_TOL: f32 = 1e-3;
			b2_not_used(K_ERROR_TOL);

			loop
			{
				//
				// Case 1: vn = 0
				//
				// 0 = A * x + b'
				//
				// solve for x:
				//
				// x = - inv(A) * b'
				//
				let mut x: B2vec2 =- b2_mul(vc.normal_mass, b);

				if x.x >= 0.0 && x.y >= 0.0
				{
					// Get the incremental impulse
					let d: B2vec2 =x - a;

					// Apply incremental impulse
					let p1: B2vec2 =d.x * normal;
					let p2: B2vec2 =d.y * normal;
					v_a -= m_a * (p1 + p2);
					w_a -= i_a * (b2_cross(cp1.r_a, p1) + b2_cross(cp2.r_a, p2));

					v_b += m_b * (p1 + p2);
					w_b += i_b * (b2_cross(cp1.r_b, p1) + b2_cross(cp2.r_b, p2));

					// Accumulate
					cp1.normal_impulse = x.x;
					cp2.normal_impulse = x.y;

					if B2_DEBUG_SOLVER
					{
						// Postconditions
						dv1 = v_b + b2_cross_scalar_by_vec(w_b, cp1.r_b) - v_a - b2_cross_scalar_by_vec(w_a, cp1.r_a);
						dv2 = v_b + b2_cross_scalar_by_vec(w_b, cp2.r_b) - v_a - b2_cross_scalar_by_vec(w_a, cp2.r_a);

						// Compute normal velocity
						vn1 = b2_dot(dv1, normal);
						vn2 = b2_dot(dv2, normal);

						b2_assert(b2_abs(vn1 - cp1.velocity_bias) < K_ERROR_TOL);
						b2_assert(b2_abs(vn2 - cp2.velocity_bias) < K_ERROR_TOL);
					}
					break;
				}

				//
				// Case 2: vn1 = 0 and x2 = 0
				//
				//   0 = a11 * x1 + a12 * 0 + b1' 
				// vn2 = a21 * x1 + a22 * 0 + b2'
				//
				x.x = - cp1.normal_mass * b.x;
				x.y = 0.0;
				//vn1 = 0.0;
				vn2 = vc.k.ex.y * x.x + b.y;
				if x.x >= 0.0 && vn2 >= 0.0
				{
					// Get the incremental impulse
					let d: B2vec2 =x - a;

					// Apply incremental impulse
					let p1: B2vec2 =d.x * normal;
					let p2: B2vec2 =d.y * normal;
					v_a -= m_a * (p1 + p2);
					w_a -= i_a * (b2_cross(cp1.r_a, p1) + b2_cross(cp2.r_a, p2));

					v_b += m_b * (p1 + p2);
					w_b += i_b * (b2_cross(cp1.r_b, p1) + b2_cross(cp2.r_b, p2));

					// Accumulate
					cp1.normal_impulse = x.x;
					cp2.normal_impulse = x.y;

if B2_DEBUG_SOLVER
{
					// Postconditions
					dv1 = v_b + b2_cross_scalar_by_vec(w_b, cp1.r_b) - v_a - b2_cross_scalar_by_vec(w_a, cp1.r_a);

					// Compute normal velocity
					vn1 = b2_dot(dv1, normal);

					b2_assert(b2_abs(vn1 - cp1.velocity_bias) < K_ERROR_TOL);
}
					break;
				}


				//
				// Case 3: vn2 = 0 and x1 = 0
				//
				// vn1 = a11 * 0 + a12 * x2 + b1' 
				//   0 = a21 * 0 + a22 * x2 + b2'
				//
				x.x = 0.0;
				x.y = - cp2.normal_mass * b.y;
				vn1 = vc.k.ey.x * x.y + b.x;
				//vn2 = 0.0;

				if x.y >= 0.0 && vn1 >= 0.0
				{
					// Resubstitute for the incremental impulse
					let d: B2vec2 =x - a;

					// Apply incremental impulse
					let p1: B2vec2 =d.x * normal;
					let p2: B2vec2 =d.y * normal;
					v_a -= m_a * (p1 + p2);
					w_a -= i_a * (b2_cross(cp1.r_a, p1) + b2_cross(cp2.r_a, p2));

					v_b += m_b * (p1 + p2);
					w_b += i_b * (b2_cross(cp1.r_b, p1) + b2_cross(cp2.r_b, p2));

					// Accumulate
					cp1.normal_impulse = x.x;
					cp2.normal_impulse = x.y;

					if B2_DEBUG_SOLVER
					{
					// Postconditions
					dv2 = v_b + b2_cross_scalar_by_vec(w_b, cp2.r_b) - v_a - b2_cross_scalar_by_vec(w_a, cp2.r_a);

					// Compute normal velocity
					vn2 = b2_dot(dv2, normal);

					b2_assert(b2_abs(vn2 - cp2.velocity_bias) < K_ERROR_TOL);
					}
					break;
				}

				//
				// Case 4: x1 = 0 and x2 = 0
				// 
				// vn1 = b1
				// vn2 = b2;
				x.x = 0.0;
				x.y = 0.0;
				vn1 = b.x;
				vn2 = b.y;

				if vn1 >= 0.0 && vn2 >= 0.0 
				{
					// Resubstitute for the incremental impulse
					let d: B2vec2 =x - a;

					// Apply incremental impulse
					let p1: B2vec2 =d.x * normal;
					let p2: B2vec2 =d.y * normal;
					v_a -= m_a * (p1 + p2);
					w_a -= i_a * (b2_cross(cp1.r_a, p1) + b2_cross(cp2.r_a, p2));

					v_b += m_b * (p1 + p2);
					w_b += i_b * (b2_cross(cp1.r_b, p1) + b2_cross(cp2.r_b, p2));

					// Accumulate
					cp1.normal_impulse = x.x;
					cp2.normal_impulse = x.y;

					break;
				}

				// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
				break;
			}
		}

		m_velocities[index_a as usize].v = v_a;
		m_velocities[index_a as usize].w = w_a;
		m_velocities[index_b as usize].v = v_b;
		m_velocities[index_b as usize].w = w_b;
	}
}

pub(crate) fn store_impulses<D:UserDataType>(this: &mut B2contactSolver, m_contacts: &[ContactPtr<D>])
{
	for vc in &this.m_velocity_constraints
	{
		let mut contact = m_contacts[vc.contact_index as usize].borrow_mut();
		let mut manifold = contact.get_base_mut().get_manifold_mut();

		for j in 0..vc.point_count as usize
		{
			manifold.points[j].normal_impulse = vc.points[j].normal_impulse;
			manifold.points[j].tangent_impulse = vc.points[j].tangent_impulse;
		}
	}
}

impl B2positionSolverManifold
{
	pub fn initialize(&mut self, pc: &B2contactPositionConstraint, xf_a: &B2Transform, xf_b: &B2Transform, index: i32)
	{
		b2_assert(pc.point_count > 0);

		match pc.mtype
		{
			B2manifoldType::ECircles =>
			{
				let point_a: B2vec2 =b2_mul_transform_by_vec2(*xf_a, pc.local_point);
				let point_b: B2vec2 =b2_mul_transform_by_vec2(*xf_b, pc.local_points[0]);
				self.normal = point_b - point_a;
				self.normal.normalize();
				self.point = 0.5 * (point_a + point_b);
				self.separation = b2_dot(point_b - point_a, self.normal) - pc.radius_a - pc.radius_b;
			}

		 B2manifoldType::EFaceA =>
			{
				self.normal = b2_mul_rot_by_vec2(xf_a.q, pc.local_normal);
				let plane_point: B2vec2 =b2_mul_transform_by_vec2(*xf_a, pc.local_point);

				let clip_point: B2vec2 =b2_mul_transform_by_vec2(*xf_b, pc.local_points[index as usize]);
				self.separation = b2_dot(clip_point - plane_point, self.normal) - pc.radius_a - pc.radius_b;
				self.point = clip_point;
			}

		 B2manifoldType::EFaceB =>
			{
				self.normal = b2_mul_rot_by_vec2(xf_b.q, pc.local_normal);
				let plane_point: B2vec2 =b2_mul_transform_by_vec2(*xf_b, pc.local_point);

				let clip_point: B2vec2 =b2_mul_transform_by_vec2(*xf_a, pc.local_points[index as usize]);
				self.separation = b2_dot(clip_point - plane_point, self.normal) - pc.radius_a - pc.radius_b;
				self.point = clip_point;

				// Ensure normal points from A to b
				self.normal = -self.normal;
			}
		}
	}
}

#[derive(Default, Copy, Clone, Debug)]
struct B2positionSolverManifold
{
	normal: B2vec2,
	point: B2vec2,
	separation: f32,
}

// Sequential solver.
pub(crate) fn  solve_position_constraints(this: &mut B2contactSolver, m_positions: &mut [B2position])-> bool
{
	let mut min_separation: f32 =0.0;

	for i in 0..this.m_position_constraints.len()
	{
		let pc = &this.m_position_constraints[i];

		let index_a: i32 =pc.index_a;
		let index_b: i32 =pc.index_b;
		let local_center_a: B2vec2 =pc.local_center_a;
		let m_a: f32 =pc.inv_mass_a;
		let i_a: f32 =pc.inv_ia;
		let local_center_b: B2vec2 =pc.local_center_b;
		let m_b: f32 =pc.inv_mass_b;
		let i_b: f32 =pc.inv_ib;
		let point_count =pc.point_count as usize;

		let mut c_a: B2vec2 =m_positions[index_a as usize].c;
		let mut a_a: f32 =m_positions[index_a as usize].a;

		let mut c_b: B2vec2 =m_positions[index_b as usize].c;
		let mut a_b: f32 =m_positions[index_b as usize].a;

		// solve normal constraints
		for j in 0..point_count
		{
			let mut xf_a = B2Transform::default();
			let mut xf_b = B2Transform::default();
			xf_a.q.set(a_a);
			xf_b.q.set(a_b);
			xf_a.p = c_a - b2_mul_rot_by_vec2(xf_a.q, local_center_a);
			xf_b.p = c_b - b2_mul_rot_by_vec2(xf_b.q, local_center_b);

			let mut psm = B2positionSolverManifold::default();
			psm.initialize(pc, &xf_a, &xf_b, j as i32);
			let normal: B2vec2 =psm.normal;

			let point: B2vec2 =psm.point;
			let separation: f32 =psm.separation;

			let r_a: B2vec2 =point - c_a;
			let r_b: B2vec2 =point - c_b;

			// Track max constraint error.
			min_separation = b2_min(min_separation, separation);

			// Prevent large corrections and allow slop.
			let c: f32 =b2_clamp(B2_BAUMGARTE * (separation + B2_LINEAR_SLOP), -B2_MAX_LINEAR_CORRECTION, 0.0);

			// Compute the effective mass.
			let rn_a: f32 =b2_cross(r_a, normal);
			let rn_b: f32 =b2_cross(r_b, normal);
			let k: f32 =m_a + m_b + i_a * rn_a * rn_a + i_b * rn_b * rn_b;

			// Compute normal impulse
			let impulse: f32 =if k > 0.0 {- c / k} else {0.0};

			let p: B2vec2 =impulse * normal;

			c_a -= m_a * p;
			a_a -= i_a * b2_cross(r_a, p);

			c_b += m_b * p;
			a_b += i_b * b2_cross(r_b, p);
		}

		m_positions[index_a as usize].c = c_a;
		m_positions[index_a as usize].a = a_a;

		m_positions[index_b as usize].c = c_b;
		m_positions[index_b as usize].a = a_b;
	}

	// We can't expect minSpeparation >= -B2_LINEAR_SLOP because we don't
	// push the separation above -B2_LINEAR_SLOP.
	return min_separation >= -3.0 * B2_LINEAR_SLOP;
}

// Sequential position solver for position constraints.
pub(crate) fn solve_toiposition_constraints(this: &mut B2contactSolver, toi_index_a: i32, toi_index_b: i32, 
	m_positions: &mut [B2position]) -> bool
{
	let mut min_separation: f32 =0.0;

	for i in 0..this.m_position_constraints.len()
	{
		let pc = &this.m_position_constraints[i];

		let index_a: i32 =pc.index_a;
		let index_b: i32 =pc.index_b;
		let local_center_a: B2vec2 =pc.local_center_a;
		let local_center_b: B2vec2 =pc.local_center_b;
		let point_count =pc.point_count as usize;

		let mut m_a: f32 =0.0;
		let mut i_a: f32 =0.0;
		if index_a == toi_index_a || index_a == toi_index_b
		{
			m_a = pc.inv_mass_a;
			i_a = pc.inv_ia;
		}

		let mut m_b: f32 =0.0;
		let mut i_b: f32 =0.0;
		if index_b == toi_index_a || index_b == toi_index_b
		{
			m_b = pc.inv_mass_b;
			i_b = pc.inv_ib;
		}

		let mut c_a: B2vec2 =m_positions[index_a as usize].c;
		let mut a_a: f32 =m_positions[index_a as usize].a;

		let mut c_b: B2vec2 =m_positions[index_b as usize].c;
		let mut a_b: f32 =m_positions[index_b as usize].a;

		// solve normal constraints
		for j in 0..point_count
		{
			let mut xf_a = B2Transform::default();
			let mut xf_b = B2Transform::default();
			xf_a.q.set(a_a);
			xf_b.q.set(a_b);
			xf_a.p = c_a - b2_mul_rot_by_vec2(xf_a.q, local_center_a);
			xf_b.p = c_b - b2_mul_rot_by_vec2(xf_b.q, local_center_b);

			let mut psm = B2positionSolverManifold::default();
			psm.initialize(pc, &xf_a, &xf_b, j as i32);
			let normal: B2vec2 =psm.normal;

			let point: B2vec2 =psm.point;
			let separation: f32 =psm.separation;

			let r_a: B2vec2 =point - c_a;
			let r_b: B2vec2 =point - c_b;

			// Track max constraint error.
			min_separation = b2_min(min_separation, separation);

			// Prevent large corrections and allow slop.
			let c: f32 =b2_clamp(B2_TOI_BAUMGARTE * (separation + B2_LINEAR_SLOP), -B2_MAX_LINEAR_CORRECTION, 0.0);

			// Compute the effective mass.
			let rn_a: f32 =b2_cross(r_a, normal);
			let rn_b: f32 =b2_cross(r_b, normal);
			let k: f32 =m_a + m_b + i_a * rn_a * rn_a + i_b * rn_b * rn_b;

			// Compute normal impulse
			let impulse: f32 = if k > 0.0 {- c / k} else {0.0};

			let p: B2vec2 =impulse * normal;

			c_a -= m_a * p;
			a_a -= i_a * b2_cross(r_a, p);

			c_b += m_b * p;
			a_b += i_b * b2_cross(r_b, p);
		}

		m_positions[index_a as usize].c = c_a;
		m_positions[index_a as usize].a = a_a;

		m_positions[index_b as usize].c = c_b;
		m_positions[index_b as usize].a = a_b;
	}

	// We can't expect minSpeparation >= -B2_LINEAR_SLOP because we don't
	// push the separation above -B2_LINEAR_SLOP.
	return min_separation >= -1.5 * B2_LINEAR_SLOP;
}
