use crate::b2_body::*;
use crate::b2_timer::*;
use crate::b2_world_callbacks::*;
use crate::b2_math::*;
use crate::b2_time_step::*;
use crate::b2_settings::*;
use super::b2_island::*;
use crate::private::dynamics::b2_contact_solver::*;

/*
Position Correction Notes
=========================
i tried the several algorithms for position correction of the 2D revolute joint.
i looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than B2_LINEAR_SLOP.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. i used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. i used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. i don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. i recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So i recommend
a system where the user can select the algorithm on a per joint basis. i would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/

pub(crate) fn solve<D: UserDataType>(this: &mut B2island<D>, profile: &mut B2Profile, step: &B2timeStep, gravity: B2vec2, allow_sleep: bool)
{
	let mut timer = B2timer::default();

	let h: f32 = step.dt;

	// Integrate velocities and apply damping. initialize the body state.
	for (i, b) in (&this.m_bodies).iter().enumerate()
	{
		let mut b = b.borrow_mut();

		let c: B2vec2 = b.m_sweep.c;
		let a:f32 = b.m_sweep.a;
		let mut v:B2vec2 = b.m_linear_velocity;
		let mut w:f32 = b.m_angular_velocity;

		// Store positions for continuous collision.
		b.m_sweep.c0 = b.m_sweep.c;
		b.m_sweep.a0 = b.m_sweep.a;

		if b.m_type == B2bodyType::B2DynamicBody
		{
			// Integrate velocities.
			v += h * b.m_inv_mass * (b.m_gravity_scale * b.m_mass * gravity + b.m_force);
			w += h * b.m_inv_i * b.m_torque;

			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Pade approximation:
			// v2 = v1 * 1 / (1 + c * dt)
			v *= 1.0 / (1.0 + h * b.m_linear_damping);
			w *= 1.0 / (1.0 + h * b.m_angular_damping);
		}

		this.m_positions[i].c = c;
		this.m_positions[i].a = a;
		this.m_velocities[i].v = v;
		this.m_velocities[i].w = w;
	}

	timer.reset();

	// Solver data
	let mut solver_data = B2solverData{
		step : *step,
		//TODO_humman
		//positions : &mut this.m_positions,
		//velocities : &mut this.m_velocities,
	};

	// initialize velocity constraints.
	let contact_solver_def = B2contactSolverDef
	{
		step : *step,
		//TODO_humman
		//contacts : &mut this.m_contacts,
		//positions : &mut this.m_positions,
		//velocities : &mut this.m_velocities,
	};

	let mut contact_solver = B2contactSolver::new::<D>(&contact_solver_def, &this.m_contacts);
	contact_solver.initialize_velocity_constraints(&this.m_positions, &this.m_velocities, &this.m_contacts);

	if step.warm_starting
	{
		contact_solver.warm_start(&mut this.m_velocities);
	}
	
	for j in &this.m_joints
	{
		j.borrow_mut().init_velocity_constraints(&mut solver_data, &mut this.m_positions, &mut this.m_velocities);
	}

	profile.solve_init = timer.get_milliseconds();

	// solve velocity constraints
	timer.reset();
	for _i in 0..step.velocity_iterations
	{
		for joint in &this.m_joints
		{
			joint.borrow_mut().solve_velocity_constraints(&mut solver_data, &mut this.m_velocities);
		}

		contact_solver.solve_velocity_constraints(&mut this.m_velocities);
	}

	// Store impulses for warm starting
	contact_solver.store_impulses(&this.m_contacts);
	profile.solve_velocity = timer.get_milliseconds();

	// Integrate positions
	for i in 0..this.m_bodies.len()
	{
		let mut c:B2vec2 = this.m_positions[i].c;
		let mut a:f32 = this.m_positions[i].a;
		let mut v:B2vec2 = this.m_velocities[i].v;
		let mut w:f32 = this.m_velocities[i].w;

		// Check for large velocities
		let translation:B2vec2 = h * v;
		if b2_dot(translation, translation) > B2_MAX_TRANSLATION_SQUARED
		{
			let ratio:f32 = B2_MAX_TRANSLATION / translation.length();
			v *= ratio;
		}

		let rotation:f32 = h * w;
		if rotation * rotation > B2_MAX_ROTATION_SQUARED
		{
			let ratio:f32 = B2_MAX_ROTATION / b2_abs(rotation);
			w *= ratio;
		}

		// Integrate
		c += h * v;
		a += h * w;

		this.m_positions[i].c = c;
		this.m_positions[i].a = a;
		this.m_velocities[i].v = v;
		this.m_velocities[i].w = w;
	}

	// solve position constraints
	timer.reset();
	let mut position_solved:bool = false;
	for _i in 0..step.position_iterations
	{
		let contacts_okay: bool = contact_solver.solve_position_constraints(&mut this.m_positions);

		let mut joints_okay: bool = true;
		for joint in &this.m_joints
		{
			let joint_okay: bool = joint.borrow_mut().solve_position_constraints(&mut solver_data, &mut this.m_positions);
			joints_okay = joints_okay && joint_okay;
		}

		if contacts_okay && joints_okay
		{
			// Exit early if the position errors are small.
			position_solved = true;
			break;
		}
	}

	// Copy state buffers back to the bodies
	for (i,body) in (&this.m_bodies).iter().enumerate()
	{
		let mut body = body.borrow_mut();
		body.m_sweep.c = this.m_positions[i].c;
		body.m_sweep.a = this.m_positions[i].a;
		body.m_linear_velocity = this.m_velocities[i].v;
		body.m_angular_velocity = this.m_velocities[i].w;
		body.synchronize_transform();
	}

	profile.solve_position = timer.get_milliseconds();

	this.report(&contact_solver.m_velocity_constraints);

	if allow_sleep
	{
		let mut min_sleep_time:f32 = B2_MAX_FLOAT;

		let lin_tol_sqr:f32 = B2_LINEAR_SLEEP_TOLERANCE * B2_LINEAR_SLEEP_TOLERANCE;
		let ang_tol_sqr:f32 = B2_ANGULAR_SLEEP_TOLERANCE * B2_ANGULAR_SLEEP_TOLERANCE;

		for b in &this.m_bodies
		{
			let mut b = b.borrow_mut();
			if b.get_type() == B2bodyType::B2StaticBody
			{
				continue;
			}

			if !b.m_flags.contains(BodyFlags::E_AUTO_SLEEP_FLAG) ||
				b.m_angular_velocity * b.m_angular_velocity > ang_tol_sqr ||
				b2_dot(b.m_linear_velocity, b.m_linear_velocity) > lin_tol_sqr
			{
				b.m_sleep_time = 0.0;
				min_sleep_time = 0.0;
			}
			else
			{
				b.m_sleep_time += h;
				min_sleep_time = b2_min(min_sleep_time, b.m_sleep_time);
			}
		}

		if min_sleep_time >= B2_TIME_TO_SLEEP && position_solved
		{
			for b in &this.m_bodies
			{
				b.borrow_mut().set_awake(false);
			}
		}
	}
}

pub(crate) fn solve_toi<D: UserDataType>(this: &mut B2island<D>, sub_step: &B2timeStep, toi_index_a: i32, toi_index_b: i32)
{
	// initialize the body state.
	for (i,b) in this.m_bodies.iter().enumerate()
	{
		let b = b.borrow();
		this.m_positions[i].c = b.m_sweep.c;
		this.m_positions[i].a = b.m_sweep.a;
		this.m_velocities[i].v = b.m_linear_velocity;
		this.m_velocities[i].w = b.m_angular_velocity;
	}

	let contact_solver_def = B2contactSolverDef{
	// contact_solver_def.contacts = m_contacts;
	// contact_solver_def.count = m_contact_count;
	// contact_solver_def.allocator = m_allocator;
	 step : *sub_step
	// contact_solver_def.positions = m_positions;
	// contact_solver_def.velocities = m_velocities;
	};
	let mut contact_solver = B2contactSolver::new::<D>(&contact_solver_def, &this.m_contacts);

	// solve position constraints.
	for _i in 0..sub_step.position_iterations
	{
		let contacts_okay:bool = contact_solver.solve_toiposition_constraints(toi_index_a, toi_index_b, &mut this.m_positions);
		if contacts_okay
		{
			break;
		}
	}

// #if 0
// 	// Is the new position really safe?
// 	for (i32 i = 0; i < m_contact_count; ++i)
// 	{
// 		B2contact* c = m_contacts[i];
// 		B2fixture* f_a = c->get_fixture_a();
// 		B2fixture* f_b = c->get_fixture_b();

// 		b2_body* b_a = f_a->get_body();
// 		b2_body* b_b = f_b->get_body();

// 		i32 index_a = c->get_child_index_a();
// 		i32 index_b = c->get_child_index_b();

// 		B2distanceInput input;
// 		input.proxy_a.set(f_a->get_shape(), index_a);
// 		input.proxy_b.set(f_b->get_shape(), index_b);
// 		input.transform_a = b_a->get_transform();
// 		input.transform_b = b_b->get_transform();
// 		input.use_radii = false;

// 		B2distanceOutput output;
// 		B2simplexCache cache;
// 		cache.count = 0;
// 		b2Distance(&output, &cache, &input);

// 		if output.distance == 0 || cache.count == 3
// 		{
// 			cache.count += 0;
// 		}
// 	}
// #endif

	// Leap of faith to new safe state.
	this.m_bodies[toi_index_a as usize].borrow_mut().m_sweep.c0 = this.m_positions[toi_index_a as usize].c;
	this.m_bodies[toi_index_a as usize].borrow_mut().m_sweep.a0 = this.m_positions[toi_index_a as usize].a;
	this.m_bodies[toi_index_b as usize].borrow_mut().m_sweep.c0 = this.m_positions[toi_index_b as usize].c;
	this.m_bodies[toi_index_b as usize].borrow_mut().m_sweep.a0 = this.m_positions[toi_index_b as usize].a;

	// No warm starting is needed for TOI events because warm
	// starting impulses were applied in the discrete solver.
	contact_solver.initialize_velocity_constraints(&this.m_positions, &this.m_velocities, &this.m_contacts);

	// solve velocity constraints.
	for _i in 0..sub_step.velocity_iterations
	{
		contact_solver.solve_velocity_constraints(&mut this.m_velocities);
	}

	// Don't store the TOI contact forces for warm starting
	// because they can be quite large.

	let h: f32 = sub_step.dt;

	// Integrate positions
	for i in 0..this.m_bodies.len()
	{
		let mut c:B2vec2 = this.m_positions[i].c;
		let mut a:f32 = this.m_positions[i].a;
		let mut v:B2vec2 = this.m_velocities[i].v;
		let mut w:f32 = this.m_velocities[i].w;

		// Check for large velocities
		let translation:B2vec2 = h * v;
		if b2_dot(translation, translation) > B2_MAX_TRANSLATION_SQUARED
		{
			let ratio:f32 = B2_MAX_TRANSLATION / translation.length();
			v *= ratio;
		}

		let rotation:f32 = h * w;
		if rotation * rotation > B2_MAX_ROTATION_SQUARED
		{
			let ratio:f32 = B2_MAX_ROTATION / b2_abs(rotation);
			w *= ratio;
		}

		// Integrate
		c += h * v;
		a += h * w;

		this.m_positions[i].c = c;
		this.m_positions[i].a = a;
		this.m_velocities[i].v = v;
		this.m_velocities[i].w = w;

		// Sync bodies
		let mut body = this.m_bodies[i].borrow_mut();
		body.m_sweep.c = c;
		body.m_sweep.a = a;
		body.m_linear_velocity = v;
		body.m_angular_velocity = w;
		body.synchronize_transform();
	}

	this.report(&contact_solver.m_velocity_constraints);
}

pub(crate) fn report<D: UserDataType>(this: &B2island<D>, constraints: &[B2contactVelocityConstraint])
{
	if this.m_listener.is_none()
	{
		return;
	}

	let listener = this.m_listener.as_ref().unwrap();

	assert_eq!(this.m_contacts.len(), constraints.len());

	for (i,c) in (&this.m_contacts).iter().enumerate()
	{
		let mut c = c.borrow_mut();
		let vc = &constraints[i];
		
		let mut impulse = B2contactImpulse::default();
		impulse.count = vc.point_count;

		for j in 0..vc.point_count
		{
			impulse.normal_impulses[j as usize] = vc.points[j as usize].normal_impulse;
			impulse.tangent_impulses[j as usize] = vc.points[j as usize].tangent_impulse;
		}

		listener.borrow_mut().post_solve(&mut *c, &impulse);
	}
}
