use super::b2_island::*;
use crate::b2_body::*;
use crate::b2_collision::*;
use crate::b2_contact::*;
use crate::b2_draw::*;
use crate::b2_fixture::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_time_of_impact::*;
use crate::b2_time_step::*;
use crate::b2_timer::*;
use crate::b2_world::*;
use crate::b2_world_callbacks::*;
use crate::double_linked_list::*;
use crate::b2_contact_manager::*;


use crate::shapes::to_derived_shape::ShapeAsDerived;

use std::cell::RefCell;
use std::rc::Rc;

pub(crate) fn b2_world_new<D: UserDataType>(gravity: B2vec2) -> B2worldPtr<D> {
	return Rc::new(RefCell::new(B2world {
		m_destruction_listener: None,
		m_debug_draw: None,

		m_body_list: DoubleLinkedList::default(),
		m_joint_list: DoubleLinkedList::default(),

		m_body_count: 0,
		m_joint_count: 0,

		m_warm_starting: true,
		m_continuous_physics: true,
		m_sub_stepping: false,

		m_step_complete: true,

		m_allow_sleep: true,
		m_gravity: gravity,

		m_new_contacts: false,
		m_locked: false,
		m_clear_forces: true,

		m_inv_dt0: 0.0,

		m_contact_manager: Rc::new(RefCell::new(B2contactManager::new())),

		m_profile: Default::default(),
	}));
}

pub(crate) fn set_destruction_listener<D: UserDataType>(
	this: &mut B2world<D>,
	listener: B2destructionListenerPtr<D>,
) {
	this.m_destruction_listener = Some(listener);
}

pub(crate) fn set_contact_filter<D: UserDataType>(
	this: &mut B2world<D>,
	filter: B2contactFilterPtr<D>,
) {
	this.m_contact_manager.borrow_mut().m_contact_filter = Some(filter);
}

pub(crate) fn set_contact_listener<D: UserDataType>(
	this: &mut B2world<D>,
	listener: B2contactListenerPtr<D>,
) {
	this.m_contact_manager.borrow_mut().m_contact_listener = Some(listener);
}

pub(crate) fn set_debug_draw<D: UserDataType>(this: &mut B2world<D>, debug_draw: B2drawTraitPtr) {
	this.m_debug_draw = Some(debug_draw);
}

pub(crate) fn create_body<D: UserDataType>(this: B2worldPtr<D>, def: &B2bodyDef<D>) -> BodyPtr<D> {
	b2_assert(this.borrow().is_locked() == false);
	if this.borrow().is_locked() {
		panic!();
	}
	let b = Rc::new(RefCell::new(B2body::new(def, this.clone())));

	// Add to world doubly linked list.
	{
		let mut this = this.borrow_mut();
		this.m_body_list.push_front(b.clone());
		this.m_body_count += 1;
	}

	return b;
}

pub(crate) fn destroy_body<D: UserDataType>(this: &mut B2world<D>, b: BodyPtr<D>) {
	b2_assert(this.m_body_count > 0);
	b2_assert(this.is_locked() == false);
	if this.is_locked() {
		panic!();
	}

	// Delete the attached joints.	
	let m_joint_list = b.borrow().m_joint_list.clone();
	for je in m_joint_list.iter() {
		let joint = upgrade(&je.borrow().joint);
		if let Some(ref m_destruction_listener) = this.m_destruction_listener {
			m_destruction_listener
				.borrow_mut()
				.say_goodbye_joint(joint.clone());
		}

		destroy_joint(this, joint);
	}
	b.borrow_mut().m_joint_list.clear();

	// Delete the attached contacts.
	let m_contact_list = b.borrow().m_contact_list.clone();
	for ce in m_contact_list.iter() {
		this.m_contact_manager
			.borrow_mut()
			.destroy(ce.borrow().contact.upgrade().unwrap().clone());
	}
	b.borrow_mut().m_contact_list.clear();

	// Delete the attached fixtures. This destroys broad-phase proxies.
	let m_fixture_list = b.borrow().m_fixture_list.clone();
	for f in m_fixture_list.iter() {
		if let Some(ref m_destruction_listener) = this.m_destruction_listener {
			m_destruction_listener
				.borrow_mut()
				.say_goodbye_fixture(f.clone());
		}

		f.borrow_mut()
			.destroy_proxies(&mut this.m_contact_manager.borrow().m_broad_phase.borrow_mut());

		b.borrow_mut().m_fixture_count -= 1;
	}
	{
		let mut b = b.borrow_mut();
		b.m_fixture_list.remove_all();
		b.m_fixture_count = 0;
	}

	// Remove world body list.
	this.m_body_list.remove(b);

	this.m_body_count -= 1;
}

pub(crate) fn create_joint<D: UserDataType>(
	this: &mut B2world<D>,
	def: &B2JointDefEnum<D>,
) -> B2jointPtr<D> {
	b2_assert(this.is_locked() == false);
	if this.is_locked() {
		panic!();
	}

	let j = B2joint::create(def);

	// Connect to the world list.
	{
		this.m_joint_list.push_front(j.clone());
		this.m_joint_count += 1;
	}

	// Connect to the bodies' doubly linked lists.
	{
		let mut j_mut = j.borrow_mut();
		let j_base = j_mut.get_base_mut();

		j_base.m_edge_a = Some(Rc::new(RefCell::new(B2jointEdge {
			joint: Rc::downgrade(&j),
			other: Rc::downgrade(&j_base.m_body_b),
			prev: None,
			next: None,
		})));

		j_base
			.m_body_a
			.borrow_mut()
			.m_joint_list
			.push_front(j_base.m_edge_a.clone().unwrap());

		j_base.m_edge_b = Some(Rc::new(RefCell::new(B2jointEdge {
			joint: Rc::downgrade(&j),
			other: Rc::downgrade(&j_base.m_body_a),
			prev: None,
			next: None,
		})));

		j_base
			.m_body_b
			.borrow_mut()
			.m_joint_list
			.push_front(j_base.m_edge_b.clone().unwrap().clone());
	}

	let (body_a, body_b, collide_connected) = match def {
		B2JointDefEnum::DistanceJoint(ref val) => (
			val.base.body_a.clone().unwrap(),
			val.base.body_b.clone().unwrap(),
			val.base.collide_connected,
		),
		B2JointDefEnum::FrictionJoint(ref val)=>(
			val.base.body_a.clone().unwrap(),
			val.base.body_b.clone().unwrap(),
			val.base.collide_connected,
		),
		B2JointDefEnum::GearJoint(ref val)=>(
			val.base.body_a.clone().unwrap(),
			val.base.body_b.clone().unwrap(),
			val.base.collide_connected,
		),
		B2JointDefEnum::MouseJoint(ref val)=>(
			val.base.body_a.clone().unwrap(),
			val.base.body_b.clone().unwrap(),
			val.base.collide_connected,
		),
		B2JointDefEnum::MotorJoint(ref val)=>(
			val.base.body_a.clone().unwrap(),
			val.base.body_b.clone().unwrap(),
			val.base.collide_connected,
		),
		B2JointDefEnum::PulleyJoint(ref val)=>(
			val.base.body_a.clone().unwrap(),
			val.base.body_b.clone().unwrap(),
			val.base.collide_connected,
		),
		B2JointDefEnum::RevoluteJoint(ref val)=>(
			val.base.body_a.clone().unwrap(),
			val.base.body_b.clone().unwrap(),
			val.base.collide_connected,
		),
		B2JointDefEnum::RopeJoint(ref val)=>(
			val.base.body_a.clone().unwrap(),
			val.base.body_b.clone().unwrap(),
			val.base.collide_connected,
		),
		B2JointDefEnum::PrismaticJoint(ref val)=>(
			val.base.body_a.clone().unwrap(),
			val.base.body_b.clone().unwrap(),
			val.base.collide_connected,
		),
		B2JointDefEnum::WeldJoint(ref val)=>(
			val.base.body_a.clone().unwrap(),
			val.base.body_b.clone().unwrap(),
			val.base.collide_connected,
		),
		B2JointDefEnum::WheelJoint(ref val)=>(
			val.base.body_a.clone().unwrap(),
			val.base.body_b.clone().unwrap(),
			val.base.collide_connected,
		)
	};

	// If the joint prevents collisions, then flag any contacts for filtering.
	if collide_connected == false {
		for edge_ in body_b.borrow().get_contact_list().iter() {
			let edge = edge_.borrow_mut();
			if Rc::ptr_eq(&edge.other.upgrade().unwrap(), &body_a) {
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.contact.upgrade().unwrap()
					.borrow_mut()
					.get_base_mut()
					.flag_for_filtering();
			}
		}
	}

	// Note: creating a joint doesn't wake the bodies.

	return j;
}

pub(crate) fn destroy_joint<D: UserDataType>(this: &mut B2world<D>, j: B2jointPtr<D>) {
	b2_assert(this.is_locked() == false);
	if this.is_locked() {
		panic!();
	}

	let collide_connected: bool;
	let body_a;
	let body_b;
	{
		let j = j.borrow();
		let j = j.get_base();
		collide_connected = j.m_collide_connected;
		body_a = j.m_body_a.clone();
		body_b = j.m_body_b.clone();
	}

	// Remove from the doubly linked list.
	this.m_joint_list.remove(j.clone());

	// Disconnect from island graph.

	// Wake up connected bodies.
	body_a.borrow_mut().set_awake(true);
	body_b.borrow_mut().set_awake(true);

	{
		let j = j.borrow();
		let j = j.get_base();
		// Remove from body 1.
		body_a
			.borrow_mut()
			.m_joint_list
			.remove(j.m_edge_a.clone().unwrap());

		// Remove from body 2
		body_b
			.borrow_mut()
			.m_joint_list
			.remove(j.m_edge_b.clone().unwrap());
	}

	//B2joint::destroy(j, &m_blockAllocator);

	b2_assert(this.m_joint_count > 0);
	this.m_joint_count -= 1;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if collide_connected == false {
		for edge in body_b.borrow().get_contact_list().iter() {
			let edge = edge.borrow();
			if Rc::ptr_eq(&edge.other.upgrade().unwrap(), &body_a) {
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.contact.upgrade().unwrap()
					.borrow_mut()
					.get_base_mut()
					.flag_for_filtering();
			}
		}
	}
}

//
pub(crate) fn set_allow_sleeping<D: UserDataType>(this: &mut B2world<D>, flag: bool) {
	if flag == this.m_allow_sleep {
		return;
	}

	this.m_allow_sleep = flag;
	if this.m_allow_sleep == false {
		for b in this.m_body_list.iter() {
			b.borrow_mut().set_awake(true);
		}
	}
}

// Find islands, integrate and solve constraints, solve position constraints
pub(crate) fn solve<D: UserDataType>(this: &mut B2world<D>, step: B2timeStep) {
	this.m_profile.solve_init = 0.0;
	this.m_profile.solve_velocity = 0.0;
	this.m_profile.solve_position = 0.0;

	let mut island;
	{
		let contact_manager = this.m_contact_manager.borrow();
		// Size the island for the worst case.
		island = B2island::new(
			this.m_body_count,
			contact_manager.m_contact_count as usize,
			this.m_joint_count,
			contact_manager
				.m_contact_listener
				.clone()
		);
	}

	// clear all the island flags.
	for b in this.m_body_list.iter() {
		b.borrow_mut().m_flags.remove(BodyFlags::E_ISLAND_FLAG);
	}
	for c in this.m_contact_manager.borrow().m_contact_list.iter() {
		c.borrow_mut()
			.get_base_mut()
			.m_flags
			.remove(ContactFlags::E_ISLAND_FLAG);
	}
	for j in this.m_joint_list.iter() {
		j.borrow_mut().get_base_mut().m_island_flag = false;
	}

	// Build and simulate all awake islands.
	let mut stack = Vec::<BodyPtr<D>>::with_capacity(this.m_body_count as usize);
	for seed in this.m_body_list.iter() {
		{
			let seed = seed.borrow();
			if seed.m_flags.contains(BodyFlags::E_ISLAND_FLAG) {
				continue;
			}

			if seed.is_awake() == false || seed.is_enabled() == false {
				continue;
			}

			// The seed can be dynamic or kinematic.
			if seed.get_type() == B2bodyType::B2StaticBody {
				continue;
			}
		}

		// reset island and stack.
		island.clear();
		stack.clear();
		stack.push(seed.clone());
		seed.borrow_mut().m_flags.insert(BodyFlags::E_ISLAND_FLAG);

		// Perform a depth first search (DFS) on the constraint graph.
		while let Some(b) = stack.pop() {
			// Grab the next body off the stack and add it to the island.
			b2_assert(b.borrow().is_enabled() == true);
			island.add_body(b.clone());

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if b.borrow().get_type() == B2bodyType::B2StaticBody {
				continue;
			}

			// Make sure the body is awake (without resetting sleep timer).
			b.borrow_mut().m_flags.insert(BodyFlags::E_AWAKE_FLAG);

			// Search all contacts connected to this body.
			for ce in b.borrow().m_contact_list.iter() {
				let contact = ce.borrow().contact.upgrade().unwrap();
				let mut contact_mut = contact.borrow_mut();
				let contact_base = contact_mut.get_base_mut();
				// Has this contact already been added to an island?
				if contact_base.m_flags.contains(ContactFlags::E_ISLAND_FLAG) {
					continue;
				}

				// Is this contact solid and touching?
				if contact_base.is_enabled() == false || contact_base.is_touching() == false {
					continue;
				}

				// Skip sensors.
				let sensor_a: bool = contact_base.m_fixture_a.borrow().m_is_sensor;
				let sensor_b: bool = contact_base.m_fixture_b.borrow().m_is_sensor;
				if sensor_a || sensor_b {
					continue;
				}

				island.add_contact(contact.clone());
				contact_base.m_flags.insert(ContactFlags::E_ISLAND_FLAG);

				let other = ce.borrow().other.upgrade().unwrap();

				// Was the other body already added to this island?
				if other.borrow().m_flags.contains(BodyFlags::E_ISLAND_FLAG) {
					continue;
				}

				stack.push(other.clone());

				other.borrow_mut().m_flags.insert(BodyFlags::E_ISLAND_FLAG);
			}

			// Search all joints connect to this body.
			for je in b.borrow().m_joint_list.iter() {
				let je = je.borrow();
				let joint = upgrade(&je.joint);
				if joint.borrow().get_base().m_island_flag == true {
					continue;
				}

				let other = upgrade(&je.other);

				// Don't simulate joints connected to diabled bodies.
				if other.borrow().is_enabled() == false {
					continue;
				}

				island.add_joint(joint.clone());
				joint.borrow_mut().get_base_mut().m_island_flag = true;

				if other.borrow().m_flags.contains(BodyFlags::E_ISLAND_FLAG) {
					continue;
				}

				stack.push(other.clone());
				other.borrow_mut().m_flags.insert(BodyFlags::E_ISLAND_FLAG);
			}
		}

		let mut profile = B2Profile::default();
		island.solve(&mut profile, &step, this.m_gravity, this.m_allow_sleep);
		this.m_profile.solve_init += profile.solve_init;
		this.m_profile.solve_velocity += profile.solve_velocity;
		this.m_profile.solve_position += profile.solve_position;

		// Post solve cleanup.
		for b in &island.m_bodies {
			// Allow static bodies to participate in other islands.
			let mut b = b.borrow_mut();
			if b.get_type() == B2bodyType::B2StaticBody {
				b.m_flags.remove(BodyFlags::E_ISLAND_FLAG);
			}
		}
	}

	{
		let timer = B2timer::default();
		// synchronize fixtures, check for out of range bodies.
		for b in this.m_body_list.iter() {
			let mut b = b.borrow_mut();
			// If a body was not in an island then it did not move.
			if !b.m_flags.contains(BodyFlags::E_ISLAND_FLAG) {
				continue;
			}

			if b.get_type() == B2bodyType::B2StaticBody {
				continue;
			}

			// update fixtures (for broad-phase).
			b.synchronize_fixtures_by_world(this);
		}

		// Look for new contacts.
		this.m_contact_manager.borrow_mut().find_new_contacts();
		this.m_profile.broadphase = timer.get_milliseconds();
	}
}

// Find TOI contacts and solve them.
pub(crate) fn solve_toi<D: UserDataType>(this: &mut B2world<D>, step: B2timeStep) {
	let mut island = B2island::new(
		2 * B2_MAX_TOICONTACTS,
		B2_MAX_TOICONTACTS,
		0,
		this.m_contact_manager
			.borrow()
			.m_contact_listener
			.clone()
	);

	if this.m_step_complete {
		for b in this.m_body_list.iter() {
			let mut b = b.borrow_mut();
			b.m_flags.remove(BodyFlags::E_ISLAND_FLAG);
			b.m_sweep.alpha0 = 0.0;
		}

		for c in this.m_contact_manager.borrow().m_contact_list.iter() {
			let mut c = c.borrow_mut();
			let c = c.get_base_mut();
			// Invalidate TOI
			c.m_flags
				.remove(ContactFlags::E_TOI_FLAG | ContactFlags::E_ISLAND_FLAG);
			c.m_toi_count = 0;
			c.m_toi = 1.0;
		}
	}

	// Find TOI events and solve them.
	loop {
		// Find the first TOI.
		let mut min_contact: Option<ContactPtr<D>> = None;
		let mut min_alpha: f32 = 1.0;

		for c_ptr in this.m_contact_manager.borrow().m_contact_list.iter() {
			let mut c = c_ptr.borrow_mut();
			let mut c_base = c.get_base_mut();
			// Is this contact disabled?
			if c_base.is_enabled() == false {
				continue;
			}

			// Prevent excessive sub-stepping.
			if c_base.m_toi_count > B2_MAX_SUB_STEPS as i32 {
				continue;
			}

			let alpha: f32;
			if c_base.m_flags.contains(ContactFlags::E_TOI_FLAG) {
				// This contact has a valid cached TOI.
				alpha = c_base.m_toi;
			} else {
				let f_a = c_base.get_fixture_a();
				let f_b = c_base.get_fixture_b();

				// Is there a sensor?
				if f_a.borrow().is_sensor() || f_b.borrow().is_sensor() {
					continue;
				}

				let b_a = f_a.borrow().get_body();
				let b_b = f_b.borrow().get_body();

				let mut alpha0;
				{
					let mut b_a = b_a.borrow_mut();
					let mut b_b = b_b.borrow_mut();
					let type_a: B2bodyType = b_a.m_type;
					let type_b: B2bodyType = b_b.m_type;
					b2_assert(
						type_a == B2bodyType::B2DynamicBody || type_b == B2bodyType::B2DynamicBody,
					);

					let active_a: bool = b_a.is_awake() && type_a != B2bodyType::B2StaticBody;
					let active_b: bool = b_b.is_awake() && type_b != B2bodyType::B2StaticBody;

					// Is at least one body active (awake and dynamic or kinematic)?
					if active_a == false && active_b == false {
						continue;
					}

					let collide_a: bool = b_a.is_bullet() || type_a != B2bodyType::B2DynamicBody;
					let collide_b: bool = b_b.is_bullet() || type_b != B2bodyType::B2DynamicBody;

					// Are these two non-bullet dynamic bodies?
					if collide_a == false && collide_b == false {
						continue;
					}

					// Compute the TOI for this contact.
					// Put the sweeps onto the same time interval.
					alpha0 = b_a.m_sweep.alpha0;

					if b_a.m_sweep.alpha0 < b_b.m_sweep.alpha0 {
						alpha0 = b_b.m_sweep.alpha0;
						b_a.m_sweep.advance(alpha0);
					} else if b_b.m_sweep.alpha0 < b_a.m_sweep.alpha0 {
						alpha0 = b_a.m_sweep.alpha0;
						b_b.m_sweep.advance(alpha0);
					}

					b2_assert(alpha0 < 1.0);
				}

				let index_a: i32 = c_base.get_child_index_a();
				let index_b: i32 = c_base.get_child_index_b();

				// Compute the time of impact in interval [0, minTOI]
				let mut input = B2toiinput::default();
				input
					.proxy_a
					.set_shape(f_a.borrow().get_shape(), index_a as usize);
				input
					.proxy_b
					.set_shape(f_b.borrow().get_shape(), index_b as usize);
				input.sweep_a = b_a.borrow().m_sweep;
				input.sweep_b = b_b.borrow().m_sweep;
				input.t_max = 1.0;

				let mut output = B2toioutput::default();
				b2_time_of_impact(&mut output, &input);

				// Beta is the fraction of the remaining portion of the .
				let beta: f32 = output.t;
				if output.state == B2toioutputState::ETouching {
					alpha = b2_min(alpha0 + (1.0 - alpha0) * beta, 1.0);
				} else {
					alpha = 1.0;
				}

				c_base.m_toi = alpha;
				c_base.m_flags.insert(ContactFlags::E_TOI_FLAG);
			}

			if alpha < min_alpha {
				// This is the minimum TOI found so far.
				min_contact = Some(c_ptr.clone());
				min_alpha = alpha;
			}
		}

		if min_contact.is_none() || 1.0 - 10.0 * B2_EPSILON < min_alpha {
			// No more TOI events. Done!
			this.m_step_complete = true;
			break;
		}

		let min_contact = min_contact.unwrap();

		// advance the bodies to the TOI.
		let f_a = min_contact.borrow().get_base().get_fixture_a();
		let f_b = min_contact.borrow().get_base().get_fixture_b();
		let b_a = f_a.borrow().get_body();
		let b_b = f_b.borrow().get_body();

		let backup1: B2Sweep = b_a.borrow().m_sweep;
		let backup2: B2Sweep = b_b.borrow().m_sweep;

		b_a.borrow_mut().advance(min_alpha);
		b_b.borrow_mut().advance(min_alpha);

		// The TOI contact likely has some new contact points.
		B2contact::update(
			&mut *min_contact.borrow_mut(),
			this
				.m_contact_manager
				.borrow()
				.m_contact_listener.clone(),
		);
		min_contact
			.borrow_mut()
			.get_base_mut()
			.m_flags
			.remove(ContactFlags::E_TOI_FLAG);
		min_contact.borrow_mut().get_base_mut().m_toi_count += 1;

		// Is the contact solid?
		if min_contact.borrow().get_base().is_enabled() == false
			|| min_contact.borrow().get_base().is_touching() == false
		{
			// Restore the sweeps.
			min_contact.borrow_mut().get_base_mut().set_enabled(false);
			b_a.borrow_mut().m_sweep = backup1;
			b_b.borrow_mut().m_sweep = backup2;
			b_a.borrow_mut().synchronize_transform();
			b_b.borrow_mut().synchronize_transform();
			continue;
		}

		b_a.borrow_mut().set_awake(true);
		b_b.borrow_mut().set_awake(true);

		// Build the island
		island.clear();
		island.add_body(b_a.clone());
		island.add_body(b_b.clone());
		island.add_contact(min_contact.clone());

		b_a.borrow_mut().m_flags.insert(BodyFlags::E_ISLAND_FLAG);
		b_b.borrow_mut().m_flags.insert(BodyFlags::E_ISLAND_FLAG);
		min_contact
			.borrow_mut()
			.get_base_mut()
			.m_flags
			.insert(ContactFlags::E_ISLAND_FLAG);

		// Get contacts on body_a and body_b.
		let bodies: [_; 2] = [b_a.clone(), b_b.clone()];
		for i in 0..2 {
			let body = bodies[i].clone();
			if body.borrow().m_type == B2bodyType::B2DynamicBody {
				let contact_list = body.borrow().m_contact_list.clone();
				for ce in contact_list.iter() {
					// if island.m_body_count == island.m_body_capacity
					// {
					// 	break;
					// }

					// if island.m_contact_count == island.m_contact_capacity
					// {
					// 	break;
					// }

					let contact_ptr = ce.borrow().contact.upgrade().unwrap();
					let mut contact = contact_ptr.borrow_mut();

					// Has this contact already been added to the island?
					if contact
						.get_base_mut()
						.m_flags
						.contains(ContactFlags::E_ISLAND_FLAG)
					{
						continue;
					}

					// Only add static, kinematic, or bullet bodies.
					let other_ptr = ce.borrow().other.upgrade().unwrap();
					let backup;
					{
						let mut other = other_ptr.borrow_mut();
						if other.m_type == B2bodyType::B2DynamicBody
							&& body.borrow().is_bullet() == false
							&& other.is_bullet() == false
						{
							continue;
						}

						{
							let contact_base = contact.get_base_mut();
							// Skip sensors.
							let sensor_a: bool = contact_base.m_fixture_a.borrow().m_is_sensor;
							let sensor_b: bool = contact_base.m_fixture_b.borrow().m_is_sensor;
							if sensor_a || sensor_b {
								continue;
							}
						}

						// Tentatively advance the body to the TOI.
						backup = other.m_sweep;
						if !other.m_flags.contains(BodyFlags::E_ISLAND_FLAG) {
							other.advance(min_alpha);
						}
					}

					// update the contact points
					B2contact::update(
						&mut *contact,
						this
							.m_contact_manager
							.borrow()
							.m_contact_listener
							.clone()
					);

					//let mut contact = contact_ptr.borrow_mut();
					let contact_base = contact.get_base_mut();

					{
						let mut other = other_ptr.borrow_mut();

						// Was the contact disabled by the user?
						if contact_base.is_enabled() == false {
							other.m_sweep = backup;
							other.synchronize_transform();
							continue;
						}

						// Are there contact points?
						if contact_base.is_touching() == false {
							other.m_sweep = backup;
							other.synchronize_transform();
							continue;
						}

						// Add the contact to the island
						contact_base.m_flags.insert(ContactFlags::E_ISLAND_FLAG);
						island.add_contact(contact_ptr.clone());

						// Has the other body already been added to the island?
						if other.m_flags.contains(BodyFlags::E_ISLAND_FLAG) {
							continue;
						}
						// Add the other body to the island.
						other.m_flags.insert(BodyFlags::E_ISLAND_FLAG);

						if other.m_type != B2bodyType::B2StaticBody {
							other.set_awake(true);
						}
					}

					island.add_body(other_ptr.clone());
				}
			}
		}

		let dt = (1.0 - min_alpha) * step.dt;
		let sub_step = B2timeStep {
			dt: dt,
			inv_dt: 1.0 / dt,
			dt_ratio: 1.0,
			position_iterations: 20,
			velocity_iterations: step.velocity_iterations,
			warm_starting: false,
		};

		{
			let index_a = b_a.borrow().m_island_index;
			let index_b = b_b.borrow().m_island_index;
			island.solve_toi(
				&sub_step,
				index_a,
				index_b,
			);
		}

		// reset island flags and synchronize broad-phase proxies.
		for body in island.m_bodies.clone() {
			let mut body = body.borrow_mut();
			body.m_flags.remove(BodyFlags::E_ISLAND_FLAG);

			if body.m_type != B2bodyType::B2DynamicBody {
				continue;
			}

			body.synchronize_fixtures_by_world(this);

			// Invalidate all contact TOIs on this displaced body.
			for ce in body.m_contact_list.iter() {
				ce.borrow()
					.contact.upgrade().unwrap()
					.borrow_mut()
					.get_base_mut()
					.m_flags
					.remove(ContactFlags::E_TOI_FLAG | ContactFlags::E_ISLAND_FLAG);
			}
		}

		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		this.m_contact_manager.borrow_mut().find_new_contacts();

		if this.m_sub_stepping {
			this.m_step_complete = false;
			break;
		}
	}
}

pub(crate) fn step<D: UserDataType>(
	this: &mut B2world<D>,
	dt: f32,
	velocity_iterations: i32,
	position_iterations: i32,
) {
	let step_timer = B2timer::default();

	// If new fixtures were added, we need to find the new contacts.
	if this.m_new_contacts {
		this.m_contact_manager.borrow_mut().find_new_contacts();
		this.m_new_contacts = false;
	}

	this.m_locked = true;

	let step = B2timeStep {
		dt: dt,
		velocity_iterations: velocity_iterations,
		position_iterations: position_iterations,
		inv_dt: if dt > 0.0 { 1.0 / dt } else { 0.0 },
		dt_ratio: this.m_inv_dt0 * dt,
		warm_starting: this.m_warm_starting,
	};
	// update contacts. This is where some contacts are destroyed.
	{
		let timer = B2timer::default();
		B2contactManager::collide(this.m_contact_manager.clone());
		this.m_profile.collide = timer.get_milliseconds();
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if this.m_step_complete && step.dt > 0.0 {
		let timer = B2timer::default();
		this.solve(step);
		this.m_profile.solve = timer.get_milliseconds();
	}

	// Handle TOI events.
	if this.m_continuous_physics && step.dt > 0.0 {
		let timer = B2timer::default();
		this.solve_toi(step);
		this.m_profile.solve_toi = timer.get_milliseconds();
	}

	if step.dt > 0.0 {
		this.m_inv_dt0 = step.inv_dt;
	}

	if this.m_clear_forces {
		this.clear_forces();
	}

	this.m_locked = false;

	this.m_profile.step = step_timer.get_milliseconds();
}

pub(crate) fn clear_forces<D: UserDataType>(this: &mut B2world<D>) {
	for body in this.m_body_list.iter() {
		let mut body = body.borrow_mut();
		body.m_force.set_zero();
		body.m_torque = 0.0;
	}
}

pub(crate) fn query_aabb<D: UserDataType, F: B2queryCallback<D>>(
	this: &B2world<D>,
	mut callback: F,
	aabb: B2AABB,
) {
	let broad_phase_ptr = this.m_contact_manager.borrow().m_broad_phase.clone();
	let broad_phase = broad_phase_ptr.borrow();
	broad_phase.query(|proxy_id:i32|->bool{
		let proxy = broad_phase.get_user_data(proxy_id);
		return callback(upgrade_opt(&proxy.unwrap().borrow().fixture));
	}, aabb);
}

// impl<'a, D: UserDataType, f> RayCastCallback for B2worldRayCastWrapper<'a, D, f>
// where
// 	f: B2rayCastCallback<D>,
// {
// 	fn ray_cast_callback(&mut self, input: &B2rayCastInput, proxy_id: i32) -> f32 {
// 		let proxy = self.broad_phase.get_user_data(proxy_id);
// 		let fixture;
// 		let index;
// 		{
// 			let proxy = proxy.as_ref().unwrap().borrow();
// 			fixture = upgrade_opt(&proxy.fixture);
// 			index = proxy.child_index;
// 		}
// 		let mut output = B2rayCastOutput::default();
// 		let hit: bool = fixture.borrow().RayCast(&mut output, input, index);

// 		if hit
// 		{
// 			let fraction: f32 = output.fraction;
// 			let point: B2vec2 = (1.0 - fraction) * input.p1 + fraction * input.p2;
// 			return self.callback.report_fixture(fixture, point, output.normal, fraction);
// 		}

// 		return input.max_fraction;
// 	}
// }

// struct B2worldRayCastWrapper<'a, D: UserDataType, f>
// where
// 	f: B2rayCastCallback<D>,
// {
// 	broad_phase: &'a B2broadPhase<FixtureProxyPtr<D>>,
// 	callback: &'a mut f,
// }

pub(crate) fn ray_cast<D: UserDataType, F: B2rayCastCallback<D>>(
	this: &B2world<D>,
	mut callback: F,
	point1: B2vec2,
	point2: B2vec2,
) {

	let broad_phase_ptr = this.m_contact_manager.borrow().m_broad_phase.clone();
	let broad_phase = broad_phase_ptr.borrow();
	// let mut wrapper = B2worldRayCastWrapper::<D, f> {
	// 	broad_phase: &*broad_phase,
	// 	callback: callback,
	// };
	let input = B2rayCastInput{
		max_fraction : 1.0,
		p1 : point1,
		p2 : point2,
	};
	broad_phase.ray_cast(|input: &B2rayCastInput, proxy_id: i32| -> f32 {
		let proxy = broad_phase.get_user_data(proxy_id);
		let fixture;
		let index;
		{
			let proxy = proxy.as_ref().unwrap().borrow();
			fixture = upgrade_opt(&proxy.fixture);
			index = proxy.child_index;
		}
		let mut output = B2rayCastOutput::default();
		let hit: bool = fixture.borrow().ray_cast(&mut output, input, index);

		if hit
		{
			let fraction: f32 = output.fraction;
			let point: B2vec2 = (1.0 - fraction) * input.p1 + fraction * input.p2;
			return callback(fixture, point, output.normal, fraction);
		}

		return input.max_fraction;
	}, &input);
}

pub(crate) fn draw_shape<D: UserDataType>(
	this: &B2world<D>,
	fixture: FixturePtr<D>,
	xf: &B2Transform,
	color: &B2color,
) {
	let mut m_debug_draw = this.m_debug_draw.as_ref().unwrap().borrow_mut();
	match fixture.borrow().get_shape().as_derived()
	{
		ShapeAsDerived::AsCircle(circle) =>
		{
			let center: B2vec2 = b2_mul_transform_by_vec2(*xf, circle.m_p);
			let radius: f32 = circle.base.m_radius;
			let axis: B2vec2 = b2_mul_rot_by_vec2(xf.q, B2vec2::new(1.0, 0.0));

			m_debug_draw.draw_solid_circle(center, radius, axis, *color);
		}
		ShapeAsDerived::AsEdge(edge)  =>
		{
			let v1: B2vec2 = b2_mul_transform_by_vec2(*xf, edge.m_vertex1);
			let v2: B2vec2 = b2_mul_transform_by_vec2(*xf, edge.m_vertex2);
			m_debug_draw.draw_segment(v1, v2, *color);

			if edge.m_one_sided == false
			{
				m_debug_draw.draw_point(v1, 4.0, *color);
				m_debug_draw.draw_point(v2, 4.0, *color);
			}

		}
		ShapeAsDerived::AsChain(chain) =>
		{
			let count = chain.m_vertices.len();
			let vertices = &chain.m_vertices;

			let mut v1: B2vec2 = b2_mul_transform_by_vec2(*xf, vertices[0]);

			for i in 1..count
			{
				let v2: B2vec2 = b2_mul_transform_by_vec2(*xf, vertices[i]);
				m_debug_draw.draw_segment(v1, v2, *color);
				v1 = v2;
			}
		}
		ShapeAsDerived::AsPolygon(poly) => 
		{
			let vertex_count = poly.m_count;
			b2_assert(vertex_count <= B2_MAX_POLYGON_VERTICES);
			let mut vertices= <[B2vec2;B2_MAX_POLYGON_VERTICES]>::default();

			for i in 0..vertex_count
			{
				vertices[i] = b2_mul_transform_by_vec2(*xf, poly.m_vertices[i]);
			}

			m_debug_draw.draw_solid_polygon(&vertices[..vertex_count], *color);
		}
	}
}

pub(crate) fn debug_draw<D: UserDataType>(this: &B2world<D>) {
	if this.m_debug_draw.is_none()
	{
		return;
	}

	let flags = {
		let m_debug_draw_ptr = this.m_debug_draw.as_ref().unwrap().clone();
		let m_debug_draw = m_debug_draw_ptr.borrow_mut();

		m_debug_draw.get_base().get_flags()
	};

	if flags.contains(B2drawShapeFlags::SHAPE_BIT)
	{
		for b in this.m_body_list.iter()
		{
			let b = b.borrow();
			let xf:B2Transform = b.get_transform();
			for f in b.get_fixture_list().iter()
			{				
				if b.get_type() == B2bodyType::B2DynamicBody && b.m_mass == 0.0
				{
					// Bad body
					this.draw_shape(f, &xf, &B2color::new(1.0, 0.0, 0.0));
				}
				else if b.is_enabled() == false
				{
					this.draw_shape(f, &xf, &B2color::new(0.5, 0.5, 0.3));
				}
				else if b.get_type() == B2bodyType::B2StaticBody
				{
					this.draw_shape(f, &xf, &B2color::new(0.5, 0.9, 0.5));
				}
				else if b.get_type() == B2bodyType::B2KinematicBody
				{
					this.draw_shape(f, &xf, &B2color::new(0.5, 0.5, 0.9));
				}
				else if b.is_awake() == false
				{
					this.draw_shape(f, &xf, &B2color::new(0.6, 0.6, 0.6));
				}
				else
				{
					this.draw_shape(f, &xf, &B2color::new(0.9, 0.7, 0.7));
				}
			}
		}
	}

	let m_debug_draw_ptr = this.m_debug_draw.as_ref().unwrap().clone();
	let mut m_debug_draw = m_debug_draw_ptr.borrow_mut();

	if flags.contains(B2drawShapeFlags::JOINT_BIT)
	{

		for j in this.m_joint_list.iter()
		{
			j.borrow().draw(&mut *m_debug_draw);
		}
	}

	if flags.contains(B2drawShapeFlags::PAIR_BIT)
	{
		let color = B2color::new(0.3, 0.9, 0.9);
		for c in this.m_contact_manager.borrow().m_contact_list.iter()
		{
			let c = c.borrow();
			let c_base = c.get_base();
			let fixture_a = c_base.get_fixture_a();
			let fixture_b = c_base.get_fixture_b();
			let index_a = c_base.get_child_index_a();
			let index_b = c_base.get_child_index_b();
			let c_a = fixture_a.borrow().get_aabb(index_a).get_center();
			let c_b = fixture_b.borrow().get_aabb(index_b).get_center();

			m_debug_draw.draw_segment(c_a, c_b, color);
		}
	}

	if flags.contains(B2drawShapeFlags::AABB_BIT)
	{
		let color = B2color::new(0.9, 0.3, 0.9);
		let bp_ptr = this.m_contact_manager.borrow().m_broad_phase.clone();
		let bp = bp_ptr.borrow();

		for b in this.m_body_list.iter()
		{
			let b = b.borrow();
			if b.is_enabled() == false
			{
				continue;
			}

			for f in b.get_fixture_list().iter()
			{
				for proxy in &f.borrow().m_proxies
				{
					let aabb: B2AABB = bp.get_fat_aabb(proxy.borrow().proxy_id);
					let vs:[B2vec2;4] = [
						B2vec2::new(aabb.lower_bound.x, aabb.lower_bound.y),
						B2vec2::new(aabb.upper_bound.x, aabb.lower_bound.y),
						B2vec2::new(aabb.upper_bound.x, aabb.upper_bound.y),
						B2vec2::new(aabb.lower_bound.x, aabb.upper_bound.y),
					];

					m_debug_draw.draw_polygon(&vs, color);
				}
			}
		}
	}

	if flags.contains(B2drawShapeFlags::CENTER_OF_MASS_BIT)
	{
		for b in this.m_body_list.iter()
		{
			let b = b.borrow();
			let mut xf = b.get_transform();
			xf.p = b.get_world_center();
			m_debug_draw.draw_transform(xf);
		}
	}
}

pub(crate) fn get_proxy_count<D: UserDataType>(this: &B2world<D>) -> i32 {
	return this
		.m_contact_manager
		.borrow()
		.m_broad_phase
		.borrow()
		.get_proxy_count();
}

pub(crate) fn get_tree_height<D: UserDataType>(this: &B2world<D>) -> i32 {
	return this
		.m_contact_manager
		.borrow()
		.m_broad_phase
		.borrow()
		.get_tree_height();
}

pub(crate) fn get_tree_balance<D: UserDataType>(this: &B2world<D>) -> i32 {
	return this
		.m_contact_manager
		.borrow()
		.m_broad_phase
		.borrow()
		.get_tree_balance();
}

pub(crate) fn get_tree_quality<D: UserDataType>(this: &B2world<D>) -> f32 {
	return this
		.m_contact_manager
		.borrow()
		.m_broad_phase
		.borrow()
		.get_tree_quality();
}

pub(crate) fn shift_origin<D: UserDataType>(this: &B2world<D>, new_origin: B2vec2) {
	b2_assert(this.is_locked() == false);
	if this.is_locked() {
		panic!();
	}

	for b in this.m_body_list.iter() {
		let mut b = b.borrow_mut();
		b.m_xf.p -= new_origin;
		b.m_sweep.c0 -= new_origin;
		b.m_sweep.c -= new_origin;
	}
	for j in this.m_joint_list.iter() {
		j.borrow_mut().shift_origin(new_origin);
	}

	this.m_contact_manager
		.borrow()
		.m_broad_phase
		.borrow_mut()
		.shift_origin(new_origin);
}
