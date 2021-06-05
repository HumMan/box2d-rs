use crate::b2_body::*;
use crate::b2_fixture::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_shape::*;
use crate::b2_world::*;
use crate::double_linked_list::*;
use crate::linked_list::*;

use std::cell::RefCell;
use std::ptr;
use std::rc::Rc;

pub fn b2_body<D: UserDataType>(bd: &B2bodyDef<D>, world: B2worldPtr<D>) -> B2body<D> {
	b2_assert(bd.position.is_valid());
	b2_assert(bd.linear_velocity.is_valid());
	b2_assert(b2_is_valid(bd.angle));
	b2_assert(b2_is_valid(bd.angular_velocity));
	b2_assert(b2_is_valid(bd.angular_damping) && bd.angular_damping >= 0.0);
	b2_assert(b2_is_valid(bd.linear_damping) && bd.linear_damping >= 0.0);

	let mut m_flags = BodyFlags::default();

	if bd.bullet {
		m_flags.insert(BodyFlags::E_BULLET_FLAG);
	}
	if bd.fixed_rotation {
		m_flags |= BodyFlags::E_FIXED_ROTATION_FLAG;
	}
	if bd.allow_sleep {
		m_flags |= BodyFlags::E_AUTO_SLEEP_FLAG;
	}
	if bd.awake && bd.body_type != B2bodyType::B2StaticBody {
		m_flags |= BodyFlags::E_AWAKE_FLAG;
	}
	if bd.enabled {
		m_flags |= BodyFlags::E_ENABLED_FLAG;
	}

	let m_xf = B2Transform::new(bd.position, B2Rot::new(bd.angle));

	return B2body::<D> {
		m_world: Rc::downgrade(&world),

		m_xf: m_xf,

		m_sweep: B2Sweep {
			local_center: B2vec2::zero(),
			c0: m_xf.p,
			c: m_xf.p,
			a0: bd.angle,
			a: bd.angle,
			alpha0: 0.0,
		},

		m_joint_list: DoubleLinkedList::default(),
		m_contact_list: DoubleLinkedList::default(),
		m_prev: None,
		m_next: None,

		m_linear_velocity: bd.linear_velocity,
		m_angular_velocity: bd.angular_velocity,

		m_linear_damping: bd.linear_damping,
		m_angular_damping: bd.angular_damping,
		m_gravity_scale: bd.gravity_scale,

		m_force: B2vec2::zero(),
		m_torque: 0.0,

		m_sleep_time: 0.0,

		m_type: bd.body_type,

		m_mass: 0.0,
		m_inv_mass: 0.0,

		m_i: 0.0,
		m_inv_i: 0.0,

		m_user_data: bd.user_data.clone(),

		m_fixture_list: LinkedList::default(),
		m_fixture_count: 0,
		m_flags: m_flags,
		m_island_index: -1,
	};
}

pub fn set_type<D: UserDataType>(self_: BodyPtr<D>, body_type: B2bodyType) {
	let world;
	let m_contact_list;
	{
		let mut self_ = self_.borrow_mut();
		world = upgrade(&self_.m_world);
		b2_assert(world.borrow().is_locked() == false);
		if world.borrow().is_locked() == true {
			return;
		}

		if self_.m_type == body_type {
			return;
		}

		self_.m_type = body_type;

		self_.reset_mass_data();

		if self_.m_type == B2bodyType::B2StaticBody {
			self_.m_linear_velocity.set_zero();
			self_.m_angular_velocity = 0.0;
			self_.m_sweep.a0 = self_.m_sweep.a;
			self_.m_sweep.c0 = self_.m_sweep.c;
			self_.m_flags.remove(BodyFlags::E_AWAKE_FLAG);
			self_.synchronize_fixtures();
		}

		self_.set_awake(true);

		self_.m_force.set_zero();
		self_.m_torque = 0.0;
		m_contact_list = self_.m_contact_list.clone();
	}

	let m_contact_manager = world.borrow().m_contact_manager.clone();

	// Delete the attached contacts.
	for contact_edge in m_contact_list.iter() {
		m_contact_manager
			.borrow_mut()
			.destroy(contact_edge.borrow().contact.upgrade().unwrap().clone());
	}

	let m_fixture_list;
	{
		let mut self_ = self_.borrow_mut();
		self_.m_contact_list.clear();
		m_fixture_list = self_.m_fixture_list.clone();
	}

	// Touch the proxies so that new contacts will be created (when appropriate)
	let broad_phase = m_contact_manager.borrow().m_broad_phase.clone();
	for fixture_edge in m_fixture_list.iter() {
		let f = fixture_edge.borrow();
		let proxy_count: usize = f.m_proxy_count as usize;
		for i in 0..proxy_count {
			broad_phase
				.borrow_mut()
				.touch_proxy(f.m_proxies[i].borrow().proxy_id);
		}
	}
}

pub fn create_fixture<D: UserDataType>(self_: BodyPtr<D>, def: &B2fixtureDef<D>) -> FixturePtr<D> {
	let mut self_mut = self_.borrow_mut();
	let world = upgrade(&self_mut.m_world);
	b2_assert(world.borrow().is_locked() == false);
	if world.borrow().is_locked() == true {
		panic!();
	}

	let fixture: FixturePtr<D> = Rc::new(RefCell::new(B2fixture::default()));
	{
		let mut fixture_mut = fixture.borrow_mut();
		B2fixture::create(&mut fixture_mut, self_.clone(), def);
	}

	if self_mut.m_flags.contains(BodyFlags::E_ENABLED_FLAG) {
		let broad_phase = world
			.borrow()
			.m_contact_manager
			.borrow()
			.m_broad_phase
			.clone();
		B2fixture::create_proxies(
			fixture.clone(),
			&mut broad_phase.borrow_mut(),
			&self_mut.m_xf,
		);
	}

	{
		self_mut.m_fixture_list.push_front(fixture.clone());
		self_mut.m_fixture_count += 1;

		fixture.borrow_mut().m_body = Some(Rc::downgrade(&self_));

		// Adjust mass properties if needed.
		if fixture.borrow().m_density > 0.0 {
			reset_mass_data(&mut self_mut);
		}
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	world.borrow_mut().m_new_contacts = true;

	return fixture;
}

pub fn create_fixture_by_shape<D: UserDataType>(
	self_: BodyPtr<D>,
	shape: ShapeDefPtr,
	density: f32,
) -> FixturePtr<D> {
	let mut def = B2fixtureDef::default();
	def.shape = Some(shape);
	def.density = density;

	return create_fixture(self_, &def);
}

pub fn destroy_fixture<D: UserDataType>(self_: BodyPtr<D>, fixture: FixturePtr<D>) {
	let world;
	let m_contact_list;
	let m_fixture_count;
	{
		let self_ = self_.borrow();
		world = upgrade(&self_.m_world.clone());
		m_contact_list = self_.m_contact_list.clone();
		m_fixture_count = self_.m_fixture_count;
	}

	b2_assert(world.borrow().is_locked() == false);
	if world.borrow().is_locked() == true {
		return;
	}

	b2_assert(ptr::eq(
		upgrade_opt(&fixture.borrow().m_body).as_ref(),
		self_.as_ref(),
	));

	// Remove the fixture from self_ body's singly linked list.
	b2_assert(m_fixture_count > 0);

	//TODO_humman имеет смысл сделать списки как https://www.reddit.com/r/rust/comments/7zsy72/writing_a_doubly_linked_list_in_rust_is_easy/
	//+ сюда же попадает enum вместо dyn trait

	self_.borrow_mut().m_fixture_list.remove(fixture.clone());

	// // destroy any contacts associated with the fixture.
	//let edge = self_mut.m_contact_list;
	let m_contact_manager = world.borrow().m_contact_manager.clone();
	for edge in m_contact_list.iter() {
		let edge_ref = edge.borrow();
		let edge_ref_contact = edge_ref.contact.upgrade().unwrap();
		let fixture_a;
		let fixture_b;
		{
			let c = edge_ref_contact.borrow();

			fixture_a = c.get_base().get_fixture_a();
			fixture_b = c.get_base().get_fixture_b();
		}

		if Rc::ptr_eq(&fixture, &fixture_a) || Rc::ptr_eq(&fixture, &fixture_b) {
			// This destroys the contact and removes it from
			// self_ body's contact list.

			m_contact_manager
				.borrow_mut()
				.destroy(edge_ref.contact.upgrade().unwrap().clone());
		}
	}

	if self_.borrow().m_flags.contains(BodyFlags::E_ENABLED_FLAG) {
		let broad_phase = m_contact_manager
			.borrow()
			.m_broad_phase
			.clone();
		fixture
			.borrow_mut()
			.destroy_proxies(&mut *broad_phase.borrow_mut());
	}

	{
		let mut fixture = fixture.borrow_mut();
		fixture.m_body = None;
		fixture.m_next = None;
		// fixture.destroy(allocator);
		// fixture.~B2fixture();
		// allocator->Free(fixture, sizeof(B2fixture));
	}
	{
		let mut self_ = self_.borrow_mut();
		self_.m_fixture_count -= 1;

		// reset the mass data.
		self_.reset_mass_data();
	}
}

pub fn reset_mass_data<D: UserDataType>(self_: &mut B2body<D>) {
	// Compute mass data from shapes. Each shape has its own density.
	self_.m_mass = 0.0;
	self_.m_inv_mass = 0.0;
	self_.m_i = 0.0;
	self_.m_inv_i = 0.0;
	self_.m_sweep.local_center.set_zero();

	// Static and kinematic bodies have zero mass.
	if self_.m_type == B2bodyType::B2StaticBody || self_.m_type == B2bodyType::B2KinematicBody {
		self_.m_sweep.c0 = self_.m_xf.p;
		self_.m_sweep.c = self_.m_xf.p;
		self_.m_sweep.a0 = self_.m_sweep.a;
		return;
	}

	b2_assert(self_.m_type == B2bodyType::B2DynamicBody);

	// Accumulate mass over all fixtures.
	let mut local_center = B2vec2::zero();
	for f_ in self_.m_fixture_list.iter() {
		let f = f_.borrow();
		if f.m_density == 0.0 {
			continue;
		}

		let mut mass_data = B2massData::default();
		f.get_mass_data(&mut mass_data);
		self_.m_mass += mass_data.mass;
		local_center += mass_data.mass * mass_data.center;
		self_.m_i += mass_data.i;
	}

	// Compute center of mass.
	if self_.m_mass > 0.0 {
		self_.m_inv_mass = 1.0 / self_.m_mass;
		local_center *= self_.m_inv_mass;
	}

	if self_.m_i > 0.0 && !self_.m_flags.contains(BodyFlags::E_FIXED_ROTATION_FLAG) {
		// Center the inertia about the center of mass.
		self_.m_i -= self_.m_mass * b2_dot(local_center, local_center);
		b2_assert(self_.m_i > 0.0);
		self_.m_inv_i = 1.0 / self_.m_i;
	} else {
		self_.m_i = 0.0;
		self_.m_inv_i = 0.0;
	}

	// Move center of mass.
	let old_center: B2vec2 = self_.m_sweep.c;
	self_.m_sweep.local_center = local_center;
	self_.m_sweep.c0 = b2_mul_transform_by_vec2(self_.m_xf, self_.m_sweep.local_center);
	self_.m_sweep.c = self_.m_sweep.c0;

	// update center of mass velocity.
	self_.m_linear_velocity +=
		b2_cross_scalar_by_vec(self_.m_angular_velocity, self_.m_sweep.c - old_center);
}

pub fn set_mass_data<D: UserDataType>(self_: &mut B2body<D>, mass_data: &B2massData) {
	let world = upgrade(&self_.m_world);
	b2_assert(world.borrow().is_locked() == false);
	if world.borrow().is_locked() == true {
		return;
	}

	if self_.m_type != B2bodyType::B2DynamicBody {
		return;
	}

	self_.m_inv_mass = 0.0;
	self_.m_i = 0.0;
	self_.m_inv_i = 0.0;

	self_.m_mass = mass_data.mass;
	if self_.m_mass <= 0.0 {
		self_.m_mass = 1.0;
	}

	self_.m_inv_mass = 1.0 / self_.m_mass;

	if mass_data.i > 0.0 && !self_.m_flags.contains(BodyFlags::E_FIXED_ROTATION_FLAG) {
		self_.m_i = mass_data.i - self_.m_mass * b2_dot(mass_data.center, mass_data.center);
		b2_assert(self_.m_i > 0.0);
		self_.m_inv_i = 1.0 / self_.m_i;
	}

	// Move center of mass.
	let old_center: B2vec2 = self_.m_sweep.c;
	self_.m_sweep.local_center = mass_data.center;
	self_.m_sweep.c0 = b2_mul_transform_by_vec2(self_.m_xf, self_.m_sweep.local_center);
	self_.m_sweep.c = self_.m_sweep.c0;

	// update center of mass velocity.
	self_.m_linear_velocity +=
		b2_cross_scalar_by_vec(self_.m_angular_velocity, self_.m_sweep.c - old_center);
}

pub fn should_collide<D: UserDataType>(self_: &B2body<D>, other: BodyPtr<D>) -> bool {
	// At least one body should be dynamic.
	if self_.m_type != B2bodyType::B2DynamicBody
		&& other.borrow().m_type != B2bodyType::B2DynamicBody
	{
		return false;
	}

	// Does a joint prevent collision?
	for jn_ in self_.m_joint_list.iter() {
		let jn = jn_.borrow();
		let jn_other = upgrade(&jn.other);
		if Rc::ptr_eq(&jn_other, &other) {
			if upgrade(&jn.joint)
				.borrow()
				.get_base()
				.get_collide_connected()
				== false
			{
				return false;
			}
		}
	}

	return true;
}

pub fn set_transform<D: UserDataType>(self_: &mut B2body<D>, position: B2vec2, angle: f32) {
	let world = upgrade(&self_.m_world);
	b2_assert(world.borrow().is_locked() == false);
	if world.borrow().is_locked() == true {
		return;
	}

	self_.m_xf.q.set(angle);
	self_.m_xf.p = position;

	self_.m_sweep.c = b2_mul_transform_by_vec2(self_.m_xf, self_.m_sweep.local_center);
	self_.m_sweep.a = angle;

	self_.m_sweep.c0 = self_.m_sweep.c;
	self_.m_sweep.a0 = angle;

	let contact_manager = world.borrow().m_contact_manager.clone();
	let broad_phase_rc = contact_manager.borrow().m_broad_phase.clone();
	let mut broad_phase = broad_phase_rc.borrow_mut();
	for f in self_.m_fixture_list.iter() {
		f.borrow_mut()
			.synchronize(&mut broad_phase, self_.m_xf, self_.m_xf);
	}
}

pub fn synchronize_fixtures<D: UserDataType>(self_: &mut B2body<D>) {
	let world = upgrade(&self_.m_world);
	synchronize_fixtures_internal(self_, &*world.borrow());
}

pub fn synchronize_fixtures_by_world<D: UserDataType>(self_: &mut B2body<D>, world: &B2world<D>) {
	synchronize_fixtures_internal(self_, world);
}

fn synchronize_fixtures_internal<D: UserDataType>(self_: &mut B2body<D>, world: &B2world<D>) {
	let contact_manager = world.m_contact_manager.clone();
	let broad_phase_rc = contact_manager.borrow().m_broad_phase.clone();
	let mut broad_phase = broad_phase_rc.borrow_mut();

	if self_.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
		let mut xf1 = B2Transform::default();
		xf1.q.set(self_.m_sweep.a0);
		xf1.p = self_.m_sweep.c0 - b2_mul_rot_by_vec2(xf1.q, self_.m_sweep.local_center);

		for f in self_.m_fixture_list.iter() {
			f.borrow_mut()
				.synchronize(&mut broad_phase, xf1, self_.m_xf);
		}
	} else {
		for f in self_.m_fixture_list.iter() {
			f.borrow_mut()
				.synchronize(&mut broad_phase, self_.m_xf, self_.m_xf);
		}
	}
}

pub fn set_enabled<D: UserDataType>(self_: BodyPtr<D>, flag: bool) {
	let world;
	let m_fixture_list;
	let broad_phase_rc;
	let mut broad_phase;
	let m_xf;
	let m_contact_list;
	{
		let mut self_ = self_.borrow_mut();

		world = upgrade(&self_.m_world);
		let contact_manager = world.borrow().m_contact_manager.clone();
		broad_phase_rc = contact_manager.borrow().m_broad_phase.clone();
		broad_phase = broad_phase_rc.borrow_mut();
		b2_assert(world.borrow().is_locked() == false);
		if world.borrow().is_locked() == true {
			return;
		}

		self_.m_flags.set(BodyFlags::E_ENABLED_FLAG, flag);
		m_fixture_list = self_.m_fixture_list.clone();
		m_xf = self_.m_xf;
		m_contact_list = self_.m_contact_list.clone();
	}

	if flag {
		// create all proxies.
		for f in m_fixture_list.iter() {
			B2fixture::create_proxies(f, &mut broad_phase, &m_xf);
		}

		// Contacts are created at the beginning of the next
		world.borrow_mut().m_new_contacts = true;
	} else {
		// destroy all proxies.
		for f in m_fixture_list.iter() {
			f.borrow_mut().destroy_proxies(&mut broad_phase);
		}

		// destroy the attached contacts.
		for ce in m_contact_list.iter() {
			world
				.borrow()
				.m_contact_manager
				.clone()
				.borrow_mut()
				.destroy(ce.borrow().contact.upgrade().unwrap());
		}
		self_.borrow_mut().m_contact_list.clear();
	}
}

pub fn set_fixed_rotation<D: UserDataType>(self_: &mut B2body<D>, flag: bool) {
	let status: bool = self_.m_flags.contains(BodyFlags::E_FIXED_ROTATION_FLAG);
	if status == flag {
		return;
	}

	self_.m_flags.set(BodyFlags::E_FIXED_ROTATION_FLAG, flag);

	self_.m_angular_velocity = 0.0;

	self_.reset_mass_data();
}