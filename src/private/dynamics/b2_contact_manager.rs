use crate::b2_body::*;
use crate::b2_contact::*;
use crate::b2_contact_manager::*;
use crate::b2_fixture::*;
use crate::b2rs_common::*;

use std::cell::RefCell;
use std::rc::Rc;

// B2contactFilter b2_defaultFilter;
// B2contactListener b2_defaultListener;

// pub fn b2_contact_manager_b2_contact_manager<D: UserDataType>() -> B2contactManager<D> {
// 	return B2contactManager::<D> {
// 		m_registers: Default::default(),
// 		m_contact_list: Default::default(),
// 		m_contact_count: 0,
// 		m_contact_filter: Some(Rc::new(RefCell::new(B2contactFilterDefault {}))),
// 		m_contact_listener: Some(Rc::new(RefCell::new(B2contactListenerDefault {}))),
// 		m_broad_phase: Rc::new(RefCell::new(B2broadPhase::<FixtureProxyPtr<D>>::new())),
// 	};
// }

pub fn b2_contact_manager_destroy<D: UserDataType>(
	self_: &mut B2contactManager<D>,
	c: ContactPtr<D>,
) {

	// if self_.m_contact_count!=self_.m_contact_list.len()
	// {
	// 	println!("m_contact_count={0}, m_contact_list.len={1}",self_.m_contact_count, self_.m_contact_list.len());
	// }
	
	// println!("m_contact_count={0}, m_contact_list.len={1}",self_.m_contact_count, self_.m_contact_list.len());

	//assert!(self_.m_contact_count>=1);
	//assert!(self_.m_contact_count==self_.m_contact_list.len());

	let fixture_a = c.borrow().get_base().get_fixture_a();
	let fixture_b = c.borrow().get_base().get_fixture_b();
	let body_a = fixture_a.borrow().get_body();
	let body_b = fixture_b.borrow().get_body();

	if let Some(ref m_contact_listener) = self_.m_contact_listener {
		let is_touching = c.borrow().get_base().is_touching();
		if is_touching {
			m_contact_listener.borrow_mut().end_contact(&mut *c.borrow_mut());
		}
	}

	// Remove from the world.
	self_.m_contact_list.remove(c.clone());

	//println!("m_contact_count={0}, m_contact_list.len={1}",self_.m_contact_count, self_.m_contact_list.len());

	// Remove from body 1
	{
		let m_node_a = c.borrow().get_base().m_node_a.as_ref().unwrap().clone();
		body_a.borrow_mut().m_contact_list.remove(m_node_a);
	}

	// Remove from body 2
	{
		let m_node_b = c.borrow().get_base().m_node_b.as_ref().unwrap().clone();
		body_b.borrow_mut().m_contact_list.remove(m_node_b);
	}

	// Call the factory.
	B2contact::destroy(&*c.borrow());
	self_.m_contact_count -= 1;

	// if self_.m_contact_count!=self_.m_contact_list.len()
	// {
	// 	println!("m_contact_count={0}, m_contact_list.len={1}",self_.m_contact_count, self_.m_contact_list.len());
	// }
	
	//assert!(self_.m_contact_count==self_.m_contact_list.len());
}

// This is the top level collision call for the time step. Here
// all the narrow phase collision is processed for the world
// contact list.
pub fn b2_contact_manager_collide<D: UserDataType>(self_: B2contactManagerPtr<D>) {
	let mut contacts_to_destroy = Vec::<ContactPtr<D>>::new();

	let (m_contact_list, m_broad_phase, m_contact_filter,m_contact_listener) = {
		let self_ = self_.borrow();
		//assert!(self_.m_contact_count==self_.m_contact_list.len());
		(self_.m_contact_list.clone(),self_.m_broad_phase.clone(),self_.m_contact_filter.clone(),self_.m_contact_listener.clone())
	};

	// update awake contacts.
	for c in m_contact_list.iter() {
		let fixture_a = c.borrow().get_base().get_fixture_a();
		let fixture_b = c.borrow().get_base().get_fixture_b();
		let index_a: i32 = c.borrow().get_base().get_child_index_a();
		let index_b: i32 = c.borrow().get_base().get_child_index_b();
		let body_a = fixture_a.borrow().get_body();
		let body_b = fixture_b.borrow().get_body();
		// Is this contact flagged for filtering?
		if c.borrow()
			.get_base()
			.m_flags
			.contains(ContactFlags::E_FILTER_FLAG)
		{
			// Should these bodies collide?
			if body_b.borrow().should_collide(body_a.clone()) == false {
				contacts_to_destroy.push(c);
				continue;
			}

			// Check user filtering.
			if let Some(m_contact_filter) = m_contact_filter.clone() {
				if m_contact_filter
					.borrow()
					.should_collide(fixture_a.clone(), fixture_b.clone())
					== false
				{
					contacts_to_destroy.push(c);
					continue;
				}
			}

			// clear the filtering flag.
			c.borrow_mut()
				.get_base_mut()
				.m_flags
				.remove(ContactFlags::E_FILTER_FLAG);
		}

		let active_a: bool =
			body_a.borrow().is_awake() && body_a.borrow().m_type != B2bodyType::B2StaticBody;
		let active_b: bool =
			body_b.borrow().is_awake() && body_b.borrow().m_type != B2bodyType::B2StaticBody;

		// At least one body must be awake and it must be dynamic or kinematic.
		if active_a == false && active_b == false {
			continue;
		}

		let proxy_id_a: i32 = fixture_a.borrow().m_proxies[index_a as usize]
			.borrow()
			.proxy_id;
		let proxy_id_b: i32 = fixture_b.borrow().m_proxies[index_b as usize]
			.borrow()
			.proxy_id;
		let overlap: bool = m_broad_phase
			.borrow()
			.test_overlap(proxy_id_a, proxy_id_b);

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if overlap == false {
			contacts_to_destroy.push(c);
			continue;
		}

		// The contact persists.
		B2contact::update(
			&mut *c.borrow_mut(),
			m_contact_listener.clone(),
		);
	}

	if contacts_to_destroy.len()>0
	{
		let mut self_ = self_.borrow_mut();
		for c in contacts_to_destroy {
			self_.destroy(c);
		}
	}
}

pub fn b2_contact_manager_find_new_contacts<D: UserDataType>(self_: &mut B2contactManager<D>) {
	let broad_phase = self_.m_broad_phase.clone();
	broad_phase.borrow_mut().update_pairs(self_);
}

pub fn b2_contact_manager_add_pair<D: UserDataType>(
	self_: &mut B2contactManager<D>,
	proxy_user_data_a: Option<FixtureProxyPtr<D>>,
	proxy_user_data_b: Option<FixtureProxyPtr<D>>,
) {

	
	//assert!(self_.m_contact_count==self_.m_contact_list.len());

	let proxy_a = proxy_user_data_a;
	let proxy_b = proxy_user_data_b;

	let fixture_a = upgrade(proxy_a.as_ref().unwrap().borrow().fixture.as_ref().unwrap());
	let fixture_b = upgrade(proxy_b.as_ref().unwrap().borrow().fixture.as_ref().unwrap());

	let index_a: i32 = proxy_a.as_ref().unwrap().borrow().child_index;
	let index_b: i32 = proxy_b.as_ref().unwrap().borrow().child_index;

	let body_a = fixture_a.borrow().get_body();
	let body_b = fixture_b.borrow().get_body();

	// Are the fixtures on the same body?
	if Rc::ptr_eq(&body_a, &body_b) {
		return;
	}

	// TODO_ERIN use a hash table to remove a potential bottleneck when both
	// bodies have a lot of contacts.
	// Does a contact already exist?
	for edge_ in body_b.borrow().get_contact_list().iter() {
		let edge = edge_.borrow();
		if Rc::ptr_eq(&edge.other.upgrade().unwrap(), &body_a) {
			let contact = edge.contact.upgrade().unwrap();
			let edge_contact = contact.borrow();
			let f_a = edge_contact.get_base().get_fixture_a();
			let f_b = edge_contact.get_base().get_fixture_b();
			let i_a: i32 = edge_contact.get_base().get_child_index_a();
			let i_b: i32 = edge_contact.get_base().get_child_index_b();

			if Rc::ptr_eq(&f_a, &fixture_a)
				&& Rc::ptr_eq(&f_b, &fixture_b)
				&& i_a == index_a
				&& i_b == index_b
			{
				// A contact already exists.
				return;
			}

			if Rc::ptr_eq(&f_a, &fixture_b)
				&& Rc::ptr_eq(&f_b, &fixture_a)
				&& i_a == index_b
				&& i_b == index_a
			{
				// A contact already exists.
				return;
			}
		}
	}

	// Does a joint override collision? Is at least one body dynamic?
	if body_b.borrow().should_collide(body_a) == false {
		return;
	}

	// Check user filtering.
	if self_.m_contact_filter.is_some()
		&& self_
			.m_contact_filter
			.as_ref()
			.unwrap()
			.borrow()
			.should_collide(fixture_a.clone(), fixture_b.clone())
			== false
	{
		return;
	}

	// Call the factory.
	let c = B2contact::create(&*self_, fixture_a, index_a, fixture_b, index_b);

	// Contact creation may swap fixtures.
	let fixture_a = c.borrow().get_base().get_fixture_a();
	let fixture_b = c.borrow().get_base().get_fixture_b();

	//let index_a: i32 = c.borrow().get_base().get_child_index_a();
	//let index_b: i32 = c.borrow().get_base().get_child_index_b();

	let body_a = fixture_a.borrow().get_body();
	let body_b = fixture_b.borrow().get_body();

	// Insert into the world.
	self_.m_contact_list.push_front(c.clone());

	// Connect to island graph.

	// Connect to body A
	{
		let m_node_a_rc = Rc::new(RefCell::new(B2contactEdge {
			contact: Rc::downgrade(&c),
			other: Rc::downgrade(&body_b),
			prev: None,
			next: None,
		}));
		c.borrow_mut().get_base_mut().m_node_a = Some(m_node_a_rc.clone());

		body_a.borrow_mut().m_contact_list.push_front(m_node_a_rc);
	}

	// Connect to body b
	{
		let m_node_b_rc = Rc::new(RefCell::new(B2contactEdge {
			contact: Rc::downgrade(&c),
			other: Rc::downgrade(&body_a),
			prev: None,
			next: None,
		}));
		c.borrow_mut().get_base_mut().m_node_b = Some(m_node_b_rc.clone());

		body_b.borrow_mut().m_contact_list.push_front(m_node_b_rc);
	}

	self_.m_contact_count += 1;
	
	//assert!(self_.m_contact_count==self_.m_contact_list.len());
}
