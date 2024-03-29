use crate::b2_body::*;
use crate::b2_broad_phase::*;
use crate::b2_collision::*;
use crate::b2_fixture::*;
use crate::b2_math::*;
use std::rc::Rc;
use std::ptr;
use crate::b2_common::*;
use crate::b2rs_common::*;

pub fn b2_fixture_default<T:UserDataType>() -> B2fixture<T> {
	return B2fixture::<T> {
		m_user_data: None,
		m_body: None,
		m_next: None,
		m_proxies: Vec::default(),
		m_proxy_count:0,
		m_shape: None,
		m_density: 0.0,
		m_filter: B2filter::default(),
		m_friction: 0.0,
		m_is_sensor: false,
		m_restitution: 0.0,
		m_restitution_threshold: 0.0
	};
}

pub fn b2_fixture_create<T:UserDataType>(
	self_: &mut B2fixture<T>,
	body: BodyPtr<T>,
	def: &B2fixtureDef<T>,
) {
	self_.m_user_data = def.user_data.clone();
	self_.m_friction = def.friction;
	self_.m_restitution = def.restitution;
	self_.m_restitution_threshold = def.restitution_threshold;

	self_.m_body = Some(Rc::downgrade(&body));
	self_.m_next = None;

	self_.m_filter = def.filter;

	self_.m_is_sensor = def.is_sensor;

	self_.m_shape = Some(def.shape.as_ref().unwrap().borrow().clone_rc());

	// Reserve proxy space
	let child_count: i32 = self_.m_shape.as_ref().unwrap().get_child_count() as i32;
	self_.m_proxies.resize_with(child_count as usize, Default::default);
	for i in 0..child_count
	{
		let mut proxy = self_.m_proxies[i as usize].as_ref().borrow_mut();
		proxy.fixture = None;
		proxy.proxy_id = E_NULL_PROXY;
	}
	self_.m_proxy_count = 0;

	self_.m_density = def.density;
}

// void b2Fixture_Destroy(b2BlockAllocator* allocator)
// {
// 	// The proxies must be destroyed before calling self_.
// 	b2_assert(m_proxyCount == 0);

// 	// Free the proxy array.
// 	i32 child_count = m_shape.get_child_count();
// 	allocator->Free(m_proxies, child_count * sizeof(B2fixtureProxy));
// 	m_proxies = None;

// 	// Free the child shape.
// 	switch (m_shape.m_type)
// 	{
// 	case B2ShapeType::ECircle:
// 		{
// 			B2circleShape* s = (B2circleShape*)m_shape;
// 			s->~B2circleShape();
// 			allocator->Free(s, sizeof(B2circleShape));
// 		}
// 		break;

// 	case B2ShapeType::EEdge:
// 		{
// 			B2edgeShape* s = (B2edgeShape*)m_shape;
// 			s->~B2edgeShape();
// 			allocator->Free(s, sizeof(B2edgeShape));
// 		}
// 		break;

// 	case B2ShapeType::EPolygon:
// 		{
// 			B2polygonShape* s = (B2polygonShape*)m_shape;
// 			s->~B2polygonShape();
// 			allocator->Free(s, sizeof(B2polygonShape));
// 		}
// 		break;

// 	case B2ShapeType::EChain:
// 		{
// 			B2chainShape* s = (B2chainShape*)m_shape;
// 			s->~B2chainShape();
// 			allocator->Free(s, sizeof(B2chainShape));
// 		}
// 		break;

// 	default:
// 		b2_assert(false);
// 		break;
// 	}

// 	m_shape = None;
// }

pub fn b2_fixture_create_proxies<T:UserDataType>(
	self_ptr: FixturePtr<T>,
	broad_phase: &mut B2broadPhase<FixtureProxyPtr<T>>,
	xf: &B2Transform,
) {
	let mut self_ = self_ptr.as_ref().borrow_mut();
	b2_assert(self_.m_proxy_count == 0);

	// create proxies in the broad-phase.
	self_.m_proxy_count = self_.m_shape.as_ref().unwrap().get_child_count() as i32;

	for i in 0..self_.m_proxy_count
	{
		let mut proxy = self_.m_proxies[i as usize].as_ref().borrow_mut();
		self_.m_shape.as_ref().unwrap().compute_aabb(&mut proxy.aabb, *xf, i as usize);
		proxy.proxy_id = broad_phase.create_proxy(proxy.aabb, &self_.m_proxies[i as usize]);
		proxy.fixture = Some(Rc::downgrade(&self_ptr));
		proxy.child_index = i;
	}
}

pub fn b2_fixture_destroy_proxies<T:UserDataType>(self_: &mut B2fixture<T>, broad_phase: &mut B2broadPhase<FixtureProxyPtr<T>>) {
	// destroy proxies in the broad-phase.
	for i in 0..self_.m_proxy_count
	{
		let mut proxy = self_.m_proxies[i as usize].as_ref().borrow_mut();
		broad_phase.destroy_proxy(proxy.proxy_id);
		proxy.proxy_id = E_NULL_PROXY;
	}

	self_.m_proxy_count = 0;
}

pub fn b2_fixture_synchronize<T:UserDataType>(
	self_: &mut B2fixture<T>,
	broad_phase: &mut B2broadPhase<FixtureProxyPtr<T>>,
	transform1: B2Transform,
	transform2: B2Transform,
) {
	if self_.m_proxy_count == 0
	{
		return;
	}

	for i in 0..self_.m_proxy_count
	{
		let mut proxy = self_.m_proxies[i as usize].as_ref().borrow_mut();

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		let mut aabb1 = B2AABB::default();
		let mut aabb2  = B2AABB::default();
		self_.m_shape.as_ref().unwrap().compute_aabb(&mut aabb1, transform1, proxy.child_index as usize);
		self_.m_shape.as_ref().unwrap().compute_aabb(&mut aabb2, transform2, proxy.child_index as usize);
		proxy.aabb.combine_two(aabb1, aabb2);

		let displacement: B2vec2 = aabb2.get_center() - aabb1.get_center();

		broad_phase.move_proxy(proxy.proxy_id, proxy.aabb, displacement);
	}
}

pub fn b2_fixture_set_filter_data<T:UserDataType>(self_: &mut B2fixture<T>, filter: B2filter) {
	self_.m_filter = filter;

	self_.refilter();
}

pub fn b2_fixture_refilter<T:UserDataType>(self_: &mut B2fixture<T>) {
	if self_.m_body.is_none()
	{
		return;
	}

	let m_body = upgrade_opt(&self_.m_body);

	// Flag associated contacts for filtering.
	for edge in m_body.borrow().get_contact_list().iter()
	{
		let contact = edge.borrow().contact.upgrade().unwrap();
		let mut contact = contact.borrow_mut();
		let fixture_a = contact.get_base().get_fixture_a();
		let fixture_b = contact.get_base().get_fixture_b();
		if ptr::eq(&*fixture_a.borrow(),self_) || ptr::eq(&*fixture_b.borrow(),self_)
		{
			contact.get_base_mut().flag_for_filtering();
		}
	}

	let world = m_body.borrow().get_world();

	// Touch each proxy so that new pairs may be created
	
	let broad_phase = world.borrow().m_contact_manager.borrow().m_broad_phase.clone();
	for i in 0..self_.m_proxy_count
	{
		broad_phase.borrow_mut().touch_proxy(self_.m_proxies[i as usize].borrow().proxy_id);
	}
}

pub fn b2_fixture_set_sensor<T:UserDataType>(self_: &mut B2fixture<T>, sensor: bool) {
	if sensor != self_.m_is_sensor
	{
		self_.m_body.as_ref().unwrap().upgrade().unwrap().borrow_mut().set_awake(true);
		self_.m_is_sensor = sensor;
	}
}