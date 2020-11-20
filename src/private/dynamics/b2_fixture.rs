use crate::b2_body::*;
use crate::b2_broad_phase::*;
use crate::b2_collision::*;
use crate::b2_fixture::*;
use crate::b2_math::*;
use std::rc::Rc;
use std::ptr;
use crate::b2_settings::*;

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
	};
}

pub fn b2_fixture_create<T:UserDataType>(
	this: &mut B2fixture<T>,
	body: BodyPtr<T>,
	def: &B2fixtureDef<T>,
) {
	this.m_user_data = def.user_data.clone();
	this.m_friction = def.friction;
	this.m_restitution = def.restitution;

	this.m_body = Some(Rc::downgrade(&body));
	this.m_next = None;

	this.m_filter = def.filter;

	this.m_is_sensor = def.is_sensor;

	this.m_shape = Some(def.shape.as_ref().unwrap().borrow().clone_rc());

	// Reserve proxy space
	let child_count: i32 = this.m_shape.as_ref().unwrap().get_child_count() as i32;
	this.m_proxies.resize_with(child_count as usize, Default::default);
	for i in 0..child_count
	{
		let mut proxy = this.m_proxies[i as usize].as_ref().borrow_mut();
		proxy.fixture = None;
		proxy.proxy_id = E_NULL_PROXY;
	}
	this.m_proxy_count = 0;

	this.m_density = def.density;
}

// void b2Fixture_Destroy(b2BlockAllocator* allocator)
// {
// 	// The proxies must be destroyed before calling this.
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
	this_ptr: FixturePtr<T>,
	broad_phase: &mut B2broadPhase<FixtureProxyPtr<T>>,
	xf: &B2Transform,
) {
	let mut this = this_ptr.as_ref().borrow_mut();
	b2_assert(this.m_proxy_count == 0);

	// create proxies in the broad-phase.
	this.m_proxy_count = this.m_shape.as_ref().unwrap().get_child_count() as i32;

	for i in 0..this.m_proxy_count
	{
		let mut proxy = this.m_proxies[i as usize].as_ref().borrow_mut();
		this.m_shape.as_ref().unwrap().compute_aabb(&mut proxy.aabb, *xf, i as usize);
		proxy.proxy_id = broad_phase.create_proxy(proxy.aabb, &this.m_proxies[i as usize]);
		proxy.fixture = Some(Rc::downgrade(&this_ptr));
		proxy.child_index = i;
	}
}

pub fn b2_fixture_destroy_proxies<T:UserDataType>(this: &mut B2fixture<T>, broad_phase: &mut B2broadPhase<FixtureProxyPtr<T>>) {
	// destroy proxies in the broad-phase.
	for i in 0..this.m_proxy_count
	{
		let mut proxy = this.m_proxies[i as usize].as_ref().borrow_mut();
		broad_phase.destroy_proxy(proxy.proxy_id);
		proxy.proxy_id = E_NULL_PROXY;
	}

	this.m_proxy_count = 0;
}

pub fn b2_fixture_synchronize<T:UserDataType>(
	this: &mut B2fixture<T>,
	broad_phase: &mut B2broadPhase<FixtureProxyPtr<T>>,
	transform1: B2Transform,
	transform2: B2Transform,
) {
	if this.m_proxy_count == 0
	{
		return;
	}

	for i in 0..this.m_proxy_count
	{
		let mut proxy = this.m_proxies[i as usize].as_ref().borrow_mut();

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		let mut aabb1 = B2AABB::default();
		let mut aabb2  = B2AABB::default();
		this.m_shape.as_ref().unwrap().compute_aabb(&mut aabb1, transform1, proxy.child_index as usize);
		this.m_shape.as_ref().unwrap().compute_aabb(&mut aabb2, transform2, proxy.child_index as usize);
		proxy.aabb.combine_two(aabb1, aabb2);

		let displacement: B2vec2 = aabb2.get_center() - aabb1.get_center();

		broad_phase.move_proxy(proxy.proxy_id, proxy.aabb, displacement);
	}
}

pub fn b2_fixture_set_filter_data<T:UserDataType>(this: &mut B2fixture<T>, filter: B2filter) {
	this.m_filter = filter;

	this.refilter();
}

pub fn b2_fixture_refilter<T:UserDataType>(this: &mut B2fixture<T>) {
	if this.m_body.is_none()
	{
		return;
	}

	let m_body = upgrade_opt(&this.m_body);

	// Flag associated contacts for filtering.
	for edge in m_body.borrow().get_contact_list().iter()
	{
		let contact = edge.borrow().contact.upgrade().unwrap();
		let mut contact = contact.borrow_mut();
		let fixture_a = contact.get_base().get_fixture_a();
		let fixture_b = contact.get_base().get_fixture_b();
		if ptr::eq(&*fixture_a.borrow(),this) || ptr::eq(&*fixture_b.borrow(),this)
		{
			contact.get_base_mut().flag_for_filtering();
		}
	}

	let world = m_body.borrow().get_world();

	// Touch each proxy so that new pairs may be created
	
	let broad_phase = world.borrow().m_contact_manager.borrow().m_broad_phase.clone();
	for i in 0..this.m_proxy_count
	{
		broad_phase.borrow_mut().touch_proxy(this.m_proxies[i as usize].borrow().proxy_id);
	}
}

pub fn b2_fixture_set_sensor<T:UserDataType>(this: &mut B2fixture<T>, sensor: bool) {
	if sensor != this.m_is_sensor
	{
		this.m_body.as_ref().unwrap().upgrade().unwrap().borrow_mut().set_awake(true);
		this.m_is_sensor = sensor;
	}
}

pub fn b2_fixture_dump<T:UserDataType>(_this: &B2fixture<T>, _body_index: i32) {
	//TODO_humman
	
	// b2Log("    B2fixtureDef fd;\n");
	// b2Log("    fd.friction = %.15lef;\n", m_friction);
	// b2Log("    fd.restitution = %.15lef;\n", m_restitution);
	// b2Log("    fd.density = %.15lef;\n", m_density);
	// b2Log("    fd.is_sensor = bool(%d);\n", m_is_sensor);
	// b2Log("    fd.filter.category_bits = uint16(%d);\n", m_filter.category_bits);
	// b2Log("    fd.filter.mask_bits = uint16(%d);\n", m_filter.mask_bits);
	// b2Log("    fd.filter.group_index = int16(%d);\n", m_filter.group_index);

	// switch (m_shape.m_type)
	// {
	// case B2ShapeType::ECircle:
	// 	{
	// 		B2circleShape* s = (B2circleShape*)m_shape;
	// 		b2Log("    B2circleShape shape;\n");
	// 		b2Log("    shape.m_radius = %.15lef;\n", s->m_radius);
	// 		b2Log("    shape.m_p.set(%.15lef, %.15lef);\n", s->m_p.x, s->m_p.y);
	// 	}
	// 	break;

	// case B2ShapeType::EEdge:
	// 	{
	// 		B2edgeShape* s = (B2edgeShape*)m_shape;
	// 		b2Log("    B2edgeShape shape;\n");
	// 		b2Log("    shape.m_radius = %.15lef;\n", s->m_radius);
	// 		b2Log("    shape.m_vertex0.set(%.15lef, %.15lef);\n", s->m_vertex0.x, s->m_vertex0.y);
	// 		b2Log("    shape.m_vertex1.set(%.15lef, %.15lef);\n", s->m_vertex1.x, s->m_vertex1.y);
	// 		b2Log("    shape.m_vertex2.set(%.15lef, %.15lef);\n", s->m_vertex2.x, s->m_vertex2.y);
	// 		b2Log("    shape.m_vertex3.set(%.15lef, %.15lef);\n", s->m_vertex3.x, s->m_vertex3.y);
	// 		b2Log("    shape.m_has_vertex0 = bool(%d);\n", s->m_has_vertex0);
	// 		b2Log("    shape.m_has_vertex3 = bool(%d);\n", s->m_has_vertex3);
	// 	}
	// 	break;

	// case B2ShapeType::EPolygon:
	// 	{
	// 		B2polygonShape* s = (B2polygonShape*)m_shape;
	// 		b2Log("    let mut shape = B2polygonShape::default();\n");
	// 		b2Log("    B2vec2 vs[%d];\n", B2_MAX_POLYGON_VERTICES);
	// 		for (i32 i = 0; i < s->m_count; ++i)
	// 		{
	// 			b2Log("    vs[%d].set(%.15lef, %.15lef);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
	// 		}
	// 		b2Log("    shape.set(vs, %d);\n", s->m_count);
	// 	}
	// 	break;

	// case B2ShapeType::EChain:
	// 	{
	// 		B2chainShape* s = (B2chainShape*)m_shape;
	// 		b2Log("    let mut $1 = B2chainShape::default();\n");
	// 		b2Log("    B2vec2 vs[%d];\n", s->m_count);
	// 		for (i32 i = 0; i < s->m_count; ++i)
	// 		{
	// 			b2Log("    vs[%d].set(%.15lef, %.15lef);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
	// 		}
	// 		b2Log("    shape.create_chain(vs, %d);\n", s->m_count);
	// 		b2Log("    shape.m_prev_vertex.set(%.15lef, %.15lef);\n", s->m_prev_vertex.x, s->m_prev_vertex.y);
	// 		b2Log("    shape.m_next_vertex.set(%.15lef, %.15lef);\n", s->m_next_vertex.x, s->m_next_vertex.y);
	// 		b2Log("    shape.m_has_prev_vertex = bool(%d);\n", s->m_has_prev_vertex);
	// 		b2Log("    shape.m_has_next_vertex = bool(%d);\n", s->m_has_next_vertex);
	// 	}
	// 	break;

	// default:
	// 	return;
	// }

	// b2Log("\n");
	// b2Log("    fd.shape = &shape;\n");
	// b2Log("\n");
	// b2Log("    bodies[%d]->create_fixture(&fd);\n", body_index);
}
