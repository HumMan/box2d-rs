use crate::b2_body::*;
use crate::b2_collision::*;
use crate::b2_contact::*;
use crate::b2_fixture::*;
use crate::b2_math::*;
use crate::b2rs_common::UserDataType;
use crate::b2_shape::*;
use crate::b2_world_callbacks::*;
use crate::b2_contact_manager::*;

pub fn b2_contact_create<D: UserDataType>(
	contact_manager: &B2contactManager<D>,
	fixture_a: FixturePtr<D>,
	index_a: i32,
	fixture_b: FixturePtr<D>,
	index_b: i32,
) -> ContactPtr<D> {

	let type1: B2ShapeType = fixture_a.borrow().get_type();
	let type2: B2ShapeType = fixture_b.borrow().get_type();

	let s_register = &contact_manager.m_registers.s_registers[type1 as usize][type2 as usize];

	let create_fcn = s_register.create_fcn.unwrap();

	if s_register.primary {
		return create_fcn(fixture_a, index_a, fixture_b, index_b);
	} else {
		return create_fcn(fixture_b, index_b, fixture_a, index_a);
	}
}

pub fn b2_contact_destroy<D: UserDataType>(self_: &dyn B2contactDynTrait<D>) {
	let contact_base = self_.get_base();

	let fixture_a = contact_base.m_fixture_a.borrow();
	let fixture_b = contact_base.m_fixture_b.borrow();

	if contact_base.m_manifold.point_count > 0
		&& fixture_a.is_sensor() == false
		&& fixture_b.is_sensor() == false
	{
		fixture_a.get_body().borrow_mut().set_awake(true);
		fixture_b.get_body().borrow_mut().set_awake(true);
	}
}

pub fn b2_contact_new<D: UserDataType>(
	f_a: FixturePtr<D>,
	index_a: i32,
	f_b: FixturePtr<D>,
	index_b: i32,
) -> B2contact<D> {
	return B2contact {
		m_flags: ContactFlags::E_ENABLED_FLAG,

		m_fixture_a: f_a.clone(),
		m_fixture_b: f_b.clone(),

		m_index_a: index_a,
		m_index_b: index_b,

		m_manifold: B2manifold {
			point_count: 0,
			..Default::default()
		},

		m_prev: None,
		m_next: None,

		m_node_a:None,
		m_node_b:None,
		// m_node_a: B2contactEdge {
		// 	contact: None,
		// 	prev: None,
		// 	next: None,
		// 	other: None,
		// },

		// m_node_b: B2contactEdge {
		// 	contact: None,
		// 	prev: None,
		// 	next: None,
		// 	other: None,
		// },

		m_toi_count: 0,

		m_friction: b2_mix_friction(f_a.borrow().m_friction, f_b.borrow().m_friction),
		m_restitution: b2_mix_restitution(f_a.borrow().m_restitution, f_b.borrow().m_restitution),
		m_restitution_threshold: b2_mix_restitution_threshold(f_a.borrow().m_restitution_threshold, f_b.borrow().m_restitution_threshold),

		m_tangent_speed: 0.0,

		m_toi: 0.0,
		//s_initialized: false,
		//s_registers: Default::default(),
	};
}

// update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
pub fn b2_contact_update<D: UserDataType>(
	self_: &mut dyn B2contactDynTrait<D>,
	listener: Option<B2contactListenerPtr<D>>
) {
	{
		let self_ = self_.get_base_mut();
		// Re-enable this contact.
		self_.m_flags |= ContactFlags::E_ENABLED_FLAG;
	}

	let old_manifold: B2manifold;

	let was_touching: bool;

	let sensor_a: bool;
	let sensor_b: bool;
	let sensor: bool;

	let body_a: BodyPtr<D>;
	let body_b: BodyPtr<D>;
	let xf_a: B2Transform;
	let xf_b: B2Transform;
	{
		let self_ = self_.get_base();
		old_manifold = self_.m_manifold;

		was_touching = self_.m_flags.contains(ContactFlags::E_TOUCHING_FLAG);

		sensor_a = self_.m_fixture_a.borrow().is_sensor();
		sensor_b = self_.m_fixture_b.borrow().is_sensor();
		sensor = sensor_a || sensor_b;
		{
			let temp = self_.m_fixture_a.borrow();
			body_a = temp.get_body();
		}
		{
			let temp = self_.m_fixture_b.borrow();
			body_b = temp.get_body();
		}
		xf_a = body_a.borrow().get_transform();
		xf_b = body_b.borrow().get_transform();
	}

	let touching: bool;

	// Is this contact a sensor?
	if sensor {
		let self_ = self_.get_base_mut();
		let shape_a = self_.m_fixture_a.borrow().get_shape();
		let shape_b = self_.m_fixture_b.borrow().get_shape();
		touching = b2_test_overlap_shapes(
			shape_a,
			self_.m_index_a as usize,
			shape_b,
			self_.m_index_b as usize,
			xf_a,
			xf_b,
		);

		// Sensors don't generate manifolds.
		self_.m_manifold.point_count = 0;
	} else {
		let mut new_manifold = B2manifold::default();
		self_.evaluate(&mut new_manifold, &xf_a, &xf_b);
		let self_ = self_.get_base_mut();
		self_.m_manifold = new_manifold;
		touching = self_.m_manifold.point_count > 0;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for i in 0..self_.m_manifold.point_count {
			let mp2 = &mut self_.m_manifold.points[i];
			mp2.normal_impulse = 0.0;
			mp2.tangent_impulse = 0.0;
			let id2: B2contactId = mp2.id;

			for j in 0..old_manifold.point_count {
				let mp1 = &old_manifold.points[j];

				if mp1.id.cf == id2.cf  {
					mp2.normal_impulse = mp1.normal_impulse;
					mp2.tangent_impulse = mp1.tangent_impulse;
					break;
				}
			}
		}

		if touching != was_touching {
			body_a.borrow_mut().set_awake(true);
			body_b.borrow_mut().set_awake(true);
		}
	}

	{
		let self_ = self_.get_base_mut();
		self_.m_flags.set(ContactFlags::E_TOUCHING_FLAG, touching);
	}

	if let Some(ref contact_listener) = listener
	{

		let mut contact_listener = contact_listener.borrow_mut();

		if was_touching == false && touching == true {
			contact_listener.begin_contact(self_);
		}

		if was_touching == true && touching == false {
			contact_listener.end_contact(self_);
		}

		if sensor == false && touching {
			contact_listener.pre_solve(self_, &old_manifold);
		}
	}
}
