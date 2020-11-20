use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use box2d_rs::joints::b2_distance_joint::*;
use box2d_rs::joints::b2_revolute_joint::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

// This is a fun demo that shows off the wheel joint
pub(crate) struct Dominos<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
}

impl<D: UserDataType> Dominos<D> {
	pub fn new<F: Facade>(global_draw: TestBedDebugDrawPtr) -> TestPtr<D, F> {
		let base = Rc::new(RefCell::new(Test::new(global_draw.clone())));
		let result_ptr = Rc::new(RefCell::new(Self {
			base: base.clone(),
			destruction_listener: Rc::new(RefCell::new(B2testDestructionListenerDefault {
				base: Rc::downgrade(&base),
			})),
			contact_listener: Rc::new(RefCell::new(B2testContactListenerDefault {
				base: Rc::downgrade(&base),
			})),
		}));

		{
			let mut self_ = result_ptr.borrow_mut();
			{
				let world = base.borrow().m_world.clone();
				let mut world = world.borrow_mut();
				world.set_destruction_listener(self_.destruction_listener.clone());
				world.set_contact_listener(self_.contact_listener.clone());
				world.set_debug_draw(global_draw);
			}
			self_.init();
		}

		return result_ptr;
	}
	fn init(&mut self) {
		let m_world = self.base.borrow().m_world.clone();
		let b1: BodyPtr<D>;

		{
			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));

			let bd = B2bodyDef::default();
			b1 = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(b1.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(6.0, 0.25);

			let mut bd = B2bodyDef::default();
			bd.position.set(-1.5, 10.0);
			let ground = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.1, 1.0);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 20.0;
			fd.friction = 0.1;

			for i in 0..10 {
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(-6.0 + 1.0 * i as f32, 11.25);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture(body.clone(), &fd);
			}
		}

		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box_angle(7.0, 0.25, B2vec2::zero(), 0.3);

			let mut bd = B2bodyDef::default();
			bd.position.set(1.0, 6.0);
			let ground = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		let b2: BodyPtr<D>;
		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.25, 1.5);

			let mut bd = B2bodyDef::default();
			bd.position.set(-7.0, 4.0);
			b2 = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(b2.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		let b3: BodyPtr<D>;
		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(6.0, 0.125);

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(-0.9, 1.0);
			bd.angle = -0.15;

			b3 = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(b3.clone(), Rc::new(RefCell::new(shape)), 10.0);
		}

		let mut jd = B2revoluteJointDef::default();
		let mut anchor = B2vec2::new(-2.0, 1.0);

		jd.initialize(b1.clone(), b3.clone(), anchor);
		jd.base.collide_connected = true;
		m_world
			.borrow_mut()
			.create_joint(&B2JointDefEnum::RevoluteJoint(jd.clone()));

		let b4: BodyPtr<D>;
		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.25, 0.25);

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(-10.0, 15.0);
			b4 = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(b4.clone(), Rc::new(RefCell::new(shape)), 10.0);
		}

		anchor.set(-7.0, 15.0);
		jd.initialize(b2, b4, anchor);
		m_world
			.borrow_mut()
			.create_joint(&B2JointDefEnum::RevoluteJoint(jd.clone()));

		let b5: BodyPtr<D>;
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(6.5, 3.0);
			b5 = B2world::create_body(m_world.clone(), &bd);

			let shape = Rc::new(RefCell::new(B2polygonShape::default()));
			let mut fd = B2fixtureDef::default();

			fd.shape = Some(shape.clone());
			fd.density = 10.0;
			fd.friction = 0.1;

			shape
				.borrow_mut()
				.set_as_box_angle(1.0, 0.1, B2vec2::new(0.0, -0.9), 0.0);
			B2body::create_fixture(b5.clone(), &fd);

			shape
				.borrow_mut()
				.set_as_box_angle(0.1, 1.0, B2vec2::new(-0.9, 0.0), 0.0);
			B2body::create_fixture(b5.clone(), &fd);

			shape
				.borrow_mut()
				.set_as_box_angle(0.1, 1.0, B2vec2::new(0.9, 0.0), 0.0);
			B2body::create_fixture(b5.clone(), &fd);
		}

		anchor.set(6.0, 2.0);
		jd.initialize(b1, b5.clone(), anchor);
		m_world
			.borrow_mut()
			.create_joint(&B2JointDefEnum::RevoluteJoint(jd.clone()));

		let b6: BodyPtr<D>;
		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(1.0, 0.1);

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(6.5, 4.1);
			b6 = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(b6.clone(), Rc::new(RefCell::new(shape)), 30.0);
		}

		anchor.set(7.5, 4.0);
		jd.initialize(b5, b6, anchor);
		m_world
			.borrow_mut()
			.create_joint(&B2JointDefEnum::RevoluteJoint(jd));

		let b7: BodyPtr<D>;
		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.1, 1.0);

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(7.4, 1.0);

			b7 = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(b7.clone(), Rc::new(RefCell::new(shape)), 10.0);
		}

		let mut djd = B2distanceJointDef::default();
		djd.base.body_a = Some(b3);
		djd.base.body_b = Some(b7);
		djd.local_anchor_a.set(6.0, 0.0);
		djd.local_anchor_b.set(0.0, -1.0);
		let d: B2vec2 = djd
			.base
			.body_b
			.as_ref()
			.unwrap()
			.borrow()
			.get_world_point(djd.local_anchor_b)
			- djd
				.base
				.body_a
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(djd.local_anchor_a);
		djd.length = d.length();
		m_world
			.borrow_mut()
			.create_joint(&B2JointDefEnum::DistanceJoint(djd));

		{
			let radius: f32 = 0.2;

			let mut shape = B2circleShape::default();
			shape.base.m_radius = radius;

			for i in 0..4 {
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(5.9 + 2.0 * radius * i as f32, 2.4);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(shape)), 10.0);
			}
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Dominos<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn step(
		&mut self,
		ui: &imgui::Ui<'_>,
		display: &F,
		target: &mut glium::Frame,
		settings: &mut Settings,
		camera: &mut Camera,
	) {
		Test::step(self.base.clone(), ui, display, target, settings, *camera);
	}
}
