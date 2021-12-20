use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::joints::b2_revolute_joint::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct Bridge<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
	m_middle: Option<BodyPtr<D>>,
}

impl<D: UserDataType> Bridge<D> {
	const E_COUNT: usize = 30;

	pub fn new<F: Facade>(global_draw: TestBedDebugDrawPtr) -> TestPtr<D, F> {
		let base = Rc::new(RefCell::new(Test::new(global_draw.clone())));
		let world = base.borrow().m_world.clone();
		let result_ptr = Rc::new(RefCell::new(Self {
			base: base.clone(),
			destruction_listener: Rc::new(RefCell::new(B2testDestructionListenerDefault {
				base: Rc::downgrade(&base),
			})),
			contact_listener: Rc::new(RefCell::new(B2testContactListenerDefault {
				base: Rc::downgrade(&base),
			})),
			m_middle: None,
		}));

		{
			let mut self_ = result_ptr.borrow_mut();
			{
				let mut world = world.borrow_mut();
				world.set_destruction_listener(self_.destruction_listener.clone());
				world.set_contact_listener(self_.contact_listener.clone());
				world.set_debug_draw(global_draw);
			}
			self_.init(world);
		}

		return result_ptr;
	}

	fn init(&mut self, world: B2worldPtr<D>) {
		let ground;
		{
			let bd = B2bodyDef::default();
			ground = B2world::create_body(world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.5, 0.125);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 20.0;
			fd.friction = 0.2;

			let mut prev_body = ground.clone();
			for i in 0..Self::E_COUNT {
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(-14.5 + 1.0 * i as f32, 5.0);
				let body = B2world::create_body(world.clone(), &bd);
				B2body::create_fixture(body.clone(), &fd);

				let anchor = B2vec2::new(-15.0 + 1.0 * i as f32, 5.0);
				let mut jd = B2revoluteJointDef::default();
				jd.initialize(prev_body.clone(), body.clone(), anchor);

				world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(jd));

				if i == (Self::E_COUNT >> 1) {
					self.m_middle = Some(body.clone());
				}
				prev_body = body;
			}

			let anchor = B2vec2::new(-15.0 + 1.0 * Self::E_COUNT as f32, 5.0);
			let mut jd = B2revoluteJointDef::default();
			jd.initialize(prev_body, ground, anchor);
			world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::RevoluteJoint(jd));
		}

		for i in 0..2 {
			let vertices = [
				B2vec2::new(-0.5, 0.0),
				B2vec2::new(0.5, 0.0),
				B2vec2::new(0.0, 1.5),
			];

			let mut shape = B2polygonShape::default();
			shape.set(&vertices);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 1.0;

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(-8.0 + 8.0 * i as f32, 12.0);
			let body = B2world::create_body(world.clone(), &bd);
			B2body::create_fixture(body.clone(), &fd);
		}

		for i in 0..3 {
			let mut shape = B2circleShape::default();
			shape.base.m_radius = 0.5;

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 1.0;

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(-6.0 + 6.0 * i as f32, 10.0);
			let body = B2world::create_body(world.clone(), &bd);
			B2body::create_fixture(body.clone(), &fd);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Bridge<D> {
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
