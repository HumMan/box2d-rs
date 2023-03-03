use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_common::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::joints::b2_revolute_joint::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct Tumbler<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_joint: Option<B2jointPtr<D>>,
	m_count: usize,
}

impl<D: UserDataType> Tumbler<D> {
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
			m_joint: None,
			m_count: 0,
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

	const E_COUNT: usize = 800;

	fn init(&mut self) {
		let m_world = self.base.borrow().m_world.clone();

		let ground;
		{
			let bd = B2bodyDef::default();
			ground = B2world::create_body(m_world.clone(), &bd);
		}

		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.allow_sleep = false;
			bd.position.set(0.0, 10.0);
			let body = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2polygonShape::default();
			shape.set_as_box_angle(0.5, 10.0, B2vec2::new(10.0, 0.0), 0.0);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 5.0);
			shape.set_as_box_angle(0.5, 10.0, B2vec2::new(-10.0, 0.0), 0.0);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 5.0);
			shape.set_as_box_angle(10.0, 0.5, B2vec2::new(0.0, 10.0), 0.0);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 5.0);
			shape.set_as_box_angle(10.0, 0.5, B2vec2::new(0.0, -10.0), 0.0);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 5.0);

			let mut jd = B2revoluteJointDef::default();
			jd.base.body_a = Some(ground);
			jd.base.body_b = Some(body);
			jd.local_anchor_a.set(0.0, 10.0);
			jd.local_anchor_b.set(0.0, 0.0);
			jd.reference_angle = 0.0;
			jd.motor_speed = 0.05 * B2_PI;
			jd.max_motor_torque = 1e8;
			jd.enable_motor = true;
			self.m_joint = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(jd)),
			);
		}

		self.m_count = 0;
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Tumbler<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn step(
		&mut self,
		ui: &imgui::Ui,
		display: &F,
		target: &mut glium::Frame,
		settings: &mut Settings,
		camera: &mut Camera,
	) {
		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		if self.m_count < Self::E_COUNT {
			let m_world = self.base.borrow().m_world.clone();

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(0.0, 10.0);
			let body = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.125, 0.125);
			B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(shape)), 1.0);

			self.m_count += 1;
		}
	}
}

//static int testIndex = RegisterTest("Benchmark", "Tumbler", Tumbler::create);
