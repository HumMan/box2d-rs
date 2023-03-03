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
use box2d_rs::joints::b2_prismatic_joint::*;
use box2d_rs::joints::b2_revolute_joint::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

// A motor driven slider crank with joint friction.
pub(crate) struct SliderCrank2<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_joint1: Option<B2jointPtr<D>>,
	m_joint2: Option<B2jointPtr<D>>,
}

impl<D: UserDataType> SliderCrank2<D> {
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
			m_joint1: None,
			m_joint2: None,
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

		let ground;
		{
			let bd = B2bodyDef::default();
			ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let mut prev_body: BodyPtr<D> = ground.clone();

			// Define crank.
			{
				let mut shape = B2polygonShape::default();
				shape.set_as_box(0.5, 2.0);

				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(0.0, 7.0);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 2.0);

				let mut rjd = B2revoluteJointDef::default();
				rjd.initialize(prev_body, body.clone(), B2vec2::new(0.0, 5.0));
				rjd.motor_speed = 1.0 * B2_PI;
				rjd.max_motor_torque = 10000.0;
				rjd.enable_motor = true;
				self.m_joint1 = Some(
					m_world
						.borrow_mut()
						.create_joint(&B2JointDefEnum::RevoluteJoint(rjd)),
				);

				prev_body = body;
			}

			// Define follower.
			{
				let mut shape = B2polygonShape::default();
				shape.set_as_box(0.5, 4.0);

				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(0.0, 13.0);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 2.0);

				let mut rjd = B2revoluteJointDef::default();
				rjd.initialize(prev_body, body.clone(), B2vec2::new(0.0, 9.0));
				rjd.enable_motor = false;
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(rjd));

				prev_body = body;
			}

			// Define piston
			{
				let mut shape = B2polygonShape::default();
				shape.set_as_box(1.5, 1.5);

				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.fixed_rotation = true;
				bd.position.set(0.0, 17.0);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 2.0);

				let mut rjd = B2revoluteJointDef::default();
				rjd.initialize(prev_body, body.clone(), B2vec2::new(0.0, 17.0));
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(rjd));

				let mut pjd = B2prismaticJointDef::default();
				pjd.initialize(
					ground,
					body.clone(),
					B2vec2::new(0.0, 17.0),
					B2vec2::new(0.0, 1.0),
				);

				pjd.max_motor_force = 1000.0;
				pjd.enable_motor = true;

				self.m_joint2 = Some(
					m_world
						.borrow_mut()
						.create_joint(&B2JointDefEnum::PrismaticJoint(pjd)),
				);
			}

			// create a payload
			{
				let mut shape = B2polygonShape::default();
				shape.set_as_box(1.5, 1.5);

				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(0.0, 23.0);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(shape)), 2.0);
			}
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for SliderCrank2<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::F) => {
					match self
						.m_joint2
						.as_ref()
						.unwrap()
						.borrow_mut()
						.as_derived_mut()
					{
						JointAsDerivedMut::EPrismaticJoint(ref mut m_joint2) => {
							m_joint2.enable_motor(!m_joint2.is_motor_enabled());
							m_joint2
								.get_base()
								.get_body_b()
								.borrow_mut()
								.set_awake(true);
						}
						_ => (),
					}
				}
				Some(VirtualKeyCode::M) => {
					match self
						.m_joint1
						.as_ref()
						.unwrap()
						.borrow_mut()
						.as_derived_mut()
					{
						JointAsDerivedMut::ERevoluteJoint(ref mut m_joint1) => {
							m_joint1.enable_motor(!m_joint1.is_motor_enabled());
							m_joint1
								.get_base()
								.get_body_b()
								.borrow_mut()
								.set_awake(true);
						}
						_ => (),
					}
				}
				_ => (),
			}
		}
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

		let mut base = self.base.borrow_mut();

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			"Keys: (f) toggle friction, (m) toggle motor",
		);
		base.m_text_line += base.m_text_increment;

		match self
			.m_joint1
			.as_ref()
			.unwrap()
			.borrow_mut()
			.as_derived_mut()
		{
			JointAsDerivedMut::ERevoluteJoint(ref mut m_joint1) => {
				let torque: f32 = m_joint1.get_motor_torque(settings.m_hertz);

				base.g_debug_draw.borrow().draw_string(
					ui,
					B2vec2::new(5.0, base.m_text_line as f32),
					&format!("Motor Torque = {0:5.0}", torque),
				);
				base.m_text_line += base.m_text_increment;
			}
			_ => (),
		}
	}
}

//static int testIndex = RegisterTest("Examples", "Slider Crank 2", SliderCrank2::create);
