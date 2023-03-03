use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_common::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_chain_shape::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;
use box2d_rs::joints::b2_revolute_joint::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

/// This tests bullet collision and provides an example of a gameplay scenario.
/// This also uses a loop shape.
pub(crate) struct Pinball<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_left_joint: Option<B2jointPtr<D>>,
	m_right_joint: Option<B2jointPtr<D>>,
	m_ball: Option<BodyPtr<D>>,
	m_button: bool,
}

impl<D: UserDataType> Pinball<D> {
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

			m_left_joint: None,
			m_right_joint: None,
			m_ball: None,
			m_button: false,
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
		// Ground body
		let ground: BodyPtr<D>;
		{
			let bd = B2bodyDef::default();
			ground = B2world::create_body(m_world.clone(), &bd);

			let vs: [B2vec2; 5] = [
				B2vec2::new(-8.0, 6.0),
				B2vec2::new(-8.0, 20.0),
				B2vec2::new(8.0, 20.0),
				B2vec2::new(8.0, 6.0),
				B2vec2::new(0.0, -2.0),
			];

			let mut loop_shape = B2chainShape::default();
			loop_shape.create_loop(&vs);
			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(loop_shape)));
			fd.density = 0.0;
			B2body::create_fixture(ground.clone(), &fd);
		}

		// Flippers
		{
			let p1 = B2vec2::new(-2.0, 0.0);
			let p2 = B2vec2::new(2.0, 0.0);

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;

			bd.position = p1;
			let left_flipper = B2world::create_body(m_world.clone(), &bd);

			bd.position = p2;
			let right_flipper = B2world::create_body(m_world.clone(), &bd);

			let mut box_shape = B2polygonShape::default();
			box_shape.set_as_box(1.75, 0.1);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(box_shape)));
			fd.density = 1.0;

			B2body::create_fixture(left_flipper.clone(), &fd);
			B2body::create_fixture(right_flipper.clone(), &fd);

			let mut jd = B2revoluteJointDef::default();
			jd.base.body_a = Some(ground);
			jd.local_anchor_b.set_zero();
			jd.enable_motor = true;
			jd.max_motor_torque = 1000.0;
			jd.enable_limit = true;

			jd.motor_speed = 0.0;
			jd.local_anchor_a = p1;
			jd.base.body_b = Some(left_flipper);
			jd.lower_angle = -30.0 * B2_PI / 180.0;
			jd.upper_angle = 5.0 * B2_PI / 180.0;
			self.m_left_joint = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(jd.clone())),
			);

			jd.motor_speed = 0.0;
			jd.local_anchor_a = p2;
			jd.base.body_b = Some(right_flipper);
			jd.lower_angle = -5.0 * B2_PI / 180.0;
			jd.upper_angle = 30.0 * B2_PI / 180.0;
			self.m_right_joint = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(jd)),
			);
		}

		// Circle character
		{
			let mut bd = B2bodyDef::default();
			bd.position.set(1.0, 15.0);
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.bullet = true;

			let m_ball = B2world::create_body(m_world.clone(), &bd);
			self.m_ball = Some(m_ball.clone());

			let mut shape = B2circleShape::default();
			shape.base.m_radius = 0.2;

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 1.0;
			B2body::create_fixture(m_ball.clone(), &fd);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Pinball<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		match key.virtual_keycode {
			Some(VirtualKeyCode::A) => {
				if key.state == ElementState::Pressed {
					self.m_button = true;
				}

				if key.state == ElementState::Released {
					self.m_button = false;
				}
			}
			_ => (),
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
		match self
			.m_left_joint
			.as_ref()
			.unwrap()
			.borrow_mut()
			.as_derived_mut()
		{
			JointAsDerivedMut::ERevoluteJoint(ref mut m_left_joint) => {
				match self
					.m_right_joint
					.as_ref()
					.unwrap()
					.borrow_mut()
					.as_derived_mut()
				{
					JointAsDerivedMut::ERevoluteJoint(ref mut m_right_joint) => {
						if self.m_button {
							m_left_joint.set_motor_speed(20.0);
							m_right_joint.set_motor_speed(-20.0);
						} else {
							m_left_joint.set_motor_speed(-10.0);
							m_right_joint.set_motor_speed(10.0);
						}
					}
					_ => (),
				}
			}
			_ => (),
		}

		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		let mut base = self.base.borrow_mut();

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			"Press 'a' to control the flippers",
		);
		base.m_text_line += base.m_text_increment;
	}
}

//static int testIndex = RegisterTest("Examples", "Pinball", Pinball::create);
