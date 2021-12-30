use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_draw::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::joints::b2_motor_joint::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

/// This test shows how to use a motor joint. A motor joint
/// can be used to animate a dynamic body. With finite motor forces
/// the body can be blocked by collision with other bodies.
pub(crate) struct MotorJoint<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
	m_joint: Option<B2jointPtr<D>>,
	m_time: f32,
	m_go: bool,
}

impl<D: UserDataType> MotorJoint<D> {
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
			m_time: 0.0,
			m_go: false,
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

		let ground: BodyPtr<D>;
		{
			let bd = B2bodyDef::default();
			ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-20.0, 0.0), B2vec2::new(20.0, 0.0));

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));

			B2body::create_fixture(ground.clone(), &fd);
		}

		// Define motorized body
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(0.0, 8.0);
			let body = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2polygonShape::default();
			shape.set_as_box(2.0, 0.5);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.friction = 0.6;
			fd.density = 2.0;
			B2body::create_fixture(body.clone(), &fd);

			let mut mjd = B2motorJointDef::default();
			mjd.initialize(ground, body);
			mjd.max_force = 1000.0;
			mjd.max_torque = 1000.0;
			self.m_joint = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::MotorJoint(mjd)),
			);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for MotorJoint<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::S) => {
					self.m_go = !self.m_go;
				}
				_ => (),
			}
		}
	}
	fn step(
		&mut self,
		ui: &imgui::Ui<'_>,
		display: &F,
		target: &mut glium::Frame,
		settings: &mut Settings,
		camera: &mut Camera,
	) {
		if self.m_go && settings.m_hertz > 0.0 {
			self.m_time += 1.0 / settings.m_hertz;
		}

		let linear_offset = B2vec2 {
			x: 6.0 * f32::sin(2.0 * self.m_time),
			y: 8.0 + 4.0 * f32::sin(1.0 * self.m_time),
		};
		let angular_offset: f32 = 4.0 * self.m_time;

		match self.m_joint.as_ref().unwrap().borrow_mut().as_derived_mut() {
			JointAsDerivedMut::EMotorJoint(ref mut m_joint) => {
				m_joint.set_linear_offset(linear_offset);
				m_joint.set_angular_offset(angular_offset);
			}
			_ => (),
		}

		{
			let base = self.base.borrow_mut();
			base.g_debug_draw.borrow_mut().draw_point(
				linear_offset,
				4.0,
				B2color::new(0.9, 0.9, 0.9),
			);
		}

		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		{
			let mut base = self.base.borrow_mut();
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Keys: (s) pause",
			);
			base.m_text_line += 15;
		}
	}
}

//static int testIndex = RegisterTest("Joints", "Motor Joint", MotorJoint::create);
