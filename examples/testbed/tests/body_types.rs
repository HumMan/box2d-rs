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
use box2d_rs::joints::b2_prismatic_joint::*;
use box2d_rs::joints::b2_revolute_joint::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct BodyTypes<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_attachment: Option<BodyPtr<D>>,
	m_platform: Option<BodyPtr<D>>,
	m_speed: f32,
}

impl<D: UserDataType> BodyTypes<D> {
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
			m_attachment: None,
			m_platform: None,
			m_speed: 0.0,
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

		// Define attachment
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(0.0, 3.0);
			self.m_attachment = Some(B2world::create_body(m_world.clone(), &bd));

			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.5, 2.0);
			B2body::create_fixture_by_shape(
				self.m_attachment.clone().unwrap(),
				Rc::new(RefCell::new(shape)),
				2.0,
			);
		}

		// Define platform
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(-4.0, 5.0);
			self.m_platform = Some(B2world::create_body(m_world.clone(), &bd));

			let mut shape = B2polygonShape::default();
			shape.set_as_box_angle(0.5, 4.0, B2vec2::new(4.0, 0.0), 0.5 * B2_PI);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.friction = 0.6;
			fd.density = 2.0;
			B2body::create_fixture(self.m_platform.as_ref().unwrap().clone(), &fd);

			let mut rjd = B2revoluteJointDef::default();
			rjd.initialize(
				self.m_attachment.clone().unwrap(),
				self.m_platform.clone().unwrap(),
				B2vec2::new(0.0, 5.0),
			);
			rjd.max_motor_torque = 50.0;
			rjd.enable_motor = true;
			m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::RevoluteJoint(rjd));

			let mut pjd = B2prismaticJointDef::default();
			pjd.initialize(
				ground,
				self.m_platform.clone().unwrap(),
				B2vec2::new(0.0, 5.0),
				B2vec2::new(1.0, 0.0),
			);

			pjd.max_motor_force = 1000.0;
			pjd.enable_motor = true;
			pjd.lower_translation = -10.0;
			pjd.upper_translation = 10.0;
			pjd.enable_limit = true;

			m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::PrismaticJoint(pjd));

			self.m_speed = 3.0;
		}

		// create a payload
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(0.0, 8.0);
			let body = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.75, 0.75);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.friction = 0.6;
			fd.density = 2.0;

			B2body::create_fixture(body, &fd);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for BodyTypes<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::D) => {
					B2body::set_type(self.m_platform.clone().unwrap(), B2bodyType::B2DynamicBody);
				}
				Some(VirtualKeyCode::S) => {
					B2body::set_type(self.m_platform.clone().unwrap(), B2bodyType::B2StaticBody);
				}
				Some(VirtualKeyCode::K) => {
					B2body::set_type(
						self.m_platform.clone().unwrap(),
						B2bodyType::B2KinematicBody,
					);
					let mut m_platform = self.m_platform.as_ref().unwrap().borrow_mut();
					m_platform.set_linear_velocity(B2vec2::new(-self.m_speed, 0.0));
					m_platform.set_angular_velocity(0.0);
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
		{
			let mut m_platform = self.m_platform.as_ref().unwrap().borrow_mut();
			// Drive the kinematic body.
			if m_platform.get_type() == B2bodyType::B2KinematicBody {
				let p: B2vec2 = m_platform.get_transform().p;
				let mut v: B2vec2 = m_platform.get_linear_velocity();

				if (p.x < -10.0 && v.x < 0.0) || (p.x > 10.0 && v.x > 0.0) {
					v.x = -v.x;
					m_platform.set_linear_velocity(v);
				}
			}
		}

		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		{
			let mut base = self.base.borrow_mut();
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Keys: (d) dynamic, (s) static, (k) kinematic",
			);
			base.m_text_line += base.m_text_increment
		}
	}
}
