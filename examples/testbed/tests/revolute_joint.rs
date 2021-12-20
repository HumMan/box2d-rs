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

use imgui::im_str;
use imgui::sys;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct RevoluteJoint<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_ball: Option<BodyPtr<D>>,
	m_joint1: Option<B2jointPtr<D>>,
	m_joint2: Option<B2jointPtr<D>>,
	m_motor_speed: f32,
	m_enable_motor: bool,
	m_enable_limit: bool,
}

impl<D: UserDataType> RevoluteJoint<D> {
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

			m_ball: None,
			m_joint1: None,
			m_joint2: None,
			m_enable_limit: true,
			m_enable_motor: false,
			m_motor_speed: 1.0,
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

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			//fd.filter.categoryBits = 2;

			B2body::create_fixture(ground.clone(), &fd);
		}

		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box_angle(0.25, 3.0, B2vec2::new(0.0, 3.0), 0.0);

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(-10.0, 20.0);
			let body = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 5.0);

			let mut jd = B2revoluteJointDef::default();
			jd.initialize(ground.clone(), body, B2vec2::new(-10.0, 20.5));
			jd.motor_speed = self.m_motor_speed;
			jd.max_motor_torque = 10000.0;
			jd.enable_motor = self.m_enable_motor;
			jd.lower_angle = -0.25 * B2_PI;
			jd.upper_angle = 0.5 * B2_PI;
			jd.enable_limit = self.m_enable_limit;

			self.m_joint1 = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(jd)),
			);
		}

		{
			let mut circle_shape = B2circleShape::default();
			circle_shape.base.m_radius = 2.0;

			let mut circle_bd = B2bodyDef::default();
			circle_bd.body_type = B2bodyType::B2DynamicBody;
			circle_bd.position.set(5.0, 30.0);

			let mut fd = B2fixtureDef::default();
			fd.density = 5.0;
			fd.filter.mask_bits = 1;
			fd.shape = Some(Rc::new(RefCell::new(circle_shape)));

			let m_ball = B2world::create_body(m_world.clone(), &circle_bd);
			self.m_ball = Some(m_ball.clone());
			B2body::create_fixture(m_ball.clone(), &fd);

			let mut polygon_shape = B2polygonShape::default();
			polygon_shape.set_as_box_angle(10.0, 0.5, B2vec2::new(-10.0, 0.0), 0.0);

			let mut polygon_bd = B2bodyDef::default();
			polygon_bd.position.set(20.0, 10.0);
			polygon_bd.body_type = B2bodyType::B2DynamicBody;
			polygon_bd.bullet = true;
			let polygon_body = B2world::create_body(m_world.clone(), &polygon_bd);
			B2body::create_fixture_by_shape(
				polygon_body.clone(),
				Rc::new(RefCell::new(polygon_shape)),
				2.0,
			);

			let mut jd = B2revoluteJointDef::default();
			jd.initialize(ground, polygon_body, B2vec2::new(19.0, 10.0));
			jd.lower_angle = -0.25 * B2_PI;
			jd.upper_angle = 0.0 * B2_PI;
			jd.enable_limit = true;
			jd.enable_motor = true;
			jd.motor_speed = 0.0;
			jd.max_motor_torque = 10000.0;

			self.m_joint2 = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(jd)),
			);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for RevoluteJoint<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn update_ui(&mut self, ui: &imgui::Ui<'_>) {
		imgui::Window::new(im_str!("Joint Controls"))
			.flags(
				imgui::WindowFlags::NO_MOVE
					| imgui::WindowFlags::NO_RESIZE
					| imgui::WindowFlags::NO_COLLAPSE,
			)
			.position([10.0, 100.0], imgui::Condition::Always)
			.size([200.0, 100.0], imgui::Condition::Always)
			.build(&ui, || unsafe {
				match self
					.m_joint1
					.as_ref()
					.unwrap()
					.borrow_mut()
					.as_derived_mut()
				{
					JointAsDerivedMut::ERevoluteJoint(ref mut m_joint1) => {
						if sys::igCheckbox(im_str!("Limit").as_ptr(), &mut self.m_enable_limit) {
							m_joint1.enable_limit(self.m_enable_limit);
						}

						if sys::igCheckbox(im_str!("Motor").as_ptr(), &mut self.m_enable_motor) {
							m_joint1.enable_motor(self.m_enable_motor);
						}

						if sys::igSliderFloat(
							im_str!("Speed").as_ptr(),
							&mut self.m_motor_speed,
							-20.0,
							20.0,
							im_str!("%.0f").as_ptr(),
							1.0,
						) {
							m_joint1.set_motor_speed(self.m_motor_speed);
						}
					}
					_ => (),
				}
			});
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

		let mut base = self.base.borrow_mut();

		match self
			.m_joint1
			.as_ref()
			.unwrap()
			.borrow_mut()
			.as_derived_mut()
		{
			JointAsDerivedMut::ERevoluteJoint(ref mut m_joint1) => {
				let torque1: f32 = m_joint1.get_motor_torque(settings.m_hertz);

				base.g_debug_draw.borrow().draw_string(
					ui,
					B2vec2::new(5.0, base.m_text_line as f32),
					&format!("Motor Torque 1 = {0:4.0}", torque1),
				);
				base.m_text_line += base.m_text_increment;
			}
			_ => (),
		}

		match self
			.m_joint2
			.as_ref()
			.unwrap()
			.borrow_mut()
			.as_derived_mut()
		{
			JointAsDerivedMut::ERevoluteJoint(ref mut m_joint2) => {
				let torque1: f32 = m_joint2.get_motor_torque(settings.m_hertz);

				base.g_debug_draw.borrow().draw_string(
					ui,
					B2vec2::new(5.0, base.m_text_line as f32),
					&format!("Motor Torque 2 = {0:4.0}", torque1),
				);
				base.m_text_line += base.m_text_increment;
			}
			_ => (),
		}
	}
}

//static int testIndex = RegisterTest("Joints", "Revolute", RevoluteJoint::Create);
