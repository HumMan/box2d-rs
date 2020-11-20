use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::joints::b2_wheel_joint::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;

use imgui::im_str;
use imgui::sys;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

// Test the wheel joint with motor, spring, and limit options.
pub(crate) struct WheelJoint<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_joint: Option<B2jointPtr<D>>,
	m_motor_speed: f32,
	m_enable_motor: bool,
	m_enable_limit: bool,
}

impl<D: UserDataType> WheelJoint<D> {
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
			m_motor_speed: 0.0,
			m_enable_motor: false,
			m_enable_limit: false,
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
		let ground: BodyPtr<D>;
		let m_world = self.base.borrow().m_world.clone();
		{
			let bd = B2bodyDef::default();
			ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		self.m_enable_limit = true;
		self.m_enable_motor = false;
		self.m_motor_speed = 10.0;

		{
			let mut shape = B2circleShape::default();
			shape.base.m_radius = 2.0;

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(0.0, 10.0);
			bd.allow_sleep = false;
			let body = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 5.0);

			let mut jd = B2wheelJointDef::default();

			// Horizontal
			jd.initialize(
				ground.clone(),
				body.clone(),
				bd.position,
				B2vec2::new(0.0, 1.0),
			);

			jd.motor_speed = self.m_motor_speed;
			jd.max_motor_torque = 10000.0;
			jd.enable_motor = self.m_enable_motor;
			jd.lower_translation = -3.0;
			jd.upper_translation = 3.0;
			jd.enable_limit = self.m_enable_limit;

			let hertz: f32 = 1.0;
			let damping_ratio: f32 = 0.7;
			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				hertz,
				damping_ratio,
				ground,
				body,
			);

			self.m_joint = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::WheelJoint(jd)),
			);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for WheelJoint<D> {
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
				match self.m_joint.as_ref().unwrap().borrow_mut().as_derived_mut() {
					JointAsDerivedMut::EWheelJoint(ref mut m_joint) => {
						if sys::igCheckbox(im_str!("Limit").as_ptr(), &mut self.m_enable_limit) {
							m_joint.enable_limit(self.m_enable_limit);
						}
						if sys::igCheckbox(im_str!("Motor").as_ptr(), &mut self.m_enable_motor) {
							m_joint.enable_motor(self.m_enable_motor);
						}
						if sys::igSliderFloat(
							im_str!("Hertz").as_ptr(),
							&mut self.m_motor_speed,
							-100.0,
							100.0,
							im_str!("%.0f").as_ptr(),
							1.0,
						) {
							m_joint.set_motor_speed(self.m_motor_speed);
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

		let mut torque: f32 = 0.0;
		match self.m_joint.as_ref().unwrap().borrow_mut().as_derived_mut() {
			JointAsDerivedMut::EWheelJoint(ref mut m_spring1) => {
				torque = m_spring1.get_motor_torque(settings.m_hertz);
			}
			_ => (),
		}
		let mut base = self.base.borrow_mut();
		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			&format!("Motor Torque = {0:4}", torque,),
		);
		base.m_text_line += base.m_text_increment;
	}
}
