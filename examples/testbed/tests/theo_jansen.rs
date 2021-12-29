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
use box2d_rs::joints::b2_distance_joint::*;
use box2d_rs::joints::b2_revolute_joint::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct TheoJansen<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_offset: B2vec2,
	m_chassis: Option<BodyPtr<D>>,
	m_wheel: Option<BodyPtr<D>>,
	m_motor_joint: Option<B2jointPtr<D>>,
	m_motor_on: bool,
	m_motor_speed: f32,
}

impl<D: UserDataType> TheoJansen<D> {
	pub fn new<F: Facade>(global_draw: TestBedDebugDrawPtr) -> TestPtr<D, F> {
		let base = Rc::new(RefCell::new(Test::new(global_draw.clone())));
		let result_ptr = Rc::new(RefCell::new(TheoJansen {
			base: base.clone(),
			destruction_listener: Rc::new(RefCell::new(B2testDestructionListenerDefault {
				base: Rc::downgrade(&base),
			})),
			contact_listener: Rc::new(RefCell::new(B2testContactListenerDefault {
				base: Rc::downgrade(&base),
			})),
			m_offset: B2vec2::zero(),
			m_chassis: None,
			m_wheel: None,
			m_motor_joint: None,
			m_motor_on: false,
			m_motor_speed: 0.0,
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
			self_.theo_jansen();
		}

		return result_ptr;
	}

	fn create_leg(&mut self, s: f32, wheel_anchor: B2vec2) {
		let m_world = self.base.borrow().m_world.clone();

		let p1 = B2vec2::new(5.4 * s, -6.1);
		let p2 = B2vec2::new(7.2 * s, -1.2);
		let p3 = B2vec2::new(4.3 * s, -1.9);
		let p4 = B2vec2::new(3.1 * s, 0.8);
		let p5 = B2vec2::new(6.0 * s, 1.5);
		let p6 = B2vec2::new(2.5 * s, 3.7);

		let mut fd1 = B2fixtureDef::default();
		let mut fd2 = B2fixtureDef::default();
		fd1.filter.group_index = -1;
		fd2.filter.group_index = -1;
		fd1.density = 1.0;
		fd2.density = 1.0;

		let mut poly1 = B2polygonShape::default();
		let mut poly2 = B2polygonShape::default();

		if s > 0.0 {
			let mut vertices: [B2vec2; 3] = [p1, p2, p3];
			poly1.set(&vertices);

			vertices[0] = B2vec2::zero();
			vertices[1] = p5 - p4;
			vertices[2] = p6 - p4;
			poly2.set(&vertices);
		} else {
			let mut vertices: [B2vec2; 3] = [p1, p3, p2];
			poly1.set(&vertices);

			vertices[0] = B2vec2::zero();
			vertices[1] = p6 - p4;
			vertices[2] = p5 - p4;
			poly2.set(&vertices);
		}

		fd1.shape = Some(Rc::new(RefCell::new(poly1)));
		fd2.shape = Some(Rc::new(RefCell::new(poly2)));

		let mut bd1 = B2bodyDef::default();
		let mut bd2 = B2bodyDef::default();
		bd1.body_type = B2bodyType::B2DynamicBody;
		bd2.body_type = B2bodyType::B2DynamicBody;
		bd1.position = self.m_offset;
		bd2.position = p4 + self.m_offset;

		bd1.angular_damping = 10.0;
		bd2.angular_damping = 10.0;

		let body1 = B2world::create_body(m_world.clone(), &bd1);
		let body2 = B2world::create_body(m_world.clone(), &bd2);

		B2body::create_fixture(body1.clone(), &fd1);
		B2body::create_fixture(body2.clone(), &fd2);

		{
			let mut jd = B2distanceJointDef::default();

			// Using a soft distance constraint can reduce some jitter.
			// It also makes the structure seem a bit more fluid by
			// acting like a suspension system.
			let damping_ratio: f32 = 0.5;
			let frequency_hz: f32 = 10.0;

			jd.initialize(
				body1.clone(),
				body2.clone(),
				p2 + self.m_offset,
				p5 + self.m_offset,
			);
			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);
			m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone()));

			jd.initialize(
				body1.clone(),
				body2.clone(),
				p3 + self.m_offset,
				p4 + self.m_offset,
			);
			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);
			m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone()));

			jd.initialize(
				body1.clone(),
				self.m_wheel.clone().unwrap(),
				p3 + self.m_offset,
				wheel_anchor + self.m_offset,
			);
			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);
			m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone()));

			jd.initialize(
				body2.clone(),
				self.m_wheel.clone().unwrap(),
				p6 + self.m_offset,
				wheel_anchor + self.m_offset,
			);
			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);
			m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone()));
		}

		{
			let mut jd = B2revoluteJointDef::default();
			jd.initialize(
				body2.clone(),
				self.m_chassis.clone().unwrap(),
				p4 + self.m_offset,
			);
			m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::RevoluteJoint(jd));
		}
	}

	fn theo_jansen(&mut self) {
		let m_world = self.base.borrow().m_world.clone();

		self.m_offset.set(0.0, 8.0);
		self.m_motor_speed = 2.0;
		self.m_motor_on = true;
		let pivot = B2vec2::new(0.0, 0.8);

		// Ground
		{
			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-50.0, 0.0), B2vec2::new(50.0, 0.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_two_sided(B2vec2::new(-50.0, 0.0), B2vec2::new(-50.0, 10.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_two_sided(B2vec2::new(50.0, 0.0), B2vec2::new(50.0, 10.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		// Balls
		for i in 0..40 {
			let mut shape = B2circleShape::default();
			shape.base.m_radius = 0.25;

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(-40.0 + 2.0 * i as f32, 0.5);

			let body = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 1.0);
		}

		// Chassis
		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(2.5, 1.0);

			let mut sd = B2fixtureDef::default();
			sd.density = 1.0;
			sd.shape = Some(Rc::new(RefCell::new(shape)));
			sd.filter.group_index = -1;
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position = pivot + self.m_offset;
			self.m_chassis = Some(B2world::create_body(m_world.clone(), &bd));
			B2body::create_fixture(self.m_chassis.clone().unwrap(), &sd);
		}

		{
			let mut shape = B2circleShape::default();
			shape.base.m_radius = 1.6;

			let mut sd = B2fixtureDef::default();
			sd.density = 1.0;
			sd.shape = Some(Rc::new(RefCell::new(shape)));
			sd.filter.group_index = -1;
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position = pivot + self.m_offset;
			self.m_wheel = Some(B2world::create_body(m_world.clone(), &bd));
			B2body::create_fixture(self.m_wheel.clone().unwrap(), &sd);
		}

		{
			let mut jd = B2revoluteJointDef::default();
			jd.initialize(
				self.m_wheel.clone().unwrap(),
				self.m_chassis.clone().unwrap(),
				pivot + self.m_offset,
			);
			jd.base.collide_connected = false;
			jd.motor_speed = self.m_motor_speed;
			jd.max_motor_torque = 400.0;
			jd.enable_motor = self.m_motor_on;
			self.m_motor_joint = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(jd)),
			);
		}

		let wheel_anchor = pivot + B2vec2::new(0.0, -0.8);

		self.create_leg(-1.0, wheel_anchor);
		self.create_leg(1.0, wheel_anchor);

		let m_wheel = self.m_wheel.clone().unwrap();
		let pos = m_wheel.borrow().get_position();
		m_wheel
			.borrow_mut()
			.set_transform(pos, 120.0 * B2_PI / 180.0);
		self.create_leg(-1.0, wheel_anchor);
		self.create_leg(1.0, wheel_anchor);

		let pos = m_wheel.borrow().get_position();
		m_wheel
			.borrow_mut()
			.set_transform(pos, -120.0 * B2_PI / 180.0);
		self.create_leg(-1.0, wheel_anchor);
		self.create_leg(1.0, wheel_anchor);
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for TheoJansen<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		match self
			.m_motor_joint
			.as_ref()
			.unwrap()
			.borrow_mut()
			.as_derived_mut()
		{
			JointAsDerivedMut::ERevoluteJoint(ref mut m_motor_joint) => {
				if key.state == ElementState::Pressed {
					match key.virtual_keycode {
						Some(VirtualKeyCode::A) => {
							m_motor_joint.set_motor_speed(-self.m_motor_speed);
						}
						Some(VirtualKeyCode::S) => {
							m_motor_joint.set_motor_speed(0.0);
						}
						Some(VirtualKeyCode::D) => {
							m_motor_joint.set_motor_speed(self.m_motor_speed);
						}
						Some(VirtualKeyCode::M) => {
							m_motor_joint.enable_motor(!m_motor_joint.is_motor_enabled());
						}
						_ => (),
					}
				}
			}
			_ => (),
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
			let mut base = self.base.borrow_mut();

			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Keys: left = a, brake = s, right = d, toggle motor = m",
			);
			base.m_text_line += base.m_text_increment;
		}

		Test::step(self.base.clone(), ui, display, target, settings, *camera);
	}
}

//static int testIndex = RegisterTest("Examples", "Theo Jansen", theo_jansen::Create);
