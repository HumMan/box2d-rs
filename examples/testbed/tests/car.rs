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
use box2d_rs::joints::b2_wheel_joint::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

// This is a fun demo that shows off the wheel joint
pub(crate) struct Car<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_car: Option<BodyPtr<D>>,
	m_wheel1: Option<BodyPtr<D>>,
	m_wheel2: Option<BodyPtr<D>>,

	m_speed: f32,
	m_spring1: Option<B2jointPtr<D>>,
	m_spring2: Option<B2jointPtr<D>>,
}

impl<D: UserDataType> Car<D> {
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
			m_car: None,
			m_wheel1: None,
			m_wheel2: None,
			m_speed: 0.0,
			m_spring1: None,
			m_spring2: None,
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

		self.m_speed = 50.0;

		let ground: BodyPtr<D>;
		{
			let bd = B2bodyDef::default();
			ground = B2world::create_body(m_world.clone(), &bd);

			let shape = Rc::new(RefCell::new(B2edgeShape::default()));

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(shape.clone());
			fd.density = 0.0;
			fd.friction = 0.6;

			shape
				.borrow_mut()
				.set_two_sided(B2vec2::new(-20.0, 0.0), B2vec2::new(20.0, 0.0));
			B2body::create_fixture(ground.clone(), &fd);

			let hs: [f32; 10] = [0.25, 1.0, 4.0, 0.0, 0.0, -1.0, -2.0, -2.0, -1.25, 0.0];

			let mut x: f32 = 20.0;
			let mut y1: f32 = 0.0;
			let dx: f32 = 5.0;

			for i in 0..10 {
				let y2: f32 = hs[i];
				shape
					.borrow_mut()
					.set_two_sided(B2vec2::new(x, y1), B2vec2::new(x + dx, y2));
				B2body::create_fixture(ground.clone(), &fd);
				y1 = y2;
				x += dx;
			}

			for i in 0..10 {
				let y2: f32 = hs[i];
				shape
					.borrow_mut()
					.set_two_sided(B2vec2::new(x, y1), B2vec2::new(x + dx, y2));
				B2body::create_fixture(ground.clone(), &fd);
				y1 = y2;
				x += dx;
			}

			shape
				.borrow_mut()
				.set_two_sided(B2vec2::new(x, 0.0), B2vec2::new(x + 40.0, 0.0));
			B2body::create_fixture(ground.clone(), &fd);

			x += 80.0;
			shape
				.borrow_mut()
				.set_two_sided(B2vec2::new(x, 0.0), B2vec2::new(x + 40.0, 0.0));
			B2body::create_fixture(ground.clone(), &fd);

			x += 40.0;
			shape
				.borrow_mut()
				.set_two_sided(B2vec2::new(x, 0.0), B2vec2::new(x + 10.0, 5.0));
			B2body::create_fixture(ground.clone(), &fd);

			x += 20.0;
			shape
				.borrow_mut()
				.set_two_sided(B2vec2::new(x, 0.0), B2vec2::new(x + 40.0, 0.0));
			B2body::create_fixture(ground.clone(), &fd);

			x += 40.0;
			shape
				.borrow_mut()
				.set_two_sided(B2vec2::new(x, 0.0), B2vec2::new(x, 20.0));
			B2body::create_fixture(ground.clone(), &fd);
		}

		// Teeter
		{
			let mut bd = B2bodyDef::default();
			bd.position.set(140.0, 1.0);
			bd.body_type = B2bodyType::B2DynamicBody;
			let body = B2world::create_body(m_world.clone(), &bd);

			let mut box_shape = B2polygonShape::default();
			box_shape.set_as_box(10.0, 0.25);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(box_shape)), 1.0);

			let mut jd = B2revoluteJointDef::default();
			jd.initialize(ground.clone(), body.clone(), body.borrow().get_position());
			jd.lower_angle = -8.0 * B2_PI / 180.0;
			jd.upper_angle = 8.0 * B2_PI / 180.0;
			jd.enable_limit = true;
			m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::RevoluteJoint(jd));

			body.borrow_mut().apply_angular_impulse(100.0, true);
		}

		// Bridge
		{
			let n: isize = 20;
			let mut shape = B2polygonShape::default();
			shape.set_as_box(1.0, 0.125);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 1.0;
			fd.friction = 0.6;

			let mut prev_body: BodyPtr<D> = ground.clone();
			for i in 0..n {
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(161.0 + 2.0 * i as f32, -0.125);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture(body.clone(), &fd);

				let anchor = B2vec2::new(160.0 + 2.0 * i as f32, -0.125);
				let mut jd = B2revoluteJointDef::default();
				jd.initialize(prev_body, body.clone(), anchor);
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(jd));

				prev_body = body;
			}

			let anchor = B2vec2::new(160.0 + 2.0 * n as f32, -0.125);
			let mut jd = B2revoluteJointDef::default();
			jd.initialize(prev_body, ground, anchor);
			m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::RevoluteJoint(jd));
		}

		// Boxes
		{
			let mut box_shape = B2polygonShape::default();
			box_shape.set_as_box(0.5, 0.5);

			let mut body: BodyPtr<D>;
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;

			bd.position.set(230.0, 0.5);
			body = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(box_shape)), 0.5);

			bd.position.set(230.0, 1.5);
			body = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(box_shape)), 0.5);

			bd.position.set(230.0, 2.5);
			body = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(box_shape)), 0.5);

			bd.position.set(230.0, 3.5);
			body = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(box_shape)), 0.5);

			bd.position.set(230.0, 4.5);
			body = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(box_shape)), 0.5);
		}

		// Car
		{
			let mut chassis = B2polygonShape::default();
			let vertices: [B2vec2; 6] = [
				B2vec2::new(-1.5, -0.5),
				B2vec2::new(1.5, -0.5),
				B2vec2::new(1.5, 0.0),
				B2vec2::new(0.0, 0.9),
				B2vec2::new(-1.15, 0.9),
				B2vec2::new(-1.5, 0.2),
			];

			chassis.set(&vertices);

			let mut circle = B2circleShape::default();
			circle.base.m_radius = 0.4;

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(0.0, 1.0);
			let m_car = B2world::create_body(m_world.clone(), &bd);
			self.m_car = Some(m_car.clone());
			B2body::create_fixture_by_shape(m_car.clone(), Rc::new(RefCell::new(chassis)), 1.0);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(circle)));
			fd.density = 1.0;
			fd.friction = 0.9;

			bd.position.set(-1.0, 0.35);
			let m_wheel1 = B2world::create_body(m_world.clone(), &bd);
			self.m_wheel1 = Some(m_wheel1.clone());
			B2body::create_fixture(m_wheel1.clone(), &fd);

			bd.position.set(1.0, 0.4);
			let m_wheel2 = B2world::create_body(m_world.clone(), &bd);
			self.m_wheel2 = Some(m_wheel2.clone());
			B2body::create_fixture(m_wheel2.clone(), &fd);

			let mut jd = B2wheelJointDef::default();
			let axis = B2vec2::new(0.0, 1.0);

			let mass1: f32 = m_wheel1.borrow().get_mass();
			let mass2: f32 = m_wheel2.borrow().get_mass();

			let hertz: f32 = 4.0;
			let damping_ratio: f32 = 0.7;
			let omega: f32 = 2.0 * B2_PI * hertz;

			jd.initialize(
				m_car.clone(),
				m_wheel1.clone(),
				m_wheel1.borrow().get_position(),
				axis,
			);
			jd.motor_speed = 0.0;
			jd.max_motor_torque = 20.0;
			jd.enable_motor = true;
			jd.stiffness = mass1 * omega * omega;
			jd.damping = 2.0 * mass1 * damping_ratio * omega;
			jd.lower_translation = -0.25;
			jd.upper_translation = 0.25;
			jd.enable_limit = true;
			self.m_spring1 = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::WheelJoint(jd)),
			);

			let mut jd = B2wheelJointDef::default();
			jd.initialize(
				m_car.clone(),
				m_wheel2.clone(),
				m_wheel2.borrow().get_position(),
				axis,
			);
			jd.motor_speed = 0.0;
			jd.max_motor_torque = 10.0;
			jd.enable_motor = false;
			jd.stiffness = mass2 * omega * omega;
			jd.damping = 2.0 * mass2 * damping_ratio * omega;
			jd.lower_translation = -0.25;
			jd.upper_translation = 0.25;
			jd.enable_limit = true;
			self.m_spring2 = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::WheelJoint(jd)),
			);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Car<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		match self
			.m_spring1
			.as_ref()
			.unwrap()
			.borrow_mut()
			.as_derived_mut()
		{
			JointAsDerivedMut::EWheelJoint(ref mut m_spring1) => {
				if key.state == ElementState::Pressed {
					match key.virtual_keycode {
						Some(VirtualKeyCode::A) => {
							m_spring1.set_motor_speed(self.m_speed);
						}
						Some(VirtualKeyCode::S) => {
							m_spring1.set_motor_speed(0.0);
						}
						Some(VirtualKeyCode::D) => {
							m_spring1.set_motor_speed(-self.m_speed);
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
				"Keys: left = a, brake = s, right = d, hz down = q, hz up = e",
			);
			base.m_text_line += base.m_text_increment;

			camera.m_center.x = self.m_car.as_ref().unwrap().borrow().get_position().x;
		}

		Test::step(self.base.clone(), ui, display, target, settings, *camera);
	}
}
