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
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;
use box2d_rs::joints::b2_friction_joint::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

// This test shows how to apply forces and torques to a body.
// It also shows how to use the friction joint that can be useful
// for overhead games.
pub(crate) struct ApplyForce<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
	m_body: Option<BodyPtr<D>>,
	key_w: bool,
	key_a: bool,
	key_d: bool,
}

impl<D: UserDataType> ApplyForce<D> {
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
			m_body: None,
			key_w: false,
			key_a: false,
			key_d: false,
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
		let world = self.base.borrow().m_world.clone();
		world.borrow_mut().set_gravity(B2vec2::zero());

		const K_RESTITUTION: f32 = 0.4;

		let ground;
		{
			let mut bd = B2bodyDef::default();
			bd.position.set(0.0, 20.0);
			ground = B2world::create_body(world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			let mut sd = B2fixtureDef::default();
			sd.shape = Some(Rc::new(RefCell::new(shape)));
			sd.density = 0.0;
			sd.restitution = K_RESTITUTION;

			// Left vertical
			shape.set_two_sided(B2vec2::new(-20.0, -20.0), B2vec2::new(-20.0, 20.0));
			sd.shape = Some(Rc::new(RefCell::new(shape)));
			B2body::create_fixture(ground.clone(), &sd);

			// Right vertical
			shape.set_two_sided(B2vec2::new(20.0, -20.0), B2vec2::new(20.0, 20.0));
			sd.shape = Some(Rc::new(RefCell::new(shape)));
			B2body::create_fixture(ground.clone(), &sd);

			// Top horizontal
			shape.set_two_sided(B2vec2::new(-20.0, 20.0), B2vec2::new(20.0, 20.0));
			sd.shape = Some(Rc::new(RefCell::new(shape)));
			B2body::create_fixture(ground.clone(), &sd);

			// Bottom horizontal
			shape.set_two_sided(B2vec2::new(-20.0, -20.0), B2vec2::new(20.0, -20.0));
			sd.shape = Some(Rc::new(RefCell::new(shape)));
			B2body::create_fixture(ground.clone(), &sd);
		}

		{
			let mut xf1 = B2Transform::default();
			xf1.q.set(0.3524 * B2_PI);
			xf1.p = xf1.q.get_xaxis();

			let vertices: [B2vec2; 3] = [
				b2_mul_transform_by_vec2(xf1, B2vec2::new(-1.0, 0.0)),
				b2_mul_transform_by_vec2(xf1, B2vec2::new(1.0, 0.0)),
				b2_mul_transform_by_vec2(xf1, B2vec2::new(0.0, 0.5)),
			];
			let mut poly1 = B2polygonShape::default();
			poly1.set(&vertices);

			let mut sd1 = B2fixtureDef::default();
			sd1.shape = Some(Rc::new(RefCell::new(poly1)));
			sd1.density = 2.0;

			let mut xf2 = B2Transform::default();
			xf2.q.set(-0.3524 * B2_PI);
			xf2.p = -xf2.q.get_xaxis();

			let vertices: [B2vec2; 3] = [
				b2_mul_transform_by_vec2(xf2, B2vec2::new(-1.0, 0.0)),
				b2_mul_transform_by_vec2(xf2, B2vec2::new(1.0, 0.0)),
				b2_mul_transform_by_vec2(xf2, B2vec2::new(0.0, 0.5)),
			];
			let mut poly2 = B2polygonShape::default();
			poly2.set(&vertices);

			let mut sd2 = B2fixtureDef::default();
			sd2.shape = Some(Rc::new(RefCell::new(poly2)));
			sd2.density = 2.0;

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;

			bd.position.set(0.0, 3.0);
			bd.angle = B2_PI;
			bd.allow_sleep = false;

			self.m_body = Some(B2world::create_body(world.clone(), &bd));
			B2body::create_fixture(self.m_body.as_ref().unwrap().clone(), &sd1);
			B2body::create_fixture(self.m_body.as_ref().unwrap().clone(), &sd2);

			let gravity: f32 = 10.0;
			let i: f32 = self.m_body.as_ref().unwrap().borrow().get_inertia();
			let mass: f32 = self.m_body.as_ref().unwrap().borrow().get_mass();

			// Compute an effective radius that can be used to
			// set the max torque for a friction joint
			// For a circle: i = 0.5 * m * r * r ==> r = sqrt(2 * i / m)
			let radius: f32 = b2_sqrt(2.0 * i / mass);

			let mut jd = B2frictionJointDef::<D>::default();
			jd.base.body_a = Some(ground.clone());
			jd.base.body_b = self.m_body.clone();
			jd.local_anchor_a.set_zero();
			jd.local_anchor_b = self.m_body.as_ref().unwrap().borrow().get_local_center();
			jd.base.collide_connected = true;
			jd.max_force = 0.5 * mass * gravity;
			jd.max_torque = 0.2 * mass * radius * gravity;

			world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::FrictionJoint(jd));
		}

		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.5, 0.5);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 1.0;
			fd.friction = 0.3;

			for i in 0..10 {
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;

				bd.position.set(0.0, 7.0 + 1.54 * i as f32);

				let body = B2world::create_body(world.clone(), &bd);

				B2body::create_fixture(body.clone(), &fd);

				let gravity: f32 = 10.0;
				let i: f32 = body.borrow().get_inertia();
				let mass: f32 = body.borrow().get_mass();

				// For a circle: i = 0.5 * m * r * r ==> r = sqrt(2 * i / m)
				let radius: f32 = b2_sqrt(2.0 * i / mass);

				let mut jd = B2frictionJointDef::<D>::default();
				jd.local_anchor_a.set_zero();
				jd.local_anchor_b.set_zero();
				jd.base.body_a = Some(ground.clone());
				jd.base.body_b = Some(body.clone());
				jd.base.collide_connected = true;
				jd.max_force = mass * gravity;
				jd.max_torque = 0.1 * mass * radius * gravity;

				world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::FrictionJoint(jd));
			}
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for ApplyForce<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		match key.virtual_keycode {
			Some(VirtualKeyCode::W) => {
				self.key_w = key.state == ElementState::Pressed;
			}
			Some(VirtualKeyCode::A) => {
				self.key_a = key.state == ElementState::Pressed;
			}
			Some(VirtualKeyCode::D) => {
				self.key_d = key.state == ElementState::Pressed;
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
		{
			let mut base = self.base.borrow_mut();
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Forward (W), Turn (A) and (D)",
			);
			base.m_text_line += base.m_text_increment;
		}

		if self.key_w {
			let mut m_body = self.m_body.as_ref().unwrap().borrow_mut();
			let f: B2vec2 = m_body.get_world_vector(B2vec2::new(0.0, -50.0));
			let p: B2vec2 = m_body.get_world_point(B2vec2::new(0.0, 3.0));
			m_body.apply_force(f, p, true);
		}

		if self.key_a {
			let mut m_body = self.m_body.as_ref().unwrap().borrow_mut();
			m_body.apply_torque(10.0, true);
		}

		if self.key_d {
			let mut m_body = self.m_body.as_ref().unwrap().borrow_mut();
			m_body.apply_torque(-10.0, true);
		}

		Test::step(self.base.clone(), ui, display, target, settings, *camera);
	}
}
