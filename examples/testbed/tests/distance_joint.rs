use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::joints::b2_distance_joint::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};

// This tests distance joints, body destruction, and joint destruction.
pub(crate) struct DistanceJoint<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: Option<B2destructionListenerPtr<D>>,
	contact_listener: B2contactListenerPtr<D>,
	test_data: Rc<RefCell<TestData<D>>>,
}

#[derive(Default)]
struct TestData<D: UserDataType> {
	m_bodies: Vec<BodyPtr<D>>,
	m_joints: Vec<B2jointPtr<D>>,

	m_world: Option<B2worldPtr<D>>,
}

pub(crate) struct B2destructionListenerCustom<D: UserDataType> {
	base: B2testDestructionListenerDefault<D>,
	test_data: Rc<RefCell<TestData<D>>>,
}

impl<D: UserDataType> B2destructionListener<D> for B2destructionListenerCustom<D> {
	fn say_goodbye_fixture(&mut self, fixture: FixturePtr<D>) {
		self.base.say_goodbye_fixture(fixture);
	}
	fn say_goodbye_joint(&mut self, joint: B2jointPtr<D>) {
		self.base.say_goodbye_joint(joint.clone());

		let mut test_data = self.test_data.borrow_mut();
		test_data.m_joints.retain(|j| !Rc::ptr_eq(j, &joint));
	}
}

impl<D: UserDataType> DistanceJoint<D> {
	pub fn new<F: Facade>(global_draw: TestBedDebugDrawPtr) -> TestPtr<D, F> {
		let base = Rc::new(RefCell::new(Test::new(global_draw.clone())));
		let world = base.borrow().m_world.clone();

		let test_data = Rc::new(RefCell::new(TestData::default()));
		test_data.borrow_mut().m_world = Some(world.clone());

		let result_ptr = Rc::new(RefCell::new(Self {
			base: base.clone(),
			destruction_listener: None,
			contact_listener: Rc::new(RefCell::new(B2testContactListenerDefault {
				base: Rc::downgrade(&base),
			})),
			test_data: test_data.clone(),
		}));

		let destruction_listener = Rc::new(RefCell::new(B2destructionListenerCustom {
			base: {
				B2testDestructionListenerDefault {
					base: Rc::downgrade(&base),
				}
			},
			test_data: test_data.clone(),
		}));

		result_ptr.borrow_mut().destruction_listener = Some(destruction_listener.clone());

		{
			let mut self_ = result_ptr.borrow_mut();
			{
				let mut world = world.borrow_mut();
				world.set_destruction_listener(destruction_listener.clone());
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
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let mut t = self.test_data.borrow_mut();
			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.5, 0.5);

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;

			bd.position.set(-5.0, 5.0);
			t.m_bodies.push(B2world::create_body(m_world.clone(), &bd));
			B2body::create_fixture_by_shape(
				t.m_bodies[0].clone(),
				Rc::new(RefCell::new(shape)),
				5.0,
			);

			bd.position.set(5.0, 5.0);
			t.m_bodies.push(B2world::create_body(m_world.clone(), &bd));
			B2body::create_fixture_by_shape(
				t.m_bodies[1].clone(),
				Rc::new(RefCell::new(shape)),
				5.0,
			);

			bd.position.set(5.0, 15.0);
			t.m_bodies.push(B2world::create_body(m_world.clone(), &bd));
			B2body::create_fixture_by_shape(
				t.m_bodies[2].clone(),
				Rc::new(RefCell::new(shape)),
				5.0,
			);

			bd.position.set(-5.0, 15.0);
			t.m_bodies.push(B2world::create_body(m_world.clone(), &bd));
			B2body::create_fixture_by_shape(
				t.m_bodies[3].clone(),
				Rc::new(RefCell::new(shape)),
				5.0,
			);

			let mut jd = B2distanceJointDef::default();
			let mut p1: B2vec2;
			let mut p2: B2vec2;
			let mut d: B2vec2;

			let frequency_hz: f32 = 2.0;
			let damping_ratio: f32 = 0.0;

			jd.base.body_a = Some(ground.clone());
			jd.base.body_b = Some(t.m_bodies[0].clone());
			jd.local_anchor_a.set(-10.0, 0.0);
			jd.local_anchor_b.set(-0.5, -0.5);
			p1 = jd
				.base
				.body_a
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_a);
			p2 = jd
				.base
				.body_b
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_b);
			d = p2 - p1;
			jd.length = d.length();

			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);

			t.m_joints.push(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone())),
			);

			jd.base.body_a = Some(ground.clone());
			jd.base.body_b = Some(t.m_bodies[1].clone());
			jd.local_anchor_a.set(10.0, 0.0);
			jd.local_anchor_b.set(0.5, -0.5);
			p1 = jd
				.base
				.body_a
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_a);
			p2 = jd
				.base
				.body_b
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_b);
			d = p2 - p1;
			jd.length = d.length();

			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);
			t.m_joints.push(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone())),
			);

			jd.base.body_a = Some(ground.clone());
			jd.base.body_b = Some(t.m_bodies[2].clone());
			jd.local_anchor_a.set(10.0, 20.0);
			jd.local_anchor_b.set(0.5, 0.5);
			p1 = jd
				.base
				.body_a
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_a);
			p2 = jd
				.base
				.body_b
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_b);
			d = p2 - p1;
			jd.length = d.length();

			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);
			t.m_joints.push(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone())),
			);

			jd.base.body_a = Some(ground.clone());
			jd.base.body_b = Some(t.m_bodies[3].clone());
			jd.local_anchor_a.set(-10.0, 20.0);
			jd.local_anchor_b.set(-0.5, 0.5);
			p1 = jd
				.base
				.body_a
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_a);
			p2 = jd
				.base
				.body_b
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_b);
			d = p2 - p1;
			jd.length = d.length();

			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);
			t.m_joints.push(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone())),
			);

			jd.base.body_a = Some(t.m_bodies[0].clone());
			jd.base.body_b = Some(t.m_bodies[1].clone());
			jd.local_anchor_a.set(0.5, 0.0);
			jd.local_anchor_b.set(-0.5, 0.0);
			p1 = jd
				.base
				.body_a
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_a);
			p2 = jd
				.base
				.body_b
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_b);
			d = p2 - p1;
			jd.length = d.length();

			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);
			t.m_joints.push(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone())),
			);

			jd.base.body_a = Some(t.m_bodies[1].clone());
			jd.base.body_b = Some(t.m_bodies[2].clone());
			jd.local_anchor_a.set(0.0, 0.5);
			jd.local_anchor_b.set(0.0, -0.5);
			p1 = jd
				.base
				.body_a
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_a);
			p2 = jd
				.base
				.body_b
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_b);
			d = p2 - p1;
			jd.length = d.length();

			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);
			t.m_joints.push(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone())),
			);

			jd.base.body_a = Some(t.m_bodies[2].clone());
			jd.base.body_b = Some(t.m_bodies[3].clone());
			jd.local_anchor_a.set(-0.5, 0.0);
			jd.local_anchor_b.set(0.5, 0.0);
			p1 = jd
				.base
				.body_a
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_a);
			p2 = jd
				.base
				.body_b
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_b);
			d = p2 - p1;
			jd.length = d.length();

			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);
			t.m_joints.push(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone())),
			);

			jd.base.body_a = Some(t.m_bodies[3].clone());
			jd.base.body_b = Some(t.m_bodies[0].clone());
			jd.local_anchor_a.set(0.0, -0.5);
			jd.local_anchor_b.set(0.0, 0.5);
			p1 = jd
				.base
				.body_a
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_a);
			p2 = jd
				.base
				.body_b
				.as_ref()
				.unwrap()
				.borrow()
				.get_world_point(jd.local_anchor_b);
			d = p2 - p1;
			jd.length = d.length();

			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				frequency_hz,
				damping_ratio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);
			t.m_joints.push(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone())),
			);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for DistanceJoint<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		let m_world = self.test_data.borrow().m_world.clone().unwrap();
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::B) => {
					if self.test_data.borrow_mut().m_bodies.len() > 0 {
						let first_body = self.test_data.borrow_mut().m_bodies[0].clone();
						m_world.borrow_mut().destroy_body(first_body);
						self.test_data.borrow_mut().m_bodies.remove(0);
					}
				}
				Some(VirtualKeyCode::J) => {
					if self.test_data.borrow_mut().m_joints.len() > 0 {
						let first_joint = self.test_data.borrow_mut().m_joints[0].clone();
						m_world.borrow_mut().destroy_joint(first_joint);
						self.test_data.borrow_mut().m_joints.remove(0);
					}
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
		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		let mut base = self.base.borrow_mut();
		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			"This demonstrates a soft distance joint.",
		);
		base.m_text_line += base.m_text_increment;
		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			"Press: (b) to delete a body, (j) to delete a joint",
		);
		base.m_text_line += base.m_text_increment;
	}
}
