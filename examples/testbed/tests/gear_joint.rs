use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::joints::b2_gear_joint::*;
use box2d_rs::joints::b2_prismatic_joint::*;
use box2d_rs::joints::b2_revolute_joint::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct GearJoint<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_joint1: Option<B2jointPtr<D>>,
	m_joint2: Option<B2jointPtr<D>>,
	m_joint3: Option<B2jointPtr<D>>,
	m_joint4: Option<B2jointPtr<D>>,
	m_joint5: Option<B2jointPtr<D>>,
}

impl<D: UserDataType> GearJoint<D> {
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
			m_joint3: None,
			m_joint4: None,
			m_joint5: None,
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
			shape.set_two_sided(B2vec2::new(50.0, 0.0), B2vec2::new(-50.0, 0.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let mut circle1 = B2circleShape::default();
			circle1.base.m_radius = 1.0;

			let mut box_shape = B2polygonShape::default();
			box_shape.set_as_box(0.5, 5.0);

			let mut circle2 = B2circleShape::default();
			circle2.base.m_radius = 2.0;
			let mut bd1 = B2bodyDef::default();
			bd1.body_type = B2bodyType::B2StaticBody;
			bd1.position.set(10.0, 9.0);
			let body1 = B2world::create_body(m_world.clone(), &bd1);
			B2body::create_fixture_by_shape(body1.clone(), Rc::new(RefCell::new(circle1)), 5.0);

			let mut bd2 = B2bodyDef::default();
			bd2.body_type = B2bodyType::B2DynamicBody;
			bd2.position.set(10.0, 8.0);
			let body2 = B2world::create_body(m_world.clone(), &bd2);
			B2body::create_fixture_by_shape(body2.clone(), Rc::new(RefCell::new(box_shape)), 5.0);

			let mut bd3 = B2bodyDef::default();
			bd3.body_type = B2bodyType::B2DynamicBody;
			bd3.position.set(10.0, 6.0);
			let body3 = B2world::create_body(m_world.clone(), &bd3);
			B2body::create_fixture_by_shape(body3.clone(), Rc::new(RefCell::new(circle2)), 5.0);

			let mut jd1 = B2revoluteJointDef::default();
			jd1.initialize(body2.clone(), body1.clone(), bd1.position);
			let joint1 = m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::RevoluteJoint(jd1));

			let mut jd2 = B2revoluteJointDef::default();
			jd2.initialize(body2.clone(), body3.clone(), bd3.position);
			let joint2 = m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::RevoluteJoint(jd2));

			let mut jd4 = B2gearJointDef::default();
			jd4.base.body_a = Some(body1);
			jd4.base.body_b = Some(body3);
			jd4.joint1 = Some(joint1);
			jd4.joint2 = Some(joint2);
			jd4.ratio = circle2.base.m_radius / circle1.base.m_radius;
			m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::GearJoint(jd4));
		}

		{
			let mut circle1 = B2circleShape::default();
			circle1.base.m_radius = 1.0;

			let mut circle2 = B2circleShape::default();
			circle2.base.m_radius = 2.0;
			let mut box_shape = B2polygonShape::default();
			box_shape.set_as_box(0.5, 5.0);

			let mut bd1 = B2bodyDef::default();
			bd1.body_type = B2bodyType::B2DynamicBody;
			bd1.position.set(-3.0, 12.0);
			let body1 = B2world::create_body(m_world.clone(), &bd1);
			B2body::create_fixture_by_shape(body1.clone(), Rc::new(RefCell::new(circle1)), 5.0);

			let mut jd1 = B2revoluteJointDef::default();
			jd1.base.body_a = Some(ground.clone());
			jd1.base.body_b = Some(body1.clone());
			jd1.local_anchor_a = ground.borrow().get_local_point(bd1.position);
			jd1.local_anchor_b = body1.borrow().get_local_point(bd1.position);
			jd1.reference_angle = body1.borrow().get_angle() - ground.borrow().get_angle();
			let m_joint1 = m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::RevoluteJoint(jd1));
			self.m_joint1 = Some(m_joint1.clone());

			let mut bd2 = B2bodyDef::default();
			bd2.body_type = B2bodyType::B2DynamicBody;
			bd2.position.set(0.0, 12.0);
			let body2 = B2world::create_body(m_world.clone(), &bd2);
			B2body::create_fixture_by_shape(body2.clone(), Rc::new(RefCell::new(circle2)), 5.0);

			let mut jd2 = B2revoluteJointDef::default();
			jd2.initialize(ground.clone(), body2.clone(), bd2.position);
			let m_joint2 = m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::RevoluteJoint(jd2));
			self.m_joint2 = Some(m_joint2.clone());

			let mut bd3 = B2bodyDef::default();
			bd3.body_type = B2bodyType::B2DynamicBody;
			bd3.position.set(2.5, 12.0);
			let body3 = B2world::create_body(m_world.clone(), &bd3);
			B2body::create_fixture_by_shape(body3.clone(), Rc::new(RefCell::new(box_shape)), 5.0);

			let mut jd3 = B2prismaticJointDef::default();
			jd3.initialize(ground, body3.clone(), bd3.position, B2vec2::new(0.0, 1.0));
			jd3.lower_translation = -5.0;
			jd3.upper_translation = 5.0;
			jd3.enable_limit = true;

			let m_joint3 = m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::PrismaticJoint(jd3));
			self.m_joint3 = Some(m_joint3.clone());

			let mut jd4 = B2gearJointDef::default();
			jd4.base.body_a = Some(body1.clone());
			jd4.base.body_b = Some(body2.clone());
			jd4.joint1 = Some(m_joint1.clone());
			jd4.joint2 = Some(m_joint2.clone());
			jd4.ratio = circle2.base.m_radius / circle1.base.m_radius;
			let m_joint4 = m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::GearJoint(jd4));
			self.m_joint4 = Some(m_joint4);

			let mut jd5 = B2gearJointDef::default();
			jd5.base.body_a = Some(body2.clone());
			jd5.base.body_b = Some(body3.clone());
			jd5.joint1 = Some(m_joint2.clone());
			jd5.joint2 = Some(m_joint3.clone());
			jd5.ratio = -1.0 / circle2.base.m_radius;
			let m_joint5 = m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::GearJoint(jd5));
			self.m_joint5 = Some(m_joint5);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for GearJoint<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
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

		let ratio;
		let value;

		match self.m_joint4.as_ref().unwrap().borrow_mut().as_derived() {
			JointAsDerived::EGearJoint(ref m_joint4) => {
				ratio = m_joint4.get_ratio();
			}
			_ => panic!(),
		}

		let m_joint1_angle;
		let m_joint2_angle;

		match self.m_joint1.as_ref().unwrap().borrow_mut().as_derived() {
			JointAsDerived::ERevoluteJoint(ref m_joint1) => {
				m_joint1_angle = m_joint1.get_joint_angle();
			}
			_ => panic!(),
		}

		match self.m_joint2.as_ref().unwrap().borrow_mut().as_derived() {
			JointAsDerived::ERevoluteJoint(ref m_joint2) => {
				m_joint2_angle = m_joint2.get_joint_angle();
			}
			_ => panic!(),
		}
		value = m_joint1_angle + ratio * m_joint2_angle;

		let mut base = self.base.borrow_mut();
		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			&format!("theta1 + {0:4.2} * theta2 = {1:4.2}", ratio, value),
		);
		base.m_text_line += base.m_text_increment;

		let ratio;

		match self.m_joint5.as_ref().unwrap().borrow_mut().as_derived() {
			JointAsDerived::EGearJoint(ref m_joint5) => {
				ratio = m_joint5.get_ratio();
			}
			_ => panic!(),
		}
		let m_joint2_angle;
		let m_joint3_translation;

		match self.m_joint2.as_ref().unwrap().borrow_mut().as_derived() {
			JointAsDerived::ERevoluteJoint(ref m_joint2) => {
				m_joint2_angle = m_joint2.get_joint_angle();
			}
			_ => panic!(),
		}

		match self.m_joint3.as_ref().unwrap().borrow_mut().as_derived() {
			JointAsDerived::EPrismaticJoint(ref m_joint3) => {
				m_joint3_translation = m_joint3.get_joint_translation();
			}
			_ => panic!(),
		}

		let value = m_joint2_angle + ratio * m_joint3_translation;

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			&format!("theta2 + {0:4.2} * delta = {1:4.2}", ratio, value),
		);
		base.m_text_line += base.m_text_increment;
	}
}
