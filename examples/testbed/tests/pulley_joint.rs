use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use box2d_rs::joints::b2_pulley_joint::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct PulleyJoint<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
	m_joint1: Option<B2jointPtr<D>>,
}

impl<D: UserDataType> PulleyJoint<D> {
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
			PulleyJoint::init(&mut self_);
		}

		return result_ptr;
	}

	fn init(self_: &mut PulleyJoint<D>) {
		let y: f32 = 16.0;
		let l: f32 = 12.0;
		let a: f32 = 1.0;
		let b: f32 = 2.0;

		let ground;
		let world = self_.base.borrow().m_world.clone();
		{
			let bd = B2bodyDef::default();
			ground = B2world::create_body(world.clone(), &bd);

			let mut circle = B2circleShape::default();
			circle.base.m_radius = 2.0;

			circle.m_p.set(-10.0, y + b + l);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(circle)), 0.0);

			circle.m_p.set(10.0, y + b + l);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(circle)), 0.0);
		}

		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(a, b);

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;

			//bd.fixed_rotation = true;
			bd.position.set(-10.0, y);
			let body1 = B2world::create_body(world.clone(), &bd);
			B2body::create_fixture_by_shape(body1.clone(), Rc::new(RefCell::new(shape)), 5.0);

			bd.position.set(10.0, y);
			let body2 = B2world::create_body(world.clone(), &bd);
			B2body::create_fixture_by_shape(body2.clone(), Rc::new(RefCell::new(shape)), 5.0);

			let mut pulley_def = B2pulleyJointDef::<D>::default();
			let anchor1 = B2vec2::new(-10.0, y + b);
			let anchor2 = B2vec2::new(10.0, y + b);
			let ground_anchor1 = B2vec2::new(-10.0, y + b + l);
			let ground_anchor2 = B2vec2::new(10.0, y + b + l);
			pulley_def.initialize(
				body1,
				body2,
				ground_anchor1,
				ground_anchor2,
				anchor1,
				anchor2,
				1.5,
			);

			self_.m_joint1 = Some(
				world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::PulleyJoint(pulley_def)),
			);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for PulleyJoint<D> {
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

		if let JointAsDerived::EPulleyJoint(ref m_joint1) =
			self.m_joint1.as_ref().unwrap().borrow().as_derived()
		{
			let ratio: f32 = m_joint1.get_ratio();
			let l: f32 = m_joint1.get_current_length_a() + ratio * m_joint1.get_current_length_b();
			let mut self_ = self.base.borrow_mut();
			self_.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				&format!("l1 + {0:4.2} * l2 = {1:4.2}", ratio, l),
			);
			self_.m_text_line += self_.m_text_increment;
		} else {
			panic!();
		}
	}
}
