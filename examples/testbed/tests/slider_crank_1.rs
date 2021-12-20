use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::joints::b2_prismatic_joint::*;
use box2d_rs::joints::b2_revolute_joint::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

// A basic slider crank created for GDC tutorial: Understanding Constraints
pub(crate) struct SliderCrank1<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
}

impl<D: UserDataType> SliderCrank1<D> {
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
			let mut bd = B2bodyDef::default();
			bd.position.set(0.0, 17.0);
			ground = B2world::create_body(m_world.clone(), &bd);
		}
		{
			let mut prev_body: BodyPtr<D> = ground.clone();
			// Define crank.
			{
				let mut shape = B2polygonShape::default();
				shape.set_as_box(4.0, 1.0);
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(-8.0, 20.0);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 2.0);
				let mut rjd = B2revoluteJointDef::default();
				rjd.initialize(prev_body, body.clone(), B2vec2::new(-12.0, 20.0));
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(rjd));
				prev_body = body;
			}
			// Define connecting rod
			{
				let mut shape = B2polygonShape::default();
				shape.set_as_box(8.0, 1.0);
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(4.0, 20.0);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 2.0);
				let mut rjd = B2revoluteJointDef::default();
				rjd.initialize(prev_body, body.clone(), B2vec2::new(-4.0, 20.0));
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(rjd));
				prev_body = body;
			}
			// Define piston
			{
				let mut shape = B2polygonShape::default();
				shape.set_as_box(3.0, 3.0);
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.fixed_rotation = true;
				bd.position.set(12.0, 20.0);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 2.0);
				let mut rjd = B2revoluteJointDef::default();
				rjd.initialize(prev_body, body.clone(), B2vec2::new(12.0, 20.0));
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(rjd));
				let mut pjd = B2prismaticJointDef::default();
				pjd.initialize(ground, body, B2vec2::new(12.0, 17.0), B2vec2::new(1.0, 0.0));
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::PrismaticJoint(pjd));
			}
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for SliderCrank1<D> {
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
	}
}
//static int testIndex = RegisterTest("Examples", "Slider Crank 1", SliderCrank1::create);
