use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::joints::b2_revolute_joint::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct MobileUnbalanced<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
}
const E_DEPTH: i32 = 4;

impl<D: UserDataType> MobileUnbalanced<D> {
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
		let ground: BodyPtr<D>;

		// create ground body.
		{
			let mut body_def = B2bodyDef::default();
			body_def.position.set(0.0, 20.0);
			ground = B2world::create_body(m_world.clone(), &body_def);
		}

		let a: f32 = 0.5;
		let h = B2vec2::new(0.0, a);

		let root: BodyPtr<D> = self.add_node(ground.clone(), B2vec2::zero(), 0, 3.0, a);

		let mut joint_def = B2revoluteJointDef::default();
		joint_def.base.body_a = Some(ground);
		joint_def.base.body_b = Some(root);
		joint_def.local_anchor_a.set_zero();
		joint_def.local_anchor_b = h;
		m_world
			.borrow_mut()
			.create_joint(&B2JointDefEnum::RevoluteJoint(joint_def));
	}

	fn add_node(
		&mut self,
		parent: BodyPtr<D>,
		local_anchor: B2vec2,
		depth: i32,
		offset: f32,
		a: f32,
	) -> BodyPtr<D> {
		let m_world = self.base.borrow().m_world.clone();

		let density: f32 = 20.0;
		let h = B2vec2::new(0.0, a);

		let p: B2vec2 = parent.borrow().get_position() + local_anchor - h;

		let mut body_def = B2bodyDef::default();
		body_def.body_type = B2bodyType::B2DynamicBody;
		body_def.position = p;
		let body = B2world::create_body(m_world.clone(), &body_def);

		let mut shape = B2polygonShape::default();
		shape.set_as_box(0.25 * a, a);
		B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), density);

		if depth == E_DEPTH {
			return body;
		}

		let a1: B2vec2 = B2vec2::new(offset, -a);
		let a2: B2vec2 = B2vec2::new(-offset, -a);
		let body1: BodyPtr<D> = self.add_node(body.clone(), a1, depth + 1, 0.5 * offset, a);
		let body2: BodyPtr<D> = self.add_node(body.clone(), a2, depth + 1, 0.5 * offset, a);

		let mut joint_def = B2revoluteJointDef::default();
		joint_def.base.body_a = Some(body.clone());
		joint_def.local_anchor_b = h;

		joint_def.local_anchor_a = a1;
		joint_def.base.body_b = Some(body1);
		m_world
			.borrow_mut()
			.create_joint(&B2JointDefEnum::RevoluteJoint(joint_def.clone()));

		joint_def.local_anchor_a = a2;
		joint_def.base.body_b = Some(body2);
		m_world
			.borrow_mut()
			.create_joint(&B2JointDefEnum::RevoluteJoint(joint_def));

		return body;
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for MobileUnbalanced<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn step(
		&mut self,
		ui: &imgui::Ui,
		display: &F,
		target: &mut glium::Frame,
		settings: &mut Settings,
		camera: &mut Camera,
	) {
		Test::step(self.base.clone(), ui, display, target, settings, *camera);
	}
}

//static int testIndex = RegisterTest("Solver", "Mobile Unbalanced", MobileUnbalanced::create);
