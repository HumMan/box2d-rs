use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

// Note: even with a restitution of 1.0, there is some energy change
// due to position correction.
pub(crate) struct Restitution<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
}

impl<D: UserDataType> Restitution<D> {
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

		const THRESHOLD: f32 = 10.0;

		let m_world = self.base.borrow().m_world.clone();
		{
			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.restitution_threshold = THRESHOLD;
			B2body::create_fixture(ground, &fd);
		}

		{
			let mut shape = B2circleShape::default();
			shape.base.m_radius = 1.0;

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 1.0;

			let restitution: [f32; 7] = [0.0, 0.1, 0.3, 0.5, 0.75, 0.9, 1.0];

			for (i, r) in restitution.iter().enumerate() {
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(-10.0 + 3.0 * i as f32, 20.0);

				let body = B2world::create_body(m_world.clone(), &bd);

				fd.restitution = *r;
				fd.restitution_threshold = THRESHOLD;
				B2body::create_fixture(body.clone(), &fd);
			}
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Restitution<D> {
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

//static int testIndex = RegisterTest("Forces", "Restitution", Restitution::create);
