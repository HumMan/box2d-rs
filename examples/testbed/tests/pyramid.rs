use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct Pyramid<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
}

impl<D: UserDataType> Pyramid<D> {
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
			let self_ = result_ptr.borrow();
			let mut base = base.borrow_mut();
			{
				let world = base.m_world.clone();
				let mut world = world.borrow_mut();
				world.set_destruction_listener(self_.destruction_listener.clone());
				world.set_contact_listener(self_.contact_listener.clone());
				world.set_debug_draw(global_draw);
			}
			Pyramid::init(&mut base);
		}

		return result_ptr;
	}

	const E_COUNT: usize = 20;

	fn init(self_: &mut Test<D>) {
		{
			let bd = B2bodyDef::default();
			let ground = B2world::create_body(self_.m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let a: f32 = 0.5;
			let mut shape = B2polygonShape::default();
			shape.set_as_box(a, a);

			let mut x = B2vec2::new(-7.0, 0.75);
			let mut y: B2vec2;
			let delta_x = B2vec2::new(0.5625, 1.25);
			let delta_y = B2vec2::new(1.125, 0.0);

			for i in 0..Self::E_COUNT {
				y = x;

				for _j in i..Self::E_COUNT {
					let mut bd = B2bodyDef::default();
					bd.body_type = B2bodyType::B2DynamicBody;
					bd.position = y;
					let body = B2world::create_body(self_.m_world.clone(), &bd);
					B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(shape)), 5.0);

					y += delta_y;
				}

				x += delta_x;
			}
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Pyramid<D> {
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

		// if self.base.m_step_count == 400 {
		// 	let contact_manager = self.base.m_world.borrow().get_contact_manager();
		// 	let broad_phase = contact_manager.borrow().get_broad_phase();
		// 	let mut broad_phase = broad_phase.borrow_mut();
		// 	let tree = broad_phase.get_tree_mut();
		// 	tree.rebuild_bottom_up();
		// }
	}
}
