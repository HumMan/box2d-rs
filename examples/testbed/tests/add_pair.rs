use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct AddPair<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
}

impl<D: UserDataType> AddPair<D> {
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
			AddPair::init(&mut self_);
		}

		return result_ptr;
	}

	//TODO_humman может тут и везде просто self?
	fn init(self_: &mut AddPair<D>) {
		let world = self_.base.borrow().m_world.clone();

		world.borrow_mut().set_gravity(B2vec2::zero());
		{
			let mut shape = B2circleShape::default();
			shape.m_p.set_zero();
			shape.base.m_radius = 0.1;

			let min_x: f32 = -6.0;
			let max_x: f32 = 0.0;
			let min_y: f32 = 4.0;
			let max_y: f32 = 6.0;

			for _i in 0..400 {
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position = B2vec2::new(
					random_float_range(min_x, max_x),
					random_float_range(min_y, max_y),
				);
				let body = B2world::create_body(world.clone(), &bd);
				B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 0.01);
			}
		}
		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(1.5, 1.5);
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(-40.0, 5.0);
			bd.bullet = true;

			let body = B2world::create_body(world.clone(), &bd);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 1.0);
			body.borrow_mut()
				.set_linear_velocity(B2vec2::new(10.0, 0.0));
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for AddPair<D> {
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
