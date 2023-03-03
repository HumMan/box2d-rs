use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;




use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

// TODO_ERIN test joints on compounds.
pub(crate) struct CompoundShapes<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_table1: Option<BodyPtr<D>>,
	m_table2: Option<BodyPtr<D>>,
	m_ship1: Option<BodyPtr<D>>,
	m_ship2: Option<BodyPtr<D>>,
}

impl<D: UserDataType> CompoundShapes<D> {
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

			m_table1: None,
			m_table2: None,
			m_ship1: None,
			m_ship2: None,
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
		{
			let mut bd = B2bodyDef::default();
			bd.position.set(0.0, 0.0);
			let body = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(50.0, 0.0), B2vec2::new(-50.0, 0.0));

			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		// Table 1
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(-15.0, 1.0);
			let m_table1 = B2world::create_body(m_world.clone(), &bd);
			self.m_table1 = Some(m_table1.clone());

			let mut top = B2polygonShape::default();
			top.set_as_box_angle(3.0, 0.5, B2vec2::new(0.0, 3.5), 0.0);

			let mut left_leg = B2polygonShape::default();
			left_leg.set_as_box_angle(0.5, 1.5, B2vec2::new(-2.5, 1.5), 0.0);

			let mut right_leg = B2polygonShape::default();
			right_leg.set_as_box_angle(0.5, 1.5, B2vec2::new(2.5, 1.5), 0.0);

			B2body::create_fixture_by_shape(m_table1.clone(), Rc::new(RefCell::new(top)), 2.0);
			B2body::create_fixture_by_shape(m_table1.clone(), Rc::new(RefCell::new(left_leg)), 2.0);
			B2body::create_fixture_by_shape(
				m_table1.clone(),
				Rc::new(RefCell::new(right_leg)),
				2.0,
			);
		}

		// Table 2
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(-5.0, 1.0);
			let m_table2 = B2world::create_body(m_world.clone(), &bd);
			self.m_table2 = Some(m_table2.clone());

			let mut top = B2polygonShape::default();
			top.set_as_box_angle(3.0, 0.5, B2vec2::new(0.0, 3.5), 0.0);

			let mut left_leg = B2polygonShape::default();
			left_leg.set_as_box_angle(0.5, 2.0, B2vec2::new(-2.5, 2.0), 0.0);

			let mut right_leg = B2polygonShape::default();
			right_leg.set_as_box_angle(0.5, 2.0, B2vec2::new(2.5, 2.0), 0.0);

			B2body::create_fixture_by_shape(m_table2.clone(), Rc::new(RefCell::new(top)), 2.0);
			B2body::create_fixture_by_shape(m_table2.clone(), Rc::new(RefCell::new(left_leg)), 2.0);
			B2body::create_fixture_by_shape(
				m_table2.clone(),
				Rc::new(RefCell::new(right_leg)),
				2.0,
			);
		}

		// Spaceship 1
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(5.0, 1.0);
			let m_ship1 = B2world::create_body(m_world.clone(), &bd);
			self.m_ship1 = Some(m_ship1.clone());

			let mut left = B2polygonShape::default();
			let vertices: [B2vec2; 3] = [
				B2vec2::new(-2.0, 0.0),
				B2vec2::new(0.0, 4.0 / 3.0),
				B2vec2::new(0.0, 4.0),
			];
			left.set(&vertices);

			let mut right = B2polygonShape::default();
			let vertices: [B2vec2; 3] = [
				B2vec2::new(2.0, 0.0),
				B2vec2::new(0.0, 4.0 / 3.0),
				B2vec2::new(0.0, 4.0),
			];
			right.set(&vertices);

			B2body::create_fixture_by_shape(m_ship1.clone(), Rc::new(RefCell::new(left)), 2.0);
			B2body::create_fixture_by_shape(m_ship1.clone(), Rc::new(RefCell::new(right)), 2.0);
		}

		// Spaceship 2
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(15.0, 1.0);
			let m_ship2 = B2world::create_body(m_world.clone(), &bd);
			self.m_ship2 = Some(m_ship2.clone());

			let mut left = B2polygonShape::default();
			let vertices: [B2vec2; 3] = [
				B2vec2::new(-2.0, 0.0),
				B2vec2::new(1.0, 2.0),
				B2vec2::new(0.0, 4.0),
			];
			left.set(&vertices);

			let mut right = B2polygonShape::default();
			let vertices: [B2vec2; 3] = [
				B2vec2::new(2.0, 0.0),
				B2vec2::new(-1.0, 2.0),
				B2vec2::new(0.0, 4.0),
			];
			right.set(&vertices);

			B2body::create_fixture_by_shape(m_ship2.clone(), Rc::new(RefCell::new(left)), 2.0);
			B2body::create_fixture_by_shape(m_ship2.clone(), Rc::new(RefCell::new(right)), 2.0);
		}
	}

	fn spawn(&mut self) {
		let m_world = self.base.borrow().m_world.clone();
		// Table 1 obstruction
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			let m_table1 = self.m_table1.as_ref().unwrap().borrow();
			bd.position = m_table1.get_position();
			bd.angle = m_table1.get_angle();

			let body = B2world::create_body(m_world.clone(), &bd);

			let mut box_shape = B2polygonShape::default();
			box_shape.set_as_box_angle(4.0, 0.1, B2vec2::new(0.0, 3.0), 0.0);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(box_shape)), 2.0);
		}

		// Table 2 obstruction
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			let m_table2 = self.m_table2.as_ref().unwrap().borrow();
			bd.position = m_table2.get_position();
			bd.angle = m_table2.get_angle();

			let body = B2world::create_body(m_world.clone(), &bd);

			let mut box_shape = B2polygonShape::default();
			box_shape.set_as_box_angle(4.0, 0.1, B2vec2::new(0.0, 3.0), 0.0);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(box_shape)), 2.0);
		}

		// Ship 1 obstruction
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			let m_ship1 = self.m_ship1.as_ref().unwrap().borrow();
			bd.position = m_ship1.get_position();
			bd.angle = m_ship1.get_angle();
			bd.gravity_scale = 0.0;

			let body = B2world::create_body(m_world.clone(), &bd);

			let mut circle = B2circleShape::default();
			circle.base.m_radius = 0.5;
			circle.m_p.set(0.0, 2.0);

			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(circle)), 2.0);
		}

		// Ship 2 obstruction
		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			let m_ship2 = self.m_ship2.as_ref().unwrap().borrow();
			bd.position = m_ship2.get_position();
			bd.angle = m_ship2.get_angle();
			bd.gravity_scale = 0.0;

			let body = B2world::create_body(m_world.clone(), &bd);

			let mut circle = B2circleShape::default();
			circle.base.m_radius = 0.5;
			circle.m_p.set(0.0, 2.0);

			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(circle)), 2.0);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for CompoundShapes<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn update_ui(&mut self, ui: &imgui::Ui) {
		ui.window("Controls")
			.flags(
				imgui::WindowFlags::NO_MOVE
					| imgui::WindowFlags::NO_RESIZE
					| imgui::WindowFlags::NO_COLLAPSE,
			)
			.position([10.0, 100.0], imgui::Condition::Always)
			.size([200.0, 100.0], imgui::Condition::Always)
			.build(|| {
				if ui.small_button("spawn") {
					self.spawn();
				}
			});
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
