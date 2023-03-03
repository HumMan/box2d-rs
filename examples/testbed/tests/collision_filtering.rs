use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::joints::b2_prismatic_joint::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

// This is a test of collision filtering.
// There is a triangle, a box, and a circle.
// There are 6 shapes. 3 large and 3 small.
// The 3 small ones always collide.
// The 3 large ones never collide.
// The boxes don't collide with triangles (except if both are small).
pub(crate) struct CollisionFiltering<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
}

impl<D: UserDataType> CollisionFiltering<D> {
	const K_SMALL_GROUP: i16 = 1;
	const K_LARGE_GROUP: i16 = -1;

	const K_TRIANGLE_CATEGORY: u16 = 0x0002;
	const K_BOX_CATEGORY: u16 = 0x0004;
	const K_CIRCLE_CATEGORY: u16 = 0x0008;

	const K_TRIANGLE_MASK: u16 = 0xFFFF;
	const K_BOX_MASK: u16 = 0xFFFF ^ Self::K_TRIANGLE_CATEGORY;
	const K_CIRCLE_MASK: u16 = 0xFFFF;
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

		// Ground body
		{
			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));

			let mut sd = B2fixtureDef::default();
			sd.shape = Some(Rc::new(RefCell::new(shape)));
			sd.friction = 0.3;

			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture(ground.clone(), &sd);
		}

		// Small triangle
		let mut vertices: [B2vec2; 3] = [
			B2vec2::new(-1.0, 0.0),
			B2vec2::new(1.0, 0.0),
			B2vec2::new(0.0, 2.0),
		];
		let polygon = Rc::new(RefCell::new(B2polygonShape::default()));
		polygon.borrow_mut().set(&vertices);

		let mut triangle_shape_def = B2fixtureDef::default();
		triangle_shape_def.shape = Some(polygon.clone());
		triangle_shape_def.density = 1.0;

		triangle_shape_def.filter.group_index = Self::K_SMALL_GROUP;
		triangle_shape_def.filter.category_bits = Self::K_TRIANGLE_CATEGORY;
		triangle_shape_def.filter.mask_bits = Self::K_TRIANGLE_MASK;

		let mut triangle_body_def = B2bodyDef::default();
		triangle_body_def.body_type = B2bodyType::B2DynamicBody;
		triangle_body_def.position.set(-5.0, 2.0);

		let body1 = B2world::create_body(m_world.clone(), &triangle_body_def);
		B2body::create_fixture(body1.clone(), &triangle_shape_def);

		// Large triangle (recycle definitions)
		vertices[0] *= 2.0;
		vertices[1] *= 2.0;
		vertices[2] *= 2.0;
		polygon.borrow_mut().set(&vertices);
		triangle_shape_def.filter.group_index = Self::K_LARGE_GROUP;
		triangle_body_def.position.set(-5.0, 6.0);
		triangle_body_def.fixed_rotation = true; // look at me!

		let body2 = B2world::create_body(m_world.clone(), &triangle_body_def);
		B2body::create_fixture(body2.clone(), &triangle_shape_def);

		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(-5.0, 10.0);
			let body = B2world::create_body(m_world.clone(), &bd);

			let mut p = B2polygonShape::default();
			p.set_as_box(0.5, 1.0);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(p)), 1.0);

			let mut jd = B2prismaticJointDef::default();
			jd.base.body_a = Some(body2);
			jd.base.body_b = Some(body);
			jd.enable_limit = true;
			jd.local_anchor_a.set(0.0, 4.0);
			jd.local_anchor_b.set_zero();
			jd.local_axis_a.set(0.0, 1.0);
			jd.lower_translation = -1.0;
			jd.upper_translation = 1.0;

			m_world
				.borrow_mut()
				.create_joint(&B2JointDefEnum::PrismaticJoint(jd));
		}

		// Small box
		polygon.borrow_mut().set_as_box(1.0, 0.5);
		let mut box_shape_def = B2fixtureDef::default();
		box_shape_def.shape = Some(polygon.clone());
		box_shape_def.density = 1.0;
		box_shape_def.restitution = 0.1;

		box_shape_def.filter.group_index = Self::K_SMALL_GROUP;
		box_shape_def.filter.category_bits = Self::K_BOX_CATEGORY;
		box_shape_def.filter.mask_bits = Self::K_BOX_MASK;

		let mut box_body_def = B2bodyDef::default();
		box_body_def.body_type = B2bodyType::B2DynamicBody;
		box_body_def.position.set(0.0, 2.0);

		let body3 = B2world::create_body(m_world.clone(), &box_body_def);
		B2body::create_fixture(body3.clone(), &box_shape_def);

		// Large box (recycle definitions)
		polygon.borrow_mut().set_as_box(2.0, 1.0);
		box_shape_def.filter.group_index = Self::K_LARGE_GROUP;
		box_body_def.position.set(0.0, 6.0);

		let body4 = B2world::create_body(m_world.clone(), &box_body_def);
		B2body::create_fixture(body4.clone(), &box_shape_def);

		// Small circle
		let circle = Rc::new(RefCell::new(B2circleShape::default()));
		circle.borrow_mut().base.m_radius = 1.0;

		let mut circle_shape_def = B2fixtureDef::default();
		circle_shape_def.shape = Some(circle.clone());
		circle_shape_def.density = 1.0;

		circle_shape_def.filter.group_index = Self::K_SMALL_GROUP;
		circle_shape_def.filter.category_bits = Self::K_CIRCLE_CATEGORY;
		circle_shape_def.filter.mask_bits = Self::K_CIRCLE_MASK;

		let mut circle_body_def = B2bodyDef::default();
		circle_body_def.body_type = B2bodyType::B2DynamicBody;
		circle_body_def.position.set(5.0, 2.0);
		let body5 = B2world::create_body(m_world.clone(), &circle_body_def);
		B2body::create_fixture(body5.clone(), &circle_shape_def);

		// Large circle
		circle.borrow_mut().base.m_radius *= 2.0;
		circle_shape_def.filter.group_index = Self::K_LARGE_GROUP;
		circle_body_def.position.set(5.0, 6.0);

		let body6 = B2world::create_body(m_world.clone(), &circle_body_def);
		B2body::create_fixture(body6.clone(), &circle_shape_def);
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for CollisionFiltering<D> {
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
