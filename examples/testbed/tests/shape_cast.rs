use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_distance::*;
use box2d_rs::b2_draw::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_common::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world_callbacks::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct ShapeCast<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_v_as: [B2vec2; B2_MAX_POLYGON_VERTICES],
	m_count_a: i32,
	m_radius_a: f32,

	m_v_bs: [B2vec2; B2_MAX_POLYGON_VERTICES],
	m_count_b: i32,
	m_radius_b: f32,

	m_transform_a: B2Transform,
	m_transform_b: B2Transform,
	m_translation_b: B2vec2,

}

impl ShapeCast<UserDataTypes> {
	pub fn new<F: Facade>(global_draw: TestBedDebugDrawPtr) -> TestPtr<UserDataTypes, F> {
		let base = Rc::new(RefCell::new(Test::new(global_draw.clone())));
		let result_ptr = Rc::new(RefCell::new(Self {
			base: base.clone(),
			destruction_listener: Rc::new(RefCell::new(B2testDestructionListenerDefault {
				base: Rc::downgrade(&base),
			})),
			contact_listener: Rc::new(RefCell::new(B2testContactListenerDefault {
				base: Rc::downgrade(&base),
			})),

			m_v_as: Default::default(),
			m_count_a: 0,
			m_radius_a: 0.0,
			m_v_bs: Default::default(),
			m_count_b: 0,
			m_radius_b: 0.0,
			m_transform_a: B2Transform::default(),
			m_transform_b: B2Transform::default(),
			m_translation_b: B2vec2::default(),
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
		if true {
			self.m_v_as[0].set(-0.5, 1.0);
			self.m_v_as[1].set(0.5, 1.0);
			self.m_v_as[2].set(0.0, 0.0);
			self.m_count_a = 3;
			self.m_radius_a = B2_POLYGON_RADIUS;

			self.m_v_bs[0].set(-0.5, -0.5);
			self.m_v_bs[1].set(0.5, -0.5);
			self.m_v_bs[2].set(0.5, 0.5);
			self.m_v_bs[3].set(-0.5, 0.5);
			self.m_count_b = 4;
			self.m_radius_b = B2_POLYGON_RADIUS;

			
			self.m_transform_a.p.set(0.0, 0.25);
			self.m_transform_a.q.set_identity();
			self.m_transform_b.p.set(-4.0, 0.0);
			self.m_transform_b.q.set_identity();
			self.m_translation_b.set(8.0, 0.0);
		}
		else if false {
			self.m_v_as[0].set(0.0, 0.0);
			self.m_count_a = 1;
			self.m_radius_a = 0.5;

			self.m_v_bs[0].set(0.0, 0.0);
			self.m_count_b = 1;
			self.m_radius_b = 0.5;

			self.m_transform_a.p.set(0.0, 0.25);
			self.m_transform_a.q.set_identity();
			self.m_transform_b.p.set(-4.0, 0.0);
			self.m_transform_b.q.set_identity();
			self.m_translation_b.set(8.0, 0.0);
		}
		else
		{
			self.m_v_as[0].set(0.0, 0.0);
			self.m_v_as[1].set(2.0, 0.0);
			self.m_count_a = 2;
			self.m_radius_a = B2_POLYGON_RADIUS;
	
			self.m_v_bs[0].set(0.0, 0.0);
			self.m_count_b = 1;
			self.m_radius_b = 0.25;
	
			// Initial overlap
			self.m_transform_a.p.set(0.0, 0.0);
			self.m_transform_a.q.set_identity();
			self.m_transform_b.p.set(-0.244360745, 0.05999358);
			self.m_transform_b.q.set_identity();
			self.m_translation_b.set(0.0, 0.0399999991);
	
		}
	}
}

impl<F: Facade> TestDyn<UserDataTypes, F> for ShapeCast<UserDataTypes> {
	fn get_base(&self) -> TestBasePtr<UserDataTypes> {
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


		let mut input = B2shapeCastInput::default();
		input
			.proxy_a
			.set_vertices(&self.m_v_as[..self.m_count_a as usize], self.m_radius_a);
		input
			.proxy_b
			.set_vertices(&self.m_v_bs[..self.m_count_b as usize], self.m_radius_b);

		input.transform_a = self.m_transform_a;
		input.transform_b = self.m_transform_b;
		input.translation_b = self.m_translation_b;	

		let mut output = B2shapeCastOutput::default();

		let hit: bool = b2_shape_cast(&mut output, input.clone());

		let mut transform_b2 = B2Transform::default();
		transform_b2.q = self.m_transform_b.q;
		transform_b2.p = self.m_transform_b.p + output.lambda * input.translation_b;


		let mut distance_input = B2distanceInput::default();
		distance_input
			.proxy_a
			.set_vertices(&self.m_v_as[..self.m_count_a as usize], self.m_radius_a);
		distance_input
			.proxy_b
			.set_vertices(&self.m_v_bs[..self.m_count_b as usize], self.m_radius_b);
			distance_input.transform_a = self.m_transform_a;

		distance_input.transform_b = transform_b2;
		distance_input.use_radii = false;
		let mut simplex_cache = B2simplexCache::default();
		simplex_cache.count = 0;
		let mut distance_output = B2distanceOutput::default();
		b2_distance_fn(&mut distance_output, &mut simplex_cache, &distance_input);
		let mut base = self.base.borrow_mut();

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			&format!(
				"hit = {0}, iters = {1}, lambda = {2}, distance = {3}",
				if hit { "true" } else { "false" },
				output.iterations,
				output.lambda,
				distance_output.distance
			),
		);
		base.m_text_line += base.m_text_increment;

		let mut vertices = <[B2vec2; B2_MAX_POLYGON_VERTICES]>::default();

		for i in 0..self.m_count_a {
			vertices[i as usize] = b2_mul_transform_by_vec2(self.m_transform_a, self.m_v_as[i as usize]);
		}
		if self.m_count_a == 1
		{
			base.g_debug_draw.borrow_mut().draw_circle(
				vertices[0],
				self.m_radius_a,
				B2color::new(0.9, 0.9, 0.9),
			);
		}
		else
		{
			base.g_debug_draw.borrow_mut().draw_polygon(
				&vertices[..self.m_count_a as usize],
				B2color::new(0.9, 0.9, 0.9),
			);
		}

		for i in 0..self.m_count_b {
			vertices[i as usize] = b2_mul_transform_by_vec2(self.m_transform_b, self.m_v_bs[i as usize]);
		}

		if self.m_count_b == 1
		{
			base.g_debug_draw.borrow_mut().draw_circle(
				vertices[0],
				self.m_radius_b,
				B2color::new(0.5, 0.9, 0.5),
			);
		}
		else
		{
			base.g_debug_draw.borrow_mut().draw_polygon(
				&vertices[..self.m_count_b as usize],
				B2color::new(0.5, 0.9, 0.5),
			);
		}

		for i in 0..self.m_count_b {
			vertices[i as usize] = b2_mul_transform_by_vec2(transform_b2, self.m_v_bs[i as usize]);
		}

		if self.m_count_b == 1
		{
			base.g_debug_draw.borrow_mut().draw_circle(
				vertices[0],
				self.m_radius_b,
				B2color::new(0.5, 0.7, 0.9),
			);
		}
		else
		{
			base.g_debug_draw.borrow_mut().draw_polygon(
				&vertices[..self.m_count_b as usize],
				B2color::new(0.5, 0.7, 0.9),
			);
		}

		if hit
		{
			let p1: B2vec2 = output.point;
			base.g_debug_draw
				.borrow_mut()
				.draw_point(p1, 10.0, B2color::new(0.9, 0.3, 0.3));
			let p2: B2vec2 = p1 + output.normal;
			base.g_debug_draw
				.borrow_mut()
				.draw_segment(p1, p2, B2color::new(0.9, 0.3, 0.3));
		}

	}
}

//static int testIndex = RegisterTest("Collision", "Shape Cast", ShapeCast::create);
