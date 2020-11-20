use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_draw::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_time_of_impact::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

use std::sync::atomic::Ordering;

pub(crate) struct TimeOfImpact<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_shape_a: B2polygonShape,
	m_shape_b: B2polygonShape,
}

impl<D: UserDataType> TimeOfImpact<D> {
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
			m_shape_a: B2polygonShape::default(),
			m_shape_b: B2polygonShape::default(),
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
		self.m_shape_a.set_as_box(25.0, 5.0);
		self.m_shape_b.set_as_box(2.5, 2.5);
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for TimeOfImpact<D> {
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

		let mut sweep_a = B2Sweep::default();
		sweep_a.c0.set(24.0, -60.0);
		sweep_a.a0 = 2.95;
		sweep_a.c = sweep_a.c0;
		sweep_a.a = sweep_a.a0;
		sweep_a.local_center.set_zero();

		let mut sweep_b = B2Sweep::default();
		sweep_b.c0.set(53.474274, -50.252514);
		sweep_b.a0 = 513.36676; // - 162.0 * B2_PI;
		sweep_b.c.set(54.595478, -51.083473);
		sweep_b.a = 513.62781; //  - 162.0 * B2_PI;
		sweep_b.local_center.set_zero();

		//sweep_b.a0 -= 300.0 * B2_PI;
		//sweep_b.a -= 300.0 * B2_PI;

		let mut input = B2toiinput::default();
		input.proxy_a.set_shape(Rc::new(self.m_shape_a), 0);
		input.proxy_b.set_shape(Rc::new(self.m_shape_b), 0);
		input.sweep_a = sweep_a;
		input.sweep_b = sweep_b;
		input.t_max = 1.0;

		let mut output = B2toioutput::default();

		b2_time_of_impact(&mut output, &input);

		let mut base = self.base.borrow_mut();

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			&format!("toi = {0}", output.t),
		);
		base.m_text_line += base.m_text_increment;

		let load_b2_toi_max_iters = B2_TOI_MAX_ITERS.load(Ordering::SeqCst);
		let load_b2_toi_max_root_iters = B2_TOI_MAX_ROOT_ITERS.load(Ordering::SeqCst);

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			&format!(
				"max toi iters = {0}, max root iters = {1}",
				load_b2_toi_max_iters, load_b2_toi_max_root_iters
			),
		);
		base.m_text_line += base.m_text_increment;

		let mut vertices: [B2vec2; B2_MAX_POLYGON_VERTICES] = Default::default();

		let mut transform_a = B2Transform::default();
		sweep_a.get_transform(&mut transform_a, 0.0);
		for i in 0..self.m_shape_a.m_count {
			vertices[i] = b2_mul_transform_by_vec2(transform_a, self.m_shape_a.m_vertices[i]);
		}
		base.g_debug_draw.borrow_mut().draw_polygon(
			&vertices[..self.m_shape_a.m_count],
			B2color::new(0.9, 0.9, 0.9),
		);

		let mut transform_b = B2Transform::default();
		sweep_b.get_transform(&mut transform_b, 0.0);
		//let localPoint = B2vec2::new(2.0, -0.1);

		for i in 0..self.m_shape_b.m_count {
			vertices[i] = b2_mul_transform_by_vec2(transform_b, self.m_shape_b.m_vertices[i]);
		}
		base.g_debug_draw.borrow_mut().draw_polygon(
			&vertices[..self.m_shape_b.m_count],
			B2color::new(0.5, 0.9, 0.5),
		);

		sweep_b.get_transform(&mut transform_b, output.t);
		for i in 0..self.m_shape_b.m_count {
			vertices[i] = b2_mul_transform_by_vec2(transform_b, self.m_shape_b.m_vertices[i]);
		}
		base.g_debug_draw.borrow_mut().draw_polygon(
			&vertices[..self.m_shape_b.m_count],
			B2color::new(0.5, 0.7, 0.9),
		);

		sweep_b.get_transform(&mut transform_b, 1.0);
		for i in 0..self.m_shape_b.m_count {
			vertices[i] = b2_mul_transform_by_vec2(transform_b, self.m_shape_b.m_vertices[i]);
		}
		base.g_debug_draw.borrow_mut().draw_polygon(
			&vertices[..self.m_shape_b.m_count],
			B2color::new(0.9, 0.5, 0.5),
		);

		if false {
			// for (let t: f32 = 0.0 t < 1.0; t += 0.1)
			// {
			// 	sweep_b.get_transform(&transform_b, t);
			// 	for i in 0..m_shape_b.m_count
			// 	{
			// 		vertices[i] = b2_mul_transform_by_vec2(transform_b, m_shape_b.m_vertices[i]);
			// 	}
			// 	g_debug_draw.draw_polygon(vertices, m_shape_b.m_count, B2color::new(0.9, 0.5, 0.5));
			// }
		}
	}
}

//static int testIndex = RegisterTest("Collision", "Time of Impact", TimeOfImpact::create);
