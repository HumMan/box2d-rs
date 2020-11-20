// This test demonstrates how to use the world ray-cast feature.
// NOTE: we are intentionally filtering one of the polygons, therefore
// the ray will always miss one type of polygon.

use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;

use box2d_rs::b2_draw::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct RayCast<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_bodies: Vec<BodyPtr<D>>,
	m_user_data: Vec<i32>,
	m_polygons: [B2polygonShape; 4],
	m_circle: B2circleShape,
	m_edge: B2edgeShape,
	m_angle: f32,
	m_mode: Mode,
}

#[derive(Debug, Copy, Clone, PartialEq)]
enum Mode {
	EClosest,
	EAny,
	EMultiple,
}

impl RayCast<UserDataTypes> {
	pub const E_MAX_BODIES: usize = 256;

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

			m_bodies: Vec::with_capacity(Self::E_MAX_BODIES),
			m_user_data: Vec::with_capacity(Self::E_MAX_BODIES),
			m_polygons: Default::default(),
			m_circle: B2circleShape::default(),
			m_edge: B2edgeShape::default(),
			m_angle: 0.0,
			m_mode: Mode::EClosest,
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
			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let vertices: [B2vec2; 3] = [
				B2vec2::new(-0.5, 0.0),
				B2vec2::new(0.5, 0.0),
				B2vec2::new(0.0, 1.5),
			];
			self.m_polygons[0].set(&vertices);
		}

		{
			let vertices: [B2vec2; 3] = [
				B2vec2::new(-0.1, 0.0),
				B2vec2::new(0.1, 0.0),
				B2vec2::new(0.0, 1.5),
			];
			self.m_polygons[1].set(&vertices);
		}

		{
			let w: f32 = 1.0;
			let b: f32 = w / (2.0 + b2_sqrt(2.0));
			let s: f32 = b2_sqrt(2.0) * b;

			let vertices: [B2vec2; 8] = [
				B2vec2::new(0.5 * s, 0.0),
				B2vec2::new(0.5 * w, b),
				B2vec2::new(0.5 * w, b + s),
				B2vec2::new(0.5 * s, w),
				B2vec2::new(-0.5 * s, w),
				B2vec2::new(-0.5 * w, b + s),
				B2vec2::new(-0.5 * w, b),
				B2vec2::new(-0.5 * s, 0.0),
			];

			self.m_polygons[2].set(&vertices);
		}

		{
			self.m_polygons[3].set_as_box(0.5, 0.5);
		}

		{
			self.m_circle.base.m_radius = 0.5;
		}

		{
			self.m_edge
				.set_two_sided(B2vec2::new(-1.0, 0.0), B2vec2::new(1.0, 0.0));
		}
	}

	fn create(&mut self, index: i32) {
		let mut bd = B2bodyDef::default();

		let x: f32 = random_float_range(-10.0, 10.0);
		let y: f32 = random_float_range(10.0, 20.0);
		bd.position.set(x, y);
		bd.angle = random_float_range(-B2_PI, B2_PI);

		//let m_body_index = self.m_bodies.len();

		bd.user_data = Some(FixtureData::Int(index));

		self.m_user_data.push(index);

		if index == 4 {
			bd.angular_damping = 0.02;
		}

		let m_world = self.base.borrow().m_world.clone();
		let new_body = B2world::create_body(m_world.clone(), &bd);

		if index < 4 {
			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(self.m_polygons[index as usize])));
			fd.friction = 0.3;
			B2body::create_fixture(new_body.clone(), &fd);
		} else if index < 5 {
			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(self.m_circle)));
			fd.friction = 0.3;

			B2body::create_fixture(new_body.clone(), &fd);
		} else {
			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(self.m_edge)));
			fd.friction = 0.3;

			B2body::create_fixture(new_body.clone(), &fd);
		}

		self.m_bodies.push(new_body);
	}

	fn destroy_body(&mut self) {
		if let Some(body) = self.m_bodies.pop() {
			self.base.borrow().m_world.borrow_mut().destroy_body(body);
		}
	}
}

impl<F: Facade> TestDyn<UserDataTypes, F> for RayCast<UserDataTypes> {
	fn get_base(&self) -> TestBasePtr<UserDataTypes> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::Key1) => {
					self.create(0);
				}
				Some(VirtualKeyCode::Key2) => {
					self.create(1);
				}
				Some(VirtualKeyCode::Key3) => {
					self.create(2);
				}
				Some(VirtualKeyCode::Key4) => {
					self.create(3);
				}
				Some(VirtualKeyCode::Key5) => {
					self.create(4);
				}
				Some(VirtualKeyCode::Key6) => {
					self.create(5);
				}
				Some(VirtualKeyCode::D) => {
					self.destroy_body();
				}
				Some(VirtualKeyCode::M) => {
					if self.m_mode == Mode::EClosest {
						self.m_mode = Mode::EAny;
					} else if self.m_mode == Mode::EAny {
						self.m_mode = Mode::EMultiple;
					} else if self.m_mode == Mode::EMultiple {
						self.m_mode = Mode::EClosest;
					}
				}
				_ => (),
			}
		}
	}
	fn step(
		&mut self,
		ui: &imgui::Ui<'_>,
		display: &F,
		target: &mut glium::Frame,
		settings: &mut Settings,
		camera: &mut Camera,
	) {
		let advance_ray: bool = settings.m_pause == false || settings.m_single_step;
		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		let m_world = self.base.borrow().m_world.clone();
		let mut base = self.base.borrow_mut();

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			"Press 1-6 to drop stuff, m to change the mode",
		);
		base.m_text_line += base.m_text_increment;

		match self.m_mode {
			Mode::EClosest => {
				base.g_debug_draw.borrow().draw_string(
					ui,
					B2vec2::new(5.0, base.m_text_line as f32),
					"Ray-cast mode: closest - find closest fixture along the ray",
				);
			}
			Mode::EAny => {
				base.g_debug_draw.borrow().draw_string(
					ui,
					B2vec2::new(5.0, base.m_text_line as f32),
					"Ray-cast mode: any - check for obstruction",
				);
			}
			Mode::EMultiple => {
				base.g_debug_draw.borrow().draw_string(
					ui,
					B2vec2::new(5.0, base.m_text_line as f32),
					"Ray-cast mode: multiple - gather multiple fixtures",
				);
			}
		}

		base.m_text_line += base.m_text_increment;

		let l: f32 = 11.0;
		let point1 = B2vec2::new(0.0, 10.0);
		let d = B2vec2::new(l * f32::cos(self.m_angle), l * f32::sin(self.m_angle));
		let point2: B2vec2 = point1 + d;

		match self.m_mode {
			Mode::EClosest => {
				let mut ray_cast_collide = None;

				// This callback finds the closest hit. Polygon 0 is filtered.
				m_world.borrow().ray_cast(
					|fixture: FixturePtr<UserDataTypes>,
					 point: B2vec2,
					 normal: B2vec2,
					 fraction: f32|
					 -> f32 {
						let body: BodyPtr<UserDataTypes> = fixture.borrow().get_body();
						if let Some(user_data) = body.borrow().get_user_data() {
							match user_data {
								FixtureData::Int(id) => {
									if id == 0 {
										// By returning -1, we instruct the calling code to ignore this fixture and
										// continue the ray-cast to the next fixture.
										return -1.0;
									}
								}
							}
						}

						ray_cast_collide = Some((point, normal));

						// By returning the current fraction, we instruct the calling code to clip the ray and
						// continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
						// are reported in order. However, by clipping, we can always get the closest fixture.
						return fraction;
					},
					point1,
					point2,
				);
				if let Some((point, normal)) = ray_cast_collide {
					base.g_debug_draw.borrow_mut().draw_point(
						point,
						5.0,
						B2color::new(0.4, 0.9, 0.4),
					);

					base.g_debug_draw.borrow_mut().draw_segment(
						point1,
						point,
						B2color::new(0.8, 0.8, 0.8),
					);

					let head: B2vec2 = point + 0.5 * normal;
					base.g_debug_draw.borrow_mut().draw_segment(
						point,
						head,
						B2color::new(0.9, 0.9, 0.4),
					);
				} else {
					base.g_debug_draw.borrow_mut().draw_segment(
						point1,
						point2,
						B2color::new(0.8, 0.8, 0.8),
					);
				}
			}
			Mode::EAny => {
				let mut ray_cast_collide = None;

				// This callback finds any hit. Polygon 0 is filtered. For this type of query we are usually
				// just checking for obstruction, so the actual fixture and hit point are irrelevant.
				m_world.borrow().ray_cast(
					|fixture: FixturePtr<UserDataTypes>,
					 point: B2vec2,
					 normal: B2vec2,
					 _fraction: f32|
					 -> f32 {
						let body: BodyPtr<UserDataTypes> = fixture.borrow().get_body();
						if let Some(user_data) = body.borrow().get_user_data() {
							match user_data {
								FixtureData::Int(id) => {
									if id == 0 {
										// By returning -1, we instruct the calling code to ignore this fixture and
										// continue the ray-cast to the next fixture.
										return -1.0;
									}
								}
							}
						}

						ray_cast_collide = Some((point, normal));

						// At this point we have a hit, so we know the ray is obstructed.
						// By returning 0, we instruct the calling code to terminate the ray-cast.
						return 0.0;
					},
					point1,
					point2,
				);
				if let Some((point, normal)) = ray_cast_collide {
					base.g_debug_draw.borrow_mut().draw_point(
						point,
						5.0,
						B2color::new(0.4, 0.9, 0.4),
					);

					base.g_debug_draw.borrow_mut().draw_segment(
						point1,
						point,
						B2color::new(0.8, 0.8, 0.8),
					);

					let head: B2vec2 = point + 0.5 * normal;
					base.g_debug_draw.borrow_mut().draw_segment(
						point,
						head,
						B2color::new(0.9, 0.9, 0.4),
					);
				} else {
					base.g_debug_draw.borrow_mut().draw_segment(
						point1,
						point2,
						B2color::new(0.8, 0.8, 0.8),
					);
				}
			}
			Mode::EMultiple => {
				let mut result_list = Vec::<(B2vec2, B2vec2)>::with_capacity(3);
				// // This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
				// // The fixtures are not necessary reported in order, so we might not capture
				// // the closest fixture.
				m_world.borrow().ray_cast(
					|fixture: FixturePtr<UserDataTypes>,
					 point: B2vec2,
					 normal: B2vec2,
					 _fraction: f32|
					 -> f32 {
						let body: BodyPtr<UserDataTypes> = fixture.borrow().get_body();
						if let Some(user_data) = body.borrow().get_user_data() {
							match user_data {
								FixtureData::Int(id) => {
									if id == 0 {
										// By returning -1, we instruct the calling code to ignore this fixture and
										// continue the ray-cast to the next fixture.
										return -1.0;
									}
								}
							}
						}

						result_list.push((point, normal));

						// By returning 1, we instruct the caller to continue without clipping the ray.
						return 1.0;
					},
					point1,
					point2,
				);

				base.g_debug_draw.borrow_mut().draw_segment(
					point1,
					point2,
					B2color::new(0.8, 0.8, 0.8),
				);

				for i in result_list {
					let p: B2vec2 = i.0;
					let n: B2vec2 = i.1;

					base.g_debug_draw
						.borrow_mut()
						.draw_point(p, 5.0, B2color::new(0.4, 0.9, 0.4));
					base.g_debug_draw.borrow_mut().draw_segment(
						point1,
						p,
						B2color::new(0.8, 0.8, 0.8),
					);

					let head: B2vec2 = p + 0.5 * n;
					base.g_debug_draw.borrow_mut().draw_segment(
						p,
						head,
						B2color::new(0.9, 0.9, 0.4),
					);
				}
			}
		}

		if advance_ray {
			self.m_angle += 0.25 * B2_PI / 180.0;
		}

		if false {
			// This case was failing.
			// {
			// 	//B2vec2 vertices[4];
			// 	//vertices[0].set(-22.875, -3.0);
			// 	//vertices[1].set(22.875, -3.0);
			// 	//vertices[2].set(22.875, 3.0);
			// 	//vertices[3].set(-22.875, 3.0);

			// 	let mut shape = B2polygonShape::default();
			// 	//shape.set(vertices, 4);
			// 	shape.set_as_box(22.875, 3.0);

			// 	let input = B2rayCastInput {
			// 		p1: B2vec2::new(10.2725,1.71372),
			// 		p2: B2vec2::new(10.2353,2.21807),
			// 		//max_fraction: 0.567623,
			// 		max_fraction: 0.56762173,
			// 	};

			// 	let mut xf = B2Transform::default();
			// 	xf.set_identity();
			// 	xf.p.set(23.0, 5.0);

			// 	let mut output = B2rayCastOutput::default();
			// 	let hit: bool;
			// 	hit = shape.RayCast(&mut output, &input, xf, 0);

			// 	let color = B2color::new(1.0, 1.0, 1.0);
			// 	let mut vs = <[B2vec2;4]>::default();
			// 	for i in 0..4
			// 	{
			// 		vs[i] = b2_mul_transform_by_vec2(xf, shape.m_vertices[i]);
			// 	}

			// 	base.g_debug_draw
			// 	.borrow_mut()
			// 	.draw_polygon(&vs, color);
			// 	base.g_debug_draw
			// 	.borrow_mut()
			// 	.draw_segment(input.p1, input.p2, color);
			// }
		}
	}
}

//static int testIndex = RegisterTest("Collision", "Ray Cast", RayCast::create);
