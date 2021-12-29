use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_collision::*;
use box2d_rs::b2_draw::*;
use box2d_rs::b2_dynamic_tree::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_common::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world_callbacks::*;

use rand::Rng;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

const E_ACTOR_COUNT: usize = 128;

// This is a fun demo that shows off the wheel joint
pub(crate) struct DynamicTree<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_world_extent: f32,
	m_proxy_extent: f32,

	m_tree: Rc<RefCell<B2dynamicTree<usize>>>,
	m_query_aabb: B2AABB,
	m_ray_cast_input: B2rayCastInput,
	m_ray_cast_output: B2rayCastOutput,
	m_ray_actor: i32,
	m_actors: [Actor; E_ACTOR_COUNT],
	m_step_count: i32,
	m_automated: bool,
}

#[derive(Default, Copy, Clone, Debug)]
struct Actor {
	pub aabb: B2AABB,
	pub fraction: f32,
	pub overlap: bool,
	pub proxy_id: i32,
}

impl<D: UserDataType> DynamicTree<D> {
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

			m_world_extent: 0.0,
			m_proxy_extent: 0.0,
			m_tree: Rc::new(RefCell::new(B2dynamicTree::new())),
			m_query_aabb: B2AABB::default(),
			m_ray_cast_input: B2rayCastInput {
				p1: B2vec2::default(),
				p2: B2vec2::default(),
				max_fraction: 0.0,
			},
			m_ray_cast_output: B2rayCastOutput::default(),
			m_ray_actor: B2_NULL_NODE,
			m_actors: [Actor::default(); E_ACTOR_COUNT],
			m_step_count: 0,
			m_automated: false,
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
		let mut m_tree = self.m_tree.borrow_mut();

		self.m_world_extent = 15.0;
		self.m_proxy_extent = 0.5;

		//srand(888);

		for i in 0..E_ACTOR_COUNT {
			let aabb = self.get_random_aabb();
			let actor = &mut self.m_actors[i];
			actor.aabb = aabb;
			actor.proxy_id = m_tree.create_proxy(actor.aabb, &i);
		}

		self.m_step_count = 0;

		let h: f32 = self.m_world_extent;
		self.m_query_aabb.lower_bound.set(-3.0, -4.0 + h);
		self.m_query_aabb.upper_bound.set(5.0, 6.0 + h);

		self.m_ray_cast_input.p1.set(-5.0, 5.0 + h);
		self.m_ray_cast_input.p2.set(7.0, -4.0 + h);
		//m_ray_cast_input.p1.set(0.0, 2.0 + h);
		//m_ray_cast_input.p2.set(0.0, -2.0 + h);
		self.m_ray_cast_input.max_fraction = 1.0;

		self.m_automated = false;
	}

	fn get_random_aabb(&self) -> B2AABB {
		let mut aabb = B2AABB::default();
		let w = B2vec2::new(2.0 * self.m_proxy_extent, 2.0 * self.m_proxy_extent);
		//aabb->lower_bound.x = -m_proxy_extent;
		//aabb->lower_bound.y = -m_proxy_extent + m_world_extent;
		aabb.lower_bound.x = random_float_range(-self.m_world_extent, self.m_world_extent);
		aabb.lower_bound.y = random_float_range(0.0, 2.0 * self.m_world_extent);
		aabb.upper_bound = aabb.lower_bound + w;
		return aabb;
	}

	fn move_aabb(&self) -> B2AABB {
		let mut aabb = B2AABB::default();
		let d = B2vec2 {
			x: random_float_range(-0.5, 0.5),
			y: random_float_range(-0.5, 0.5),
		};
		//d.x = 2.0;
		//d.y = 0.0;
		aabb.lower_bound += d;
		aabb.upper_bound += d;

		let c0: B2vec2 = 0.5 * (aabb.lower_bound + aabb.upper_bound);
		let min = B2vec2::new(-self.m_world_extent, 0.0);
		let max = B2vec2::new(self.m_world_extent, 2.0 * self.m_world_extent);
		let c: B2vec2 = b2_clamp_vec2(c0, min, max);

		aabb.lower_bound += c - c0;
		aabb.upper_bound += c - c0;
		return aabb;
	}

	fn create_proxy(&mut self) {
		let mut rng = rand::thread_rng();
		for _i in 0..E_ACTOR_COUNT {
			let j: usize = rng.gen::<usize>() % E_ACTOR_COUNT;

			if self.m_actors[j].proxy_id == B2_NULL_NODE {
				let aabb = self.get_random_aabb();
				let actor = &mut self.m_actors[j];
				actor.aabb = aabb;
				actor.proxy_id = self.m_tree.borrow_mut().create_proxy(actor.aabb, &j);
				return;
			}
		}
	}

	fn destroy_proxy(&mut self) {
		let mut rng = rand::thread_rng();
		for _i in 0..E_ACTOR_COUNT {
			let j: usize = rng.gen::<usize>() % E_ACTOR_COUNT;

			if self.m_actors[j].proxy_id != B2_NULL_NODE {
				let actor = &mut self.m_actors[j];
				self.m_tree.borrow_mut().destroy_proxy(actor.proxy_id);
				actor.proxy_id = B2_NULL_NODE;
				return;
			}
		}
	}

	fn move_proxy(&mut self) {
		let mut rng = rand::thread_rng();
		for _i in 0..E_ACTOR_COUNT {
			let j: usize = rng.gen::<usize>() % E_ACTOR_COUNT;

			if self.m_actors[j].proxy_id == B2_NULL_NODE {
				continue;
			}

			let new_aabb = self.move_aabb();
			let actor = &mut self.m_actors[j];
			let aabb0: B2AABB = actor.aabb;
			actor.aabb = new_aabb;
			let displacement: B2vec2 = actor.aabb.get_center() - aabb0.get_center();
			self.m_tree
				.borrow_mut()
				.move_proxy(actor.proxy_id, actor.aabb, displacement);
			return;
		}
	}

	fn action(&mut self) {
		let mut rng = rand::thread_rng();
		let choice = rng.gen::<usize>() % 20;

		match choice {
			0 => {
				self.create_proxy();
			}
			1 => {
				self.destroy_proxy();
			}
			_ => {
				self.move_proxy();
			}
		}
	}

	fn query(&mut self) {
		let m_tree = self.m_tree.clone();
		let m_tree = m_tree.borrow();
		let m_query_aabb = self.m_query_aabb;
		m_tree.query(
			|proxy_id: i32| -> bool {
				let actor = m_tree.get_user_data(proxy_id).unwrap();
				let actor = &mut self.m_actors[actor as usize];
				actor.overlap = b2_test_overlap(m_query_aabb, actor.aabb);
				return true;
			},
			m_query_aabb,
		);

		for i in 0..E_ACTOR_COUNT {
			if self.m_actors[i].proxy_id == B2_NULL_NODE {
				continue;
			}

			let overlap: bool = b2_test_overlap(self.m_query_aabb, self.m_actors[i].aabb);
			b2_not_used(overlap);
			b2_assert(overlap == self.m_actors[i].overlap);
		}
	}

	fn ray_cast(&mut self) {
		self.m_ray_actor = B2_NULL_NODE;
		let mut input = self.m_ray_cast_input;
		let m_tree = self.m_tree.clone();
		let m_tree = m_tree.borrow();

		// Ray cast against the dynamic tree.
		m_tree.ray_cast(
			|input: &B2rayCastInput, proxy_id: i32| -> f32 {
				let actor = m_tree.get_user_data(proxy_id).unwrap();

				let mut output = B2rayCastOutput::default();
				let hit: bool = self.m_actors[actor].aabb.ray_cast(&mut output, input);

				if hit {
					self.m_ray_cast_output = output;
					self.m_ray_actor = actor as i32;
					self.m_actors[self.m_ray_actor as usize].fraction = output.fraction;
					return output.fraction;
				}

				return input.max_fraction;
			},
			&input,
		);

		// Brute force ray cast.
		let mut brute_actor = B2_NULL_NODE;
		let mut brute_output = B2rayCastOutput::default();
		for i in 0..E_ACTOR_COUNT {
			if self.m_actors[i].proxy_id == B2_NULL_NODE {
				continue;
			}

			let mut output = B2rayCastOutput::default();
			let hit: bool = self.m_actors[i].aabb.ray_cast(&mut output, &input);
			if hit {
				brute_actor = i as i32;
				brute_output = output;
				input.max_fraction = output.fraction;
			}
		}

		if brute_actor != B2_NULL_NODE {
			b2_assert(brute_output.fraction == self.m_ray_cast_output.fraction);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for DynamicTree<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::A) => {
					self.m_automated = !self.m_automated;
				}
				Some(VirtualKeyCode::C) => {
					self.create_proxy();
				}
				Some(VirtualKeyCode::D) => {
					self.destroy_proxy();
				}
				Some(VirtualKeyCode::M) => {
					self.move_proxy();
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
		b2_not_used(settings);

		self.m_ray_actor = B2_NULL_NODE;
		for i in 0..E_ACTOR_COUNT {
			self.m_actors[i].fraction = 1.0;
			self.m_actors[i].overlap = false;
		}

		if self.m_automated == true {
			let action_count: i32 = b2_max(1, E_ACTOR_COUNT as i32 >> 2);

			for _i in 0..action_count {
				self.action();
			}
		}

		self.query();
		self.ray_cast();

		let mut base = self.base.borrow_mut();

		for i in 0..E_ACTOR_COUNT {
			let actor = &mut self.m_actors[i];
			if actor.proxy_id == B2_NULL_NODE {
				continue;
			}

			let mut c = B2color::new(0.9, 0.9, 0.9);
			if i as i32 == self.m_ray_actor && actor.overlap {
				c.set(0.9, 0.6, 0.6);
			} else if i as i32 == self.m_ray_actor {
				c.set(0.6, 0.9, 0.6);
			} else if actor.overlap {
				c.set(0.6, 0.6, 0.9);
			}

			base.g_debug_draw.borrow_mut().draw_aabb(actor.aabb, c);
		}

		let c = B2color::new(0.7, 0.7, 0.7);
		base.g_debug_draw
			.borrow_mut()
			.draw_aabb(self.m_query_aabb, c);

		base.g_debug_draw.borrow_mut().draw_segment(
			self.m_ray_cast_input.p1,
			self.m_ray_cast_input.p2,
			c,
		);

		let c1 = B2color::new(0.2, 0.9, 0.2);
		let c2 = B2color::new(0.9, 0.2, 0.2);
		base.g_debug_draw
			.borrow_mut()
			.draw_point(self.m_ray_cast_input.p1, 6.0, c1);
		base.g_debug_draw
			.borrow_mut()
			.draw_point(self.m_ray_cast_input.p2, 6.0, c2);

		if self.m_ray_actor != B2_NULL_NODE {
			let cr = B2color::new(0.2, 0.2, 0.9);
			let p: B2vec2 = self.m_ray_cast_input.p1
				+ self.m_actors[self.m_ray_actor as usize].fraction
					* (self.m_ray_cast_input.p2 - self.m_ray_cast_input.p1);
			base.g_debug_draw.borrow_mut().draw_point(p, 6.0, cr);
		}

		{
			let height: i32 = self.m_tree.borrow().get_height();
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				&format!("dynamic tree height = {0}", height),
			);
			base.m_text_line += base.m_text_increment;
		}

		self.m_step_count += 1;

		base.g_debug_draw
			.borrow_mut()
			.flush(display, target, *camera);
	}
}
