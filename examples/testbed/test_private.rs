use super::draw::*;
use super::settings::*;
use super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_collision::*;
use box2d_rs::b2_contact::*;
use box2d_rs::b2_draw::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_common::B2_MAX_MANIFOLD_POINTS;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_shape::*;
use box2d_rs::b2_time_step::*;
use box2d_rs::b2_world::*;
use box2d_rs::joints::b2_mouse_joint::*;
use box2d_rs::shapes::b2_circle_shape::*;

use glium::backend::Facade;

use std::cell::RefCell;
use std::rc::Rc;

pub(crate) fn say_goodbye_joint<D: UserDataType>(self_: &mut Test<D>, joint: B2jointPtr<D>) {
	if let Some(ref m_mouse_joint) = self_.m_mouse_joint {
		let m_mouse_joint: B2jointPtr<D> = m_mouse_joint.clone();
		if Rc::ptr_eq(&m_mouse_joint, &joint) {
			self_.m_mouse_joint = None;
			return;
		}
	}

	//self_.joint_destroyed(joint);
}

pub(crate) fn pre_solve<D: UserDataType>(
	self_: &mut Test<D>,
	contact: &dyn B2contactDynTrait<D>,
	old_manifold: &B2manifold,
) {
	let manifold = contact.get_base().get_manifold();

	if manifold.point_count == 0 {
		return;
	}

	let fixture_a = contact.get_base().get_fixture_a();
	let fixture_b = contact.get_base().get_fixture_b();

	let mut state1 = <[B2pointState; B2_MAX_MANIFOLD_POINTS]>::default();
	let mut state2 = <[B2pointState; B2_MAX_MANIFOLD_POINTS]>::default();
	b2_get_point_states(&mut state1, &mut state2, old_manifold, manifold);

	let mut world_manifold = B2worldManifold::default();
	contact.get_base().get_world_manifold(&mut world_manifold);

	for i in 0..manifold.point_count {
		self_.m_points.push(ContactPoint {
			fixture_a: fixture_a.clone(),
			fixture_b: fixture_b.clone(),
			position: world_manifold.points[i],
			normal: world_manifold.normal,
			state: state2[i],
			normal_impulse: manifold.points[i].normal_impulse,
			tangent_impulse: manifold.points[i].tangent_impulse,
			separation: world_manifold.separations[i],
		});
	}
}

pub(crate) fn draw_title<D: UserDataType>(self_: &mut Test<D>, ui: &imgui::Ui<'_>, string: &str) {
	self_
		.g_debug_draw
		.borrow_mut()
		.draw_string(ui, B2vec2::new(5.0, 5.0), string);
	self_.m_text_line = 26;
}

pub(crate) fn mouse_down<D: UserDataType>(self_: &mut Test<D>, p: B2vec2) {
	self_.m_mouse_world = p;

	if self_.m_mouse_joint.is_some() {
		return;
	}

	// Make a small box.
	let mut aabb = B2AABB::default();
	let mut d = B2vec2::default();
	d.set(0.001, 0.001);
	aabb.lower_bound = p - d;
	aabb.upper_bound = p + d;

	// query the world for overlapping shapes.
	let mut fixture_found = None;
	self_.m_world.borrow().query_aabb(
		|fixture: FixturePtr<D>| -> bool {
			let body = fixture.borrow().get_body();
			if body.borrow().get_type() == B2bodyType::B2DynamicBody {
				let inside: bool = fixture.borrow().test_point(p);
				if inside {
					fixture_found = Some(fixture);

					// We are done, terminate the query.
					return false;
				}
			}

			// Continue the query.
			return true;
		},
		aabb,
	);

	if let Some(fixture) = fixture_found {
		let body_ptr = fixture.borrow().get_body();
		let mut jd;
		{
			let frequencyHz: f32 = 5.0;
			let dampingRatio: f32 = 0.7;
	
			jd = B2mouseJointDef {
				base: B2jointDef {
					jtype: B2jointType::EMouseJoint,
					body_a: Some(self_.m_ground_body.clone()),
					body_b: Some(body_ptr.clone()),
					..Default::default()
				},
				target: p,
				max_force: 1000.0 * body_ptr.borrow().get_mass(),
				..Default::default()
			};
			b2_linear_stiffness(&mut jd.stiffness, &mut jd.damping, frequencyHz, dampingRatio, jd.base.body_a.clone().unwrap(), jd.base.body_b.clone().unwrap());
		}

		self_.m_mouse_joint = Some(self_.m_world.borrow_mut().create_joint(&B2JointDefEnum::<D>::MouseJoint(jd)));
		body_ptr.borrow_mut().set_awake(true);
	}
}

pub(crate) fn spawn_bomb<D: UserDataType>(self_: &mut Test<D>, world_pt: B2vec2) {
	self_.m_bomb_spawn_point = world_pt;
	self_.m_bomb_spawning = true;
}

pub(crate) fn complete_bomb_spawn<D: UserDataType>(self_: &mut Test<D>, p: B2vec2) {
	if self_.m_bomb_spawning == false {
		return;
	}

	const MULTIPLIER: f32 = 30.0;
	let mut vel: B2vec2 = self_.m_bomb_spawn_point - p;
	vel *= MULTIPLIER;
	launch_bomb(self_, self_.m_bomb_spawn_point, vel);
	self_.m_bomb_spawning = false;
}

pub(crate) fn shift_mouse_down<D: UserDataType>(self_: &mut Test<D>, p: B2vec2) {
	self_.m_mouse_world = p;

	if self_.m_mouse_joint.is_some() {
		return;
	}

	spawn_bomb(self_, p);
}

pub(crate) fn mouse_up<D: UserDataType>(self_: &mut Test<D>, p: B2vec2) {
	if let Some(ref mouse_joint) = self_.m_mouse_joint {
		self_
			.m_world
			.borrow_mut()
			.destroy_joint(mouse_joint.clone());
		self_.m_mouse_joint = None;
	}

	if self_.m_bomb_spawning {
		complete_bomb_spawn(self_, p);
	}
}

pub(crate) fn mouse_move<D: UserDataType>(self_: &mut Test<D>, p: B2vec2) {
	self_.m_mouse_world = p;

	if let Some(ref mouse_joint) = self_.m_mouse_joint {
		match mouse_joint.borrow_mut().as_derived_mut() {
			JointAsDerivedMut::EMouseJoint(mouse_joint) => {
				mouse_joint.set_target(p);
			}
			_ => panic!(),
		}
	}
}

//TODO_humman bomb launch
pub(crate) fn launch_bomb_rand<D: UserDataType>(self_: &mut Test<D>) {
	let p = B2vec2::new(random_float_range(-15.0, 15.0), 30.0);
	let v: B2vec2 = -5.0 * p;
	launch_bomb(self_, p, v);
}

pub(crate) fn launch_bomb<D: UserDataType>(
	self_: &mut Test<D>,
	position: B2vec2,
	velocity: B2vec2,
) {
	if let Some(ref bomb) = self_.m_bomb {
		self_.m_world.borrow_mut().destroy_body(bomb.clone());
		self_.m_bomb = None;
	}

	let bd = B2bodyDef {
		body_type: B2bodyType::B2DynamicBody,
		position: position,
		bullet: true,
		..Default::default()
	};
	let m_bomb_ptr = B2world::create_body(self_.m_world.clone(), &bd);
	self_.m_bomb = Some(m_bomb_ptr.clone());
	m_bomb_ptr.borrow_mut().set_linear_velocity(velocity);

	let circle = B2circleShape {
		base: B2Shape {
			m_type: B2ShapeType::ECircle,
			m_radius: 0.3,
			..Default::default()
		},
		..Default::default()
	};

	let mut fd = B2fixtureDef::default();
	fd.shape = Some(Rc::new(RefCell::from(circle)));
	fd.density = 20.0;
	fd.restitution = 0.0;

	// let min_v: B2vec2 = position - B2vec2::new(0.3, 0.3);
	// let max_v: B2vec2 = position + B2vec2::new(0.3, 0.3);

	// let aabb = B2AABB {
	// 	lower_bound: min_v,
	// 	upper_bound: max_v,
	// };
	B2body::create_fixture(m_bomb_ptr, &fd);
}

pub(crate) fn step<D: UserDataType, F: Facade>(
	self_: TestBasePtr<D>,
	ui: &imgui::Ui<'_>,
	display: &F,
	target: &mut glium::Frame,
	settings: &mut Settings,
	g_camera: Camera,
) {
	let mut time_step: f32 = if settings.m_hertz > 0.0 {
		1.0 / settings.m_hertz
	} else {
		0.0
	};

	{
		let mut self_ = self_.borrow_mut();

		if settings.m_pause {
			if settings.m_single_step {
				settings.m_single_step = false;
			} else {
				time_step = 0.0;
			}

			self_.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				"****PAUSED****",
			);
			self_.m_text_line += self_.m_text_increment;
		}

		let mut flags: B2drawShapeFlags = Default::default();
		flags.set(B2drawShapeFlags::SHAPE_BIT, settings.m_draw_shapes);
		flags.set(B2drawShapeFlags::JOINT_BIT, settings.m_draw_joints);
		flags.set(B2drawShapeFlags::AABB_BIT, settings.m_draw_aabbs);
		flags.set(B2drawShapeFlags::CENTER_OF_MASS_BIT, settings.m_draw_coms);
		self_
			.g_debug_draw
			.borrow_mut()
			.get_base_mut()
			.set_flags(flags);
	}

	{
		let m_world = {
			let mut self_ = self_.borrow_mut();
			self_.m_points.clear();
			self_.m_world.clone()
		};

		let mut m_world = m_world.borrow_mut();

		m_world.set_allow_sleeping(settings.m_enable_sleep);
		m_world.set_warm_starting(settings.m_enable_warm_starting);
		m_world.set_continuous_physics(settings.m_enable_continuous);
		m_world.set_sub_stepping(settings.m_enable_sub_stepping);

		m_world.step(
			time_step,
			settings.m_velocity_iterations,
			settings.m_position_iterations,
		);

		m_world.debug_draw();
	}
	{
		let mut self_ = self_.borrow_mut();
		self_
			.g_debug_draw
			.borrow_mut()
			.flush(display, target, g_camera);

		if time_step > 0.0 {
			self_.m_step_count += 1;
		}

		if settings.m_draw_stats {
			let m_world = self_.m_world.clone();
			let m_world = m_world.borrow();
			let body_count: usize = m_world.get_body_count();
			let contact_count: usize = m_world.get_contact_count();
			let joint_count: usize = m_world.get_joint_count();
			self_.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				&format!(
					"bodies/contacts/joints = {0}/{1}/{2}",
					body_count, contact_count, joint_count
				),
			);
			self_.m_text_line += self_.m_text_increment;

			let proxy_count: i32 = m_world.get_proxy_count();
			let height: i32 = m_world.get_tree_height();
			let balance: i32 = m_world.get_tree_balance();
			let quality: f32 = m_world.get_tree_quality();
			self_.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				&format!(
					"proxies/height/balance/quality = {0}/{1}/{2}/{3}",
					proxy_count, height, balance, quality
				),
			);
			self_.m_text_line += self_.m_text_increment;
		}

		// Track maximum profile times
		{
			let p: B2Profile = self_.m_world.borrow().get_profile();
			self_.m_max_profile.step = b2_max(self_.m_max_profile.step, p.step);
			self_.m_max_profile.collide = b2_max(self_.m_max_profile.collide, p.collide);
			self_.m_max_profile.solve = b2_max(self_.m_max_profile.solve, p.solve);
			self_.m_max_profile.solve_init = b2_max(self_.m_max_profile.solve_init, p.solve_init);
			self_.m_max_profile.solve_velocity =
				b2_max(self_.m_max_profile.solve_velocity, p.solve_velocity);
			self_.m_max_profile.solve_position =
				b2_max(self_.m_max_profile.solve_position, p.solve_position);
			self_.m_max_profile.solve_toi = b2_max(self_.m_max_profile.solve_toi, p.solve_toi);
			self_.m_max_profile.broadphase = b2_max(self_.m_max_profile.broadphase, p.broadphase);

			self_.m_total_profile.step += p.step;
			self_.m_total_profile.collide += p.collide;
			self_.m_total_profile.solve += p.solve;
			self_.m_total_profile.solve_init += p.solve_init;
			self_.m_total_profile.solve_velocity += p.solve_velocity;
			self_.m_total_profile.solve_position += p.solve_position;
			self_.m_total_profile.solve_toi += p.solve_toi;
			self_.m_total_profile.broadphase += p.broadphase;
		}

		if settings.m_draw_profile {
			let p: B2Profile = self_.m_world.borrow().get_profile();

			let mut ave_profile = B2Profile::default();
			if self_.m_step_count > 0 {
				let scale: f32 = 1.0 / self_.m_step_count as f32;
				ave_profile.step = scale * self_.m_total_profile.step;
				ave_profile.collide = scale * self_.m_total_profile.collide;
				ave_profile.solve = scale * self_.m_total_profile.solve;
				ave_profile.solve_init = scale * self_.m_total_profile.solve_init;
				ave_profile.solve_velocity = scale * self_.m_total_profile.solve_velocity;
				ave_profile.solve_position = scale * self_.m_total_profile.solve_position;
				ave_profile.solve_toi = scale * self_.m_total_profile.solve_toi;
				ave_profile.broadphase = scale * self_.m_total_profile.broadphase;
			}

			let g_debug_draw = self_.g_debug_draw.clone();
			let g_debug_draw = g_debug_draw.borrow();
			//let m_text_line = &mut self_.m_text_line;
			//let m_text_increment = self_.m_text_increment;

			g_debug_draw.draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				&format!(
					"step [ave] (max) = {0:5.2} {1:6.2} {2:6.2}",
					p.step, ave_profile.step, self_.m_max_profile.step
				),
			);
			self_.m_text_line += self_.m_text_increment;
			g_debug_draw.draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				&format!(
					"collide [ave] (max) = {0:5.2} {1:6.2} {2:6.2}",
					p.collide, ave_profile.collide, self_.m_max_profile.collide
				),
			);
			self_.m_text_line += self_.m_text_increment;
			g_debug_draw.draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				&format!(
					"solve [ave] (max) = {0:5.2} {1:6.2} {2:6.2}",
					p.solve, ave_profile.solve, self_.m_max_profile.solve
				),
			);
			self_.m_text_line += self_.m_text_increment;
			g_debug_draw.draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				&format!(
					"solve init [ave] (max) = {0:5.2} {1:6.2} {2:6.2}",
					p.solve_init, ave_profile.solve_init, self_.m_max_profile.solve_init
				),
			);
			self_.m_text_line += self_.m_text_increment;
			g_debug_draw.draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				&format!(
					"solve velocity [ave] (max) = {0:5.2} {1:6.2} {2:6.2}",
					p.solve_velocity,
					ave_profile.solve_velocity,
					self_.m_max_profile.solve_velocity
				),
			);
			self_.m_text_line += self_.m_text_increment;
			g_debug_draw.draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				&format!(
					"solve position [ave] (max) = {0:5.2} {1:6.2} {2:6.2}",
					p.solve_position,
					ave_profile.solve_position,
					self_.m_max_profile.solve_position
				),
			);
			self_.m_text_line += self_.m_text_increment;
			g_debug_draw.draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				&format!(
					"solve_toi [ave] (max) = {0:5.2} {1:6.2} {2:6.2}",
					p.solve_toi, ave_profile.solve_toi, self_.m_max_profile.solve_toi
				),
			);
			self_.m_text_line += self_.m_text_increment;
			g_debug_draw.draw_string(
				ui,
				B2vec2::new(5.0, self_.m_text_line as f32),
				&format!(
					"broad-phase [ave] (max) = {0:5.2} {1:6.2} {2:6.2}",
					p.broadphase, ave_profile.broadphase, self_.m_max_profile.broadphase
				),
			);
			self_.m_text_line += self_.m_text_increment;
		}

		if self_.m_bomb_spawning {
			let mut g_debug_draw = self_.g_debug_draw.borrow_mut();

			let c = B2color::new(0.0, 0.0, 1.0);
			g_debug_draw.draw_point(self_.m_bomb_spawn_point, 4.0, c);

			let c = B2color::new(0.8, 0.8, 0.8);
			g_debug_draw.draw_segment(self_.m_mouse_world, self_.m_bomb_spawn_point, c);
		}

		if settings.m_draw_contact_points {
			const K_IMPULSE_SCALE: f32 = 0.1;
			const K_AXIS_SCALE: f32 = 0.3;

			let mut g_debug_draw = self_.g_debug_draw.borrow_mut();

			for point in &self_.m_points {
				if point.state == B2pointState::B2AddState {
					// Add
					g_debug_draw.draw_point(point.position, 10.0, B2color::new(0.3, 0.95, 0.3));
				} else if point.state == B2pointState::B2PersistState {
					// Persist
					g_debug_draw.draw_point(point.position, 5.0, B2color::new(0.3, 0.3, 0.95));
				}

				if settings.m_draw_contact_normals == true {
					let p1: B2vec2 = point.position;
					let p2: B2vec2 = p1 + K_AXIS_SCALE * point.normal;
					g_debug_draw.draw_segment(p1, p2, B2color::new(0.9, 0.9, 0.9));
				} else if settings.m_draw_contact_impulse == true {
					let p1: B2vec2 = point.position;
					let p2: B2vec2 = p1 + K_IMPULSE_SCALE * point.normal_impulse * point.normal;
					g_debug_draw.draw_segment(p1, p2, B2color::new(0.9, 0.9, 0.3));
				}

				if settings.m_draw_friction_impulse == true {
					let tangent: B2vec2 = b2_cross_vec_by_scalar(point.normal, 1.0);
					let p1: B2vec2 = point.position;
					let p2: B2vec2 = p1 + K_IMPULSE_SCALE * point.tangent_impulse * tangent;
					g_debug_draw.draw_segment(p1, p2, B2color::new(0.9, 0.9, 0.3));
				}
			}
		}
	}
}

// pub(crate) fn shift_origin<D: UserDataType>(self_: &mut Test<D>, new_origin: B2vec2) {
// 	self_.m_world.borrow_mut().shift_origin(new_origin);
// }
