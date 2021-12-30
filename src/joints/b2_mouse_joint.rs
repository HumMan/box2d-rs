use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;

use crate::private::dynamics::joints::b2_mouse_joint as private;

impl<D: UserDataType> Default for B2mouseJointDef<D> {
	fn default() -> Self {
		return Self {
			base: B2jointDef {
				jtype: B2jointType::EMouseJoint,
				..Default::default()
			},
			target: B2vec2::zero(),
			max_force: 0.0,
			stiffness: 0.0,
			damping: 0.0,
		};
	}
}

impl<D: UserDataType> ToDerivedJoint<D> for B2mouseJoint<D> {
	fn as_derived(&self) -> JointAsDerived<D> {
		return JointAsDerived::EMouseJoint(self);
	}
	fn as_derived_mut(&mut self) -> JointAsDerivedMut<D> {
		return JointAsDerivedMut::EMouseJoint(self);
	}
}

/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
#[derive(Clone)]
pub struct B2mouseJointDef<D: UserDataType> {
	pub base: B2jointDef<D>,

	/// The initial world target point. This is assumed
	/// to coincide with the body anchor initially.
	pub target: B2vec2,

	/// The maximum constraint force that can be exerted
	/// to move the candidate body. Usually you will express
	/// as some multiple of the weight (multiplier * mass * gravity).
	pub max_force: f32,

	/// The linear stiffness in N/m
	pub stiffness: f32,

	/// The linear damping in N*s/m
	pub damping: f32,
}

impl<D: UserDataType> B2jointTraitDyn<D> for B2mouseJoint<D> {
	fn get_base(&self) -> &B2joint<D> {
		return &self.base;
	}
	fn get_base_mut(&mut self) -> &mut B2joint<D> {
		return &mut self.base;
	}
	fn get_anchor_a(&self) -> B2vec2 {
		return self.m_target_a;
	}
	fn get_anchor_b(&self) -> B2vec2 {
		return self
			.base
			.m_body_b
			.borrow()
			.get_world_point(self.m_local_anchor_b);
	}

	/// Get the reaction force given the inverse time step.
	/// Unit is n.
	fn get_reaction_force(&self, inv_dt: f32) -> B2vec2 {
		return inv_dt * self.m_impulse;
	}

	/// Get the reaction torque given the inverse time step.
	/// Unit is n*m. This is always zero for a distance joint.
	fn get_reaction_torque(&self, inv_dt: f32) -> f32 {
		return inv_dt * 0.0;
	}

	fn init_velocity_constraints(
		&mut self,
		data: &B2solverData,
		positions: &[B2position],
		velocities: &mut [B2velocity],
	) {
		private::init_velocity_constraints(self, data, positions, velocities);
	}
	fn solve_velocity_constraints(
		&mut self,
		data: &B2solverData,
		velocities: &mut [B2velocity],
	) {
		private::solve_velocity_constraints(self, data, velocities);
	}
	fn solve_position_constraints(
		&mut self,
		data: &B2solverData,
		positions: &mut [B2position],
	) -> bool {
		return private::solve_position_constraints(self, data, positions);
	}

	/// Implement B2joint::shift_origin
	fn shift_origin(&mut self, new_origin: B2vec2) {
		self.m_target_a -= new_origin;
	}
}
impl<D: UserDataType> B2mouseJoint<D> {
	/// Use this to update the target point.
	pub fn set_target(&mut self, target: B2vec2) {
		if target != self.m_target_a {
			self.base.m_body_b.borrow_mut().set_awake(true);
			self.m_target_a = target;
		}
	}
	pub fn get_target(&self) -> B2vec2 {
		return self.m_target_a;
	}

	/// Set/get the maximum force in Newtons.
	pub fn set_max_force(&mut self, force: f32) {
		self.m_max_force = force;
	}
	pub fn get_max_force(&self) -> f32 {
		return self.m_max_force;
	}

	/// Set/get the linear stiffness in N/m
	pub fn set_stiffness(&mut self, stiffness: f32) { self.m_stiffness = stiffness; }
	pub fn get_stiffness(&self) -> f32 { return self.m_stiffness; }

	/// Set/get linear damping in N*s/m
	pub fn set_damping(&mut self, damping: f32) { self.m_damping = damping; }
	pub fn get_damping(&self) -> f32 { return self.m_damping; }
	

	pub fn new(def: &B2mouseJointDef<D>) -> Self {
		let m_target_a = def.target;
		let body_b_transform = def.base.body_b.as_ref().unwrap().borrow().get_transform();
		return B2mouseJoint {
			base: B2joint::new(&def.base),

			m_target_a: m_target_a,
			m_local_anchor_b: b2_mul_t_transform_by_vec2(body_b_transform, m_target_a),

			m_max_force: def.max_force,
			m_impulse: B2vec2::zero(),

			m_stiffness: def.stiffness,
			m_damping: def.damping,

			m_beta: 0.0,
			m_gamma: 0.0,
			//m_index_a: 0,
			m_index_b: 0,
			m_r_b: B2vec2::zero(),
			m_local_center_b: B2vec2::zero(),
			m_inv_mass_b: 0.0,
			m_inv_ib: 0.0,
			m_mass: B2Mat22::default(),
			m_c: B2vec2::zero(),
		};
	}
}

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in the testbed. If you want to learn how to
/// use the mouse joint, look at the testbed.
pub struct B2mouseJoint<D: UserDataType> {
	pub(crate) base: B2joint<D>,
	pub(crate) m_local_anchor_b: B2vec2,
	pub(crate) m_target_a: B2vec2,
	pub(crate) m_stiffness: f32,
	pub(crate) m_damping: f32,
	pub(crate) m_beta: f32,

	// Solver shared
	pub(crate) m_impulse: B2vec2,
	pub(crate) m_max_force: f32,
	pub(crate) m_gamma: f32,

	// Solver temp
	//pub(crate) m_index_a: i32,
	pub(crate) m_index_b: i32,
	pub(crate) m_r_b: B2vec2,
	pub(crate) m_local_center_b: B2vec2,
	pub(crate) m_inv_mass_b: f32,
	pub(crate) m_inv_ib: f32,
	pub(crate) m_mass: B2Mat22,
	pub(crate) m_c: B2vec2,
}
