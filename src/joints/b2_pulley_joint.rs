use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::private::dynamics::joints::b2_pulley_joint as private;

//const B2_MIN_PULLEY_LENGTH: f32 = 2.0;

impl<D: UserDataType> Default for B2pulleyJointDef<D> {
	fn default() -> Self {
		return Self {
			base: B2jointDef {
				jtype: B2jointType::EPulleyJoint,
				collide_connected: true,
				..Default::default()
			},
			ground_anchor_a: B2vec2::new(-1.0, 1.0),
			ground_anchor_b: B2vec2::new(1.0, 1.0),
			local_anchor_a: B2vec2::new(-1.0, 0.0),
			local_anchor_b: B2vec2::new(1.0, 0.0),
			length_a: 0.0,
			length_b: 0.0,
			ratio: 1.0,
		};
	}
}

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, and a pulley ratio.
#[derive(Clone)]
pub struct B2pulleyJointDef<D: UserDataType> {
	pub base: B2jointDef<D>,
	/// The first ground anchor in world coordinates. This point never moves.
	pub ground_anchor_a: B2vec2,

	/// The second ground anchor in world coordinates. This point never moves.
	pub ground_anchor_b: B2vec2,

	/// The local anchor point relative to body_a's origin.
	pub local_anchor_a: B2vec2,

	/// The local anchor point relative to body_b's origin.
	pub local_anchor_b: B2vec2,

	/// The a reference length for the segment attached to body_a.
	pub length_a: f32,

	/// The a reference length for the segment attached to body_b.
	pub length_b: f32,

	/// The pulley ratio, used to simulate a block-and-tackle.
	pub ratio: f32,
}

impl<D: UserDataType> B2pulleyJointDef<D> {
	/// initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
	pub fn initialize(
		&mut self,
		body_a: BodyPtr<D>,
		body_b: BodyPtr<D>,
		ground_anchor_a: B2vec2,
		ground_anchor_b: B2vec2,
		anchor_a: B2vec2,
		anchor_b: B2vec2,
		ratio: f32,
	) {
		self.base.body_a = Some(body_a.clone());
		self.base.body_b = Some(body_b.clone());
		self.ground_anchor_a = ground_anchor_a;
		self.ground_anchor_b = ground_anchor_b;
		self.local_anchor_a = body_a.borrow().get_local_point(anchor_a);
		self.local_anchor_b = body_b.borrow().get_local_point(anchor_b);

		let d_a: B2vec2 = anchor_a - ground_anchor_a;
		self.length_a = d_a.length();
		let d_b: B2vec2 = anchor_b - ground_anchor_b;
		self.length_b = d_b.length();
		self.ratio = ratio;
		b2_assert(ratio > B2_EPSILON);
	}
}

impl<D: UserDataType> ToDerivedJoint<D> for B2pulleyJoint<D> {
	fn as_derived(&self) -> JointAsDerived<D> {
		return JointAsDerived::EPulleyJoint(self);
	}
	fn as_derived_mut(&mut self) -> JointAsDerivedMut<D> {
		return JointAsDerivedMut::EPulleyJoint(self);
	}
}

impl<D: UserDataType> B2jointTraitDyn<D> for B2pulleyJoint<D> {
	fn get_base(&self) -> &B2joint<D> {
		return &self.base;
	}
	fn get_base_mut(&mut self) -> &mut B2joint<D> {
		return &mut self.base;
	}
	fn get_anchor_a(&self) -> B2vec2 {
		return self
			.base
			.m_body_a
			.borrow()
			.get_world_point(self.m_local_anchor_a);
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
		let p: B2vec2 = self.m_impulse * self.m_u_b;
		return inv_dt * p;
	}

	fn get_reaction_torque(&self, inv_dt: f32) -> f32 {
		b2_not_used(inv_dt);
		return 0.0;
	}
	fn init_velocity_constraints(
		&mut self,
		data: &mut B2solverData,
		positions: &mut [B2position],
		velocities: &mut [B2velocity],
	) {
		private::init_velocity_constraints(self, data, positions, velocities);
	}
	fn solve_velocity_constraints(
		&mut self,
		data: &mut B2solverData,
		velocities: &mut [B2velocity],
	) {
		private::solve_velocity_constraints(self, data, velocities);
	}
	fn solve_position_constraints(
		&mut self,
		data: &mut B2solverData,
		positions: &mut [B2position],
	) -> bool {
		return private::solve_position_constraints(self, data, positions);
	}

	/// Implement B2joint::shift_origin
	fn shift_origin(&mut self, new_origin: B2vec2) {
		self.m_ground_anchor_a -= new_origin;
		self.m_ground_anchor_b -= new_origin;
	}

}

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// Warning: the pulley joint can get a bit squirrelly by itself. They often
/// work better when combined with prismatic joints. You should also cover the
/// the anchor points with static shapes to prevent one side from going to
/// zero length.
impl<D: UserDataType> B2pulleyJoint<D> {
	/// Get the first ground anchor.
	pub fn get_ground_anchor_a(&self) -> B2vec2 {
		return self.m_ground_anchor_a;
	}

	/// Get the second ground anchor.
	pub fn get_ground_anchor_b(&self) -> B2vec2 {
		return self.m_ground_anchor_b;
	}

	/// Get the current length of the segment attached to body_a.
	pub fn get_length_a(&self) -> f32 {
		return self.m_length_a;
	}

	/// Get the current length of the segment attached to body_b.
	pub fn get_length_b(&self) -> f32 {
		return self.m_length_b;
	}

	/// Get the pulley ratio.
	pub fn get_ratio(&self) -> f32 {
		return self.m_ratio;
	}

	/// Get the current length of the segment attached to body_a.
	pub fn get_current_length_a(&self) -> f32 {
		let p: B2vec2 = self
			.base
			.m_body_a
			.borrow()
			.get_world_point(self.m_local_anchor_a);
		let s: B2vec2 = self.m_ground_anchor_a;
		let d: B2vec2 = p - s;
		return d.length();
	}

	/// Get the current length of the segment attached to body_b.
	pub fn get_current_length_b(&self) -> f32 {
		let p: B2vec2 = self
			.base
			.m_body_b
			.borrow()
			.get_world_point(self.m_local_anchor_b);
		let s: B2vec2 = self.m_ground_anchor_b;
		let d: B2vec2 = p - s;
		return d.length();
	}

	pub(crate) fn new(def: &B2pulleyJointDef<D>) -> B2pulleyJoint<D> {
		return private::new(def);
	}
}

pub struct B2pulleyJoint<D: UserDataType> {
	pub(crate) base: B2joint<D>,
	pub(crate) m_ground_anchor_a: B2vec2,
	pub(crate) m_ground_anchor_b: B2vec2,
	pub(crate) m_length_a: f32,
	pub(crate) m_length_b: f32,

	// Solver shared
	pub(crate) m_local_anchor_a: B2vec2,
	pub(crate) m_local_anchor_b: B2vec2,
	pub(crate) m_constant: f32,
	pub(crate) m_ratio: f32,
	pub(crate) m_impulse: f32,

	// Solver temp
	pub(crate) m_index_a: i32,
	pub(crate) m_index_b: i32,
	pub(crate) m_u_a: B2vec2,
	pub(crate) m_u_b: B2vec2,
	pub(crate) m_r_a: B2vec2,
	pub(crate) m_r_b: B2vec2,
	pub(crate) m_local_center_a: B2vec2,
	pub(crate) m_local_center_b: B2vec2,
	pub(crate) m_inv_mass_a: f32,
	pub(crate) m_inv_mass_b: f32,
	pub(crate) m_inv_ia: f32,
	pub(crate) m_inv_ib: f32,
	pub(crate) m_mass: f32,
}
