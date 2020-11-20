use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_time_step::*;
use crate::private::dynamics::joints::b2_weld_joint as private;

impl<D: UserDataType> Default for B2weldJointDef<D> {
	fn default() -> Self {
		return Self {
			base: B2jointDef {
				jtype: B2jointType::EWeldJoint,
				..Default::default()
			},
			local_anchor_a: B2vec2::zero(),
			local_anchor_b: B2vec2::zero(),
			reference_angle: 0.0,
			stiffness : 0.0,
			damping : 0.0,	
		};
	}
}

/// Weld joint definition. You need to specify local anchor points
/// where they are attached and the relative body angle. The position
/// of the anchor points is important for computing the reaction torque.
#[derive(Clone)]
pub struct B2weldJointDef<D: UserDataType> {
	pub base: B2jointDef<D>,

	/// The local anchor point relative to body_a's origin.
	pub local_anchor_a: B2vec2,

	/// The local anchor point relative to body_b's origin.
	pub local_anchor_b: B2vec2,

	/// The body_b angle minus body_a angle in the reference state (radians).
	pub reference_angle: f32,

	/// The rotational stiffness in n*m
	/// Disable softness with a value of 0
	pub stiffness: f32,

	/// The rotational damping in n*m*s
	pub damping: f32,

}

impl<D: UserDataType> B2weldJointDef<D> {
	/// Initialize the bodies, anchors, reference angle, stiffness, and damping.
	/// * `body_a` - the first body connected by this joint
	/// * `body_b` - the second body connected by this joint
	/// * `anchor` - the point of connection in world coordinates
	pub fn initialize(&mut self, body_a: BodyPtr<D>, body_b: BodyPtr<D>, anchor: B2vec2) {
		self.base.body_a = Some(body_a.clone());
		self.base.body_b = Some(body_b.clone());
		self.local_anchor_a = body_a.borrow().get_local_point(anchor);
		self.local_anchor_b = body_b.borrow().get_local_point(anchor);
		self.reference_angle = body_b.borrow().get_angle() - body_a.borrow().get_angle();
	}
}

impl<D: UserDataType> ToDerivedJoint<D> for B2weldJoint<D> {
	fn as_derived(&self) -> JointAsDerived<D> {
		return JointAsDerived::EWeldJoint(self);
	}
	fn as_derived_mut(&mut self) -> JointAsDerivedMut<D> {
		return JointAsDerivedMut::EWeldJoint(self);
	}
}

/// A weld joint essentially glues two bodies together. A weld joint may
/// distort somewhat because the island constraint solver is approximate.
pub struct B2weldJoint<D: UserDataType> {
	pub(crate) base: B2joint<D>,
	pub(crate) m_stiffness : f32,
	pub(crate) m_damping: f32,
	pub(crate) m_bias: f32,

	// Solver shared
	pub(crate) m_local_anchor_a: B2vec2,
	pub(crate) m_local_anchor_b: B2vec2,
	pub(crate) m_reference_angle: f32,
	pub(crate) m_gamma: f32,
	pub(crate) m_impulse: B2Vec3,

	// Solver temp
	pub(crate) m_index_a: i32,
	pub(crate) m_index_b: i32,
	pub(crate) m_r_a: B2vec2,
	pub(crate) m_r_b: B2vec2,
	pub(crate) m_local_center_a: B2vec2,
	pub(crate) m_local_center_b: B2vec2,
	pub(crate) m_inv_mass_a: f32,
	pub(crate) m_inv_mass_b: f32,
	pub(crate) m_inv_ia: f32,
	pub(crate) m_inv_ib: f32,
	pub(crate) m_mass: B2Mat33,
}

impl<D: UserDataType> B2weldJoint<D> {
	/// The local anchor point relative to body_a's origin.
	pub fn get_local_anchor_a(&self) -> B2vec2 {
		return self.m_local_anchor_a;
	}

	/// The local anchor point relative to body_b's origin.
	pub fn get_local_anchor_b(&self) -> B2vec2 {
		return self.m_local_anchor_b;
	}

	/// Get the reference angle.
	pub fn get_reference_angle(&self) -> f32 {
		return self.m_reference_angle;
	}

		/// Set/get stiffness in n*m
		pub fn  set_stiffness(&mut self,hz: f32) { self.m_stiffness = hz; }
		pub fn  get_frequency(&self)-> f32 { return self.m_stiffness; }
	
		/// Set/get damping in n*m*s
		pub fn  set_damping(&mut self,damping: f32) { self.m_damping = damping; }
		pub fn  get_damping(&self) -> f32{ return self.m_damping; }


	pub(crate) fn new(def: &B2weldJointDef<D>) -> Self {
		return Self {
			base: B2joint::new(&def.base),

			m_local_anchor_a: def.local_anchor_a,
			m_local_anchor_b: def.local_anchor_b,

			m_stiffness: def.stiffness,
			m_damping: def.damping,
			m_bias: 0.0,		

			m_reference_angle: def.reference_angle,
			m_gamma: 0.0,
			m_impulse: B2Vec3::zero(),
		

			m_index_a: 0,
			m_index_b: 0,
			m_r_a: B2vec2::zero(),
			m_r_b: B2vec2::zero(),
			m_local_center_a: B2vec2::zero(),
			m_local_center_b: B2vec2::zero(),
			m_inv_mass_a: 0.0,
			m_inv_mass_b: 0.0,
			m_inv_ia: 0.0,
			m_inv_ib: 0.0,
			m_mass: B2Mat33::zero(),
		}
	}
}

impl<D: UserDataType> B2jointTraitDyn<D> for B2weldJoint<D> {
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
		let p = B2vec2::new(self.m_impulse.x, self.m_impulse.y);
		return inv_dt * p;
	}

	fn get_reaction_torque(&self, inv_dt: f32) -> f32 {
		return inv_dt * self.m_impulse.z;
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

	/// dump joint to dmLog
	fn dump(&self) {
		//private::dump(self);
	}
}
