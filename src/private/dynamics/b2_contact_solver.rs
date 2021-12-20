use crate::b2_collision::*;
use crate::b2_math::*;
use crate::b2_time_step::*;
use crate::b2_settings::*;
use crate::b2rs_common::UserDataType;
use crate::b2_contact::*;

use super::b2_contact_solver_private as private;

#[derive(Default,Copy, Clone)]
pub(crate) struct B2velocityConstraintPoint
{
	pub(crate)  r_a: B2vec2,
	 pub(crate)  r_b: B2vec2,
	 pub(crate)  normal_impulse: f32,
	 pub(crate) tangent_impulse: f32,
	pub(crate) normal_mass: f32,
	pub(crate) tangent_mass: f32,
	pub(crate) velocity_bias: f32,
}

#[derive(Default,Copy, Clone)]
pub(crate) struct B2contactVelocityConstraint
{
	pub(crate) points: [B2velocityConstraintPoint;B2_MAX_MANIFOLD_POINTS],
	pub(crate) normal: B2vec2,
	pub(crate) normal_mass: B2Mat22,
	pub(crate) k: B2Mat22,
	pub(crate) index_a: i32,
	pub(crate) index_b: i32,
	pub(crate) inv_mass_a: f32,
	pub(crate) inv_mass_b: f32,
	pub(crate) inv_ia: f32,
	pub(crate) inv_ib: f32,
	pub(crate) friction: f32,
	pub(crate) restitution: f32,
	pub(crate) tangent_speed: f32,
	pub(crate) point_count: i32,
	pub(crate) contact_index: i32,
}

pub(crate) struct B2contactSolverDef
{
	pub(crate) step: B2timeStep,
	//pub(crate) contacts: &'a mut  Vec<ContactPtr<D>>,
	//pub(crate) positions: &'a mut  Vec<B2position>,
	//pub(crate) velocities: &'a mut  Vec<B2velocity>,
	//b2StackAllocator* allocator;
}

impl B2contactSolver
{
	pub fn new<D: UserDataType>(def: &B2contactSolverDef, 
		contacts: &Vec<ContactPtr<D>>)->Self{
		return private::new(def, contacts);
	}

	pub fn  initialize_velocity_constraints<D: UserDataType>(&mut self,
		m_positions: &[B2position], m_velocities: &[B2velocity], m_contacts: &[ContactPtr<D>]){
		private::initialize_velocity_constraints(self, m_positions, m_velocities, m_contacts);
	}

	pub fn  warm_start(&mut self, m_velocities: &mut [B2velocity]){
		private::warm_start(self, m_velocities);
	}
	pub fn  solve_velocity_constraints(&mut self, m_velocities: &mut [B2velocity]){
		private::solve_velocity_constraints(self, m_velocities);
	}
	pub fn  store_impulses<D: UserDataType>(&mut self, m_contacts: &[ContactPtr<D>]){
		private::store_impulses(self, m_contacts);
	}

	pub fn  solve_position_constraints(&mut self, m_positions: &mut [B2position]) -> bool
	{
		return private::solve_position_constraints(self, m_positions);
	}
	pub fn  solve_toiposition_constraints(&mut self, toi_index_a: i32, toi_index_b: i32, 
		m_positions: &mut [B2position]) -> bool
	{
		return private::solve_toiposition_constraints(self, toi_index_a, toi_index_b, m_positions);
	}

}

#[derive(Default,Clone)]
pub(crate) struct B2contactSolver
{
	pub(crate) m_step: B2timeStep,
	//m_positions: Vec<B2position>,
	//m_velocities: Vec<B2velocity>,
	//m_contacts: Vec<ContactPtr<D>>,
	//m_count: i32,
	//b2StackAllocator* m_allocator;
	pub(crate) m_position_constraints: Vec<B2contactPositionConstraint>,
	pub(crate) m_velocity_constraints: Vec<B2contactVelocityConstraint>,
}

#[derive(Default, Copy, Clone)]
pub(crate) struct B2contactPositionConstraint
{
	pub(crate) local_points: [B2vec2; B2_MAX_MANIFOLD_POINTS],
	pub(crate) local_normal: B2vec2,
	pub(crate) local_point: B2vec2,
	pub(crate) index_a: i32,
	pub(crate) index_b: i32,
	pub(crate) inv_mass_a: f32, 
	pub(crate) inv_mass_b: f32,
	pub(crate) local_center_a: B2vec2, 
	pub(crate) local_center_b: B2vec2,
	pub(crate) inv_ia: f32, 
	pub(crate) inv_ib: f32,
	pub(crate) mtype: B2manifoldType,
	pub(crate) radius_a: f32,
	pub(crate) radius_b: f32,
	pub(crate)  point_count: i32,
}