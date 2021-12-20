use crate::b2_body::*;
use crate::b2_contact::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::b2_world_callbacks::*;
use super::b2_contact_solver::*;

use crate::private::dynamics::b2_island_private as private;

/// This is an internal class.
impl<D: UserDataType> B2island<D> {
	pub fn new(
		body_capacity: usize,
		contact_capacity: usize,
		joint_capacity: usize,
		listener: Option<B2contactListenerPtr<D>>,
	) -> Self {
		let mut m_positions = Vec::<B2position>::new();
		let mut m_velocities = Vec::<B2velocity>::new();
		m_positions.resize(body_capacity, Default::default());
		m_velocities.resize(body_capacity, Default::default());
		return Self
		{
			m_listener: listener,
			m_bodies: Vec::with_capacity(body_capacity),
			m_contacts: Vec::with_capacity(contact_capacity),
			m_joints: Vec::with_capacity(joint_capacity),
		
			m_positions,
			m_velocities,
		}
	}

	pub fn clear(&mut self) {
		self.m_bodies.clear();
		self.m_contacts.clear();
		self.m_joints.clear();
	}

	pub fn solve(&mut self, profile: &mut B2Profile, step: &B2timeStep, gravity: B2vec2, allow_sleep: bool)
	{
		private::solve(self, profile, step, gravity, allow_sleep);
	}

	pub fn solve_toi(&mut self, sub_step: &B2timeStep, toi_index_a: i32, toi_index_b: i32)
	{
		private::solve_toi(self, sub_step, toi_index_a, toi_index_b);
	}

	pub fn add_body(&mut self, body: BodyPtr<D>) {
		body.borrow_mut().m_island_index = self.m_bodies.len() as i32;
		self.m_bodies.push(body);
	}

	pub fn add_contact(&mut self, contact: ContactPtr<D>) {
		self.m_contacts.push(contact);
	}

	pub fn add_joint(&mut self, joint: B2jointPtr<D>) {
		self.m_joints.push(joint);
	}

	pub fn report(&self, constraints: &[B2contactVelocityConstraint])
	{
		private::report(self, constraints);
	}
}

/// This is an internal class.
pub(crate) struct B2island<D: UserDataType> {
	pub(crate) m_listener: Option<B2contactListenerPtr<D>>,
	pub(crate) m_bodies: Vec<BodyPtr<D>>,
	pub(crate) m_contacts: Vec<ContactPtr<D>>,
	pub(crate) m_joints: Vec<B2jointPtr<D>>,

	pub(crate) m_positions: Vec<B2position>,
	pub(crate) m_velocities: Vec<B2velocity>,
}
