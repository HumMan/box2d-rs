use crate::b2_body::*;
use crate::b2_draw::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;

use crate::b2rs_double_linked_list::*;
use crate::b2rs_linked_list::*;

use std::cell::RefCell;
use std::rc::{Rc, Weak};

#[cfg(feature="serde_support")]
use serde::{Serialize, Deserialize};

use crate::joints::b2_distance_joint::*;
use crate::joints::b2_friction_joint::*;
use crate::joints::b2_gear_joint::*;
use crate::joints::b2_motor_joint::*;
use crate::joints::b2_mouse_joint::*;
use crate::joints::b2_prismatic_joint::*;
use crate::joints::b2_pulley_joint::*;
use crate::joints::b2_revolute_joint::*;
use crate::joints::b2_weld_joint::*;
use crate::joints::b2_wheel_joint::*;

use crate::private::dynamics::b2_joint as private;

pub enum B2JointDefEnum<D: UserDataType> {
	DistanceJoint(B2distanceJointDef<D>),
	FrictionJoint(B2frictionJointDef<D>),
	GearJoint(B2gearJointDef<D>),
	MouseJoint(B2mouseJointDef<D>),
	MotorJoint(B2motorJointDef<D>),
	PulleyJoint(B2pulleyJointDef<D>),
	RevoluteJoint(B2revoluteJointDef<D>),
	PrismaticJoint(B2prismaticJointDef<D>),
	WeldJoint(B2weldJointDef<D>),
	WheelJoint(B2wheelJointDef<D>),
}


#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum B2jointType {
	EUnknownJoint,
	EDistanceJoint,
	EFrictionJoint,
	EGearJoint,
	EMotorJoint,
	EMouseJoint,
	EPrismaticJoint,
	EPulleyJoint,
	ERevoluteJoint,
	EWeldJoint,
	EWheelJoint,	
}

impl Default for B2jointType {
	fn default() -> Self {
		return B2jointType::EUnknownJoint;
	}
}

#[derive(Default, Clone, Copy, Debug)]
pub(crate) struct B2jacobian {
	linear: B2vec2,
	angular_a: f32,
	angular_b: f32,
}

pub type B2jointEdgePtr<D> = Rc<RefCell<B2jointEdge<D>>>;
pub type B2jointEdgeWeakPtr<D> = Weak<RefCell<B2jointEdge<D>>>;

impl<D: UserDataType> LinkedListNode<B2jointEdge<D>> for B2jointEdge<D> {
	fn get_next(&self) -> Option<B2jointEdgePtr<D>> {
		return self.next.clone();
	}
	fn set_next(&mut self, value: Option<B2jointEdgePtr<D>>) {
		self.next = value;
	}
	fn take_next(&mut self) -> Option<B2jointEdgePtr<D>> {
		return self.next.take();
	}
}

impl<D: UserDataType> DoubleLinkedListNode<B2jointEdge<D>> for B2jointEdge<D> {
	fn get_prev(&self) -> Option<B2jointEdgeWeakPtr<D>> {
		return self.prev.clone();
	}
	fn set_prev(&mut self, value: Option<B2jointEdgeWeakPtr<D>>) {
		self.prev = value;
	}
}

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
pub struct B2jointEdge<D: UserDataType> {
	///< provides quick access to the other body attached.
	pub(crate) other: BodyWeakPtr<D>,
	///< the joint
	pub(crate) joint: B2jointWeakPtr<D>,
	///< the previous joint edge in the body's joint list
	pub(crate) prev: Option<B2jointEdgeWeakPtr<D>>,
	///< the next joint edge in the body's joint list
	pub(crate) next: Option<B2jointEdgePtr<D>>,
}

impl<D: UserDataType> Default for B2jointDef<D> {
	fn default() -> Self {
		return Self {
			jtype: B2jointType::EUnknownJoint,
			user_data: None,
			body_a: None,
			body_b: None,
			collide_connected: false,
		};
	}
}

impl<D: UserDataType> LinkedListNode<dyn B2jointTraitDyn<D>> for dyn B2jointTraitDyn<D> {
	fn get_next(&self) -> Option<B2jointPtr<D>> {
		return self.get_base().m_next.clone();
	}
	fn set_next(&mut self, value: Option<B2jointPtr<D>>) {
		self.get_base_mut().m_next = value;
	}
	fn take_next(&mut self) -> Option<B2jointPtr<D>> {
		return self.get_base_mut().m_next.take();
	}
}

impl<D: UserDataType> DoubleLinkedListNode<dyn B2jointTraitDyn<D>> for dyn B2jointTraitDyn<D> {
	fn get_prev(&self) -> Option<B2jointWeakPtr<D>> {
		return self.get_base().m_prev.clone();
	}
	fn set_prev(&mut self, value: Option<B2jointWeakPtr<D>>) {
		self.get_base_mut().m_prev = value.clone();
	}
}

/// Joint definitions are used to construct joints.
#[derive(Clone)]
pub struct B2jointDef<D: UserDataType> {
	/// The joint type is set automatically for concrete joint types.
	pub jtype: B2jointType,

	/// Use this to attach application specific data to your joints.
	pub user_data: Option<D::Joint>,

	/// The first attached body.
	pub body_a: Option<BodyPtr<D>>,

	/// The second attached body.
	pub body_b: Option<BodyPtr<D>>,

	/// Set this flag to true if the attached bodies should collide.
	pub collide_connected: bool,
}

pub type B2jointPtr<D> = Rc<RefCell<dyn B2jointTraitDyn<D>>>;
pub type B2jointWeakPtr<D> = Weak<RefCell<dyn B2jointTraitDyn<D>>>;


/// Utility to compute linear stiffness values from frequency and damping ratio
pub fn b2_linear_stiffness<D: UserDataType>(stiffness: &mut f32, damping: &mut f32,
	frequency_hertz: f32, damping_ratio: f32,
	body_a: BodyPtr<D>, body_b: BodyPtr<D>)
{
	private::b2_linear_stiffness(stiffness, damping, frequency_hertz, damping_ratio, body_a, body_b);
}

/// Utility to compute rotational stiffness values frequency and damping ratio
pub fn b2_angular_stiffness<D: UserDataType>(stiffness: &mut f32, damping: &mut f32,
	frequency_hertz: f32, damping_ratio: f32,
	body_a: BodyPtr<D>, body_b: BodyPtr<D>)
{
	private::b2_angular_stiffness(stiffness, damping, frequency_hertz, damping_ratio, body_a, body_b);
}


/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
impl<D: UserDataType> B2joint<D> {
	/// Get the type of the concrete joint.
	pub fn get_type(&self) -> B2jointType {
		return self.m_type;
	}

	/// Get the first body attached to this joint.
	pub fn get_body_a(&self) -> BodyPtr<D> {
		return self.m_body_a.clone();
	}

	/// Get the second body attached to this joint.
	pub fn get_body_b(&self) -> BodyPtr<D> {
		return self.m_body_b.clone();
	}

	/// Get the next joint the world joint list.
	pub fn get_next(&self) -> B2jointPtr<D> {
		return self.m_next.as_ref().unwrap().clone();
	}

	/// Get the user data pointer.
	pub fn get_user_data(&self) -> Option<D::Joint> {
		return self.m_user_data.clone();
	}

	/// Set the user data pointer.
	pub fn set_user_data(&mut self, data: D::Joint) {
		self.m_user_data = Some(data);
	}

	/// Short-cut function to determine if either body is enabled.
	pub fn is_enabled(&self) -> bool {
		return private::is_enabled(self);
	}

	/// Get collide connected.
	/// Note: modifying the collide connect flag won't work correctly because
	/// the flag is only checked when fixture AABBs begin to overlap.
	pub fn get_collide_connected(&self) -> bool {
		return self.m_collide_connected;
	}

	// protected:

	// 	static B2joint* create(const B2jointDef* def, b2BlockAllocator* allocator);
	pub(crate) fn create(def: &B2JointDefEnum<D>) -> B2jointPtr<D> {
		return private::create(def);
	}
	pub(crate) fn new(def: &B2jointDef<D>) -> B2joint<D> {
		return private::new(def);
	}
}

pub trait B2jointTraitDyn<D: UserDataType>: ToDerivedJoint<D> {
	fn get_base(&self) -> &B2joint<D>;
	fn get_base_mut(&mut self) -> &mut B2joint<D>;
	/// Get the anchor point on body_a in world coordinates.
	fn get_anchor_a(&self) -> B2vec2;

	/// Get the anchor point on body_b in world coordinates.
	fn get_anchor_b(&self) -> B2vec2;

	/// Get the reaction force on body_b at the joint anchor in Newtons.
	fn get_reaction_force(&self, inv_dt: f32) -> B2vec2;

	/// Get the reaction torque on body_b in n*m.
	fn get_reaction_torque(&self, inv_dt: f32) -> f32;

	/// Shift the origin for any points stored in world coordinates.
	fn shift_origin(&mut self, new_origin: B2vec2) {
		b2_not_used(new_origin);
	}

	/// Debug draw this joint
	fn draw(&self, draw: &mut dyn B2drawTrait) {
		private::draw(self, draw);
	} 

	fn init_velocity_constraints(
		&mut self,
		data: &B2solverData,
		positions: &[B2position],
		velocities: &mut [B2velocity],
	);
	fn solve_velocity_constraints(
		&mut self,
		data: &B2solverData,
		velocities: &mut [B2velocity],
	);

	// This returns true if the position errors are within tolerance.
	fn solve_position_constraints(
		&mut self,
		data: &B2solverData,
		positions: &mut [B2position],
	) -> bool;
}

pub trait ToDerivedJoint<D: UserDataType> {
	fn as_derived(&self) -> JointAsDerived<D>;
	fn as_derived_mut(&mut self) -> JointAsDerivedMut<D>;
}

pub enum JointAsDerived<'a, D: UserDataType> {
	EDistanceJoint(&'a B2distanceJoint<D>),
	EFrictionJoint(&'a B2frictionJoint<D>),
	EGearJoint(&'a B2gearJoint<D>),
	EMouseJoint(&'a B2mouseJoint<D>),
	EMotorJoint(&'a B2motorJoint<D>),
	EPulleyJoint(&'a B2pulleyJoint<D>),
	ERevoluteJoint(&'a B2revoluteJoint<D>),
	EPrismaticJoint(&'a B2prismaticJoint<D>),
	EWeldJoint(&'a B2weldJoint<D>),
	EWheelJoint(&'a B2wheelJoint<D>),
}

pub enum JointAsDerivedMut<'a, D: UserDataType> {
	EDistanceJoint(&'a mut B2distanceJoint<D>),
	EFrictionJoint(&'a mut B2frictionJoint<D>),
	EGearJoint(&'a mut B2gearJoint<D>),
	EMouseJoint(&'a mut B2mouseJoint<D>),
	EMotorJoint(&'a mut B2motorJoint<D>),
	EPulleyJoint(&'a mut B2pulleyJoint<D>),
	ERevoluteJoint(&'a mut B2revoluteJoint<D>),
	EPrismaticJoint(&'a mut B2prismaticJoint<D>),
	EWeldJoint(&'a mut B2weldJoint<D>),
	EWheelJoint(&'a mut B2wheelJoint<D>),
}

#[derive(Clone)]
pub struct B2joint<D: UserDataType> {
	// protected:
	pub(crate) m_type: B2jointType,
	pub(crate) m_prev: Option<B2jointWeakPtr<D>>,
	pub(crate) m_next: Option<B2jointPtr<D>>,
	pub(crate) m_edge_a: Option<B2jointEdgePtr<D>>,
	pub(crate) m_edge_b: Option<B2jointEdgePtr<D>>,
	pub(crate) m_body_a: BodyPtr<D>,
	pub(crate) m_body_b: BodyPtr<D>,

	pub(crate) m_index: i32,

	pub(crate) m_island_flag: bool,
	pub(crate) m_collide_connected: bool,

	pub(crate) m_user_data: Option<D::Joint>,
}
