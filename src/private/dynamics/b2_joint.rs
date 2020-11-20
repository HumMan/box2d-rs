use crate::b2_joint::*;
use crate::b2_draw::*;
use crate::b2_settings::*;
use crate::b2_math::*;
use crate::b2_body::*;

use crate::joints::b2_distance_joint::*;
use crate::joints::b2_friction_joint::*;
use crate::joints::b2_gear_joint::*;
use crate::joints::b2_motor_joint::*;
use crate::joints::b2_mouse_joint::*;
use crate::joints::b2_prismatic_joint::*;
use crate::joints::b2_pulley_joint::*;
use crate::joints::b2_revolute_joint::*;
use crate::joints::b2_rope_joint::*;
use crate::joints::b2_weld_joint::*;
use crate::joints::b2_wheel_joint::*;

use std::cell::RefCell;
use std::rc::{Rc};

pub(crate) fn b2_linear_stiffness<D: UserDataType>(stiffness: &mut f32, damping: &mut f32,
	frequency_hertz: f32, damping_ratio: f32,
	body_a: BodyPtr<D>, body_b: BodyPtr<D>)
{
	let mass_a: f32 = body_a.borrow().get_mass();
	let mass_b: f32 = body_b.borrow().get_mass();
	let mass: f32;
	if mass_a > 0.0 && mass_b > 0.0
	{
		mass = mass_a * mass_b / (mass_a + mass_b);
	}
	else if mass_a > 0.0
	{
		mass = mass_a;
	}
	else
	{
		mass = mass_b;
	}

	let omega: f32 = 2.0 * B2_PI * frequency_hertz;
	*stiffness = mass * omega * omega;
	*damping = 2.0 * mass * damping_ratio * omega;
}

pub(crate) fn b2_angular_stiffness<D: UserDataType>(stiffness: &mut f32, damping: &mut f32,
	frequency_hertz: f32, damping_ratio: f32,
	body_a: BodyPtr<D>, body_b: BodyPtr<D>)
{
	let ia: f32 = body_a.borrow().get_inertia();
	let ib: f32 = body_b.borrow().get_inertia();
	let i: f32;
	if ia > 0.0 && ib > 0.0
	{
		i = ia * ib / (ia + ib);
	}
	else if ia > 0.0
	{
		i = ia;
	}
	else
	{
		i = ib;
	}

	let omega: f32 = 2.0 * B2_PI * frequency_hertz;
	*stiffness = i * omega * omega;
	*damping = 2.0 * i * damping_ratio * omega;
}


pub(crate) fn create<D: UserDataType>(def: &B2JointDefEnum<D>) -> B2jointPtr<D>
{
	match def
	{
		B2JointDefEnum::DistanceJoint(ref def)=> Rc::new(RefCell::new(B2distanceJoint::new(def))),
		B2JointDefEnum::FrictionJoint(ref def)=> Rc::new(RefCell::new(B2frictionJoint::new(def))),
		B2JointDefEnum::GearJoint(ref def)=> Rc::new(RefCell::new(B2gearJoint::new(def))),
		B2JointDefEnum::MouseJoint(ref def)=> Rc::new(RefCell::new(B2mouseJoint::new(def))),
		B2JointDefEnum::MotorJoint(ref def)=> Rc::new(RefCell::new(B2motorJoint::new(def))),
		B2JointDefEnum::PulleyJoint(ref def)=> Rc::new(RefCell::new(B2pulleyJoint::new(def))),
		B2JointDefEnum::RevoluteJoint(ref def)=> Rc::new(RefCell::new(B2revoluteJoint::new(def))),
		B2JointDefEnum::RopeJoint(ref def)=> Rc::new(RefCell::new(B2ropeJoint::new(def))),
		B2JointDefEnum::PrismaticJoint(ref def)=> Rc::new(RefCell::new(B2prismaticJoint::new(def))),
		B2JointDefEnum::WeldJoint(ref def)=> Rc::new(RefCell::new(B2weldJoint::new(def))),
		B2JointDefEnum::WheelJoint(ref def)=> Rc::new(RefCell::new(B2wheelJoint::new(def))),	
	}
}

pub(crate) fn new<D: UserDataType>(def: &B2jointDef<D>)-> B2joint<D>
{
	b2_assert(!Rc::ptr_eq(def.body_a.as_ref().unwrap(), def.body_b.as_ref().unwrap()));

	return B2joint
	{
		m_type : def.jtype,
		m_prev : None,
		m_next : None,
		m_body_a : def.body_a.clone().unwrap(),
		m_body_b : def.body_b.clone().unwrap(),
		m_index : 0,
		m_collide_connected : def.collide_connected,
		m_island_flag : false,
		m_user_data : def.user_data.clone(),
		m_edge_a:None,
		m_edge_b:None,
	}
}

pub(crate) fn  is_enabled<D: UserDataType>(this: &B2joint<D>) -> bool
{
	return this.m_body_a.borrow().is_enabled() && this.m_body_b.borrow().is_enabled();
}

pub(crate) fn  draw<D: UserDataType, T: B2jointTraitDyn<D>+?Sized>(_self: &T, draw: &mut dyn B2drawTrait)
{
	let xf1 = _self.get_base().m_body_a.borrow().get_transform();
	let xf2 = _self.get_base().m_body_b.borrow().get_transform();
	let x1: B2vec2 = xf1.p;
	let x2: B2vec2 = xf2.p;
	let p1: B2vec2 = _self.get_anchor_a();
	let p2: B2vec2 = _self.get_anchor_b();

	let color = B2color::new(0.5, 0.8, 0.8);

	match _self.as_derived()
	{
		JointAsDerived::EDistanceJoint(ref _def)=>{
			draw.draw_segment(p1, p2, color);
		},
		JointAsDerived::EPulleyJoint(ref pulley)=>{
			let s1:B2vec2 = pulley.get_ground_anchor_a();
			let s2:B2vec2 = pulley.get_ground_anchor_b();
			draw.draw_segment(s1, p1, color);
			draw.draw_segment(s2, p2, color);
			draw.draw_segment(s1, s2, color);
		},
		JointAsDerived::EMouseJoint(ref _def)=>{
			let c = B2color::new(0.0, 1.0, 0.0);
			draw.draw_point(p1, 4.0, c);
			draw.draw_point(p2, 4.0, c);

			let c = B2color::new(0.8, 0.8, 0.8);
			draw.draw_segment(p1, p2, c);
		},
		_=>{
			draw.draw_segment(x1, p1, color);
			draw.draw_segment(p1, p2, color);
			draw.draw_segment(x2, p2, color);
		}
	}
}
