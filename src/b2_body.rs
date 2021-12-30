use std::cell::RefCell;
use std::rc::{Rc, Weak};

#[cfg(feature="serde_support")]
use serde::{Serialize, Deserialize};

use crate::b2_contact::*;
use crate::b2_fixture::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2rs_common::UserDataType;
use crate::b2_shape::*;
use crate::b2_world::*;
use crate::private::dynamics::b2_body as private;

use crate::linked_list::*;
use crate::double_linked_list::*;

use bitflags::bitflags;

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub enum B2bodyType {
	B2StaticBody = 0,
	B2KinematicBody,
	B2DynamicBody,
}

impl Default for B2bodyType {
	fn default() -> Self {
		return B2bodyType::B2StaticBody;
	}
}

impl<D: UserDataType> Default for B2bodyDef<D> {
	/// This constructor sets the body definition default values.
	fn default() -> Self {
		return Self {
			position: B2vec2::zero(),
			angle: 0.0,
			linear_velocity: B2vec2::zero(),
			angular_velocity: 0.0,
			linear_damping: 0.0,
			angular_damping: 0.0,
			allow_sleep: true,
			awake: true,
			fixed_rotation: false,
			bullet: false,
			body_type: B2bodyType::B2StaticBody,
			enabled: true,
			gravity_scale: 1.0,
			user_data: None,
		};
	}
}

impl<D:UserDataType> LinkedListNode<B2body<D>> for B2body<D>
{
    fn get_next(&self) -> Option<BodyPtr<D>> {
        return self.m_next.clone();
	}
	fn set_next(&mut self, value: Option<BodyPtr<D>>)
    {
        self.m_next = value;
    }
	fn take_next(&mut self) -> Option<BodyPtr<D>> {
		return self.m_next.take();
	}
}

impl<D:UserDataType> DoubleLinkedListNode<B2body<D>> for B2body<D>
{ 	
	fn get_prev(&self) -> Option<BodyWeakPtr<D>>
	{
		return self.m_prev.clone();
	}
	fn set_prev(&mut self, value: Option<BodyWeakPtr<D>>)
	{
		self.m_prev = value;
	}
}

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct B2bodyDef<D: UserDataType> {
	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	pub body_type: B2bodyType,

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	pub position: B2vec2,

	/// The world angle of the body in radians.
	pub angle: f32,

	/// The linear velocity of the body's origin in world co-ordinates.
	pub linear_velocity: B2vec2,

	/// The angular velocity of the body.
	pub angular_velocity: f32,

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Units are 1/time
	pub linear_damping: f32,

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Units are 1/time
	pub angular_damping: f32,

	/// Set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	pub allow_sleep: bool,

	/// Is this body initially awake or sleeping?
	pub awake: bool,

	/// Should this body be prevented from rotating? Useful for characters.
	pub fixed_rotation: bool,

	/// Is this a fast moving body that should be prevented from tunneling through
	/// other moving bodies? Note that all bodies are prevented from tunneling through
	/// kinematic and static bodies. This setting is only considered on dynamic bodies.
	/// @warning You should use this flag sparingly since it increases processing time.
	pub bullet: bool,

	/// Does this body start out enabled?
	pub enabled: bool,

	/// Use this to store application specific body data.
	pub user_data: Option<D::Body>,

	/// Scale the gravity applied to this body.
	pub gravity_scale: f32,
}

pub type BodyPtr<D> = Rc<RefCell<B2body<D>>>;
pub type BodyWeakPtr<D> = Weak<RefCell<B2body<D>>>;

/// A rigid body. These are created via B2world::create_body.
#[derive(Default)]
pub struct B2body<D: UserDataType>
//<BodyData>
{
	pub(crate) m_type: B2bodyType,

	pub(crate) m_flags: BodyFlags,

	pub(crate) m_island_index: i32,

	pub(crate) m_xf: B2Transform, // the body origin transform
	pub(crate) m_sweep: B2Sweep,  // the swept motion for CCD

	pub(crate) m_linear_velocity: B2vec2,
	pub(crate) m_angular_velocity: f32,

	pub(crate) m_force: B2vec2,
	pub(crate) m_torque: f32,

	pub(crate) m_world: B2worldWeakPtr<D>,
	pub(crate) m_prev: Option<BodyWeakPtr<D>>,
	pub(crate) m_next: Option<BodyPtr<D>>,

	pub(crate) m_fixture_list: LinkedList<B2fixture<D>>,
	pub(crate) m_fixture_count: i32,

	pub(crate) m_joint_list: DoubleLinkedList<B2jointEdge<D>>,
	pub(crate) m_contact_list: DoubleLinkedList<B2contactEdge<D>>,

	pub(crate) m_mass: f32,
	pub(crate) m_inv_mass: f32,

	// Rotational inertia about the center of mass.
	pub(crate) m_i: f32,
	pub(crate) m_inv_i: f32,

	pub(crate) m_linear_damping: f32,
	pub(crate) m_angular_damping: f32,
	pub(crate) m_gravity_scale: f32,

	pub(crate) m_sleep_time: f32,

	pub(crate) m_user_data: Option<D::Body>,
}


impl<D: UserDataType> Drop for B2body<D>
{
    fn drop(&mut self) {
		self.m_fixture_list.remove_all();
    }
}


bitflags! {
	/// m_flags
	#[derive(Default)]
	#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
	pub struct BodyFlags: u16 {
		const E_ISLAND_FLAG		= 0x0001;
		const E_AWAKE_FLAG			= 0x0002;
		const E_AUTO_SLEEP_FLAG		= 0x0004;
		const E_BULLET_FLAG		= 0x0008;
		const E_FIXED_ROTATION_FLAG	= 0x0010;
		const E_ENABLED_FLAG		= 0x0020;
		const E_TOI_FLAG			= 0x0040;
	}
}

impl<D: UserDataType> B2body<D> {
	/// Creates a fixture and attach it to this body. Use this function if you need
	/// to set some fixture parameters, like friction. Otherwise you can create the
	/// fixture directly from a shape.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// Contacts are not created until the next time step.
	/// * `def` - the fixture definition.
	/// @warning This function is locked during callbacks.
	pub fn create_fixture(self_: BodyPtr<D>, def: &B2fixtureDef<D>) -> FixturePtr<D> {
		return private::create_fixture(self_, def);
	}

	/// Creates a fixture from a shape and attach it to this body.
	/// This is a convenience function. Use B2fixtureDef if you need to set parameters
	/// like friction, restitution, user data, or filtering.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// * `shape` - the shape to be cloned.
	/// * `density` - the shape density (set to zero for static bodies).
	/// @warning This function is locked during callbacks.
	pub fn create_fixture_by_shape(self_: BodyPtr<D>, shape: ShapeDefPtr, density: f32) -> FixturePtr<D> {
		return private::create_fixture_by_shape(self_, shape, density);
	}

	/// destroy a fixture. This removes the fixture from the broad-phase and
	/// destroys all contacts associated with this fixture. This will
	/// automatically adjust the mass of the body if the body is dynamic and the
	/// fixture has positive density.
	/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	/// * `fixture` - the fixture to be removed.
	/// @warning This function is locked during callbacks.
	pub fn destroy_fixture(self_: BodyPtr<D>, fixture: FixturePtr<D>) {
		private::destroy_fixture(self_, fixture);
	}

	/// Set the position of the body's origin and rotation.
	/// Manipulating a body's transform may cause non-physical behavior.
	/// Note: contacts are updated on the next call to B2world::step.
	/// * `position` - the world position of the body's local origin.
	/// * `angle` - the world rotation in radians.
	pub fn set_transform(&mut self, position: B2vec2, angle: f32) {
		private::set_transform(self, position, angle);
	}

	/// Get the body transform for the body's origin.
	/// @return the world transform of the body's origin.
	pub fn get_transform(&self) -> B2Transform {
		return inline::get_transform(self);
	}

	/// Get the world body origin position.
	/// @return the world position of the body's origin.
	pub fn get_position(&self) -> B2vec2 {
		return inline::get_position(self);
	}

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	pub fn get_angle(&self) -> f32 {
		return inline::get_angle(self);
	}

	/// Get the world position of the center of mass.
	pub fn get_world_center(&self) -> B2vec2 {
		return inline::get_world_center(self);
	}

	/// Get the local position of the center of mass.
	pub fn get_local_center(&self) -> B2vec2 {
		return inline::get_local_center(self);
	}

	/// Set the linear velocity of the center of mass.
	/// * `v` - the new linear velocity of the center of mass.
	pub fn set_linear_velocity(&mut self, v: B2vec2) {
		inline::set_linear_velocity(self, v);
	}

	/// Get the linear velocity of the center of mass.
	/// @return the linear velocity of the center of mass.
	pub fn get_linear_velocity(&self) -> B2vec2 {
		return inline::get_linear_velocity(self);
	}

	/// Set the angular velocity.
	/// * `omega` - the new angular velocity in radians/second.
	pub fn set_angular_velocity(&mut self, omega: f32) {
		inline::set_angular_velocity(self, omega);
	}

	/// Get the angular velocity.
	/// @return the angular velocity in radians/second.
	pub fn get_angular_velocity(&self) -> f32 {
		return inline::get_angular_velocity(self);
	}

	/// Apply a force at a world point. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// * `force` - the world force vector, usually in Newtons (n).
	/// * `point` - the world position of the point of application.
	/// * `wake` - also wake up the body
	pub fn apply_force(&mut self, force: B2vec2, point: B2vec2, wake: bool) {
		inline::apply_force(self, force, point, wake);
	}

	/// Apply a force to the center of mass. This wakes up the body.
	/// * `force` - the world force vector, usually in Newtons (n).
	/// * `wake` - also wake up the body
	pub fn apply_force_to_center(&mut self, force: B2vec2, wake: bool) {
		inline::apply_force_to_center(self, force, wake);
	}

	/// Apply a torque. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// * `torque` - about the z-axis (out of the screen), usually in n-m.
	/// * `wake` - also wake up the body
	pub fn apply_torque(&mut self, torque: f32, wake: bool) {
		inline::apply_torque(self, torque, wake);
	}

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// * `impulse` - the world impulse vector, usually in n-seconds or kg-m/s.
	/// * `point` - the world position of the point of application.
	/// * `wake` - also wake up the body
	pub fn apply_linear_impulse(&mut self, impulse: B2vec2, point: B2vec2, wake: bool) {
		inline::apply_linear_impulse(self, impulse, point, wake);
	}

	/// Apply an impulse to the center of mass. This immediately modifies the velocity.
	/// * `impulse` - the world impulse vector, usually in n-seconds or kg-m/s.
	/// * `wake` - also wake up the body
	pub fn apply_linear_impulse_to_center(&mut self, impulse: B2vec2, wake: bool) {
		inline::apply_linear_impulse_to_center(self, impulse, wake);
	}

	/// Apply an angular impulse.
	/// * `impulse` - the angular impulse in units of kg*m*m/s
	/// * `wake` - also wake up the body
	pub fn apply_angular_impulse(&mut self, impulse: f32, wake: bool) {
		inline::apply_angular_impulse(self, impulse, wake);
	}

	/// Get the total mass of the body.
	/// @return the mass, usually in kilograms (kg).
	pub fn get_mass(&self) -> f32 {
		return inline::get_mass(self);
	}

	/// Get the rotational inertia of the body about the local origin.
	/// @return the rotational inertia, usually in kg-m^2.
	pub fn get_inertia(&self) -> f32 {
		return inline::get_inertia(self);
	}

	/// Get the mass data of the body.
	/// @return a struct containing the mass, inertia and center of the body.
	pub fn get_mass_data(&self, data: &mut B2massData) {
		inline::get_mass_data(self, data);
	}

	/// Set the mass properties to override the mass properties of the fixtures.
	/// Note that this changes the center of mass position.
	/// Note that creating or destroying fixtures can also alter the mass.
	/// This function has no effect if the body isn't dynamic.
	/// * `data` - the mass properties.
	pub fn set_mass_data(&mut self, mass_data: &B2massData) {
		private::set_mass_data(self, mass_data);
	}

	/// This resets the mass properties to the sum of the mass properties of the fixtures.
	/// This normally does not need to be called unless you called set_mass_data to override
	/// the mass and you later want to reset the mass.
	pub fn reset_mass_data(&mut self) {
		private::reset_mass_data(self);
	}

	/// Get the world coordinates of a point given the local coordinates.
	/// * `local_point` - a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.
	pub fn get_world_point(&self, local_point: B2vec2) -> B2vec2 {
		return inline::get_world_point(self, local_point);
	}

	/// Get the world coordinates of a vector given the local coordinates.
	/// * `local_vector` - a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	pub fn get_world_vector(&self, local_vector: B2vec2) -> B2vec2 {
		return inline::get_world_vector(self, local_vector);
	}

	/// Gets a local point relative to the body's origin given a world point.
	/// * `world_point` - a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	pub fn get_local_point(&self, world_point: B2vec2) -> B2vec2 {
		return inline::get_local_point(self, world_point);
	}

	/// Gets a local vector given a world vector.
	/// * `world_vector` - a vector in world coordinates.
	/// @return the corresponding local vector.
	pub fn get_local_vector(&self, world_vector: B2vec2) -> B2vec2 {
		return inline::get_local_vector(self, world_vector);
	}

	/// Get the world linear velocity of a world point attached to this body.
	/// * `world_point` - a point in world coordinates.
	/// @return the world velocity of a point.
	pub fn get_linear_velocity_from_world_point(&self, world_point: B2vec2) -> B2vec2 {
		return inline::get_linear_velocity_from_world_point(self, world_point);
	}

	/// Get the world velocity of a local point.
	/// * `local_point` - a point in local coordinates.
	/// @return the world velocity of a point.
	pub fn get_linear_velocity_from_local_point(&self, local_point: B2vec2) -> B2vec2 {
		return inline::get_linear_velocity_from_local_point(self, local_point);
	}

	/// Get the linear damping of the body.
	pub fn get_linear_damping(&self) -> f32 {
		return inline::get_linear_damping(self);
	}

	/// Set the linear damping of the body.
	pub fn set_linear_damping(&mut self, linear_damping: f32) {
		inline::set_linear_damping(self, linear_damping);
	}

	/// Get the angular damping of the body.
	pub fn get_angular_damping(&self) -> f32 {
		return inline::get_angular_damping(self);
	}

	/// Set the angular damping of the body.
	pub fn set_angular_damping(&mut self, angular_damping: f32) {
		inline::set_angular_damping(self, angular_damping);
	}

	/// Get the gravity scale of the body.
	pub fn get_gravity_scale(&self) -> f32 {
		return inline::get_gravity_scale(self);
	}

	/// Set the gravity scale of the body.
	pub fn set_gravity_scale(&mut self, scale: f32) {
		inline::set_gravity_scale(self, scale);
	}

	/// Set the type of this body. This may alter the mass and velocity.
	pub fn set_type(self_: BodyPtr<D>, body_type: B2bodyType) {
		private::set_type(self_, body_type);
	}

	/// Get the type of this body.
	pub fn get_type(&self) -> B2bodyType {
		return inline::get_type(self);
	}

	/// Should this body be treated like a bullet for continuous collision detection?
	pub fn set_bullet(&mut self, flag: bool) {
		inline::set_bullet(self, flag);
	}

	/// Is this body treated like a bullet for continuous collision detection?
	pub fn is_bullet(&self) -> bool {
		return inline::is_bullet(self);
	}

	/// You can disable sleeping on this body. If you disable sleeping, the
	/// body will be woken.
	pub fn set_sleeping_allowed(&mut self, flag: bool) {
		inline::set_sleeping_allowed(self, flag);
	}

	/// Is this body allowed to sleep
	pub fn is_sleeping_allowed(&self) -> bool {
		return inline::is_sleeping_allowed(self);
	}

	/// Set the sleep state of the body. A sleeping body has very
	/// low CPU cost.
	/// * `flag` - set to true to wake the body, false to put it to sleep.
	pub fn set_awake(&mut self, flag: bool) {
		inline::set_awake(self, flag);
	}

	/// Get the sleeping state of this body.
	/// @return true if the body is awake.
	pub fn is_awake(&self) -> bool {
		return inline::is_awake(self);
	}

	/// Allow a body to be disabled. A disabled body is not simulated and cannot
	/// be collided with or woken up.
	/// If you pass a flag of true, all fixtures will be added to the broad-phase.
	/// If you pass a flag of false, all fixtures will be removed from the
	/// broad-phase and all contacts will be destroyed.
	/// Fixtures and joints are otherwise unaffected. You may continue
	/// to create/destroy fixtures and joints on disabled bodies.
	/// Fixtures on a disabled body are implicitly disabled and will
	/// not participate in collisions, ray-casts, or queries.
	/// Joints connected to a disabled body are implicitly disabled.
	/// An diabled body is still owned by a B2world object and remains
	/// in the body list.
	pub fn set_enabled(self_: BodyPtr<D>, flag: bool) {
		private::set_enabled(self_, flag);
	}

	/// Get the active state of the body.
	pub fn is_enabled(&self) -> bool {
		return inline::is_enabled(self);
	}

	/// Set this body to have fixed rotation. This causes the mass
	/// to be reset.
	pub fn set_fixed_rotation(&mut self, flag: bool) {
		private::set_fixed_rotation(self, flag);
	}

	/// Does this body have fixed rotation?
	pub fn is_fixed_rotation(&self) -> bool {
		return inline::is_fixed_rotation(self);
	}

	/// Get the list of all fixtures attached to this body.
	pub fn get_fixture_list(&self) -> &LinkedList<B2fixture<D>> {
		return inline::get_fixture_list(self);
	}
	// pub fn get_fixture_list_mut(&mut self) -> &mut Option<FixturePtr<D>> {
	// 	return inline::get_fixture_list_mut(self);
	// }

	/// Get the list of all joints attached to this body.
	pub fn get_joint_list(&self) -> &DoubleLinkedList<B2jointEdge<D>> {
		return inline::get_joint_list(self);
	}
	pub fn get_joint_list_mut(&mut self) -> &mut DoubleLinkedList<B2jointEdge<D>> {
		return inline::get_joint_list_mut(self);
	}

	/// Get the list of all contacts attached to this body.
	/// @warning this list changes during the time step and you may
	/// miss some collisions if you don't use B2contactListener.
	pub fn get_contact_list(&self) -> &DoubleLinkedList<B2contactEdge<D>> {
		return inline::get_contact_list(self);
	}
	// pub fn get_contact_list_mut(&mut self) -> &mut Option<ContactEdgePtr<D>> {
	// 	return inline::get_contact_list_mut(self);
	// }

	/// Get the next body in the world's body list.
	pub fn get_next(&self) -> Option<BodyPtr<D>> {
		return inline::get_next(self);
	}

	/// Get the user data pointer that was provided in the body definition.
	pub fn get_user_data(&self) -> Option<D::Body> {
		return inline::get_user_data(self);
	}

	/// Set the user data. Use this to store your application specific data.
	pub fn set_user_data(&mut self, data: &D::Body) {
		inline::set_user_data(self, data);
	}

	/// Get the parent world of this body.
	pub fn get_world(&self) -> B2worldPtr<D> {
		return inline::get_world(self);
	}

	// private:

	pub(crate) fn new(bd: &B2bodyDef<D>, world: B2worldPtr<D>) -> Self {
		return private::b2_body(bd, world);
	}

	pub(crate) fn synchronize_fixtures(&mut self) {
		private::synchronize_fixtures(self);
	}
	pub(crate) fn synchronize_fixtures_by_world(&mut self, world: &B2world<D>) {
		private::synchronize_fixtures_by_world(self, world);
	}
	pub(crate) fn synchronize_transform(&mut self) {
		inline::synchronize_transform(self);
	}

	// This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collide_connected flag.
	pub(crate) fn should_collide(&self, other: BodyPtr<D>) -> bool {
		return private::should_collide(self, other);
	}

	pub(crate) fn advance(&mut self, t: f32) {
		inline::advance(self, t);
	}
}

//TODO_humman остальные модули так же переделать
mod inline {
	use super::*;

	pub fn get_type<D: UserDataType>(this: &B2body<D>) -> B2bodyType {
		return this.m_type;
	}

	pub fn get_transform<D: UserDataType>(this: &B2body<D>) -> B2Transform {
		return this.m_xf;
	}

	pub fn get_position<D: UserDataType>(this: &B2body<D>) -> B2vec2 {
		return this.m_xf.p;
	}

	pub fn get_angle<D: UserDataType>(this: &B2body<D>) -> f32 {
		return this.m_sweep.a;
	}

	pub fn get_world_center<D: UserDataType>(this: &B2body<D>) -> B2vec2 {
		return this.m_sweep.c;
	}

	pub fn get_local_center<D: UserDataType>(this: &B2body<D>) -> B2vec2 {
		return this.m_sweep.local_center;
	}

	pub fn set_linear_velocity<D: UserDataType>(this: &mut B2body<D>, v: B2vec2) {
		if this.m_type == B2bodyType::B2StaticBody {
			return;
		}

		if b2_dot(v, v) > 0.0 {
			this.set_awake(true);
		}

		this.m_linear_velocity = v;
	}

	pub fn get_linear_velocity<D: UserDataType>(this: &B2body<D>) -> B2vec2 {
		return this.m_linear_velocity;
	}

	pub fn set_angular_velocity<D: UserDataType>(this: &mut B2body<D>, w: f32) {
		if this.m_type == B2bodyType::B2StaticBody {
			return;
		}

		if w * w > 0.0 {
			this.set_awake(true);
		}

		this.m_angular_velocity = w;
	}

	pub fn get_angular_velocity<D: UserDataType>(this: &B2body<D>) -> f32 {
		return this.m_angular_velocity;
	}

	pub fn get_mass<D: UserDataType>(this: &B2body<D>) -> f32 {
		return this.m_mass;
	}

	pub fn get_inertia<D: UserDataType>(this: &B2body<D>) -> f32 {
		return this.m_i
			+ this.m_mass * b2_dot(this.m_sweep.local_center, this.m_sweep.local_center);
	}

	pub fn get_mass_data<D: UserDataType>(this: &B2body<D>, data: &mut B2massData) {
		data.mass = this.m_mass;
		data.i =
			this.m_i + this.m_mass * b2_dot(this.m_sweep.local_center, this.m_sweep.local_center);
		data.center = this.m_sweep.local_center;
	}

	pub fn get_world_point<D: UserDataType>(this: &B2body<D>, local_point: B2vec2) -> B2vec2 {
		return b2_mul_transform_by_vec2(this.m_xf, local_point);
	}

	pub fn get_world_vector<D: UserDataType>(this: &B2body<D>, local_vector: B2vec2) -> B2vec2 {
		return b2_mul_rot_by_vec2(this.m_xf.q, local_vector);
	}

	pub fn get_local_point<D: UserDataType>(this: &B2body<D>, world_point: B2vec2) -> B2vec2 {
		return b2_mul_t_transform_by_vec2(this.m_xf, world_point);
	}

	pub fn get_local_vector<D: UserDataType>(this: &B2body<D>, world_vector: B2vec2) -> B2vec2 {
		return b2_mul_t_rot_by_vec2(this.m_xf.q, world_vector);
	}

	pub fn get_linear_velocity_from_world_point<D: UserDataType>(
		this: &B2body<D>,
		world_point: B2vec2,
	) -> B2vec2 {
		return this.m_linear_velocity
			+ b2_cross_scalar_by_vec(this.m_angular_velocity, world_point - this.m_sweep.c);
	}

	pub fn get_linear_velocity_from_local_point<D: UserDataType>(
		this: &B2body<D>,
		local_point: B2vec2,
	) -> B2vec2 {
		return this.get_linear_velocity_from_world_point(this.get_world_point(local_point));
	}

	pub fn get_linear_damping<D: UserDataType>(this: &B2body<D>) -> f32 {
		return this.m_linear_damping;
	}

	pub fn set_linear_damping<D: UserDataType>(this: &mut B2body<D>, linear_damping: f32) {
		this.m_linear_damping = linear_damping;
	}

	pub fn get_angular_damping<D: UserDataType>(this: &B2body<D>) -> f32 {
		return this.m_angular_damping;
	}

	pub fn set_angular_damping<D: UserDataType>(this: &mut B2body<D>, angular_damping: f32) {
		this.m_angular_damping = angular_damping;
	}

	pub fn get_gravity_scale<D: UserDataType>(this: &B2body<D>) -> f32 {
		return this.m_gravity_scale;
	}

	pub fn set_gravity_scale<D: UserDataType>(this: &mut B2body<D>, scale: f32) {
		this.m_gravity_scale = scale;
	}

	pub fn set_bullet<D: UserDataType>(this: &mut B2body<D>, flag: bool) {
		this.m_flags.set(BodyFlags::E_BULLET_FLAG, flag);
	}

	pub fn is_bullet<D: UserDataType>(this: &B2body<D>) -> bool {
		return this.m_flags.contains(BodyFlags::E_BULLET_FLAG);
	}

	pub fn set_awake<D: UserDataType>(this: &mut B2body<D>, flag: bool) {

		if this.m_type == B2bodyType::B2StaticBody
		{
			return;
	
		}

		this.m_flags.set(BodyFlags::E_AWAKE_FLAG, flag);
		if flag {
			this.m_sleep_time = 0.0;
		} else {
			this.m_sleep_time = 0.0;
			this.m_linear_velocity.set_zero();
			this.m_angular_velocity = 0.0;
			this.m_force.set_zero();
			this.m_torque = 0.0;
		}
	}

	pub fn is_awake<D: UserDataType>(this: &B2body<D>) -> bool {
		return this.m_flags.contains(BodyFlags::E_AWAKE_FLAG);
	}

	pub fn is_enabled<D: UserDataType>(this: &B2body<D>) -> bool {
		return this.m_flags.contains(BodyFlags::E_ENABLED_FLAG);
	}

	pub fn is_fixed_rotation<D: UserDataType>(this: &B2body<D>) -> bool {
		return this.m_flags.contains(BodyFlags::E_FIXED_ROTATION_FLAG);
	}

	pub fn set_sleeping_allowed<D: UserDataType>(this: &mut B2body<D>, flag: bool) {
		this.m_flags.set(BodyFlags::E_AUTO_SLEEP_FLAG, flag);
		if flag {
		} else {
			this.set_awake(true);
		}
	}

	pub fn is_sleeping_allowed<D: UserDataType>(this: &B2body<D>) -> bool {
		return this.m_flags.contains(BodyFlags::E_AUTO_SLEEP_FLAG);
	}

	// pub fn get_fixture_list_mut<D: UserDataType>(
	// 	this: &mut B2body<D>,
	// ) -> &mut Option<FixturePtr<D>> {
	// 	return &mut this.m_fixture_list;
	// }

	pub fn get_fixture_list<D: UserDataType>(this: &B2body<D>) -> &LinkedList<B2fixture<D>> {
		return &this.m_fixture_list;
	}

	pub fn get_joint_list_mut<D: UserDataType>(
		this: &mut B2body<D>,
	) -> &mut DoubleLinkedList<B2jointEdge<D>> {
		return &mut this.m_joint_list;
	}

	pub fn get_joint_list<D: UserDataType>(this: &B2body<D>) -> &DoubleLinkedList<B2jointEdge<D>> {
		return &this.m_joint_list;
	}

	// pub fn get_contact_list_mut<D: UserDataType>(
	// 	this: &mut B2body<D>,
	// ) -> &mut Option<ContactEdgePtr<D>> {
	// 	return &mut this.m_contact_list.head;
	// }

	pub fn get_contact_list<D: UserDataType>(this: &B2body<D>) -> &DoubleLinkedList<B2contactEdge<D>> {
		return &this.m_contact_list;
	}

	pub fn get_next<D: UserDataType>(this: &B2body<D>) -> Option<BodyPtr<D>> {
		return this.m_next.clone();
	}

	pub fn set_user_data<D: UserDataType>(this: &mut B2body<D>, data: &D::Body) {
		this.m_user_data = Some(data.clone());
	}

	pub fn get_user_data<D: UserDataType>(this: &B2body<D>) -> Option<D::Body> {
		return this.m_user_data.clone();
	}

	pub fn apply_force<D: UserDataType>(
		this: &mut B2body<D>,
		force: B2vec2,
		point: B2vec2,
		wake: bool,
	) {
		if this.m_type != B2bodyType::B2DynamicBody {
			return;
		}

		if wake && !this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.set_awake(true);
		}

		// Don't accumulate a force if the body is sleeping.
		if this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.m_force += force;
			this.m_torque += b2_cross(point - this.m_sweep.c, force);
		}
	}

	pub fn apply_force_to_center<D: UserDataType>(this: &mut B2body<D>, force: B2vec2, wake: bool) {
		if this.m_type != B2bodyType::B2DynamicBody {
			return;
		}

		if wake && !this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.set_awake(true);
		}

		// Don't accumulate a force if the body is sleeping
		if this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.m_force += force;
		}
	}

	pub fn apply_torque<D: UserDataType>(this: &mut B2body<D>, torque: f32, wake: bool) {
		if this.m_type != B2bodyType::B2DynamicBody {
			return;
		}

		if wake && !this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.set_awake(true);
		}

		// Don't accumulate a force if the body is sleeping
		if this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.m_torque += torque;
		}
	}

	pub fn apply_linear_impulse<D: UserDataType>(
		this: &mut B2body<D>,
		impulse: B2vec2,
		point: B2vec2,
		wake: bool,
	) {
		if this.m_type != B2bodyType::B2DynamicBody {
			return;
		}

		if wake && !this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.set_awake(true);
		}

		// Don't accumulate velocity if the body is sleeping
		if this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.m_linear_velocity += this.m_inv_mass * impulse;
			this.m_angular_velocity += this.m_inv_i * b2_cross(point - this.m_sweep.c, impulse);
		}
	}

	pub fn apply_linear_impulse_to_center<D: UserDataType>(
		this: &mut B2body<D>,
		impulse: B2vec2,
		wake: bool,
	) {
		if this.m_type != B2bodyType::B2DynamicBody {
			return;
		}
		if wake && !this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.set_awake(true);
		}

		// Don't accumulate velocity if the body is sleeping
		if this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.m_linear_velocity += this.m_inv_mass * impulse;
		}
	}

	pub fn apply_angular_impulse<D: UserDataType>(this: &mut B2body<D>, impulse: f32, wake: bool) {
		if this.m_type != B2bodyType::B2DynamicBody {
			return;
		}

		if wake && !this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.set_awake(true);
		}

		// Don't accumulate velocity if the body is sleeping
		if this.m_flags.contains(BodyFlags::E_AWAKE_FLAG) {
			this.m_angular_velocity += this.m_inv_i * impulse;
		}
	}

	pub fn synchronize_transform<D: UserDataType>(this: &mut B2body<D>) {
		this.m_xf.q.set(this.m_sweep.a);
		this.m_xf.p = this.m_sweep.c - b2_mul_rot_by_vec2(this.m_xf.q, this.m_sweep.local_center);
	}

	pub fn advance<D: UserDataType>(this: &mut B2body<D>, alpha: f32) {
		// advance to the new safe time. This doesn't sync the broad-phase.
		this.m_sweep.advance(alpha);
		this.m_sweep.c = this.m_sweep.c0;
		this.m_sweep.a = this.m_sweep.a0;
		this.m_xf.q.set(this.m_sweep.a);
		this.m_xf.p = this.m_sweep.c - b2_mul_rot_by_vec2(this.m_xf.q, this.m_sweep.local_center);
	}

	pub fn get_world<D: UserDataType>(this: &B2body<D>) -> B2worldPtr<D> {
		return this.m_world.upgrade().unwrap();
	}
}
