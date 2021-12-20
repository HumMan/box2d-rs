use std::cell::RefCell;
use std::rc::{Rc, Weak};

use crate::b2_body::*;
use crate::b2_collision::*;
use crate::b2_contact::*;
use crate::b2_contact_manager::*;
use crate::b2_draw::*;
use crate::b2_fixture::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2rs_common::*;
use crate::b2_time_step::*;
use crate::b2_world_callbacks::*;
use crate::double_linked_list::*;

use crate::private::dynamics::b2_world as private;

pub type B2worldPtr<D> = Rc<RefCell<B2world<D>>>;
pub type B2worldWeakPtr<D> = Weak<RefCell<B2world<D>>>;

impl<D: UserDataType> B2world<D> {
	/// Construct a world object.
	/// * `gravity` - the world gravity vector.
	pub fn new(gravity: B2vec2) -> B2worldPtr<D> {
		return private::b2_world_new(gravity);
	}

	/// Register a destruction listener. The listener is owned by you and must
	/// remain in scope.
	pub fn set_destruction_listener(&mut self, listener: B2destructionListenerPtr<D>) {
		private::set_destruction_listener(self, listener);
	}

	/// Register a contact filter to provide specific control over collision.
	/// Otherwise the default filter is used (b2_defaultFilter). The listener is
	/// owned by you and must remain in scope.
	pub fn set_contact_filter(&mut self, filter: B2contactFilterPtr<D>) {
		private::set_contact_filter(self, filter);
	}

	/// Register a contact event listener. The listener is owned by you and must
	/// remain in scope.
	pub fn set_contact_listener(&mut self, listener: B2contactListenerPtr<D>) {
		private::set_contact_listener(self, listener);
	}

	/// Register a routine for debug drawing. The debug draw functions are called
	/// inside with debug_draw method. The debug draw object is owned
	/// by you and must remain in scope.
	pub fn set_debug_draw(&mut self, debug_draw: B2drawTraitPtr) {
		private::set_debug_draw(self, debug_draw);
	}

	/// create a rigid body given a definition. No reference to the definition
	/// is retained.
	/// @warning This function is locked during callbacks.
	pub fn create_body(self_: B2worldPtr<D>, def: &B2bodyDef<D>) -> BodyPtr<D> {
		return private::create_body(self_, def);
	}

	/// destroy a rigid body given a definition. No reference to the definition
	/// is retained. This function is locked during callbacks.
	/// @warning This automatically deletes all associated shapes and joints.
	/// @warning This function is locked during callbacks.
	pub fn destroy_body(&mut self, b: BodyPtr<D>) {
		private::destroy_body(self, b);
	}

	/// create a joint to constrain bodies together. No reference to the definition
	/// is retained. This may cause the connected bodies to cease colliding.
	/// @warning This function is locked during callbacks.
	pub fn create_joint(&mut self, def: &B2JointDefEnum<D>) -> B2jointPtr<D> {
		return private::create_joint(self, def);
	}

	/// destroy a joint. This may cause the connected bodies to begin colliding.
	/// @warning This function is locked during callbacks.
	pub fn destroy_joint(&mut self, j: B2jointPtr<D>) {
		private::destroy_joint(self, j);
	}

	/// Take a time step. This performs collision detection, integration,
	/// and constraint solution.
	/// * `time_step` - the amount of time to simulate, this should not vary.
	/// * `velocity_iterations` - for the velocity constraint solver.
	/// * `position_iterations` - for the position constraint solver.
	pub fn step(&mut self, dt: f32, velocity_iterations: i32, position_iterations: i32) {
		private::step(self, dt, velocity_iterations, position_iterations);
	}

	/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
	/// after each call to step. The default behavior is modified by calling set_auto_clear_forces.
	/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
	/// a fixed sized time step under a variable frame-rate.
	/// When you perform sub-stepping you will disable auto clearing of forces and instead call
	/// clear_forces after all sub-steps are complete in one pass of your game loop.
	/// [see](set_auto_clear_forces)
	pub fn clear_forces(&mut self) {
		private::clear_forces(self);
	}

	/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
	pub fn debug_draw(&self) {
		private::debug_draw(self);
	}

	/// query the world for all fixtures that potentially overlap the
	/// provided AABB.
	/// * `callback` - a user implemented callback class.
	/// * `aabb` - the query box.
	pub fn query_aabb<F: B2queryCallback<D>>(&self, callback: F, aabb: B2AABB) {
		private::query_aabb(self, callback, aabb);
	}

	/// Ray-cast the world for all fixtures in the path of the ray. Your callback
	/// controls whether you get the closest point, any point, or n-points.
	/// The ray-cast ignores shapes that contain the starting point.
	/// * `callback` - a user implemented callback class.
	/// * `point1` - the ray starting point
	/// * `point2` - the ray ending point
	pub fn ray_cast<F: B2rayCastCallback<D>>(&self, callback: F, point1: B2vec2, point2: B2vec2) {
		private::ray_cast(self, callback, point1, point2);
	}
	/// Get the world body list. With the returned body, use b2_body::get_next to get
	/// the next body in the world list. A None body indicates the end of the list.
	/// @return the head of the world body list.
	pub fn get_body_list(&self) -> DoubleLinkedList<B2body<D>> {
		return inline::get_body_list(self);
	}

	/// Get the world joint list. With the returned joint, use B2joint::get_next to get
	/// the next joint in the world list. A None joint indicates the end of the list.
	/// @return the head of the world joint list.
	pub fn get_joint_list(&self) -> DoubleLinkedList<dyn B2jointTraitDyn<D>> {
		return inline::get_joint_list(self);
	}

	/// Get the world contact list. With the returned contact, use B2contact::get_next to get
	/// the next contact in the world list. A None contact indicates the end of the list.
	/// @return the head of the world contact list.
	/// @warning contacts are created and destroyed in the middle of a time step.
	/// Use B2contactListener to avoid missing contacts.
	pub fn get_contact_list(&self) -> DoubleLinkedList<dyn B2contactDynTrait<D>> {
		return inline::get_contact_list(self);
	}

	/// Enable/disable sleep.
	pub fn set_allow_sleeping(&mut self, flag: bool) {
		private::set_allow_sleeping(self, flag);
	}
	pub fn get_allow_sleeping(&self) -> bool {
		return self.m_allow_sleep;
	}

	/// Enable/disable warm starting. For testing.
	pub fn set_warm_starting(&mut self, flag: bool) {
		self.m_warm_starting = flag;
	}
	pub fn get_warm_starting(&self) -> bool {
		return self.m_warm_starting;
	}

	/// Enable/disable continuous physics. For testing.
	pub fn set_continuous_physics(&mut self, flag: bool) {
		self.m_continuous_physics = flag;
	}
	pub fn get_continuous_physics(&self) -> bool {
		return self.m_continuous_physics;
	}

	/// Enable/disable single stepped continuous physics. For testing.
	pub fn set_sub_stepping(&mut self, flag: bool) {
		self.m_sub_stepping = flag;
	}
	pub fn get_sub_stepping(&self) -> bool {
		return self.m_sub_stepping;
	}

	/// Get the number of broad-phase proxies.
	pub fn get_proxy_count(&self) -> i32 {
		return private::get_proxy_count(self);
	}

	/// Get the number of bodies.
	pub fn get_body_count(&self) -> usize {
		return inline::get_body_count(self);
	}

	/// Get the number of joints.
	pub fn get_joint_count(&self) -> usize {
		return inline::get_joint_count(self);
	}

	/// Get the number of contacts (each may have 0 or more contact points).
	pub fn get_contact_count(&self) -> usize {
		return inline::get_contact_count(self);
	}

	/// Get the height of the dynamic tree.
	pub fn get_tree_height(&self) -> i32 {
		return private::get_tree_height(self);
	}

	/// Get the balance of the dynamic tree.
	pub fn get_tree_balance(&self) -> i32 {
		return private::get_tree_balance(self);
	}

	/// Get the quality metric of the dynamic tree. The smaller the better.
	/// The minimum is 1.
	pub fn get_tree_quality(&self) -> f32 {
		return private::get_tree_quality(self);
	}

	/// Change the global gravity vector.
	pub fn set_gravity(&mut self, gravity: B2vec2) {
		inline::set_gravity(self, gravity);
	}
	/// Get the global gravity vector.
	pub fn get_gravity(&self) -> B2vec2 {
		return inline::get_gravity(self);
	}

	/// Is the world locked (in the middle of a time step).
	pub fn is_locked(&self) -> bool {
		return inline::is_locked(self);
	}

	/// Set flag to control automatic clearing of forces after each time step.
	pub fn set_auto_clear_forces(&mut self, flag: bool) {
		inline::set_auto_clear_forces(self, flag);
	}

	/// Get the flag that controls automatic clearing of forces after each time step.
	pub fn get_auto_clear_forces(&self) -> bool {
		return inline::get_auto_clear_forces(self);
	}

	/// Shift the world origin. Useful for large worlds.
	/// The body shift formula is: position -= new_origin
	/// * `new_origin` - the new origin with respect to the old origin
	pub fn shift_origin(&self, new_origin: B2vec2) {
		private::shift_origin(self, new_origin);
	}

	/// Get the contact manager for testing.
	pub fn get_contact_manager(&self) -> B2contactManagerPtr<D> {
		return inline::get_contact_manager(self);
	}

	/// Get the current profile.
	pub fn get_profile(&self) -> B2Profile {
		return inline::get_profile(self);
	}

	// private:

	pub(crate) fn solve(&mut self, step: B2timeStep) {
		private::solve(self, step);
	}
	pub(crate) fn solve_toi(&mut self, step: B2timeStep) {
		private::solve_toi(self, step);
	}

	pub(crate) fn draw_shape(&self, fixture: FixturePtr<D>, xf: &B2Transform, color: &B2color) {
		private::draw_shape(self, fixture, xf, color);
	}
}

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
pub struct B2world<D: UserDataType> {
	// private:
	pub(crate) m_contact_manager: B2contactManagerPtr<D>,

	pub(crate) m_body_list: DoubleLinkedList<B2body<D>>,
	pub(crate) m_joint_list: DoubleLinkedList<dyn B2jointTraitDyn<D>>,

	pub(crate) m_body_count: usize,
	pub(crate) m_joint_count: usize,

	pub(crate) m_gravity: B2vec2,
	pub(crate) m_allow_sleep: bool,

	pub(crate) m_destruction_listener: Option<B2destructionListenerPtr<D>>,
	pub(crate) m_debug_draw: Option<B2drawTraitPtr>,

	// This is used to compute the time step ratio to
	// support a variable time step.
	pub(crate) m_inv_dt0: f32,

	pub(crate) m_new_contacts: bool,
	pub(crate) m_locked: bool,
	pub(crate) m_clear_forces: bool,

	// These are for debugging the solver.
	pub(crate) m_warm_starting: bool,
	pub(crate) m_continuous_physics: bool,
	pub(crate) m_sub_stepping: bool,

	pub(crate) m_step_complete: bool,

	pub(crate) m_profile: B2Profile,
}

impl<D: UserDataType> Drop for B2world<D>
{
    fn drop(&mut self) {
		self.m_body_list.remove_all();
		self.m_joint_list.remove_all();
    }
}

mod inline {
	use super::*;

	pub fn get_body_list<D: UserDataType>(this: &B2world<D>) -> DoubleLinkedList<B2body<D>> {
		return this.m_body_list.clone();
	}

	pub fn get_joint_list<D: UserDataType>(this: &B2world<D>) -> DoubleLinkedList<dyn B2jointTraitDyn<D>> {
		return this.m_joint_list.clone();
	}

	pub fn get_contact_list<D: UserDataType>(this: &B2world<D>) -> DoubleLinkedList<dyn B2contactDynTrait<D>> {
		return this.m_contact_manager.borrow().m_contact_list.clone();
	}

	pub fn get_body_count<D: UserDataType>(this: &B2world<D>) -> usize {
		return this.m_body_count;
	}

	pub fn get_joint_count<D: UserDataType>(this: &B2world<D>) -> usize {
		return this.m_joint_count;
	}

	pub fn get_contact_count<D: UserDataType>(this: &B2world<D>) -> usize {
		return this.m_contact_manager.borrow().m_contact_count;
	}

	pub fn set_gravity<D: UserDataType>(this: &mut B2world<D>, gravity: B2vec2) {
		this.m_gravity = gravity;
	}

	pub fn get_gravity<D: UserDataType>(this: &B2world<D>) -> B2vec2 {
		return this.m_gravity;
	}

	pub fn is_locked<D: UserDataType>(this: &B2world<D>) -> bool {
		return this.m_locked;
	}

	pub fn set_auto_clear_forces<D: UserDataType>(this: &mut B2world<D>, flag: bool) {
		this.m_clear_forces = flag;
	}

	/// Get the flag that controls automatic clearing of forces after each time step.
	pub fn get_auto_clear_forces<D: UserDataType>(this: &B2world<D>) -> bool {
		return this.m_clear_forces;
	}

	pub fn get_contact_manager<D: UserDataType>(this: &B2world<D>) -> B2contactManagerPtr<D> {
		return this.m_contact_manager.clone();
	}

	pub(crate) fn get_profile<D: UserDataType>(this: &B2world<D>) -> B2Profile {
		return this.m_profile;
	}
}
