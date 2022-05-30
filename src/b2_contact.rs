use crate::b2_body::*;
use crate::b2_fixture::*;
use crate::b2_math::*;


use std::cell::RefCell;
use std::rc::{Rc, Weak};

use crate::b2_collision::*;
use crate::b2rs_common::UserDataType;
use crate::b2_world_callbacks::*;
use crate::b2_contact_manager::*;

use crate::b2rs_linked_list::LinkedListNode;
use crate::b2rs_double_linked_list::DoubleLinkedListNode;

use crate::private::dynamics::b2_contact as private;

use bitflags::bitflags;

use std::sync::atomic::{AtomicBool};

//box2d-rs: from \src\private\dynamics\b2_contact_solver_private.rs
//not sure should it be public and atomic as it used only in box_stack example
pub static G_BLOCK_SOLVE: AtomicBool = AtomicBool::new(true);

/// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
/// For example, anything slides on ice.
pub fn b2_mix_friction(friction1: f32, friction2: f32) -> f32 {
	return b2_sqrt(friction1 * friction2);
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
pub fn b2_mix_restitution(restitution1: f32, restitution2: f32) -> f32 {
	return if restitution1 > restitution2 {
		restitution1
	} else {
		restitution2
	};
}

/// Restitution mixing law. This picks the lowest value.
pub fn b2_mix_restitution_threshold(threshold1: f32, threshold2: f32) -> f32 {
	return if threshold1 < threshold2 {threshold1} else {threshold2};
}

pub type ContactEdgePtr<D> = Rc<RefCell<B2contactEdge<D>>>;
pub type ContactEdgeWeakPtr<D> = Weak<RefCell<B2contactEdge<D>>>;

impl<D:UserDataType> LinkedListNode<B2contactEdge<D>> for B2contactEdge<D>
{
    fn get_next(&self) -> Option<ContactEdgePtr<D>> {
        return self.next.clone();
	}
	fn set_next(&mut self, value: Option<ContactEdgePtr<D>>)
    {
        self.next = value;
    }
	fn take_next(&mut self) -> Option<ContactEdgePtr<D>> {
		return self.next.take();
	}
}

impl<D:UserDataType> DoubleLinkedListNode<B2contactEdge<D>> for B2contactEdge<D>
{ 	
	fn get_prev(&self) -> Option<ContactEdgeWeakPtr<D>>
	{
		return self.prev.clone();
	}
	fn set_prev(&mut self, value: Option<ContactEdgeWeakPtr<D>>)
	{
		self.prev = value;
	}
}

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
#[derive(Clone)]
pub struct B2contactEdge<D: UserDataType> {
	///< provides quick access to the other body attached.
	pub other: BodyWeakPtr<D>,
	///< the contact
	pub contact: ContactWeakPtr<D>,
	///< the previous contact edge in the body's contact list
	pub prev: Option<ContactEdgeWeakPtr<D>>,
	///< the next contact edge in the body's contact list
	pub next: Option<ContactEdgePtr<D>>,
}

pub type ContactPtr<D> = Rc<RefCell<dyn B2contactDynTrait<D>>>;
pub type ContactWeakPtr<D> = Weak<RefCell<dyn B2contactDynTrait<D>>>;

pub trait B2contactDynTrait<D: UserDataType> {
	fn get_base<'a>(&'a self) -> &'a B2contact<D>;
	fn get_base_mut<'a>(&'a mut self) -> &'a mut B2contact<D>;
	/// evaluate this contact with your own manifold and transforms.
	fn evaluate(&self, manifold: &mut B2manifold, xf_a: &B2Transform, xf_b: &B2Transform);
}

impl<D:UserDataType> LinkedListNode<dyn B2contactDynTrait<D>> for dyn B2contactDynTrait<D>
{ 	
	fn get_next(&self) -> Option<Rc<RefCell<dyn B2contactDynTrait<D>>>>
	{
		return self.get_base().m_next.clone();
	}
	fn set_next(&mut self, value: Option<Rc<RefCell<dyn B2contactDynTrait<D>>>>)
	{
		self.get_base_mut().m_next = value;
	}
	fn take_next(&mut self) -> Option<Rc<RefCell<dyn B2contactDynTrait<D>>>> {
		return self.get_base_mut().m_next.take();
	}
}

impl<D:UserDataType> DoubleLinkedListNode<dyn B2contactDynTrait<D>> for dyn B2contactDynTrait<D>
{ 	
	fn get_prev(&self) -> Option<Weak<RefCell<dyn B2contactDynTrait<D>>>>
	{
		return self.get_base().m_prev.clone();
	}
	fn set_prev(&mut self, value: Option<Weak<RefCell<dyn B2contactDynTrait<D>>>>)
	{
		self.get_base_mut().m_prev = value;
	}
}

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
impl<D: UserDataType> B2contact<D> {
	/// Get the contact manifold. Do not modify the manifold unless you understand the
	/// internals of Box2D.
	pub fn get_manifold(&self) -> &B2manifold {
		return inline::get_manifold(self);
	}

	pub fn get_manifold_mut(&mut self) -> &mut B2manifold {
		return inline::get_manifold_mut(self);
	}

	/// Get the world manifold.
	pub fn get_world_manifold(&self, world_manifold: &mut B2worldManifold) {
		inline::get_world_manifold(self, world_manifold);
	}

	/// Is this contact touching?
	pub fn is_touching(&self) -> bool {
		return inline::is_touching(self);
	}

	/// Enable/disable this contact. This can be used inside the pre-solve
	/// contact listener. The contact is only disabled for the current
	/// time step (or sub-step in continuous collisions).
	pub fn set_enabled(&mut self, flag: bool) {
		inline::set_enabled(self, flag);
	}

	/// Has this contact been disabled?
	pub fn is_enabled(&self) -> bool {
		return inline::is_enabled(self);
	}

	/// Get the next contact in the world's contact list.
	pub fn get_next(&self) -> Option<ContactPtr<D>> {
		return inline::get_next(self);
	}

	/// Get fixture A in this contact.
	pub fn get_fixture_a(&self) -> FixturePtr<D> {
		return inline::get_fixture_a(self);
	}

	/// Get the child primitive index for fixture A.
	pub fn get_child_index_a(&self) -> i32 {
		return inline::get_child_index_a(self);
	}

	/// Get fixture b in this contact.
	pub fn get_fixture_b(&self) -> FixturePtr<D> {
		return inline::get_fixture_b(self);
	}

	/// Get the child primitive index for fixture b.
	pub fn get_child_index_b(&self) -> i32 {
		return inline::get_child_index_b(self);
	}

	/// Override the default friction mixture. You can call this in B2contactListener::pre_solve.
	/// This value persists until set or reset.
	pub fn set_friction(&mut self, friction: f32) {
		inline::set_friction(self, friction);
	}

	/// Get the friction.
	pub fn get_friction(&self) -> f32 {
		return inline::get_friction(self);
	}

	/// reset the friction mixture to the default value.
	pub fn reset_friction(&mut self) {
		inline::reset_friction(self);
	}

	/// Override the default restitution mixture. You can call this in B2contactListener::pre_solve.
	/// The value persists until you set or reset.
	pub fn set_restitution(&mut self, restitution: f32) {
		inline::set_restitution(self, restitution);
	}

	/// Get the restitution.
	pub fn get_restitution(&self) -> f32 {
		return inline::get_restitution(self);
	}
	/// reset the restitution to the default value.
	pub fn reset_restitution(&mut self) {
		inline::reset_restitution(self);
	}

	/// Override the default restitution velocity threshold mixture. You can call this in b2ContactListener::PreSolve.
	/// The value persists until you set or reset.
	pub fn  set_restitution_threshold(&mut self, threshold:f32){
		inline::set_restitution_threshold(self, threshold);
	}

	/// Get the restitution threshold.
	pub fn  get_restitution_threshold(&self)->f32{
		return inline::get_restitution_threshold(self);
	}

	/// Reset the restitution threshold to the default value.
	pub fn  reset_restitution_threshold(&mut self){
		inline::reset_restitution_threshold(self);
	}

	/// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
	pub fn set_tangent_speed(&mut self, speed: f32) {
		inline::set_tangent_speed(self, speed);
	}

	/// Get the desired tangent speed. In meters per second.
	pub fn get_tangent_speed(&self) -> f32 {
		return inline::get_tangent_speed(self);
	}

	//protected:

	/// Flag this contact for filtering. Filtering will occur the next time step.
	pub(crate) fn flag_for_filtering(&mut self) {
		inline::flag_for_filtering(self);
	}

	pub(crate) fn create(
		contact_manager: &B2contactManager<D>,
		fixture_a: FixturePtr<D>,
		index_a: i32,
		fixture_b: FixturePtr<D>,
		index_b: i32,
	) -> ContactPtr<D> {
		return private::b2_contact_create(contact_manager, fixture_a, index_a, fixture_b, index_b);
	}
	pub(crate) fn destroy(self_: &dyn B2contactDynTrait<D>) {
		private::b2_contact_destroy(self_);
	}

	pub(crate) fn new(f_a: FixturePtr<D>, index_a: i32, f_b: FixturePtr<D>, index_b: i32) -> Self {
		return private::b2_contact_new(f_a, index_a, f_b, index_b);
	}

	pub(crate) fn update(
		self_dyn: &mut dyn B2contactDynTrait<D>,
		listener: Option<B2contactListenerPtr<D>>,
	) where
		Self: Sized,
	{
		private::b2_contact_update(self_dyn, listener);
	}
}

bitflags! {
	// Flags stored in m_flags
	#[derive(Default)]
	pub(crate) struct ContactFlags: u32 {
		// Used when crawling contact graph when forming islands.
		const E_ISLAND_FLAG		= 0x0001;

		// Set when the shapes are touching.
		const E_TOUCHING_FLAG		= 0x0002;

		// This contact can be disabled (by user)
		const E_ENABLED_FLAG		= 0x0004;

		// This contact needs filtering because a fixture filter was changed.
		const E_FILTER_FLAG		= 0x0008;

		// This bullet contact had a TOI event
		const E_BULLET_HIT_FLAG		= 0x0010;

		// This contact has a valid TOI in m_toi
		const E_TOI_FLAG			= 0x0020;
	}
}



//#[derive(Default)]
pub struct B2contact<D: UserDataType> {
	//protected:

	pub(crate) m_flags: ContactFlags,

	// World pool and list pointers.
	pub(crate) m_prev: Option<ContactWeakPtr<D>>,
	pub(crate) m_next: Option<ContactPtr<D>>,

	// Nodes for connecting bodies.
	pub(crate) m_node_a: Option<ContactEdgePtr<D>>,
	pub(crate) m_node_b: Option<ContactEdgePtr<D>>,

	pub(crate) m_fixture_a: FixturePtr<D>,
	pub(crate) m_fixture_b: FixturePtr<D>,

	pub(crate) m_index_a: i32,
	pub(crate) m_index_b: i32,

	pub(crate) m_manifold: B2manifold,

	pub(crate) m_toi_count: i32,
	pub(crate) m_toi: f32,

	pub(crate) m_friction: f32,
	pub(crate) m_restitution: f32,
	pub(crate) m_restitution_threshold: f32,

	pub(crate) m_tangent_speed: f32,
}

mod inline {
	use super::*;

	pub fn get_manifold<D: UserDataType>(self_: &B2contact<D>) -> &B2manifold {
		return &self_.m_manifold;
	}

	pub fn get_manifold_mut<D: UserDataType>(self_: &mut B2contact<D>) -> &mut B2manifold {
		return &mut self_.m_manifold;
	}

	pub fn get_world_manifold<D: UserDataType>(
		self_: &B2contact<D>,
		world_manifold: &mut B2worldManifold,
	) {
		let body_a = self_.m_fixture_a.borrow().get_body();
		let body_b = self_.m_fixture_b.borrow().get_body();
		let shape_a = self_.m_fixture_a.borrow().get_shape();
		let shape_b = self_.m_fixture_b.borrow().get_shape();

		world_manifold.initialize(
			&self_.m_manifold,
			body_a.borrow().get_transform(),
			shape_a.get_base().m_radius,
			body_b.borrow().get_transform(),
			shape_b.get_base().m_radius,
		);
	}

	pub fn set_enabled<D: UserDataType>(self_: &mut B2contact<D>, flag: bool) {
		self_.m_flags.set(ContactFlags::E_ENABLED_FLAG, flag);
	}

	pub fn is_enabled<D: UserDataType>(self_: &B2contact<D>) -> bool {
		return self_.m_flags.contains(ContactFlags::E_ENABLED_FLAG);
	}

	pub fn is_touching<D: UserDataType>(self_: &B2contact<D>) -> bool {
		return self_.m_flags.contains(ContactFlags::E_TOUCHING_FLAG);
	}

	pub fn get_next<D: UserDataType>(self_: &B2contact<D>) -> Option<ContactPtr<D>> {
		return self_.m_next.clone();
	}

	pub fn get_fixture_a<D: UserDataType>(self_: &B2contact<D>) -> FixturePtr<D> {
		return self_.m_fixture_a.clone();
	}

	pub fn get_child_index_a<D: UserDataType>(self_: &B2contact<D>) -> i32 {
		return self_.m_index_a;
	}

	pub fn get_fixture_b<D: UserDataType>(self_: &B2contact<D>) -> FixturePtr<D> {
		return self_.m_fixture_b.clone();
	}

	pub fn get_child_index_b<D: UserDataType>(self_: &B2contact<D>) -> i32 {
		return self_.m_index_b;
	}

	pub fn flag_for_filtering<D: UserDataType>(self_: &mut B2contact<D>) {
		self_.m_flags.set(ContactFlags::E_FILTER_FLAG, true);
	}

	pub fn set_friction<D: UserDataType>(self_: &mut B2contact<D>, friction: f32) {
		self_.m_friction = friction;
	}

	pub fn get_friction<D: UserDataType>(self_: &B2contact<D>) -> f32 {
		return self_.m_friction;
	}

	pub fn reset_friction<D: UserDataType>(self_: &mut B2contact<D>) {
		self_.m_friction = b2_mix_friction(
			self_.m_fixture_a.borrow().m_friction,
			self_.m_fixture_b.borrow().m_friction,
		);
	}

	pub fn set_restitution<D: UserDataType>(self_: &mut B2contact<D>, restitution: f32) {
		self_.m_restitution = restitution;
	}

	pub fn get_restitution<D: UserDataType>(self_: &B2contact<D>) -> f32 {
		return self_.m_restitution;
	}

	pub fn reset_restitution<D: UserDataType>(self_: &mut B2contact<D>) {
		self_.m_restitution = b2_mix_restitution(
			self_.m_fixture_a.borrow().m_restitution,
			self_.m_fixture_b.borrow().m_restitution,
		);
	}

	pub fn set_restitution_threshold<D: UserDataType>(self_: &mut B2contact<D>, threshold: f32) {
		self_.m_restitution_threshold = threshold;
	}

	pub fn get_restitution_threshold<D: UserDataType>(self_: &B2contact<D>) -> f32 {
		return self_.m_restitution_threshold;
	}

	pub fn reset_restitution_threshold<D: UserDataType>(self_: &mut B2contact<D>) {
		self_.m_restitution_threshold = b2_mix_restitution_threshold(
			self_.m_fixture_a.borrow().m_restitution_threshold,
			self_.m_fixture_b.borrow().m_restitution_threshold,
		);
	}

	pub fn set_tangent_speed<D: UserDataType>(self_: &mut B2contact<D>, speed: f32) {
		self_.m_tangent_speed = speed;
	}

	pub fn get_tangent_speed<D: UserDataType>(self_: &B2contact<D>) -> f32 {
		return self_.m_tangent_speed;
	}
}
