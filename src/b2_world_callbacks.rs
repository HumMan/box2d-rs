use crate::b2_collision::*;
use crate::b2_contact::*;
use crate::b2_fixture::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2rs_common::*;

use std::cell::RefCell;
use std::rc::Rc;

use crate::private::dynamics::b2_world_callbacks as private;

pub type B2destructionListenerPtr<D> = Rc<RefCell<dyn B2destructionListener<D>>>;
pub type B2contactFilterPtr<D> = Rc<RefCell<dyn B2contactFilter<D>>>;
pub type B2contactListenerPtr<D> = Rc<RefCell<dyn B2contactListener<D>>>;

/// Joints and fixtures are destroyed when their associated
/// body is destroyed. Implement this listener so that you
/// may nullify references to these joints and shapes.
pub trait B2destructionListener<D: UserDataType> {
	/// Called when any joint is about to be destroyed due
	/// to the destruction of one of its attached bodies.
	fn say_goodbye_joint(&mut self, joint: B2jointPtr<D>);

	/// Called when any fixture is about to be destroyed due
	/// to the destruction of its parent body.
	fn say_goodbye_fixture(&mut self, fixture: FixturePtr<D>);
}

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
pub trait B2contactFilter<D: UserDataType> {
	/// Return true if contact calculations should be performed between these two shapes.
	/// <p style="background:rgba(255,181,77,0.16);padding:0.75em;">
	/// <strong>Warning:</strong> for performance reasons this is only called when the AABBs begin to overlap.
	/// </p>
	fn should_collide(&self, fixture_a: FixturePtr<D>, fixture_b: FixturePtr<D>) -> bool {
		return private::should_collide(fixture_a, fixture_b);
	}
}

pub struct B2contactFilterDefault;

impl<D: UserDataType> B2contactFilter<D> for B2contactFilterDefault {}

/// Contact impulses for reporting. Impulses are used instead of forces because
/// sub-step forces may approach infinity for rigid body collisions. These
/// match up one-to-one with the contact points in B2manifold.
#[derive(Default, Copy, Clone, Debug)]
pub struct B2contactImpulse {
	pub normal_impulses: [f32; B2_MAX_MANIFOLD_POINTS],
	pub tangent_impulses: [f32; B2_MAX_MANIFOLD_POINTS],
	pub count: i32,
}

/// Implement this class to get contact information. You can use these results for
/// things like sounds and game logic. You can also get contact results by
/// traversing the contact lists after the time step. However, you might miss
/// some contacts because continuous physics leads to sub-stepping.
/// Additionally you may receive multiple callbacks for the same contact in a
/// single time step.
/// You should strive to make your callbacks efficient because there may be
/// many callbacks per time step.
/// <p style="background:rgba(255,181,77,0.16);padding:0.75em;">
/// <strong>Warning:</strong> You cannot create/destroy Box2D entities inside these callbacks.
/// </p>
pub trait B2contactListener<D: UserDataType> {
	/// Called when two fixtures begin to touch.
	fn begin_contact(&mut self, contact: &mut dyn B2contactDynTrait<D>) {
		b2_not_used(contact);
	}

	/// Called when two fixtures cease to touch.
	fn end_contact(&mut self, contact: &mut dyn B2contactDynTrait<D>) {
		b2_not_used(contact);
	}

	/// This is called after a contact is updated. This allows you to inspect a
	/// contact before it goes to the solver. If you are careful, you can modify the
	/// contact manifold (e.g. disable contact).
	/// A copy of the old manifold is provided so that you can detect changes.
	/// Note: this is called only for awake bodies.
	/// Note: this is called even when the number of contact points is zero.
	/// Note: this is not called for sensors.
	/// Note: if you set the number of contact points to zero, you will not
	/// get an end_contact callback. However, you may get a begin_contact callback
	/// the next step.
	fn pre_solve(&mut self, contact: &mut dyn B2contactDynTrait<D>, old_manifold: &B2manifold) {
		b2_not_used(contact);
		b2_not_used(old_manifold);
	}

	/// This lets you inspect a contact after the solver is finished. This is useful
	/// for inspecting impulses.
	/// Note: the contact manifold does not include time of impact impulses, which can be
	/// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
	/// in a separate data structure.
	/// Note: this is only called for contacts that are touching, solid, and awake.
	fn post_solve(&mut self, contact: &mut dyn B2contactDynTrait<D>, impulse: &B2contactImpulse) {
		b2_not_used(contact);
		b2_not_used(impulse);
	}
}

pub struct B2contactListenerDefault;

impl<D: UserDataType> B2contactListener<D> for B2contactListenerDefault {}

/// Called for each fixture found in the query AABB.
/// 
/// @return false to terminate the query.
pub trait B2queryCallback<D: UserDataType>: FnMut(
	/*fixture:*/ FixturePtr<D>
) -> bool {}
impl<F, D: UserDataType> B2queryCallback<D> for F where F: FnMut(FixturePtr<D>) -> bool {}

/// Called for each fixture found in the query. You control how the ray cast
/// proceeds by returning a f32:
/// return -1: ignore this fixture and continue
/// return 0: terminate the ray cast
/// return fraction: clip the ray to this point
/// return 1: don't clip the ray and continue
/// * `fixture` - the fixture hit by the ray
/// * `point` - the point of initial intersection
/// * `normal` - the normal vector at the point of intersection
/// * `fraction` - the fraction along the ray at the point of intersection
/// 
/// @return -1 to filter, 0 to terminate, fraction to clip the ray for
/// closest hit, 1 to continue
pub trait B2rayCastCallback<D:UserDataType>: FnMut(	
	/*fixture:*/ FixturePtr<D>,
	/*point:*/ B2vec2,
	/*normal:*/ B2vec2,
	/*fraction:*/ f32) -> f32 {}

impl<F, D: UserDataType> B2rayCastCallback<D> for F where
	F: FnMut(FixturePtr<D>, B2vec2, B2vec2, f32) -> f32
{}
