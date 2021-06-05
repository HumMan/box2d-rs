use crate::b2_body::*;
use crate::b2_broad_phase::*;
use crate::b2_collision::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_shape::*;
use crate::linked_list::*;
use crate::private::dynamics::b2_fixture as private;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

#[cfg(feature="serde_support")]
use serde::{Serialize, Deserialize};

impl Default for B2filter {
	fn default() -> Self {
		return B2filter {
			category_bits: 0x0001,
			mask_bits: 0xFFFF,
			group_index: 0,
		};
	}
}

/// This holds contact filtering data.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct B2filter {
	/// The collision category bits. Normally you would just set one bit.
	pub category_bits: u16,

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	pub mask_bits: u16,

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). Zero means no collision group. Non-zero group
	/// filtering always wins against the mask bits.
	pub group_index: i16,
}

impl<D: UserDataType> Default for B2fixtureDef<D> {
	/// The constructor sets the default fixture definition values.
	fn default() -> Self {
		return B2fixtureDef {
			shape: None,
			user_data: None,
			friction: 0.2,
			restitution: 0.0,
			density: 0.0,
			is_sensor: false,
			filter: B2filter::default(),
		};
	}
}

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
#[derive(Clone)]
pub struct B2fixtureDef<D: UserDataType> {
	/// The shape, this must be set. The shape will be cloned, so you
	/// can create the shape on the stack.
	pub shape: Option<ShapeDefPtr>,

	/// Use this to store application specific fixture data.
	pub user_data: Option<D::Fixture>,

	/// The friction coefficient, usually in the range [0,1].
	pub friction: f32,

	/// The restitution (elasticity) usually in the range [0,1].
	pub restitution: f32,

	/// The density, usually in kg/m^2.
	pub density: f32,

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	pub is_sensor: bool,

	/// Contact filtering data.
	pub filter: B2filter,
}

pub type FixturePtr<D> = Rc<RefCell<B2fixture<D>>>;
pub type FixtureWeakPtr<D> = Weak<RefCell<B2fixture<D>>>;
pub type FixtureProxyPtr<D> = Rc<RefCell<B2fixtureProxy<D>>>;

/// This proxy is used internally to connect fixtures to the broad-phase.
#[derive(Default)]
pub struct B2fixtureProxy<D: UserDataType> {
	pub(crate) aabb: B2AABB,
	pub(crate) fixture: Option<FixtureWeakPtr<D>>,
	pub(crate) child_index: i32,
	pub(crate) proxy_id: i32,
}

impl<D:UserDataType> LinkedListNode<B2fixture<D>> for B2fixture<D>
{
    fn get_next(&self) -> Option<FixturePtr<D>> {
        return self.m_next.clone();
	}
	fn set_next(&mut self, value: Option<FixturePtr<D>>)
    {
        self.m_next = value;
    }
	fn take_next(&mut self) -> Option<FixturePtr<D>> {
		return self.m_next.take();
	}
}

/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via b2_body::create_fixture.
/// @warning you cannot reuse fixtures.
#[derive(Default, Clone)]
pub struct B2fixture<D: UserDataType> {
	pub(crate) m_density: f32,

	pub(crate) m_next: Option<FixturePtr<D>>,
	pub(crate) m_body: Option<BodyWeakPtr<D>>,

	pub(crate) m_shape: Option<ShapePtr>,

	pub(crate) m_friction: f32,
	pub(crate) m_restitution: f32,

	pub(crate) m_proxies: Vec<FixtureProxyPtr<D>>,
	pub(crate) m_proxy_count: i32,

	pub(crate) m_filter: B2filter,

	pub(crate) m_is_sensor: bool,

	pub(crate) m_user_data: Option<D::Fixture>,
}

impl<D: UserDataType> B2fixture<D> {
	/// Get the type of the child shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	pub fn get_type(&self) -> B2ShapeType {
		return inline::get_type(self);
	}

	/// Get the child shape. You can modify the child shape, however you should not change the
	/// number of vertices because this will crash some collision caching mechanisms.
	/// Manipulating the shape may lead to non-physical behavior.
	pub fn get_shape(&self) -> ShapePtr {
		return inline::get_shape(self);
	}

	/// Set if this fixture is a sensor.
	pub fn set_sensor(&mut self, sensor: bool) {
		private::b2_fixture_set_sensor(self, sensor);
	}

	/// Is this fixture a sensor (non-solid)?
	/// @return the true if the shape is a sensor.
	pub fn is_sensor(&self) -> bool {
		return inline::is_sensor(self);
	}

	/// Set the contact filtering data. This will not update contacts until the next time
	/// step when either parent body is active and awake.
	/// This automatically calls refilter.
	pub fn set_filter_data(&mut self, filter: B2filter) {
		private::b2_fixture_set_filter_data(self, filter);
	}

	/// Get the contact filtering data.
	pub fn get_filter_data(&self) -> B2filter {
		return inline::get_filter_data(self);
	}

	/// Call this if you want to establish collision that was previously disabled by B2contactFilter::should_collide.
	pub fn refilter(&mut self) {
		private::b2_fixture_refilter(self);
	}

	/// Get the parent body of this fixture. This is None if the fixture is not attached.
	/// @return the parent body.
	pub fn get_body(&self) -> BodyPtr<D> {
		return inline::get_body(self);
	}

	/// Get the next fixture in the parent body's fixture list.
	/// @return the next shape.
	pub fn get_next(&self) -> Option<FixturePtr<D>> {
		return inline::get_next(self);
	}

	/// Get the user data that was assigned in the fixture definition. Use this to
	/// store your application specific data.
	pub fn get_user_data(&self) -> Option<D::Fixture> {
		return inline::get_user_data(self);
	}

	/// Set the user data. Use this to store your application specific data.
	pub fn set_user_data(&mut self, data: &D::Fixture) {
		inline::set_user_data(self, data);
	}

	/// Test a point for containment in this fixture.
	/// * `p` - a point in world coordinates.
	pub fn test_point(&self, p: B2vec2) -> bool {
		return inline::test_point(self, p);
	}

	/// Cast a ray against this shape.
	/// * `output` - the ray-cast results.
	/// * `input` - the ray-cast input parameters.
	/// * `child_index` - the child shape index (e.g. edge index)
	pub fn ray_cast(
		&self,
		output: &mut B2rayCastOutput,
		input: &B2rayCastInput,
		child_index: i32,
	) -> bool {
		return inline::ray_cast(self, output, input, child_index);
	}

	/// Get the mass data for this fixture. The mass data is based on the density and
	/// the shape. The rotational inertia is about the shape's origin. This operation
	/// may be expensive.
	pub fn get_mass_data(&self, mass_data: &mut B2massData) {
		inline::get_mass_data(self, mass_data);
	}

	/// Set the density of this fixture. This will _not_ automatically adjust the mass
	/// of the body. You must call b2_body::reset_mass_data to update the body's mass.
	pub fn set_density(&mut self, density: f32) {
		inline::set_density(self, density);
	}

	/// Get the density of this fixture.
	pub fn get_density(&self) -> f32 {
		return inline::get_density(self);
	}

	/// Get the coefficient of friction.
	pub fn get_friction(&self) -> f32 {
		return inline::get_friction(self);
	}

	/// Set the coefficient of friction. This will _not_ change the friction of
	/// existing contacts.
	pub fn set_friction(&mut self, friction: f32) {
		return inline::set_friction(self, friction);
	}

	/// Get the coefficient of restitution.
	pub fn get_restitution(&self) -> f32 {
		return inline::get_restitution(self);
	}

	/// Set the coefficient of restitution. This will _not_ change the restitution of
	/// existing contacts.
	pub fn set_restitution(&mut self, restitution: f32) {
		inline::set_restitution(self, restitution);
	}

	/// Get the fixture's AABB. This AABB may be enlarge and/or stale.
	/// If you need a more accurate AABB, compute it using the shape and
	/// the body transform.
	pub fn get_aabb(&self, child_index: i32) -> B2AABB {
		return inline::get_aabb(self, child_index);
	}

	pub(crate) fn default() -> Self {
		return private::b2_fixture_default();
	}

	// We need separation create/destroy functions from the constructor/destructor because
	// the destructor cannot access the allocator (no destructor arguments allowed by c++).
	pub(crate) fn create(
		this: &mut B2fixture<D>,
		body: BodyPtr<D>,
		def: &B2fixtureDef<D>,
	) {
		private::b2_fixture_create(this, body, def);
	}

	// These support body activation/deactivation.
	pub(crate) fn create_proxies(
		this_ptr: FixturePtr<D>,
		broad_phase: &mut B2broadPhase<FixtureProxyPtr<D>>,
		xf: &B2Transform,
	) {
		private::b2_fixture_create_proxies(this_ptr, broad_phase, xf);
	}
	pub(crate) fn destroy_proxies(
		&mut self,
		broad_phase: &mut B2broadPhase<FixtureProxyPtr<D>>,
	) {
		private::b2_fixture_destroy_proxies(self, broad_phase);
	}

	pub(crate) fn synchronize(
		&mut self,
		broad_phase: &mut B2broadPhase<FixtureProxyPtr<D>>,
		xf1: B2Transform,
		xf2: B2Transform,
	) {
		private::b2_fixture_synchronize(self, broad_phase, xf1, xf2);
	}
}

mod inline {
	use super::*;

	pub fn get_type<T: UserDataType>(this: &B2fixture<T>) -> B2ShapeType {
		return this.m_shape.as_ref().unwrap().get_type();
	}

	pub fn get_shape<T: UserDataType>(this: &B2fixture<T>) -> ShapePtr {
		return this.m_shape.as_ref().unwrap().clone();
	}

	pub fn is_sensor<T: UserDataType>(this: &B2fixture<T>) -> bool {
		return this.m_is_sensor;
	}

	pub fn get_filter_data<T: UserDataType>(this: &B2fixture<T>) -> B2filter {
		return this.m_filter;
	}

	pub fn get_user_data<D: UserDataType>(this: &B2fixture<D>) -> Option<D::Fixture> {
		return this.m_user_data.clone();
	}

	pub fn set_user_data<D: UserDataType>(this: &mut B2fixture<D>, data: &D::Fixture) {
		this.m_user_data = Some(data.clone());
	}

	pub fn get_body<T: UserDataType>(this: &B2fixture<T>) -> BodyPtr<T> {
		return this.m_body.as_ref().unwrap().upgrade().unwrap();
	}

	pub fn get_next<T: UserDataType>(this: &B2fixture<T>) -> Option<FixturePtr<T>> {
		return this.m_next.clone();
	}

	pub fn set_density<T: UserDataType>(this: &mut B2fixture<T>, density: f32) {
		b2_assert(b2_is_valid(density) && density >= 0.0);
		this.m_density = density;
	}

	pub fn get_density<T: UserDataType>(this: &B2fixture<T>) -> f32 {
		return this.m_density;
	}

	pub fn get_friction<T: UserDataType>(this: &B2fixture<T>) -> f32 {
		return this.m_friction;
	}

	pub fn set_friction<T: UserDataType>(this: &mut B2fixture<T>, friction: f32) {
		this.m_friction = friction;
	}

	pub fn get_restitution<T: UserDataType>(this: &B2fixture<T>) -> f32 {
		return this.m_restitution;
	}

	pub fn set_restitution<T: UserDataType>(this: &mut B2fixture<T>, restitution: f32) {
		this.m_restitution = restitution;
	}

	pub fn test_point<T: UserDataType>(this: &B2fixture<T>, p: B2vec2) -> bool {
		return this.m_shape.as_ref().unwrap().test_point(
			this.m_body
				.as_ref()
				.unwrap()
				.upgrade()
				.unwrap()
				.borrow()
				.get_transform(),
			p,
		);
	}

	pub fn ray_cast<T: UserDataType>(
		this: &B2fixture<T>,
		output: &mut B2rayCastOutput,
		input: &B2rayCastInput,
		child_index: i32,
	) -> bool {
		return this.m_shape.as_ref().unwrap().ray_cast(
			output,
			input,
			this.m_body
				.as_ref()
				.unwrap()
				.upgrade()
				.unwrap()
				.borrow()
				.get_transform(),
			child_index as usize,
		);
	}

	pub fn get_mass_data<T: UserDataType>(this: &B2fixture<T>, mass_data: &mut B2massData) {
		this.m_shape
			.as_ref()
			.unwrap()
			.compute_mass(mass_data, this.m_density);
	}

	pub fn get_aabb<T: UserDataType>(this: &B2fixture<T>, child_index: i32) -> B2AABB {
		b2_assert(0 <= child_index && child_index < this.m_proxies.len() as i32);
		return this.m_proxies[child_index as usize].as_ref().borrow().aabb;
	}
}
