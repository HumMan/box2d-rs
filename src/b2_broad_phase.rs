use crate::b2_collision::*;
use crate::b2_dynamic_tree::*;
use crate::b2_math::*;
use crate::private::collision::b2_broad_phase as private;

use std::cell::RefCell;
use std::rc::Rc;

#[derive(Default, Clone, Copy, Debug)]
pub struct B2pair {
	pub proxy_id_a: i32,
	pub proxy_id_b: i32,
}

pub type B2broadPhasePtr<UserDataType> = Rc<RefCell<B2broadPhase<UserDataType>>>;

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
#[derive(Debug)]
pub struct B2broadPhase<UserDataType> {
	pub(crate) m_tree: B2dynamicTree<UserDataType>,

	pub(crate) m_proxy_count: i32,

	pub(crate) m_move_buffer: Vec<i32>,
	pub(crate) m_move_capacity: i32,
	pub(crate) m_move_count: i32,

	pub(crate) pairs: B2broadPhasePairs,
}

//because of &mut
#[derive(Debug, Default)]
pub struct B2broadPhasePairs {
	pub(crate) m_pair_buffer: Vec<B2pair>,
	pub(crate) m_pair_capacity: i32,
	pub(crate) m_pair_count: i32,
}

pub const E_NULL_PROXY: i32 = -1;

impl<UserDataType: Default + Clone> B2broadPhase<UserDataType> {
	pub fn new() -> Self {
		return private::b2_broad_phase_b2_broad_phase();
	}

	//~B2broadPhase();

	/// create a proxy with an initial AABB. Pairs are not reported until
	/// UpdatePairs is called.
	pub fn create_proxy(&mut self, aabb: B2AABB, user_data: &UserDataType) -> i32 {
		return private::b2_broad_phase_create_proxy(self, aabb, user_data);
	}

	/// destroy a proxy. It is up to the client to remove any pairs.
	pub fn destroy_proxy(&mut self, proxy_id: i32) {
		private::b2_broad_phase_destroy_proxy(self, proxy_id);
	}

	/// Call move_proxy as many times as you like, then when you are done
	/// call UpdatePairs to finalized the proxy pairs (for your time step).
	pub fn move_proxy(&mut self, proxy_id: i32, aabb: B2AABB, displacement: B2vec2) {
		private::b2_broad_phase_move_proxy(self, proxy_id, aabb, displacement);
	}

	/// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
	pub fn touch_proxy(&mut self, proxy_id: i32) {
		private::b2_broad_phase_touch_proxy(self, proxy_id);
	}

	/// Get the fat AABB for a proxy.
	pub fn get_fat_aabb(&self, proxy_id: i32) -> B2AABB {
		return inline::get_fat_aabb(self, proxy_id);
	}

	/// Get user data from a proxy. Returns None if the id is invalid.
	pub fn get_user_data(&self, proxy_id: i32) -> Option<UserDataType> {
		return inline::get_user_data(self, proxy_id);
	}

	/// Test overlap of fat AABBs.
	pub fn test_overlap(&self, proxy_id_a: i32, proxy_id_b: i32) -> bool {
		return inline::test_overlap(self, proxy_id_a, proxy_id_b);
	}

	/// Get the number of proxies.
	pub fn get_proxy_count(&self) -> i32 {
		return inline::get_proxy_count(self);
	}

	/// update the pairs. This results in pair callbacks. This can only add pairs.
	pub fn update_pairs<CallbackType: AddPairTrait<UserDataType>>(
		&mut self,
		callback: &mut CallbackType,
	) {
		inline::update_pairs(self, callback);
	}

	/// query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	pub fn query<F:  QueryCallback>(&self, callback: F, aabb: B2AABB) {
		inline::query(self, callback, aabb);
	}

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// * `input` - the ray-cast input data. The ray extends from p1 to p1 + max_fraction * (p2 - p1).
	/// * `callback` - a callback class that is called for each proxy that is hit by the ray.
	pub fn ray_cast<T: RayCastCallback>(&self, callback: T, input: &B2rayCastInput) {
		inline::ray_cast(self, callback, input);
	}

	/// Get the height of the embedded tree.
	pub fn get_tree_height(&self) -> i32 {
		return inline::get_tree_height(self);
	}

	/// Get the balance of the embedded tree.
	pub fn get_tree_balance(&self) -> i32 {
		return inline::get_tree_balance(self);
	}

	/// Get the quality metric of the embedded tree.
	pub fn get_tree_quality(&self) -> f32 {
		return inline::get_tree_quality(self);
	}

	/// Shift the world origin. Useful for large worlds.
	/// The shift formula is: position -= new_origin
	/// * `new_origin` - the new origin with respect to the old origin
	pub fn shift_origin(&mut self, new_origin: B2vec2) {
		inline::shift_origin(self, new_origin);
	}

	pub fn get_tree_mut(&mut self)->&mut B2dynamicTree<UserDataType>{
		return &mut self.m_tree;
	}

	pub(crate) fn buffer_move(&mut self, proxy_id: i32) {
		private::b2_broad_phase_buffer_move(self, proxy_id);
	}

	pub(crate) fn un_buffer_move(&mut self, proxy_id: i32) {
		private::b2_broad_phase_un_buffer_move(self, proxy_id);
	}
}

pub trait AddPairTrait<UserDataType> {
	fn add_pair(
		&mut self,
		proxy_user_data_a: Option<UserDataType>,
		proxy_user_data_b: Option<UserDataType>,
	);
}

mod inline {
	use super::*;

	pub fn get_user_data<UserDataType: Default + Clone>(
		self_: &B2broadPhase<UserDataType>,
		proxy_id: i32,
	) -> Option<UserDataType> {
		return self_.m_tree.get_user_data(proxy_id);
	}

	pub fn test_overlap<T: Default + Clone>(
		self_: &B2broadPhase<T>,
		proxy_id_a: i32,
		proxy_id_b: i32,
	) -> bool {
		let aabb_a: B2AABB = self_.m_tree.get_fat_aabb(proxy_id_a);
		let aabb_b: B2AABB = self_.m_tree.get_fat_aabb(proxy_id_b);
		return b2_test_overlap(aabb_a, aabb_b);
	}

	pub fn get_fat_aabb<T: Default + Clone>(self_: &B2broadPhase<T>, proxy_id: i32) -> B2AABB {
		return self_.m_tree.get_fat_aabb(proxy_id);
	}

	pub fn get_proxy_count<T: Default + Clone>(self_: &B2broadPhase<T>) -> i32 {
		return self_.m_proxy_count;
	}

	pub fn get_tree_height<T: Default + Clone>(self_: &B2broadPhase<T>) -> i32 {
		return self_.m_tree.get_height();
	}

	pub fn get_tree_balance<T: Default + Clone>(self_: &B2broadPhase<T>) -> i32 {
		return self_.m_tree.get_max_balance();
	}

	pub fn get_tree_quality<T: Default + Clone>(self_: &B2broadPhase<T>) -> f32 {
		return self_.m_tree.get_area_ration();
	}

	pub fn update_pairs<T: Default + Clone, CallbackType: AddPairTrait<T>>(
		self_: &mut B2broadPhase<T>,
		callback: &mut CallbackType,
	) {
		// reset pair buffer
		self_.pairs.m_pair_count=0;

		// Perform tree queries for all moving proxies.
		for i in 0..self_.m_move_count {
			let m_query_proxy_id = self_.m_move_buffer[i as usize];
			if m_query_proxy_id == E_NULL_PROXY {
				continue;
			}

			// We have to query the tree with the fat AABB so that
			// we don't fail to create a pair that may touch later.
			let fat_aabb: B2AABB = self_.m_tree.get_fat_aabb(m_query_proxy_id);
			{
				let pairs = &mut self_.pairs;
				let tree = &self_.m_tree;
				// query tree, create pairs and add them pair buffer.
				tree.query(|proxy_id:i32|->bool{
					let moved = tree.was_moved(proxy_id);
					return private::b2_broad_phase_query_callback(pairs, m_query_proxy_id, proxy_id, moved);
				}, fat_aabb);
			}
		}

		// Send pairs to caller
		for i in 0..self_.pairs.m_pair_count {
			let primary_pair: B2pair = self_.pairs.m_pair_buffer[i as usize];
			let user_data_a = self_.m_tree.get_user_data(primary_pair.proxy_id_a);
			let user_data_b = self_.m_tree.get_user_data(primary_pair.proxy_id_b);

			callback.add_pair(user_data_a, user_data_b);
		}

		// clear move flags
		for i in 0..self_.m_move_count {
			let proxy_id: i32 = self_.m_move_buffer[i as usize];
			if proxy_id == E_NULL_PROXY {
				continue;
			}

			self_.m_tree.clear_moved(proxy_id);
		}

		// reset move buffer
		self_.m_move_count = 0;
	}

	pub fn query<UserDataType: Default + Clone, F:  QueryCallback>(
		self_: &B2broadPhase<UserDataType>,
		callback: F,
		aabb: B2AABB,
	) {
		self_.m_tree.query(callback, aabb);
	}

	pub fn ray_cast<UserDataType: Default + Clone, T: RayCastCallback>(
		self_: &B2broadPhase<UserDataType>,
		callback: T,
		input: &B2rayCastInput,
	) {
		self_.m_tree.ray_cast(callback, input);
	}

	pub fn shift_origin<UserDataType: Default + Clone>(
		self_: &mut B2broadPhase<UserDataType>,
		new_origin: B2vec2,
	) {
		self_.m_tree.shift_origin(new_origin);
	}
}
