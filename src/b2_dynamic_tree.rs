use crate::b2_collision::*;
use crate::b2_growable_stack::B2growableStack;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::private::collision::b2_dynamic_tree as private;

pub const B2_NULL_NODE: i32 = -1;

/// A node in the dynamic tree. The client does not interact with this directly.
#[derive(Default, Clone, Debug)]
pub struct B2treeNode<UserDataType> {
	/// Enlarged AABB
	pub(crate) aabb: B2AABB,

	pub(crate) user_data: Option<UserDataType>,

	//box2d-rs: parent=next for free nodes
	//union
	//{
	//	 parent:i32,
	//	 next:i32,
	//},
	pub(crate) parent: i32,

	pub(crate) child1: i32,
	pub(crate) child2: i32,

	// leaf = 0, free node = -1
	pub(crate) height: i32,

	pub(crate) moved: bool,
}

impl<UserDataType> B2treeNode<UserDataType> {
	pub fn is_leaf(&self) -> bool {
		return self.child1 == B2_NULL_NODE;
	}
}

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than pointers.
#[derive(Default, Clone, Debug)]
pub struct B2dynamicTree<UserDataType> {
	pub(crate) m_root: i32,

	pub(crate) m_nodes: Vec<B2treeNode<UserDataType>>,
	pub(crate) m_node_count: i32,
	pub(crate) m_node_capacity: i32,

	pub(crate) m_free_list: i32,

	pub(crate) m_insertion_count: i32,
}

impl<UserDataType: Clone + Default> B2dynamicTree<UserDataType> {
	/// Constructing the tree initializes the node pool.
	pub fn new() -> Self {
		return private::b2_dynamic_tree();
	}

	/// destroy the tree, freeing the node pool.
	//~B2dynamicTree();

	/// create a proxy. Provide a tight fitting AABB and a user_data pointer.
	pub fn create_proxy(&mut self, aabb: B2AABB, user_data: &UserDataType) -> i32 {
		return private::create_proxy(self, aabb, user_data);
	}

	/// destroy a proxy. This asserts if the id is invalid.
	pub fn destroy_proxy(&mut self, proxy_id: i32) {
		private::destroy_proxy(self, proxy_id);
	}

	/// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
	/// then the proxy is removed from the tree and re-inserted. Otherwise
	/// the function returns immediately.
	/// 
	/// @return true if the proxy was re-inserted.
	pub fn move_proxy(&mut self, proxy_id: i32, aabb1: B2AABB, displacement: B2vec2) -> bool {
		return private::move_proxy(self, proxy_id, aabb1, displacement);
	}

	/// Get proxy user data.
	/// 
	/// @return the proxy user data or 0 if the id is invalid.
	pub fn get_user_data(&self, proxy_id: i32) -> Option<UserDataType> {
		return inline::get_user_data(self, proxy_id);
	}

	pub fn was_moved(&self, proxy_id: i32) -> bool {
		return inline::was_moved(self, proxy_id);
	}
	pub fn clear_moved(&mut self, proxy_id: i32) {
		inline::clear_moved(self, proxy_id);
	}

	/// Get the fat AABB for a proxy.
	pub fn get_fat_aabb(&self, proxy_id: i32) -> B2AABB {
		return inline::get_fat_aabb(self, proxy_id);
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

	/// Validate this tree. For testing.
	pub fn validate(&self) {
		private::validate(self);
	}

	/// Compute the height of the binary tree in O(n) time. Should not be
	/// called often.
	pub fn get_height(&self) -> i32 {
		return private::get_height(self);
	}

	/// Get the maximum balance of an node in the tree. The balance is the difference
	/// in height of the two children of a node.
	pub fn get_max_balance(&self) -> i32 {
		return private::get_max_balance(self);
	}

	/// Get the ratio of the sum of the node areas to the root area.
	pub fn get_area_ration(&self) -> f32 {
		return private::get_area_ratio(self);
	}

	/// Build an optimal tree. Very expensive. For testing.
	pub fn rebuild_bottom_up(&mut self) {
		private::rebuild_bottom_up(self);
	}

	/// Shift the world origin. Useful for large worlds.
	/// The shift formula is: position -= new_origin
	/// * `new_origin` - the new origin with respect to the old origin
	pub fn shift_origin(&mut self, new_origin: B2vec2) {
		private::shift_origin(self, new_origin);
	}

	pub(crate) fn allocate_node(&mut self) -> i32 {
		return private::allocate_node(self);
	}
	pub(crate) fn free_node(&mut self, node: i32) {
		private::free_node(self, node);
	}

	pub(crate) fn insert_leaf(&mut self, node: i32) {
		private::insert_leaf(self, node);
	}
	pub(crate) fn remove_leaf(&mut self, node: i32) {
		private::remove_leaf(self, node);
	}

	pub(crate) fn balance(&mut self, index: i32) -> i32 {
		return private::balance(self, index);
	}

	pub(crate) fn compute_height(&self) -> i32 {
		return private::compute_height(self);
	}
	pub(crate) fn compute_height_by_node(&self, node_id: i32) -> i32 {
		return private::compute_height_by_node(self, node_id);
	}

	pub(crate) fn validate_structure(&self, index: i32) {
		private::validate_structure(self, index);
	}
	pub(crate) fn validate_metrics(&self, index: i32) {
		private::validate_metrics(self, index);
	}
}

// pub trait QueryCallback {
// 	fn query_callback(&mut self, proxy_id: i32) -> bool;
// }
pub trait QueryCallback: FnMut(i32) -> bool {}
impl <F> QueryCallback for F where F: FnMut(i32) -> bool {}

// pub trait RayCastCallback {
// 	fn ray_cast_callback(&mut self, input: &B2rayCastInput, proxy_id: i32) -> f32;
// }
pub trait RayCastCallback: FnMut(&B2rayCastInput, i32) -> f32 {}
impl <F> RayCastCallback for F where F: FnMut(&B2rayCastInput, i32) -> f32 {}

mod inline {
	use super::*;

	pub fn get_user_data<UserDataType: Clone + Default>(
		self_: &B2dynamicTree<UserDataType>,
		proxy_id: i32,
	) -> Option<UserDataType> {
		//b2_assert(0 <= proxy_id && proxy_id < self_.m_nodeCapacity);
		return self_.m_nodes[proxy_id as usize].user_data.clone();
	}

	pub fn was_moved<UserDataType: Clone + Default>(
		self_: &B2dynamicTree<UserDataType>,
		proxy_id: i32,
	) -> bool {
		//b2_assert(0 <= proxy_id && proxy_id < self_.m_nodeCapacity);
		return self_.m_nodes[proxy_id as usize].moved;
	}

	pub fn clear_moved<UserDataType: Clone + Default>(
		self_: &mut B2dynamicTree<UserDataType>,
		proxy_id: i32,
	) {
		//b2_assert(0 <= proxy_id && proxy_id < self_.m_nodeCapacity);
		self_.m_nodes[proxy_id as usize].moved = false;
	}

	pub fn get_fat_aabb<UserDataType: Clone + Default>(
		self_: &B2dynamicTree<UserDataType>,
		proxy_id: i32,
	) -> B2AABB {
		//b2_assert(0 <= proxy_id && proxy_id < self_.m_nodeCapacity);
		return self_.m_nodes[proxy_id as usize].aabb;
	}

	pub fn query<UserDataType, F:  QueryCallback>(
		self_: &B2dynamicTree<UserDataType>,
		mut callback: F,
		aabb: B2AABB,
	) {
		let mut stack = B2growableStack::<i32>::new();
		stack.push(&self_.m_root);

		while stack.get_count() > 0 {
			let node_id: i32 = stack.pop();
			if node_id == B2_NULL_NODE {
				continue;
			}

			let node = &self_.m_nodes[node_id as usize];

			if b2_test_overlap(node.aabb, aabb) {
				if node.is_leaf() {
					let proceed: bool = callback(node_id);
					if proceed == false {
						return;
					}
				} else {
					stack.push(&node.child1);
					stack.push(&node.child2);
				}
			}
		}
	}

	pub fn ray_cast<T: RayCastCallback, UserDataType>(
		self_: &B2dynamicTree<UserDataType>,
		mut callback: T,
		input: &B2rayCastInput,
	) {
		let p1: B2vec2 = input.p1;
		let p2: B2vec2 = input.p2;
		let mut r: B2vec2 = p2 - p1;
		b2_assert(r.length_squared() > 0.0);
		r.normalize();

		// v is perpendicular to the segment.
		let v: B2vec2 = b2_cross_scalar_by_vec(1.0, r);
		let abs_v: B2vec2 = b2_abs_vec2(v);

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)

		let mut max_fraction: f32 = input.max_fraction;

		// Build a bounding box for the segment.
		let mut segment_aabb = B2AABB::default();
		{
			let t: B2vec2 = p1 + max_fraction * (p2 - p1);
			segment_aabb.lower_bound = b2_min_vec2(p1, t);
			segment_aabb.upper_bound = b2_max_vec2(p1, t);
		}

		let mut stack = B2growableStack::<i32>::new();
		stack.push(&self_.m_root);

		while stack.get_count() > 0 {
			let node_id: i32 = stack.pop();
			if node_id == B2_NULL_NODE {
				continue;
			}

			let node = &self_.m_nodes[node_id as usize];

			if b2_test_overlap(node.aabb, segment_aabb) == false {
				continue;
			}

			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			let c: B2vec2 = node.aabb.get_center();
			let h: B2vec2 = node.aabb.get_extents();
			let separation: f32 = b2_abs(b2_dot(v, p1 - c)) - b2_dot(abs_v, h);
			if separation > 0.0 {
				continue;
			}

			if node.is_leaf() {
				let sub_input = B2rayCastInput {
					p1: input.p1,
					p2: input.p2,
					max_fraction: max_fraction,
				};

				let value: f32 = callback(&sub_input, node_id);

				if value == 0.0 {
					// The client has terminated the ray cast.
					return;
				}

				if value > 0.0 {
					// update segment bounding box.
					max_fraction = value;
					let t: B2vec2 = p1 + max_fraction * (p2 - p1);
					segment_aabb.lower_bound = b2_min_vec2(p1, t);
					segment_aabb.upper_bound = b2_max_vec2(p1, t);
				}
			} else {
				stack.push(&node.child1);
				stack.push(&node.child2);
			}
		}
	}
}
