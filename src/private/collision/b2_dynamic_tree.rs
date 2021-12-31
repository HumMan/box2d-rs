use crate::b2_collision::*;
use crate::b2_dynamic_tree::*;
use crate::b2_math::*;
use crate::b2_common::*;
use std::convert::TryInto;

pub fn b2_dynamic_tree<T: Clone + Default>() -> B2dynamicTree<T> {
	let m_root = B2_NULL_NODE;

	let m_node_capacity: i32 = 16;
	let m_node_count: i32 = 0;
	let mut m_nodes = Vec::<B2treeNode<T>>::new();
	m_nodes.resize_with(m_node_capacity as usize, Default::default);

	// Build a linked list for the free list.
	for i in 0..m_node_capacity - 1 {
		m_nodes[i as usize].parent = (i + 1) as i32;//parent is next free node
		m_nodes[i as usize].height = -1;
	}
	m_nodes[(m_node_capacity - 1) as usize].parent = B2_NULL_NODE;
	m_nodes[(m_node_capacity - 1) as usize].height = -1;

	return B2dynamicTree::<T> {
		m_root,
		m_node_capacity,
		m_node_count,
		m_nodes,
		m_free_list: 0,
		m_insertion_count: 0,
	};
}

// Allocate a node from the pool. Grow the pool if necessary.
pub fn allocate_node<T: Clone + Default>(self_: &mut B2dynamicTree<T>) -> i32 {
	// Expand the node pool as needed.
	if self_.m_free_list == B2_NULL_NODE {
		b2_assert(self_.m_node_count == self_.m_node_capacity);

		// The free list is empty. Rebuild a bigger pool.
		self_.m_node_capacity *= 2;
		self_.m_nodes
			.resize(self_.m_node_capacity as usize, B2treeNode::<T>::default());

		// Build a linked list for the free list. The parent
		// pointer becomes the "next" pointer.
		for i in self_.m_node_count..self_.m_node_capacity - 1 {
			self_.m_nodes[i as usize].parent = i + 1; //next free node
			self_.m_nodes[i as usize].height = -1;
		}
		self_.m_nodes[(self_.m_node_capacity - 1) as usize].parent = B2_NULL_NODE;
		self_.m_nodes[(self_.m_node_capacity - 1) as usize].height = -1;
		self_.m_free_list = self_.m_node_count;
	}

	// Peel a node off the free list.
	let node_id: usize = (self_.m_free_list).try_into().unwrap();
	self_.m_free_list = self_.m_nodes[node_id].parent; //next free node
	self_.m_nodes[node_id].parent = B2_NULL_NODE;
	self_.m_nodes[node_id].child1 = B2_NULL_NODE;
	self_.m_nodes[node_id].child2 = B2_NULL_NODE;
	self_.m_nodes[node_id].height = 0;
	self_.m_nodes[node_id].user_data = None;
	self_.m_nodes[node_id].moved = false;
	self_.m_node_count += 1;
	return node_id.try_into().unwrap();
}

// Return a node to the pool.
pub fn free_node<T: Clone + Default>(self_: &mut B2dynamicTree<T>, node_id: i32) {
	b2_assert(0 <= node_id && node_id < self_.m_node_capacity);
	b2_assert(0 < self_.m_node_count);
	self_.m_nodes[node_id as usize].parent = self_.m_free_list; //next
	self_.m_nodes[node_id as usize].height = -1;
	self_.m_free_list = node_id;
	self_.m_node_count -= 1;
}

// create a proxy in the tree as a leaf node. We return the index
// of the node instead of a pointer so that we can grow
// the node pool.
pub fn create_proxy<T: Clone + Default>(
	self_: &mut B2dynamicTree<T>,
	aabb: B2AABB,
	user_data: &T,
) -> i32 {
	let proxy_id: i32 = allocate_node(self_);

	// Fatten the aabb.
	let r = B2vec2::new(B2_AABB_EXTENSION, B2_AABB_EXTENSION);
	self_.m_nodes[proxy_id as usize].aabb.lower_bound = aabb.lower_bound - r;
	self_.m_nodes[proxy_id as usize].aabb.upper_bound = aabb.upper_bound + r;
	self_.m_nodes[proxy_id as usize].user_data = Some(user_data.clone());
	self_.m_nodes[proxy_id as usize].height = 0;
	self_.m_nodes[proxy_id as usize].moved = true;

	insert_leaf(self_, proxy_id);

	return proxy_id;
}

pub fn destroy_proxy<T: Clone + Default>(self_: &mut B2dynamicTree<T>, proxy_id: i32) {
	b2_assert(0 <= proxy_id && proxy_id < self_.m_node_capacity);
	b2_assert(self_.m_nodes[proxy_id as usize].is_leaf());

	remove_leaf(self_, proxy_id);
	free_node(self_, proxy_id);
}

pub fn move_proxy<T: Clone + Default>(
	self_: &mut B2dynamicTree<T>,
	proxy_id: i32,
	aabb: B2AABB,
	displacement: B2vec2,
) -> bool {
	b2_assert(0 <= proxy_id && proxy_id < self_.m_node_capacity);

	b2_assert(self_.m_nodes[proxy_id as usize].is_leaf());
	let r = B2vec2::new(B2_AABB_EXTENSION, B2_AABB_EXTENSION);
	// Extend AABB
	let mut fat_aabb = B2AABB {
		lower_bound: aabb.lower_bound - r,
		upper_bound: aabb.upper_bound + r,
	};

	// Predict AABB movement
	let d: B2vec2 = B2_AABB_MULTIPLIER * displacement;

	if d.x < 0.0 {
		fat_aabb.lower_bound.x += d.x;
	} else {
		fat_aabb.upper_bound.x += d.x;
	}

	if d.y < 0.0 {
		fat_aabb.lower_bound.y += d.y;
	} else {
		fat_aabb.upper_bound.y += d.y;
	}

	let tree_aabb: B2AABB = self_.m_nodes[proxy_id as usize].aabb;
	if tree_aabb.contains(&aabb) {
		// The tree AABB still contains the object, but it might be too large.
		// Perhaps the object was moving fast but has since gone to sleep.
		// The huge AABB is larger than the new fat AABB.
		let huge_aabb = B2AABB {
			lower_bound: fat_aabb.lower_bound - 4.0 * r,
			upper_bound: fat_aabb.upper_bound + 4.0 * r,
		};

		if huge_aabb.contains(&tree_aabb) {
			// The tree AABB contains the object AABB and the tree AABB is
			// not too large. No tree update needed.
			return false;
		}

		// Otherwise the tree AABB is huge and needs to be shrunk
	}

	self_.remove_leaf(proxy_id);

	self_.m_nodes[proxy_id as usize].aabb = fat_aabb;

	self_.insert_leaf(proxy_id);

	self_.m_nodes[proxy_id as usize].moved = true;

	return true;
}

pub fn insert_leaf<T: Clone + Default>(self_: &mut B2dynamicTree<T>, leaf: i32) {
	self_.m_insertion_count += 1;

	if self_.m_root == B2_NULL_NODE {
		self_.m_root = leaf;
		self_.m_nodes[self_.m_root as usize].parent = B2_NULL_NODE; 
		return;
	}

	// Find the best sibling for this node
	let leaf_aabb: B2AABB = self_.m_nodes[leaf as usize].aabb;
	let mut index: i32 = self_.m_root;
	while self_.m_nodes[index as usize].is_leaf() == false {
		let child1: i32 = self_.m_nodes[index as usize].child1;
		let child2: i32 = self_.m_nodes[index as usize].child2;

		let area: f32 = self_.m_nodes[index as usize].aabb.get_perimeter();

		let mut combined_aabb = B2AABB::default();
		combined_aabb.combine_two(self_.m_nodes[index as usize].aabb, leaf_aabb);
		let combined_area: f32 = combined_aabb.get_perimeter();

		// Cost of creating a new parent for this node and the new leaf
		let cost: f32 = 2.0 * combined_area;

		// Minimum cost of pushing the leaf further down the tree
		let inheritance_cost: f32 = 2.0 * (combined_area - area);

		// Cost of descending into child1
		let cost1: f32;
		if self_.m_nodes[child1 as usize].is_leaf() {
			let mut aabb = B2AABB::default();
			aabb.combine_two(leaf_aabb, self_.m_nodes[child1 as usize].aabb);
			cost1 = aabb.get_perimeter() + inheritance_cost;
		} else {
			let mut aabb = B2AABB::default();
			aabb.combine_two(leaf_aabb, self_.m_nodes[child1 as usize].aabb);
			let old_area: f32 = self_.m_nodes[child1 as usize].aabb.get_perimeter();
			let new_area: f32 = aabb.get_perimeter();
			cost1 = (new_area - old_area) + inheritance_cost;
		}

		// Cost of descending into child2
		let cost2: f32;
		if self_.m_nodes[child2 as usize].is_leaf() {
			let mut aabb = B2AABB::default();
			aabb.combine_two(leaf_aabb, self_.m_nodes[child2 as usize].aabb);
			cost2 = aabb.get_perimeter() + inheritance_cost;
		} else {
			let mut aabb = B2AABB::default();
			aabb.combine_two(leaf_aabb, self_.m_nodes[child2 as usize].aabb);
			let old_area: f32 = self_.m_nodes[child2 as usize].aabb.get_perimeter();
			let new_area: f32 = aabb.get_perimeter();
			cost2 = new_area - old_area + inheritance_cost;
		}

		// Descend according to the minimum cost.
		if cost < cost1 && cost < cost2 {
			break;
		}

		// Descend
		if cost1 < cost2 {
			index = child1;
		} else {
			index = child2;
		}
	}

	let sibling: i32 = index;

	// create a new parent.
	let old_parent: i32 = self_.m_nodes[sibling as usize].parent; 
	let new_parent: i32 = self_.allocate_node();
	self_.m_nodes[new_parent as usize].parent = old_parent; 
	self_.m_nodes[new_parent as usize].user_data = None;
	let temp_aabb = self_.m_nodes[sibling as usize].aabb;
	self_.m_nodes[new_parent as usize]
		.aabb
		.combine_two(leaf_aabb, temp_aabb);
	self_.m_nodes[new_parent as usize].height = self_.m_nodes[sibling as usize].height + 1;

	if old_parent != B2_NULL_NODE {
		// The sibling was not the root.
		if self_.m_nodes[old_parent as usize].child1 == sibling {
			self_.m_nodes[old_parent as usize].child1 = new_parent;
		} else {
			self_.m_nodes[old_parent as usize].child2 = new_parent;
		}

		self_.m_nodes[new_parent as usize].child1 = sibling;
		self_.m_nodes[new_parent as usize].child2 = leaf;
		self_.m_nodes[sibling as usize].parent = new_parent; 
		self_.m_nodes[leaf as usize].parent = new_parent; 
	} else {
		// The sibling was the root.
		self_.m_nodes[new_parent as usize].child1 = sibling;
		self_.m_nodes[new_parent as usize].child2 = leaf;
		self_.m_nodes[sibling as usize].parent = new_parent; 
		self_.m_nodes[leaf as usize].parent = new_parent; 
		self_.m_root = new_parent;
	}

	// Walk back up the tree fixing heights and AABBs
	index = self_.m_nodes[leaf as usize].parent; 
	while index != B2_NULL_NODE {
		index = self_.balance(index);

		let child1: i32 = self_.m_nodes[index as usize].child1;
		let child2: i32 = self_.m_nodes[index as usize].child2;

		b2_assert(child1 != B2_NULL_NODE);
		b2_assert(child2 != B2_NULL_NODE);

		self_.m_nodes[index as usize].height = 1 + b2_max(
			self_.m_nodes[child1 as usize].height,
			self_.m_nodes[child2 as usize].height,
		);
		let temp_aabb1 = self_.m_nodes[child1 as usize].aabb;
		let temp_aabb2 = self_.m_nodes[child2 as usize].aabb;
		self_.m_nodes[index as usize]
			.aabb
			.combine_two(temp_aabb1, temp_aabb2);

		index = self_.m_nodes[index as usize].parent; 
	}

	//validate();
}

pub fn remove_leaf<T: Clone + Default>(self_: &mut B2dynamicTree<T>, leaf: i32) {
	if leaf == self_.m_root {
		self_.m_root = B2_NULL_NODE;
		return;
	}

	let parent: i32 = self_.m_nodes[leaf as usize].parent; 
	let grand_parent: i32 = self_.m_nodes[parent as usize].parent; 
	let sibling: i32;
	if self_.m_nodes[parent as usize].child1 == leaf {
		sibling = self_.m_nodes[parent as usize].child2;
	} else {
		sibling = self_.m_nodes[parent as usize].child1;
	}

	if grand_parent != B2_NULL_NODE {
		// destroy parent and connect sibling to grandParent.
		if self_.m_nodes[grand_parent as usize].child1 == parent {
			self_.m_nodes[grand_parent as usize].child1 = sibling;
		} else {
			self_.m_nodes[grand_parent as usize].child2 = sibling;
		}
		self_.m_nodes[sibling as usize].parent = grand_parent; 
		self_.free_node(parent);

		// Adjust ancestor bounds.
		let mut index: i32 = grand_parent;
		while index != B2_NULL_NODE {
			index = self_.balance(index);

			let child1: i32 = self_.m_nodes[index as usize].child1;
			let child2: i32 = self_.m_nodes[index as usize].child2;

			let temp_aabb1 = self_.m_nodes[child1 as usize].aabb;
			let temp_aabb2 = self_.m_nodes[child2 as usize].aabb;

			self_.m_nodes[index as usize]
				.aabb
				.combine_two(temp_aabb1, temp_aabb2);
			self_.m_nodes[index as usize].height = 1 + b2_max(
				self_.m_nodes[child1 as usize].height,
				self_.m_nodes[child2 as usize].height,
			);

			index = self_.m_nodes[index as usize].parent; 
		}
	} else {
		self_.m_root = sibling;
		self_.m_nodes[sibling as usize].parent = B2_NULL_NODE; 
		self_.free_node(parent);
	}

	//validate();
}

// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
pub fn balance<T: Clone + Default>(self_: &mut B2dynamicTree<T>, i_a: i32) -> i32 {
	b2_assert(i_a != B2_NULL_NODE);

	let mut a = self_.m_nodes[i_a as usize].clone();
	if a.is_leaf() || a.height < 2 {
		return i_a;
	}

	let i_b: i32 = a.child1;
	let i_c: i32 = a.child2;
	b2_assert(0 <= i_b && i_b < self_.m_node_capacity);
	b2_assert(0 <= i_c && i_c < self_.m_node_capacity);

	let mut b = self_.m_nodes[i_b as usize].clone();
	let mut c = self_.m_nodes[i_c as usize].clone();

	let balance: i32 = c.height - b.height;

	

	// Rotate c up
	if balance > 1 {
		let i_f: i32 = c.child1;
		let i_g: i32 = c.child2;
		let mut f = self_.m_nodes[i_f as usize].clone();
		let mut g = self_.m_nodes[i_g as usize].clone();
		b2_assert(0 <= i_f && i_f < self_.m_node_capacity);
		b2_assert(0 <= i_g && i_g < self_.m_node_capacity);

		// Swap A and c
		c.child1 = i_a;
		c.parent = a.parent;
		a.parent = i_c;

		// A's old parent should point to c
		if c.parent != B2_NULL_NODE {
			if self_.m_nodes[c.parent as usize].child1 == i_a {
				self_.m_nodes[c.parent as usize].child1 = i_c;
			} else {
				b2_assert(self_.m_nodes[c.parent as usize].child2 == i_a);
				self_.m_nodes[c.parent as usize].child2 = i_c;
			}
		} else {
			self_.m_root = i_c;
		}

		// Rotate
		if f.height > g.height {
			c.child2 = i_f;
			a.child2 = i_g;
			g.parent = i_a;
			a.aabb.combine_two(b.aabb, g.aabb);
			c.aabb.combine_two(a.aabb, f.aabb);

			a.height = 1 + b2_max(b.height, g.height);
			c.height = 1 + b2_max(a.height, f.height);
		} else {
			c.child2 = i_g;
			a.child2 = i_f;
			f.parent = i_a;
			a.aabb.combine_two(b.aabb, f.aabb);
			c.aabb.combine_two(a.aabb, g.aabb);

			a.height = 1 + b2_max(b.height, f.height);
			c.height = 1 + b2_max(a.height, g.height);
		}

		//box2d-rs: because of borrowing and lifetime problems
		self_.m_nodes[i_a as usize] = a;
		self_.m_nodes[i_c as usize] = c;
		self_.m_nodes[i_f as usize] = f;
		self_.m_nodes[i_g as usize] = g;

		return i_c;
	}
	// Rotate b up
	if balance < -1 {
		let i_d: i32 = b.child1;
		let i_e: i32 = b.child2;
		let mut d = self_.m_nodes[i_d as usize].clone();
		let mut e = self_.m_nodes[i_e as usize].clone();
		b2_assert(0 <= i_d && i_d < self_.m_node_capacity);
		b2_assert(0 <= i_e && i_e < self_.m_node_capacity);

		// Swap A and b
		b.child1 = i_a;
		b.parent = a.parent;
		a.parent = i_b;

		// A's old parent should point to b
		if b.parent != B2_NULL_NODE {
			if self_.m_nodes[b.parent as usize].child1 == i_a {
				self_.m_nodes[b.parent as usize].child1 = i_b;
			} else {
				b2_assert(self_.m_nodes[b.parent as usize].child2 == i_a);
				self_.m_nodes[b.parent as usize].child2 = i_b;
			}
		} else {
			self_.m_root = i_b;
		}

		// Rotate
		if d.height > e.height {
			b.child2 = i_d;
			a.child1 = i_e;
			e.parent = i_a;
			a.aabb.combine_two(c.aabb, e.aabb);
			b.aabb.combine_two(a.aabb, d.aabb);

			a.height = 1 + b2_max(c.height, e.height);
			b.height = 1 + b2_max(a.height, d.height);
		} else {
			b.child2 = i_e;
			a.child1 = i_d;
			d.parent = i_a;
			a.aabb.combine_two(c.aabb, d.aabb);
			b.aabb.combine_two(a.aabb, e.aabb);

			a.height = 1 + b2_max(c.height, d.height);
			b.height = 1 + b2_max(a.height, e.height);
		}

		//box2d-rs: because of borrowing and lifetime problems
		self_.m_nodes[i_a as usize] = a;
		self_.m_nodes[i_b as usize] = b;
		self_.m_nodes[i_e as usize] = e;
		self_.m_nodes[i_d as usize] = d;

		return i_b;
	}

	return i_a;
}

pub fn get_height<T: Clone + Default>(self_: &B2dynamicTree<T>) -> i32 {
	if self_.m_root == B2_NULL_NODE {
		return 0;
	}

	return self_.m_nodes[self_.m_root as usize].height;
}

//
pub fn get_area_ratio<T: Clone + Default>(self_: &B2dynamicTree<T>) -> f32 {
	if self_.m_root == B2_NULL_NODE {
		return 0.0;
	}

	let root = &self_.m_nodes[self_.m_root as usize];
	let root_area: f32 = root.aabb.get_perimeter();

	let mut total_area: f32 = 0.0;
	for i in 0..self_.m_node_capacity {
		let node = &self_.m_nodes[i as usize];
		if node.height < 0 {
			// Free node in pool
			continue;
		}

		total_area += node.aabb.get_perimeter();
	}

	return total_area / root_area;
}

// Compute the height of a sub-tree.
pub fn compute_height_by_node<T: Clone + Default>(
	self_: &B2dynamicTree<T>,
	node_id: i32,
) -> i32 {
	b2_assert(0 <= node_id && node_id < self_.m_node_capacity);
	let node = &self_.m_nodes[node_id as usize];

	if node.is_leaf() {
		return 0;
	}

	let height1: i32 = self_.compute_height_by_node(node.child1);
	let height2: i32 = self_.compute_height_by_node(node.child2);
	return 1 + b2_max(height1, height2);
}

pub fn compute_height<T: Clone + Default>(self_: &B2dynamicTree<T>) -> i32 {
	let height: i32 = compute_height_by_node(self_, self_.m_root);
	return height;
}

pub fn validate_structure<T: Clone + Default>(self_: &B2dynamicTree<T>, index: i32) {
	if index == B2_NULL_NODE {
		return;
	}

	

	if index == self_.m_root {
		b2_assert(self_.m_nodes[index as usize].parent == B2_NULL_NODE);
	}

	let node = &self_.m_nodes[index as usize];

	let child1: i32 = node.child1;
	let child2: i32 = node.child2;

	if node.is_leaf() {
		b2_assert(child1 == B2_NULL_NODE);
		b2_assert(child2 == B2_NULL_NODE);
		b2_assert(node.height == 0);
		return;
	}

	b2_assert(0 <= child1 && child1 < self_.m_node_capacity);
	b2_assert(0 <= child2 && child2 < self_.m_node_capacity);

	b2_assert(self_.m_nodes[child1 as usize].parent == index);
	b2_assert(self_.m_nodes[child2 as usize].parent == index);

	self_.validate_structure(child1);
	self_.validate_structure(child2);
}

pub fn validate_metrics<T: Clone + Default>(self_: &B2dynamicTree<T>, index: i32) {
	if index == B2_NULL_NODE {
		return;
	}

	let node = &self_.m_nodes[index as usize];

	let child1: i32 = node.child1;
	let child2: i32 = node.child2;

	if node.is_leaf() {
		b2_assert(child1 == B2_NULL_NODE);
		b2_assert(child2 == B2_NULL_NODE);
		b2_assert(node.height == 0);
		return;
	}

	b2_assert(0 <= child1 && child1 < self_.m_node_capacity);
	b2_assert(0 <= child2 && child2 < self_.m_node_capacity);

	let height1: i32 = self_.m_nodes[child1 as usize].height;
	let height2: i32 = self_.m_nodes[child2 as usize].height;
	let height: i32;
	height = 1 + b2_max(height1, height2);
	b2_assert(node.height == height);

	let mut aabb = B2AABB::default();
	aabb.combine_two(
		self_.m_nodes[child1 as usize].aabb,
		self_.m_nodes[child2 as usize].aabb,
	);

	b2_assert(is_equal(aabb.lower_bound, node.aabb.lower_bound));
	b2_assert(is_equal(aabb.upper_bound, node.aabb.upper_bound));

	self_.validate_metrics(child1);
	self_.validate_metrics(child2);
}

pub fn validate<T: Clone + Default>(self_: &B2dynamicTree<T>) {
	if B2_DEBUG
	{
		self_.validate_structure(self_.m_root);
		self_.validate_metrics(self_.m_root);

		let mut free_count: i32 = 0;
		let mut free_index: i32 = self_.m_free_list;
		while free_index != B2_NULL_NODE {
			b2_assert(0 <= free_index && free_index < self_.m_node_capacity);
			free_index = self_.m_nodes[free_index as usize].parent;
			free_count += 1;
		}

		b2_assert(self_.get_height() == self_.compute_height());

		b2_assert(self_.m_node_count + free_count == self_.m_node_capacity);
	}
}

pub fn get_max_balance<T: Clone + Default>(self_: &B2dynamicTree<T>) -> i32 {
	let mut max_balance: i32 = 0;
	for i in 0..self_.m_node_capacity {
		let node = &self_.m_nodes[i as usize];
		if node.height <= 1 {
			continue;
		}

		b2_assert(node.is_leaf() == false);

		let child1: i32 = node.child1;
		let child2: i32 = node.child2;
		let balance: i32 =
			b2_abs_i32(self_.m_nodes[child2 as usize].height - self_.m_nodes[child1 as usize].height);
		max_balance = b2_max(max_balance, balance);
	}

	return max_balance;
}

pub fn rebuild_bottom_up<T: Clone + Default>(self_: &mut B2dynamicTree<T>) {
	let mut nodes = Vec::<i32>::new();
	nodes.resize(self_.m_node_count as usize, -1);
	let mut count: i32 = 0;

	// Build array of leaves. Free the rest.
	for i in 0..self_.m_node_capacity {
		if self_.m_nodes[i as usize].height < 0 {
			// free node in pool
			continue;
		}

		if self_.m_nodes[i as usize].is_leaf() {
			self_.m_nodes[i as usize].parent = B2_NULL_NODE;
			nodes[count as usize] = i;
			count += 1;
		} else {
			self_.free_node(i);
		}
	}

	while count > 1 {
		let mut min_cost: f32 = B2_MAX_FLOAT;
		let mut i_min: i32 = -1;
		let mut j_min: i32 = -1;
		for i in 0..count {
			let aabbi: B2AABB = self_.m_nodes[nodes[i as usize] as usize].aabb;

			for j in i + 1..count {
				let aabbj: B2AABB = self_.m_nodes[nodes[j as usize] as usize].aabb;
				let mut b = B2AABB::default();
				b.combine_two(aabbi, aabbj);
				let cost: f32 = b.get_perimeter();
				if cost < min_cost {
					i_min = i;
					j_min = j;
					min_cost = cost;
				}
			}
		}

		let index1: i32 = nodes[i_min as usize];
		let index2: i32 = nodes[j_min as usize];
		let mut child1 = self_.m_nodes[index1 as usize].clone();
		let mut child2 = self_.m_nodes[index2 as usize].clone();

		let parent_index: i32 = self_.allocate_node();
		let parent = &mut self_.m_nodes[parent_index as usize];
		parent.child1 = index1;
		parent.child2 = index2;
		parent.height = 1 + b2_max(child1.height, child2.height);
		parent.aabb.combine_two(child1.aabb, child2.aabb);
		parent.parent = B2_NULL_NODE;

		child1.parent = parent_index;
		child2.parent = parent_index;

		self_.m_nodes[index1 as usize] = child1;
		self_.m_nodes[index2 as usize] = child2;

		nodes[j_min as usize] = nodes[(count - 1) as usize];
		nodes[i_min as usize] = parent_index;
		count-=1;
	}

	self_.m_root = nodes[0];

	self_.validate();
}

pub fn shift_origin<T: Clone + Default>(
	self_: &mut B2dynamicTree<T>,
	new_origin: B2vec2,
) {
	// Build array of leaves. Free the rest.
	for i in 0..self_.m_node_capacity {
		self_.m_nodes[i as usize].aabb.lower_bound -= new_origin;
		self_.m_nodes[i as usize].aabb.upper_bound -= new_origin;
	}
}
