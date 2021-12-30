use crate::b2_broad_phase::*;
use crate::b2_collision::*;
use crate::b2_dynamic_tree::*;
use crate::b2_math::*;

pub fn b2_broad_phase_b2_broad_phase<UserDataType: Default + Clone>() -> B2broadPhase<UserDataType> {
	let m_proxy_count: i32 = 0;

	let m_pair_capacity: i32 = 16;
	let m_pair_count: i32 = 0;
	let mut m_pair_buffer = Vec::<B2pair>::new();
	m_pair_buffer.resize_with(m_pair_capacity as usize, Default::default);

	let m_move_capacity: i32 = 16;
	let m_move_count: i32 = 0;
	let mut m_move_buffer = Vec::<i32>::new();
	m_move_buffer.resize_with(m_move_capacity as usize, Default::default);

	return B2broadPhase::<UserDataType> {
		m_tree: B2dynamicTree::<UserDataType>::new(),
		m_proxy_count,
		m_move_buffer,
		m_move_capacity,
		m_move_count,
		pairs: B2broadPhasePairs {
			m_pair_buffer,
			m_pair_capacity,
			m_pair_count,
		},
	};
}

pub fn b2_broad_phase_create_proxy<T: Default + Clone>(
	self_: &mut B2broadPhase<T>,
	aabb: B2AABB,
	user_data: &T,
) -> i32 {
	let proxy_id: i32 = self_.m_tree.create_proxy(aabb, user_data);
	self_.m_proxy_count += 1;
	self_.buffer_move(proxy_id);
	return proxy_id;
}

pub fn b2_broad_phase_destroy_proxy<T: Default + Clone>(self_: &mut B2broadPhase<T>, proxy_id: i32) {
	self_.un_buffer_move(proxy_id);
	self_.m_proxy_count -= 1;
	self_.m_tree.destroy_proxy(proxy_id);
}

pub fn b2_broad_phase_move_proxy<T: Default + Clone>(
	self_: &mut B2broadPhase<T>,
	proxy_id: i32,
	aabb: B2AABB,
	displacement: B2vec2,
) {
	let buffer: bool = self_.m_tree.move_proxy(proxy_id, aabb, displacement);
	if buffer {
		self_.buffer_move(proxy_id);
	}
}

pub fn b2_broad_phase_touch_proxy<T: Default + Clone>(self_: &mut B2broadPhase<T>, proxy_id: i32) {
	self_.buffer_move(proxy_id);
}

pub fn b2_broad_phase_buffer_move<T: Default + Clone>(self_: &mut B2broadPhase<T>, proxy_id: i32) {
	if self_.m_move_count == self_.m_move_capacity {
		self_.m_move_capacity *= 2;
		self_.m_move_buffer
			.resize_with(self_.m_move_capacity as usize, Default::default);
	}

	self_.m_move_buffer[self_.m_move_count as usize] = proxy_id;
	self_.m_move_count += 1;
}

pub fn b2_broad_phase_un_buffer_move<T: Default + Clone>(self_: &mut B2broadPhase<T>, proxy_id: i32) {
	for i in 0..self_.m_move_count {
		if self_.m_move_buffer[i as usize] == proxy_id {
			self_.m_move_buffer[i as usize] = E_NULL_PROXY;
		}
	}
}

// This is called from B2dynamicTree::query when we are gathering pairs.
pub fn b2_broad_phase_query_callback(self_: &mut B2broadPhasePairs, m_query_proxy_id: i32, proxy_id: i32, moved:bool) -> bool {
	// A proxy cannot form a pair with itself.
	if proxy_id == m_query_proxy_id {
		return true;
	}

	//moved to closure
	//let moved: bool = self_.m_tree.WasMoved(proxy_id);
	if moved && proxy_id > m_query_proxy_id {
		// Both proxies are moving. Avoid duplicate pairs.
		return true;
	}

	// Grow the pair buffer as needed.
	if self_.m_pair_count == self_.m_pair_capacity {
		self_.m_pair_capacity = self_.m_pair_capacity + (self_.m_pair_capacity >> 1);
		self_.m_pair_buffer
			.resize_with(self_.m_pair_capacity as usize, Default::default);
	}

	self_.m_pair_buffer[self_.m_pair_count as usize].proxy_id_a = b2_min(proxy_id, m_query_proxy_id);
	self_.m_pair_buffer[self_.m_pair_count as usize].proxy_id_b = b2_max(proxy_id, m_query_proxy_id);
	self_.m_pair_count += 1;

	return true;
}
