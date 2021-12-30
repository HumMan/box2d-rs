//box2d-rs: waiting const_generics feature
//#![feature(const_generics)]

use crate::b2_common::*;

const B2_GROWABLE_STACK_SIZE: usize = 256;

/// This is a growable LIFO stack with an initial capacity of n.
/// If the stack size exceeds the initial capacity, the heap is used
/// to increase the size of the stack.
pub struct B2growableStack<T: Copy + Default> {
	m_stack: Vec<T>,
	m_array: [T; B2_GROWABLE_STACK_SIZE],
	m_count: usize,
	// m_capacity: usize
}

impl<T: Copy + Default> B2growableStack<T> {
	pub fn new() -> Self {
		return B2growableStack {
			m_stack: Vec::<T>::default(),
			m_array: [T::default(); B2_GROWABLE_STACK_SIZE],
			m_count: 0,
			// m_capacity : B2_GROWABLE_STACK_SIZE
		};
	}

	pub fn push(&mut self, element: &T) {
		if self.m_count >= B2_GROWABLE_STACK_SIZE {
			if self.m_count > self.m_stack.len() {
				self.m_stack.resize(self.m_stack.len() * 2, T::default());
			}
			self.m_stack[self.m_count] = *element;
			self.m_count += 1;
		} else {
			self.m_array[self.m_count] = *element;
			self.m_count += 1;
		}
	}

	pub fn pop(&mut self) -> T {
		b2_assert(self.m_count > 0);
		self.m_count -= 1;
		if self.m_count >= B2_GROWABLE_STACK_SIZE {
			return self.m_array[self.m_count];
		} else {
			return self.m_array[self.m_count];
		}
	}

	pub fn get_count(&self) -> usize {
		return self.m_count;
	}
}
