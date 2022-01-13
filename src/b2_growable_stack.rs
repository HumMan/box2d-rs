use crate::b2_common::*;

/// This is a growable LIFO stack with an initial capacity of n.
/// If the stack size exceeds the initial capacity, the heap is used
/// to increase the size of the stack.
pub struct B2growableStack<T: Copy + Default, const N: usize> {

	value:B2rsStackOrVec<T,N>
	
	//box2d-rs: using enum because problems to have ref to own field
	// T* m_stack;
	// T m_array[N];
	// int32 m_count;
	// int32 m_capacity;
}

struct B2rsStaticArrayWithSize<T: Copy + Default, const N: usize>
{
	m_array: [T; N],
	m_count: usize,
}

enum B2rsStackOrVec<T: Copy + Default, const N: usize>
{
	Stack(B2rsStaticArrayWithSize<T,N>),
	DynVec(Vec<T>)
}

impl<T: Copy + Default, const N: usize> B2growableStack<T,N> {
	pub fn new() -> Self {
		return B2growableStack {
			value: B2rsStackOrVec::Stack(B2rsStaticArrayWithSize {
				m_array: [T::default(); N],
				m_count: 0,
			})
		};
	}

	pub fn push(&mut self, element: &T) {
		match self.value
		{
			B2rsStackOrVec::Stack(ref mut stack)=>{
				if stack.m_count >= N {
					let mut v = Vec::<T>::with_capacity(stack.m_count+1);
					v.extend_from_slice(&stack.m_array);
					v.push(*element);
					self.value = B2rsStackOrVec::DynVec(v);
				}else{
					stack.m_array[stack.m_count] = *element;
					stack.m_count += 1;
				}
			},
			B2rsStackOrVec::DynVec(ref mut vec)=>{
				vec.push(*element);
			}
		}
	}

	pub fn pop(&mut self) -> T {
		match self.value
		{
			B2rsStackOrVec::Stack(ref mut stack)=>{
				b2_assert(stack.m_count > 0);
				stack.m_count-=1;
				return stack.m_array[stack.m_count];
			},
			B2rsStackOrVec::DynVec(ref mut vec)=>{
				b2_assert(vec.len() > 0);
				return vec.pop().unwrap();
			}
		}
	}

	pub fn get_count(&self) -> usize {
		match self.value
		{
			B2rsStackOrVec::Stack(ref stack)=>{
				return stack.m_count;
			},
			B2rsStackOrVec::DynVec(ref vec)=>{
				return vec.len();
			}
		}
	}
}
