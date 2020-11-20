use crate::b2_broad_phase::*;
use crate::b2_contact::*;
use crate::b2_settings::*;
use crate::b2_world_callbacks::*;
use crate::b2_fixture::*;
use crate::double_linked_list::*;
use crate::private::dynamics::b2_contact_manager as private;
use crate::private::dynamics::b2_contact_registers::*;

use std::cell::RefCell;
use std::rc::Rc;

// impl<D: UserDataType> Default for B2contactManager<D>
// {
// 	fn default() -> Self
// 	{
// 		return private::b2_contact_manager_b2_contact_manager();
// 	}
// }

impl<D: UserDataType> B2contactManager<D> {
	
	//TODO_humman переместить в приват и использовать B2contactFilterDefault
	pub fn new() ->Self{
		return Self{
			m_registers: Default::default(),
			m_broad_phase: Rc::new(RefCell::new(B2broadPhase::new())),
			m_contact_list: Default::default(),
			m_contact_count: 0,
			m_contact_filter: Some(Rc::new(RefCell::new(B2contactFilterDefault{}))),
			m_contact_listener: Default::default(),
		};
	}

	pub fn find_new_contacts(&mut self) {
		private::b2_contact_manager_find_new_contacts(self);
	}

	pub fn destroy(&mut self, c: ContactPtr<D>) {
		private::b2_contact_manager_destroy(self, c);
	}

	pub fn collide(self_: B2contactManagerPtr<D>) {
		private::b2_contact_manager_collide(self_);
	}

	pub fn get_broad_phase(&self)->B2broadPhasePtr<FixtureProxyPtr<D>>{
		return self.m_broad_phase.clone();
	}
}

// impl<D: UserDataType> Debug for dyn B2contactFilter<D> {
//     fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
//         write!(f, "B2contactFilter")
//     }
// }

// impl<D: UserDataType> Debug for dyn B2contactListener<D> {
//     fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
//         write!(f, "B2contactListener")
//     }
// }

pub type B2contactManagerPtr<D> = Rc<RefCell<B2contactManager<D>>>;

// Delegate of B2world.
pub struct B2contactManager<D: UserDataType> {
	pub(crate) m_registers: ContactRegisters<D>,
	pub(crate) m_broad_phase: B2broadPhasePtr<FixtureProxyPtr<D>>,
	pub(crate) m_contact_list: DoubleLinkedList<dyn B2contactDynTrait<D>>,
	pub(crate) m_contact_count: usize,
	pub(crate) m_contact_filter: Option<B2contactFilterPtr<D>>,
	pub(crate) m_contact_listener: Option<B2contactListenerPtr<D>>,
}

impl<D: UserDataType> Drop for B2contactManager<D>
{
    fn drop(&mut self) {
		self.m_contact_list.remove_all();
    }
}

impl<D:UserDataType> AddPairTrait<FixtureProxyPtr<D>> for B2contactManager<D>
{
	// Broad-phase callback.
	fn add_pair(&mut self, proxy_user_data_a: Option<FixtureProxyPtr<D>>, proxy_user_data_b: Option<FixtureProxyPtr<D>>) {
		private::b2_contact_manager_add_pair(self, proxy_user_data_a, proxy_user_data_b);
	}
}