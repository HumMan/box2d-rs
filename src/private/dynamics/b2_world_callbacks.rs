use crate::b2_fixture::*;
use crate::b2_settings::*;

// Return true if contact calculations should be performed between these two shapes.
// If you implement your own collision filter you may want to build from this implementation.
pub fn should_collide<D:UserDataType>(fixture_a: FixturePtr<D>, fixture_b: FixturePtr<D>) -> bool
{
	let filter_a = fixture_a.borrow().get_filter_data();
	let filter_b = fixture_b.borrow().get_filter_data();

	if filter_a.group_index == filter_b.group_index && filter_a.group_index != 0
	{
		return filter_a.group_index > 0;
	}

	let collide:bool = (filter_a.mask_bits & filter_b.category_bits) != 0 && (filter_a.category_bits & filter_b.mask_bits) != 0;
	return collide;
}
