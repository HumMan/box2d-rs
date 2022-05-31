#[cfg(feature="serde_support")]
use serde::{de::DeserializeOwned, Serialize};

use std::rc::{Rc,Weak};

pub fn upgrade<T: ?Sized>(v: &Weak<T>) -> Rc<T> {
	return v.upgrade().unwrap();
}

pub fn upgrade_opt<T: ?Sized>(v: &Option<Weak<T>>) -> Rc<T> {
	return v.as_ref().unwrap().upgrade().unwrap();
}

#[cfg(not(feature="serde_support"))]
pub trait UserDataType: Default + Clone + 'static {
    type Fixture: Default + Clone + std::fmt::Debug;
    type Body: Default + Clone + std::fmt::Debug;
    type Joint: Default + Clone + std::fmt::Debug;
}

#[cfg(feature="serde_support")]
pub trait UserDataType: Default + Clone + Serialize + DeserializeOwned + 'static {
    type Fixture: Default + Clone + Serialize + DeserializeOwned + std::fmt::Debug;
    type Body: Default + Clone + Serialize + DeserializeOwned + std::fmt::Debug;
    type Joint: Default + Clone + Serialize + DeserializeOwned + std::fmt::Debug;
}
