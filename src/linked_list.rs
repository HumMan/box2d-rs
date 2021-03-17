#[cfg(feature="serde_support")]
use serde::{
    ser::{SerializeSeq, SerializeStruct},
    Deserialize, Serialize, Serializer,
};
use std::cell::RefCell;
use std::ptr;
use std::rc::Rc;

pub trait LinkedListNode<T:LinkedListNode<T> + ?Sized> {
    fn get_next(&self) -> Option<Rc<RefCell<T>>>;
    fn take_next(&mut self) -> Option<Rc<RefCell<T>>>;
    fn set_next(&mut self, value: Option<Rc<RefCell<T>>>);
}

pub struct LinkedList<T:LinkedListNode<T> + ?Sized> {
    head: Option<Rc<RefCell<T>>>,
}

impl<T:LinkedListNode<T> + ?Sized> Default for LinkedList<T> {
    fn default() -> Self {
        return Self { head: None };
    }
}

impl<T:LinkedListNode<T> + ?Sized> Clone for LinkedList<T> {
    fn clone(&self) -> Self {
        return Self {
            head: self.head.clone(),
        };
    }
}

#[cfg(feature="serde_support")]
impl<T> Serialize for LinkedList<T>
where
    T: LinkedListNode<T> + ?Sized + Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let len = self.iter().count();
        let mut state = serializer.serialize_seq(Some(len))?;
        for jn_ in self.iter() {
            state.serialize_element(&*jn_.borrow())?;
        }
        state.end()
    }
}

impl<T> LinkedList<T>
where
    T: LinkedListNode<T> + ?Sized,
{
    pub fn remove_all(&mut self) {
        let mut cur_link = self.head.take();
        while let Some(boxed_node) = cur_link {
            cur_link = boxed_node.borrow_mut().take_next();
        }
    }

    pub fn remove(&mut self, node_to_remove: Rc<RefCell<T>>) {
        let mut found: bool = false;

        match self.head.clone() {
            Some(ref head_node) => {
                if ptr::eq(head_node.as_ref(), node_to_remove.as_ref()) {
                    self.head = head_node.borrow().get_next();
                    found = true;
                } else {
                    let mut node = head_node.clone();
                    loop {
                        let next;
                        {
                            next = node.borrow().get_next();
                        }

                        if let Some(next) = next {
                            if ptr::eq(next.as_ref(), node_to_remove.as_ref()) {
                                node.borrow_mut()
                                    .set_next(node_to_remove.borrow().get_next());
                                found = true;
                                break;
                            }

                            node = next;
                        } else {
                            break;
                        }
                    }
                }
            }
            None => {}
        }

        assert!(found);
    }

    pub fn push_front(&mut self, node_to_push: Rc<RefCell<T>>) {
        node_to_push.borrow_mut().set_next(self.head.take());
        self.head = Some(node_to_push);
    }

    pub fn front(&self) -> Option<&Rc<RefCell<T>>> {
        return self.head.as_ref();
    }

    pub fn front_mut(&mut self) -> &mut Option<Rc<RefCell<T>>> {
        return &mut self.head;
    }
    pub fn clear(&mut self) {
        self.head = None;
    }
    pub fn iter(&self) -> Iter<T> {
        return Iter {
            next: self.head.clone(),
        };
    }
}

pub struct Iter<T>
where
    T: LinkedListNode<T> + ?Sized,
{
    next: Option<Rc<RefCell<T>>>,
}

impl<T> Iterator for Iter<T>
where
    T: LinkedListNode<T> + ?Sized,
{
    type Item = Rc<RefCell<T>>;
    fn next(&mut self) -> Option<Self::Item> {
        self.next.clone().map(|node| {
            self.next = node.borrow().get_next();
            node
        })
    }
}