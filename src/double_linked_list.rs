use std::cell::RefCell;
use std::ptr;
use std::rc::{Rc, Weak};
use crate::b2_settings::upgrade;
use crate::linked_list::*;
use std::fmt;

pub trait DoubleLinkedListNode<T: DoubleLinkedListNode<T> + ?Sized>: LinkedListNode<T> {
    fn get_prev(&self) -> Option<Weak<RefCell<T>>>;
    fn set_prev(&mut self, value: Option<Weak<RefCell<T>>>);
    //fn remove_from_list(&mut self);
}

pub struct DoubleLinkedList<T: DoubleLinkedListNode<T> + ?Sized> {
    head: Option<Rc<RefCell<T>>>,
}

impl<T> fmt::Debug for dyn DoubleLinkedListNode<T> 
    {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Node")
    }
}

impl<T> fmt::Debug for DoubleLinkedList<T> 
where
    T: DoubleLinkedListNode<T> + fmt::Debug + ?Sized,
    {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
         f.debug_list().entries(self.iter()).finish()
    }
}

impl<T:DoubleLinkedListNode<T> + ?Sized> Default for DoubleLinkedList<T> {
    fn default() -> Self {
        return Self { head: None };
    }
}

impl<T:DoubleLinkedListNode<T> + ?Sized> Clone for DoubleLinkedList<T> {
    fn clone(&self) -> Self {
        return Self {
            head: self.head.clone(),
        };
    }
}

impl<T> DoubleLinkedList<T>
where
    T: DoubleLinkedListNode<T> + ?Sized,
{
    pub fn remove_all(&mut self) {
        let mut cur_link = self.head.take();
        while let Some(boxed_node) = cur_link {
            cur_link = boxed_node.borrow_mut().take_next();
        }
    }
    pub fn remove(&mut self, node_to_remove: Rc<RefCell<T>>) {

        //self.validate();

        //let original_len = self.len();
        //assert!(self.contains(node_to_remove.clone()));

        let (prev,next);
        {
            let node_to_remove = node_to_remove.borrow();
            prev = node_to_remove.get_prev();
            next = node_to_remove.get_next();
        }

        if let Some(prev) = prev.clone()
		{
			upgrade(&prev).borrow_mut().set_next(next.clone())
		}
		if let Some(next) = next
		{
			next.borrow_mut().set_prev(prev);
        }
        
        if self.head.is_some() {
            if ptr::eq(
                self.head.as_ref().unwrap().as_ref(),
                node_to_remove.as_ref(),
            ) {
                let next = self.head.as_ref().unwrap().borrow().get_next();
                self.head = next;
            }
        }
         else {
            //println!("{:#?}", self);
            assert!(false);
        }

        //assert!(original_len==self.len()+1);
        //assert!(!self.contains(node_to_remove.clone()));

        //self.validate();
    }

    // pub fn len(&self)->usize{
    //     let mut result = 0;
    //     for _i in self.iter(){
    //         result+=1;
    //     }
    //     result
    // }

    // pub fn contains(&self, node_to_check: Rc<RefCell<T>>)->bool{
    //     for node in self.iter(){
    //         if ptr::eq(
    //             node_to_check.as_ref(),
    //             node.as_ref(),
    //         ) {
    //             return true;
    //         }
    //     }
    //     return false;
    // }

    // pub fn validate(&self){
    //     let mut list = Vec::new();
    //     for node in self.iter(){
    //        list.push(node.clone());
    //     }
    //     if list.len()==1
    //     {
    //         let node = list[0].clone();
    //         let prev = node.borrow().get_prev();
    //         let next = node.borrow().get_next();
    //         assert!(prev.is_none());
    //         assert!(next.is_none());
    //     }
    //     else
    //     {
    //         for (i, node) in list.iter().enumerate(){
    //             let prev = node.borrow().get_prev();
    //             let next = node.borrow().get_next();
                
    //             if i==0 {
    //                 assert!(prev.is_none());
    //                 assert!(!next.is_none());

    //                 assert!(ptr::eq(
    //                     next.as_ref().unwrap().as_ref(),
    //                     list[i+1].as_ref(),
    //                 ));
    //             }
    //             if i==list.len()-1 {
    //                 assert!(!prev.is_none());
    //                 assert!(next.is_none());

    //                 assert!(ptr::eq(
    //                     upgrade(prev.as_ref().unwrap()).as_ref(),
    //                     list[i-1].as_ref(),
    //                 ));
    //             }
    //             if i>=1 && i<list.len()-1 {
    //                 assert!(!prev.is_none());
    //                 assert!(!next.is_none());

    //                 assert!(ptr::eq(
    //                     upgrade(prev.as_ref().unwrap()).as_ref(),
    //                     list[i-1].as_ref(),
    //                 ));

    //                 assert!(ptr::eq(
    //                     next.as_ref().unwrap().as_ref(),
    //                     list[i+1].as_ref(),
    //                 ));
    //             }
    //         }
    //     }
    // }

    pub fn push_front(&mut self, node_to_push: Rc<RefCell<T>>) {
       // assert!(!self.contains(node_to_push.clone()));
        //self.validate();

        //let original_len = self.len();
        {
            if let Some(head) = self.head.as_ref()
            {
                head.borrow_mut().set_prev(Some(Rc::downgrade(&node_to_push)));
            }

            let mut node = node_to_push.borrow_mut();
            node.set_prev(None);
            node.set_next(self.head.take());
        }
        self.head = Some(node_to_push.clone());
        //assert!(original_len+1==self.len());
        //assert!(self.contains(node_to_push.clone()));
        //self.validate(); 
    }
    
    pub fn clear(&mut self){
        self.head = None;
    }
    pub fn iter(&self)->Iter<T>{
        return Iter{
            next: self.head.clone()
        }
    }
}

pub struct Iter<T>
where
    T: DoubleLinkedListNode<T> + ?Sized,
{
    next: Option<Rc<RefCell<T>>>
}

impl<T> Iterator for Iter<T> 
where
    T: DoubleLinkedListNode<T> + ?Sized,
{
    type Item = Rc<RefCell<T>>;
    fn next(&mut self) -> Option<Self::Item> {
        self.next.clone().map(|node| {
            self.next = node.borrow().get_next();
            node
        })
    }
}