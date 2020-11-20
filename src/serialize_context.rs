use std::marker::PhantomData;

pub(crate) struct WithContext<'ctx, T: ?Sized, A> {
    pub(crate) context: &'ctx T,
    pub(crate) phantom: PhantomData<A>,
}

impl<'ctx, T: ?Sized, A> WithContext<'ctx, T, A>{
    pub(crate) fn new(context: &'ctx T) -> Self{
        Self{
            context,
            phantom: PhantomData
        }
    }
}