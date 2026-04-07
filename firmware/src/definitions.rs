use core::convert::TryFrom;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

pub trait AtomicEnumRepr: Copy + TryFrom<u8> {
    fn to_u8(self) -> u8;
}

pub struct AtomicEnum<E: AtomicEnumRepr> {
    inner: AtomicU8,
    _marker: PhantomData<E>,
}

impl<E: AtomicEnumRepr> AtomicEnum<E> {
    pub const fn new(value: u8) -> Self {
        Self {
            inner: AtomicU8::new(value),
            _marker: PhantomData,
        }
    }

    /*
    pub fn new(value: E) -> Self {
        Self {
            inner: AtomicU8::new(value.to_u8()),
            _marker: PhantomData,
        }
    }
    */

    pub fn load(&self) -> E {
        let raw = self.inner.load(Ordering::Relaxed);
        E::try_from(raw).ok().unwrap()
    }

    pub fn store(&self, value: E) {
        self.inner.store(value.to_u8(), Ordering::Relaxed);
    }
}
