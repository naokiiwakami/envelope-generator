use core::convert::TryFrom;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

#[derive(Clone, Copy, PartialEq, Debug, defmt::Format)]
#[repr(u8)]
pub enum PotKind {
    Attack = 0,
    Decay = 1,
    Sustain = 2,
    Release = 3,
    Extra1 = 4,
    Extra2 = 5,
    CvADepth = 6,
    CvBDepth = 7,
}

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

    pub fn load(&self) -> E {
        let raw = self.inner.load(Ordering::Relaxed);
        E::try_from(raw).ok().unwrap()
    }

    pub fn store(&self, value: E) {
        self.inner.store(value.to_u8(), Ordering::Relaxed);
    }
}

#[derive(Clone, Copy, PartialEq, Debug, defmt::Format)]
#[repr(u8)]
pub enum CvKind {
    A,
    B,
}
