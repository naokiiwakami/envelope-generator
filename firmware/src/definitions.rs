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
    CvDepthA = 6,
    CvDepthB = 7,
}

impl TryFrom<u8> for PotKind {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Attack),
            1 => Ok(Self::Decay),
            2 => Ok(Self::Sustain),
            3 => Ok(Self::Release),
            4 => Ok(Self::Extra1),
            5 => Ok(Self::Extra2),
            6 => Ok(Self::CvDepthA),
            7 => Ok(Self::CvDepthB),
            _ => Err(()),
        }
    }
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
