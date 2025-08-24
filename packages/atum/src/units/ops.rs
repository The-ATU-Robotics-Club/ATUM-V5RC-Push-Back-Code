macro_rules! impl_ops {
    ($struct:ident, $inner:ty) => {
        use ::core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};
        impl Add for $struct {
            type Output = Self;
            #[inline]
            fn add(self, rhs: Self) -> Self::Output {
                Self(self.0 + rhs.0)
            }
        }
        impl Sub for $struct {
            type Output = Self;
            #[inline]
            fn sub(self, rhs: Self) -> Self::Output {
                Self(self.0 - rhs.0)
            }
        }
        impl Mul for $struct {
            type Output = Self;
            #[inline]
            fn mul(self, rhs: Self) -> Self::Output {
                Self(self.0 * rhs.0)
            }
        }
        impl Div for $struct {
            type Output = Self;
            #[inline]
            fn div(self, rhs: Self) -> Self::Output {
                Self(self.0 / rhs.0)
            }
        }

        // Struct op with inner type
        impl Add<$inner> for $struct {
            type Output = Self;
            #[inline]
            fn add(self, rhs: $inner) -> Self::Output {
                Self(self.0 + rhs)
            }
        }
        impl Sub<$inner> for $struct {
            type Output = Self;
            #[inline]
            fn sub(self, rhs: $inner) -> Self::Output {
                Self(self.0 - rhs)
            }
        }
        impl Mul<$inner> for $struct {
            type Output = Self;
            #[inline]
            fn mul(self, rhs: $inner) -> Self::Output {
                Self(self.0 * rhs)
            }
        }
        impl Div<$inner> for $struct {
            type Output = Self;
            #[inline]
            fn div(self, rhs: $inner) -> Self::Output {
                Self(self.0 / rhs)
            }
        }
        impl AddAssign for $struct {
            #[inline]
            fn add_assign(&mut self, rhs: Self) {
                self.0 += rhs.0;
            }
        }
        impl SubAssign for $struct {
            #[inline]
            fn sub_assign(&mut self, rhs: Self) {
                self.0 -= rhs.0;
            }
        }
        impl MulAssign for $struct {
            #[inline]
            fn mul_assign(&mut self, rhs: Self) {
                self.0 *= rhs.0;
            }
        }
        impl DivAssign for $struct {
            #[inline]
            fn div_assign(&mut self, rhs: Self) {
                self.0 /= rhs.0;
            }
        }
        impl Neg for $struct {
            type Output = Self;
            #[inline]
            fn neg(self) -> Self::Output {
                Self(-self.0)
            }
        }
    };
}

pub(crate) use impl_ops;
