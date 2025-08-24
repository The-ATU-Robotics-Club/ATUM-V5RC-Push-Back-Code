macro_rules! impl_float {
    ($struct:ident, $inner:ty) => {
        use ::vexide::float::Float;
        impl Float for $struct {
            fn floor(self) -> Self {
                Self(self.0.floor())
            }
            fn ceil(self) -> Self {
                Self(self.0.ceil())
            }
            fn round(self) -> Self {
                Self(self.0.round())
            }
            fn round_ties_even(self) -> Self {
                Self(self.0.round_ties_even())
            }
            fn trunc(self) -> Self {
                Self(self.0.trunc())
            }
            fn fract(self) -> Self {
                Self(self.0.fract())
            }
            fn abs(self) -> Self {
                Self(self.0.abs())
            }
            fn signum(self) -> Self {
                Self(self.0.signum())
            }
            fn copysign(self, sign: Self) -> Self {
                Self(self.0.copysign(sign.0))
            }
            fn mul_add(self, a: Self, b: Self) -> Self {
                Self(self.0.mul_add(a.0, b.0))
            }
            fn div_euclid(self, rhs: Self) -> Self {
                Self(self.0.div_euclid(rhs.0))
            }
            fn rem_euclid(self, rhs: Self) -> Self {
                Self(self.0.rem_euclid(rhs.0))
            }
            fn powi(self, n: i32) -> Self {
                Self(self.0.powi(n))
            }
            fn powf(self, n: Self) -> Self {
                Self(self.0.powf(n.0))
            }
            fn sqrt(self) -> Self {
                Self(self.0.sqrt())
            }
            fn exp(self) -> Self {
                Self(self.0.exp())
            }
            fn exp2(self) -> Self {
                Self(self.0.exp2())
            }
            fn ln(self) -> Self {
                Self(self.0.ln())
            }
            fn log(self, base: Self) -> Self {
                Self(self.0.log(base.0))
            }
            fn log2(self) -> Self {
                Self(self.0.log2())
            }
            fn log10(self) -> Self {
                Self(self.0.log10())
            }
            #[allow(deprecated)]
            fn abs_sub(self, other: Self) -> Self {
                Self(self.0.abs_sub(other.0))
            }
            fn cbrt(self) -> Self {
                Self(self.0.cbrt())
            }
            fn hypot(self, other: Self) -> Self {
                Self(self.0.hypot(other.0))
            }
            fn sin(self) -> Self {
                Self(self.0.sin())
            }
            fn cos(self) -> Self {
                Self(self.0.cos())
            }
            fn tan(self) -> Self {
                Self(self.0.tan())
            }
            fn asin(self) -> Self {
                Self(self.0.asin())
            }
            fn acos(self) -> Self {
                Self(self.0.acos())
            }
            fn atan(self) -> Self {
                Self(self.0.atan())
            }
            fn atan2(self, other: Self) -> Self {
                Self(self.0.atan2(other.0))
            }
            fn sin_cos(self) -> (Self, Self) {
                let (s, c) = self.0.sin_cos();
                (Self(s), Self(c))
            }
            fn exp_m1(self) -> Self {
                Self(self.0.exp_m1())
            }
            fn ln_1p(self) -> Self {
                Self(self.0.ln_1p())
            }
            fn sinh(self) -> Self {
                Self(self.0.sinh())
            }
            fn cosh(self) -> Self {
                Self(self.0.cosh())
            }
            fn tanh(self) -> Self {
                Self(self.0.tanh())
            }
            fn asinh(self) -> Self {
                Self(self.0.asinh())
            }
            fn acosh(self) -> Self {
                Self(self.0.acosh())
            }
            fn atanh(self) -> Self {
                Self(self.0.atanh())
            }
        }
    };
}

pub(crate) use impl_float;
