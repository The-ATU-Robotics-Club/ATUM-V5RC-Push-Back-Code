use nalgebra::{SVector, SMatrix};

const TOTAL_STATES: usize = 6;
const TOTAL_INPUTS: usize = 2;
const TOTAL_OUTPUTS: usize = 8;

type State = SVector<f64, TOTAL_STATES>;
type StateCovariance = SMatrix<f64, TOTAL_STATES, TOTAL_STATES>;
type Input = SVector<f64, TOTAL_INPUTS>;
// type KalmanGain = 
type Output = SVector<f64, TOTAL_OUTPUTS>;
type OutputCovariance = SMatrix<f64, TOTAL_OUTPUTS, TOTAL_OUTPUTS>;

pub struct Ukf {
}

impl Ukf {
}
