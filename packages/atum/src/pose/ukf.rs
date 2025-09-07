use alloc::vec::{self, Vec};
use nalgebra::{Cholesky, SMatrix, SVector};
use vexide::float::Float;


const TOTAL_STATES: usize = 6;
const TOTAL_INPUTS: usize = 2;
const TOTAL_OUTPUTS: usize = 8;
const SIGMA_POINTS: usize = TOTAL_STATES*2 +1;
type State = SVector<f64, TOTAL_STATES>;
type StateCovariance = SMatrix<f64, TOTAL_STATES, TOTAL_STATES>;
type Input = SVector<f64, TOTAL_INPUTS>;
// type KalmanGain = 
type Output = SVector<f64, TOTAL_OUTPUTS>;
type OutputCovariance = SMatrix<f64, TOTAL_OUTPUTS, TOTAL_OUTPUTS>;


pub struct Ukf { //Variables Go here
    alpha: f64,
    beta: f64,
    lambda: f64,
    n:f64,
    n_plus:f64,
    x:State,
    p:StateCovariance,

    // Physics Model units
    d1:f64,
    d2:f64,
    d3:f64,
    d4:f64,
}

impl Ukf { //Functions go here
    fn new(x: State) -> Self {
        let kappa= 0.0;
        let beta = 2.0;
        let alpha = 0.001;
        let n = 6.0;
        let lambda = alpha*alpha*(n-kappa)-n;

        let kt =0.142275; // Torque Constant in Newton Meters per Amp
        let r = 2.61; // Resistance of motor in Ohms
        let kv = 5.23; // how many radians per second per volt
        let nof_m = 8.0; // Number of meters
        let rb = 0.15113; // radius of robot In meters
        let rw = 0.041275; //radius of wheel In meters
        let j = 1.001; // Moment of Inertia
        let g = 0.75; // Gear ratio
        let m = 8.0; // Mass of robot in kg
        let c1 = -(g * g * kt * n) / (kv * r * rw * rw);
        let c2 = (g * kt * n) / (r * rw);

        let p: StateCovariance = SMatrix::from_diagonal(&SVector::from_row_slice(&[
        0.330,      // variance or std dev for x
        0.330,      // variance or std dev for y
        0.01745,  // variance or std dev for θ (≈ 1° in radians)
        0.0,      // vx
        0.0,      // vy
        0.0,     // ω
        ]));

        Self{
            alpha,
            beta,
            lambda,
            n,
            n_plus:lambda+n,
            x,
            p,
            d1:2.0 * c1 / m,
            d2:c2 / m,
            d3:2.0 * rb * rb * c1 / j,
            d4:rb * c2 / j,

        }

    }

    fn f(&self, x: &State, u: &SVector<f64, 2>) -> State {
        let dt = 0.01;
        let mut new_x = *x;
        let dh = 0.5 * x[5] * dt;
        new_x[0] += dt * (x[3] * (x[2] + dh).sin() + x[4] * (x[2] + dh).cos());
        new_x[1] += dt * (x[3] * (x[2] + dh).cos() - x[4] * (x[2] + dh).sin());
        new_x[2] += dt * x[5];
        new_x[3] += dt * (self.d1 * x[3] + self.d2 * (u[0] + u[1]));
        new_x[4] += dt * self.d1 * x[4];
        new_x[5] += dt * (self.d3 * x[5] + self.d4 * (u[0] - u[1]));

        new_x
    }
    fn sigma_points(&self, x: &State) -> Vec<State> {
        let n = TOTAL_STATES;
        let lambda = self.lambda;

        // Scale covariance
        let scaled_p = (n as f64 + lambda) * self.p;

        // Cholesky factorization (lower triangular L such that scaled_p = L * Lᵀ)
        let chol = Cholesky::new(scaled_p).expect("P must be positive definite");
        let l = chol.l();

        let mut sigmas = Vec::with_capacity(2 * n + 1);

        // First sigma point = mean
        sigmas.push(*x);

        // For each column of L, create ± sigma points
        for i in 0..n {
            let col = l.column(i).into_owned(); // Convert to full vector
            sigmas.push(x + col);
            sigmas.push(x - col);
        }

        sigmas
    }
    
    // Generate UKF weights
    fn weights(&self) -> (SVector<f64,SIGMA_POINTS>, SVector<f64, SIGMA_POINTS>) {
        let n_plus_lambda = TOTAL_STATES as f64 + self.lambda;
        let mut wm = SVector::<f64, SIGMA_POINTS>::zeros();
        let mut wc = SVector::<f64, SIGMA_POINTS>::zeros();

        wm[0] = self.lambda / n_plus_lambda;
        wc[0] = wm[0] + (1.0 - self.alpha * self.alpha + self.beta);

        for i in 1..SIGMA_POINTS {
            wm[i] = 1.0 / (2.0 * n_plus_lambda);
            wc[i] = wm[i];
        }

        (wm, wc)
    }
    pub fn predict(&mut self, u: &Input) {
        // 1. generate sigma points
        let sigmas = self.sigma_points(&self.x);

        // 2. get weights
        let (wm, wc) = self.weights();

        // 3. propagate each sigma point through motion model
        let mut propagated = Vec::with_capacity(SIGMA_POINTS);
        for i in 0..SIGMA_POINTS {
            propagated.push(self.f(&sigmas[i], u));
        }

        // 4. compute predicted mean
        let mut x_pred = State::zeros();
        for i in 0..SIGMA_POINTS {
            x_pred += wm[i] * propagated[i];
        }

        // 5. compute predicted covariance (diagonal-only)
        let mut p_pred = StateCovariance::zeros();
        for i in 0..SIGMA_POINTS {
            let diff = propagated[i] - x_pred;
            p_pred += wc[i] * (diff.clone() * diff.transpose());
        }

        // 6. update state and covariance
        self.x = x_pred;
        self.p = p_pred;
    }
    




}
