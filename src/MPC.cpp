#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration

// too many time steps makes it do crazy things
size_t N = 10;
// Larger than our actuator delay
double dt = .125;	// greatly drives Computing Time

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 75*.44704;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

		fg[0] = 0;

		// Minimize the error by calculating it based on our state
		for (int t = 0; t < N; t++){
      //fg[0] += 2000*CppAD::pow(vars[cte_start + t] - ref_cte, 2); // improtant to give high weight to
      fg[0] += 1500*CppAD::pow(vars[cte_start + t] - ref_cte, 2); // improtant to give high weight to
      //fg[0] += 15*CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += 150*CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      //fg[0] += 150*CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
			fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
		}

		// Add smoothness to our turns, by taxing any change in acceleration and yaw
		for (int t = 0; t < N - 1; t++) {
			fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);
			fg[0] += 5*CppAD::pow(vars[a_start + t], 2);
		}

		// Add further smoothness by taxing the rate of change of our acceleration and yaw
		for (int t=0; t < N - 2; t++){
      //fg[0] += 2000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);  // important to make sure we don't jerk too hard
      fg[0] += 1500*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);  // important to make sure we don't jerk too hard
			fg[0] += 50*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t ], 2);
		}

		// Cost is located at fg[start_pos] for each variable, so we must move values for each variable back 1.
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];


		for (int t = 0; t < N-1; t++) { // set to t = 1 if 0 sate is @ t - 1
			// Future state t+1, or could do t if 0 state is t-1
      AD<double> x1 = vars[x_start + t + 1];
			AD<double> y1 = vars[y_start + t + 1];
			AD<double> psi1 = vars[psi_start + t + 1];
			AD<double> v1 = vars[v_start + t + 1];
			AD<double> cte1 = vars[cte_start + t + 1];
			AD<double> epsi1 = vars[epsi_start + t + 1];

			// State at time t
			AD<double> x0 = vars[x_start + t ];
			AD<double> y0 = vars[y_start + t ]  ;
			AD<double> psi0 = vars[psi_start + t ];
			AD<double> v0 = vars[v_start + t ];
			AD<double> cte0 = vars[cte_start + t ];
			AD<double> epsi0 = vars[epsi_start + t ];

			// Actuation at time t
			AD<double> delta0 = vars[delta_start + t ];
			AD<double> a0 = vars[a_start + t];

			AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0; // if using 3rd power polynom
			//AD<double> psides0 = CppAD::atan(coeffs[1]);
			AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0 *x0 + 2*coeffs[2]*x0 + coeffs[1]);

			// State predicted in future time step
			fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
			fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
			fg[2 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[2 + epsi_start + t] = epsi1 - ((psi0 - psides0) + (v0 * delta0 / Lf * dt));
		}
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];

	// set size of variable array
  size_t n_vars = 6*N + 2*(N-1);
	// set size of constraints
	size_t n_constraints = 6*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

	// Set all non-actuators upper and lowerlimits
	// to the max negative and positive values.
	for (int i = 0; i < delta_start; i++) {
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}


	// The upper and lower limits of delta are set to -25 and 25
	// degrees (values in radians).
	// NOTE: Feel free to change this to something else.
	for (int i = delta_start; i < a_start; i++) {
		vars_lowerbound[i] = -0.436332*Lf;	// Don't need the LF
		vars_upperbound[i] = 0.436332*Lf;		// Don't need the LF
	}

	// Acceleration/decceleration upper and lower limits.
	// NOTE: Feel free to change this to something else.
	for (int i = a_start; i < n_vars; i++) {
		vars_lowerbound[i] = -1.0;
		vars_upperbound[i] = 1.0;
	}


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	for (int i = 0; i < n_constraints; i++) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}
	// set each constraint case = to initial state, as we don't have a time machine.
	constraints_lowerbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi;

	constraints_upperbound[x_start] = x;
	constraints_upperbound[y_start] = y;
	constraints_upperbound[psi_start] = psi;
	constraints_upperbound[v_start] = v;
	constraints_upperbound[cte_start] = cte;
	constraints_upperbound[epsi_start] = epsi;
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";		// consider decreasing based on N and dt
																											// Simulator sends values every .02S

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;

	vector<double> result;

	result.push_back(solution.x[delta_start]);		// Actuators
	result.push_back(solution.x[a_start]);				// Acutators

	for (int i = 0; i < N-1; i++){
		result.push_back(solution.x[x_start + i + 1]);	// Path Painting x cords
		result.push_back(solution.x[y_start + i + 1]);	// Path Painting y cords
	}

	return result;

 }
