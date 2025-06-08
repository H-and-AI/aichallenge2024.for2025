# include "MPC.h"
# include <cppad/cppad.hpp>
# include <cppad/ipopt/solve.hpp>
# include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
//Predict unitl 10 time step ahead
size_t N = 10;
// Set dt to 0.1 since the latency is 0.1s(we have to predict at least 0.1s ahead)
double dt = 0.1;

const double Lf = 2.67;
//基準のCTE，角度差を0で初期化
double ref_cte = 0;
double ref_epsi = 0;
//目標スピードを100.0MPHに設定
double ref_v = 100.0;

/*
0~N-1まではxの値
N~2N-1まではyの値
2N~3N-1まではpsiの値
3N~4N-1まではvの値
4N~5N-1まではcte値
5N~6N-1まではepsiの値
6N~7N-1まではdeltaの値
7N~8N-1まではaの値を保存
*/
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
  // Fitted polynomial coefficients（目標軌道のフィッティング）
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    //コスト関数を設定
    fg[0] = 0;

    for (int i = 0; i < N - 1; i++) {
      fg[0] += 2000 * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += 1800 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    for (int i = 0; i < N - 1; i++) {
      fg[0] += 3 * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += 5 * CppAD::pow(vars[a_start + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      fg[0] += 100 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += 10 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    //Setup constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (int i = 0; i < N-1; i++){
      // The state at time t+1.
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      AD<double> psides0 =  CppAD::atan(coeffs[1] + 2.0 * coeffs[2] * x0 + 3.0 * coeffs[3] * x0 * x0);


      //制約条件
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);

      fg[2 + psi_start + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);

    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

//モデル予測制御を解く
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



  size_t n_vars = 6 * N + 2 * (N-1);
  // Set the number of constraints
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set lower and upper limits for variables.
  for(int i = 0; i < delta_start; i++){
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  //ハンドル角の制限
  for(int i = delta_start; i < a_start; i++){
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  //アクセル量の制限
  for(int i = a_start; i < n_vars; i++){
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

  
  std::string options;
  options += "Integer print_level  0\n";
  
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
 
  options += "Numeric max_cpu_time          0.5\n";

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
  std::cout << "Cost " << cost << std::endl;

   
  vector<double> result;
  //ハンドル量とアクセル量の最適解
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  //10ステップ先までの予測軌道
  for(int i = 0; i < N-1; i++){
    result.push_back(solution.x[x_start+i+1]);
    result.push_back(solution.x[y_start+i+1]);
  }
  return result;
}
