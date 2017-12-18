#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

enum vars_index {
    x_start_idx = 0,
    y_start_idx,
    psi_start_idx,
    v_start_idx,
    cte_start_idx,
    epsi_start_idx,
    delta_start_idx,
    a_start_idx,
    n_start_idx
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

};

#endif /* MPC_H */
