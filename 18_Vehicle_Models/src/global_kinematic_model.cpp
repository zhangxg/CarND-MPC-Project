// In this quiz you'll implement the global kinematic model.
#include <math.h>
#include <iostream>
// works with below relative path works, also withe the symbolic link
// #include "../../src/Eigen-3.3/Eigen/Dense"
#include "Eigen/Eigen/Dense"
/**
using #include "../../src/Eigen-3.3/Eigen/Dense" generates:
1.72723e-77 1.72723e-77 4.94066e-323 0;
uisng #include "Eigen/Eigen/Dense" generates: 
0 0 0 0 
the result is close, but why the difference in precision. 

the answer is: both methods generates the same result as below results show
1) 0.212132 0.212132 0.798488 1.3
2) 0.212132 0.212132 0.798488 1.3

the difference is the next_step in previous try is not initialized, 
arbitrary value is given.
=====================
to run the code, using the below command:
g++ global_kinematic_model -o main; ./main
the g++ does not work. then what's the difference between g++ and gcc?
*/

using namespace std;

//
// Helper functions
//
double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2;

// TODO: Implement the global kinematic model.
// Return the next state.
//
// NOTE: state is [x, y, psi, v]
// NOTE: actuators is [delta, a]
Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, 
                                double dt) {
  Eigen::VectorXd next_state(state.size());
  //TODO complete the next_state calculation ...
  double x, y, psi, v;
  x = state[0];
  y = state[1];
  psi = state[2];
  v = state[3];

  double delta, a;
  delta = actuators[0];
  a = actuators[1];
  
  x += v * dt * cos(psi);
  y += v * dt * sin(psi);
  psi += v * delta * dt / Lf;
  v += a * dt;

  next_state[0] = x;
  next_state[1] = y;
  next_state[2] = psi;
  next_state[3] = v;

  return next_state;
}

int main() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);

  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;

  Eigen::VectorXd next_state = globalKinematic(state, actuators, 0.3);

  std::cout << next_state << std::endl;
}