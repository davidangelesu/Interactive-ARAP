
#include <Eigen/Core>
#include <Eigen/Sparse>



// Inputs:
//   K  pre-constructed bi-linear term of energy combining rotations and
//     positions
//   U  #V by dim list of current positions (see output)
// Outputs:
//   U  #V by dim list of new positions (see input)
void arap_single_iteration(
  const std::vector<Eigen::Matrix<double,3,-1>> & K,
  const Eigen::MatrixXd & bc,
  const Eigen::MatrixXi& F,
  Eigen::MatrixXd & U);
