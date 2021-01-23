
#include <Eigen/Core>
#include <Eigen/Sparse>



// Inputs:
//   K  pre-constructed bi-linear term of energy combining rotations and
//     positions
//   U  #V by dim list of current positions (see output)
// Outputs:
//   U  #V by dim list of new positions (see input)
void arap_single_iteration(
  const std::vector<Eigen::Matrix<double,3,-1>>& K,
  const Eigen::MatrixXd& constraints,
  const Eigen::VectorXi& constraintIndices,
  const Eigen::MatrixXd& V,
  const Eigen::MatrixXi& F,
  Eigen::MatrixXd& U,
  Eigen::SparseMatrix<double>& m_systemMatrix,
  bool use_uniform_weights);
