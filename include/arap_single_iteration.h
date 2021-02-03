
#include <Eigen/Core>
#include <Eigen/Sparse>



/**
 * Compute a single ARAP iteration
 * @param K pre-constructed bi-linear term of energy combining rotations and positions
 * @param constraints Control points, i.e. constraint vertices
 * @param constraintIndices Control points indices, i.e. indices of constraint vertices
 * @param V Original vertices
 * @param F Faces of the mesh
 * @param U Deformed vertices
 * @param m_systemMatrix System matrix for global optimization
 * @param use_uniform_weights Weight flag
 * @param uniform_weight Value to use as uniform weights
 */
void arap_single_iteration(
  const std::vector<Eigen::Matrix<double,3,-1>>& K,
  const Eigen::MatrixXd& constraints,
  const Eigen::VectorXi& constraintIndices,
  const Eigen::MatrixXd& V,
  const Eigen::MatrixXi& F,
  Eigen::MatrixXd& U,
  Eigen::SparseMatrix<double>& m_systemMatrix,
  bool use_uniform_weights,
  double uniform_weight);
