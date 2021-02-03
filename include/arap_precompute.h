#include <Eigen/Core>
#include <Eigen/Sparse>

/**
 * Precomputes part of the covariance matrix
 * @param V Original vertices
 * @param F Faces of the mesh
 * @param K Variable to precompute: K = P * D (see eq. 5 in https://igl.ethz.ch/projects/ARAP/arap_web.pdf)
 * @param use_uniform_weights Weight flag
 * @param uniform_weight Value to use as uniform weights
 */
void arap_precompute(
	const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& F,
	std::vector<Eigen::Matrix<double,3,-1>>& K,
  bool use_uniform_weights,
  double uniform_weight);

