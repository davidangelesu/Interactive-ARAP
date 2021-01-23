#include <Eigen/Core>
#include <Eigen/Sparse>

/// <summary>
/// Precomputes part of the covariance matrix
/// </summary>
/// <param name="V"></param>
/// <param name="F"></param>
/// <param name="L"></param>
/// <param name="K"></param>
void arap_precompute(
	const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& F,
	std::vector<Eigen::Matrix<double,3,-1>>& K,
  bool use_uniform_weights);

