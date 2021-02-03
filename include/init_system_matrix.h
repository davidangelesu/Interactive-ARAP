#include <Eigen/Core>
#include <Eigen/Sparse>

/**
 * Init the system matrix used for global optimization, i.e. new vertex positions
 * @param V Vertices of the mesh
 * @param F Faces of the mesh
 * @param systemMatrix Variable to initialize
 * @param use_uniform_weights Whether to use uniform or cotangent weights
 * @param uniform_weight Value to use for uniform weights
 */
void init_system_matrix(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::SparseMatrix<double> &systemMatrix, bool use_uniform_weights, double uniform_weight);
