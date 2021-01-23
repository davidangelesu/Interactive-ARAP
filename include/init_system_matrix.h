#include <Eigen/Core>
#include <Eigen/Sparse>

void init_system_matrix(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::SparseMatrix<double> &systemMatrix, bool use_uniform_weights);
