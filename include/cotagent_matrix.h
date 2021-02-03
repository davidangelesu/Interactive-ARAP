#ifndef COT_MAT
#define COT_MAT
#include <Eigen/Core>
#include <Eigen/Sparse>


/**
 * Constructs the cotagent Stiffness Matrix from given mesh V, F
 * @tparam DerivedV Derived Value from Matrix V
 * @tparam DerivedF Derived Value from Matrix V
 * @tparam Scalar Type of scalar values used to store the cotangent values (e.g. double)
 * @tparam WeightFlag Flag variable for weights
 * @param V Eigen Matrix where each row represents the Vertex position.
 * @param F Eigen Matrix where each row represents the 3 Vertices that compose that specific face f
 * @param L Output Matrix (cotagent Stiffness Matrix)/ Discrete Laplacian Operator<
 * @param use_uniform_weights Weight flag
 * @param uniform_weight Value to use as uniform weights
 */
template<typename DerivedV, typename DerivedF, typename Scalar, typename WeightFlag>
void cotagent_matrix(
	const Eigen::MatrixBase<DerivedV> & V,
	const Eigen::MatrixBase<DerivedF> & F,
	Eigen::SparseMatrix<Scalar> & L,
  WeightFlag use_uniform_weights,
  Scalar uniform_weight);

#endif
