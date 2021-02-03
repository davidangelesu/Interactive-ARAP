#ifndef COT_MAT
#define COT_MAT
#include <Eigen/Core>
#include <Eigen/Sparse>


/// <summary>
/// Constructs the cotagent Stiffness Matrix from given mesh V, F
/// </summary>
/// <typeparam name="DerivedV"> derived Value from Matrix V</typeparam>
/// <typeparam name="DerivedF"> derived Value from Matrix V</typeparam>
/// <typeparam name="Scalar"> type of scalar values used to store the cotagent values (eg. double)</typeparam>
/// <param name="V">Eigen Matrix where each row represents the Vertex position.</param>
/// <param name="F">Eigen Matrix where each row represents the 3 Verices that compose that specific face f</param>
/// <param name="L">Output Matrix (cotagent Stiffness Matrix)/ Discrete Laplacian Operator</param>
template<typename DerivedV, typename DerivedF, typename Scalar, typename WeightFlag>
void cotagent_matrix(
	const Eigen::MatrixBase<DerivedV> & V,
	const Eigen::MatrixBase<DerivedF> & F,
	Eigen::SparseMatrix<Scalar> & L,
  WeightFlag use_uniform_weights,
  Scalar uniform_weight);

#endif
