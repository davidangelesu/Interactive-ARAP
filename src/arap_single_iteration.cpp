#include "../include/arap_single_iteration.h"
#include <igl/polar_svd3x3.h>
#include <igl/min_quad_with_fixed.h>

void arap_single_iteration(
  const igl::min_quad_with_fixed_data<double> & data,
  const Eigen::SparseMatrix<double> & K,
  const Eigen::MatrixXd & bc,
  Eigen::MatrixXd & U)
{
  // REPLACE WITH YOUR CODE
	//number of vertex
	unsigned int numVertex = U.rows();

	//*********local optimization
	Eigen::MatrixXd C = K.transpose() * U; //3x*3
	Eigen::MatrixXd R(3 * numVertex, 3);
	//looping though vertices
	for (int k = 0; k < numVertex; k++) {
		Eigen::Matrix3d  C_k = C.block(k * 3, 0, 3, 3);
		Eigen::Matrix3d R_k;
		igl::polar_svd3x3(C_k, R_k);
		R.block(k * 3, 0, 3, 3) = R_k;
	}
	
	//**********Global Optimization
	Eigen::MatrixXd B = K * R;
	igl::min_quad_with_fixed_solve(data, B,bc,Eigen::VectorXd(), U);
	std::cout << "U has been updated\n";
}
