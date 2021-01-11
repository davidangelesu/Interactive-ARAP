#include "arap_single_iteration.h"
#include <igl/polar_svd3x3.h>
#include <igl/min_quad_with_fixed.h>


void arap_single_iteration(
  const std::vector<Eigen::Matrix<double, 3, -1>>& K,
  const Eigen::MatrixXd & bc,
  const Eigen::MatrixXi& F,
  Eigen::MatrixXd & U)
{
	//number of vertex
	unsigned int numVertex = U.rows();
	unsigned int numFaces = F.rows();

	//*********local optimization
	std::vector<Eigen::Matrix3d> R (numVertex);
	std::vector<Eigen::Matrix<double, 3, -1>> P_prime(numVertex);
	

	//create P_prime matrices
	for (int f = 0; f < numFaces; f++) {

		//loop through face vertex
		for (int oppositeVertex = 0; oppositeVertex < 3; oppositeVertex++) {
			int i = (oppositeVertex + 1) % 3;
			int j = (oppositeVertex + 2) % 3;

			int v_i = F(f, i);
			int v_j = F(f, j);
			//edges
			Eigen::Vector3d e_prime_ij = U.row(F(f, i)) - U.row(F(f, j));
			P_prime[v_i].conservativeResize(P_prime[v_i].rows(), P_prime[v_i].cols() + 1);
			P_prime[v_i].col(P_prime[v_i].cols() - 1) =e_prime_ij;
		}
	}

	//solve for each rotation matrix
	for (int i = 0; i < numVertex; i++) {
		//covariance matrix
		Eigen::Matrix3d S_i = K[i] * P_prime[i].transpose();

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(S_i,Eigen::ComputeFullU|Eigen::ComputeFullV);
		R[i] = svd.matrixV() * svd.matrixU().transpose();
	}
	std::cout << "ROTATIONS COMPUTED\n";
	//**********Global Optimization 
	//TODO  Implement !!  use as Reference SolveForVertexPositions()  
}
