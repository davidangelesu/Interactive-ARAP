#include "../include/arap_precompute.h"
#include <iostream>
#include "../include/cotagent_matrix.h"
#include "../include/cotagent.h"


void arap_precompute(const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& F,
	std::vector<Eigen::Matrix<double, 3, -1>> & K ) {


	unsigned int numVertex = V.rows();
	unsigned int numFaces = F.rows();

	

	//----------------------------Compute K-------------------------------
	//calculate Constant part of covariance Matrix S (Or also in other references denoted as  C).
	//Different way to implement Logically as the code in example repo
	K = std::vector<Eigen::Matrix<double, 3, -1>>(numVertex);
	
	//calculate cotagent matrix (with weights) w_i,j
	Eigen::SparseMatrix<double> L;
	cotagent_matrix(V, F, L);

	//loop through faces
	for (int f = 0; f < numFaces; f++) {

		//loop through face vertex
		for (int oppositeVertex = 0; oppositeVertex < 3; oppositeVertex++) {
			int i = (oppositeVertex + 1) % 3;
			int j = (oppositeVertex + 2) % 3;

			int v_i = F(f, i);
			int v_j = F(f, j);
			Eigen::Vector3d e_ij = V.row(F(f, i)) - V.row(F(f, j));
			//add an additional column 
			K[v_i].conservativeResize(K[v_i].rows(), K[v_i].cols() + 1);
			K[v_i].col(K[v_i].cols() - 1) = L.coeff(v_i, v_j) * e_ij;
		}
	}

};
