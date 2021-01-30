#include "../include/arap_precompute.h"
#include <iostream>
#include "../include/cotagent_matrix.h"
#include "../include/cotagent.h"
#include <igl/adjacency_list.h>


void arap_precompute(const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& F,
	std::vector<Eigen::Matrix<double, 3, -1>> & K,
  bool use_uniform_weights, double uniform_weight) {


	unsigned int numVertex = V.rows();
	unsigned int numFaces = F.rows();

	

	//----------------------------Compute K-------------------------------
	//calculate Constant part of covariance Matrix S (Or also in other references denoted as  C).
	//Different way to implement Logically as the code in example repo
	K = std::vector<Eigen::Matrix<double, 3, -1>>(numVertex);
	
	//calculate cotagent matrix (with weights) w_i,j
	Eigen::SparseMatrix<double> L;
	cotagent_matrix(V, F, L, use_uniform_weights, uniform_weight);

    std::vector<std::vector<double>> neighbors(numVertex);
    igl::adjacency_list(F, neighbors);

    for (unsigned int i = 0; i < numVertex; ++i) {
        for (unsigned int j : neighbors[i]) {
			Eigen::Vector3d e_ij = V.row(i) - V.row(j);
			//add an additional column 
			K[i].conservativeResize(K[i].rows(), K[i].cols() + 1);
			K[i].col(K[i].cols() - 1) = L.coeff(i, j) * e_ij;
		}
	}

};
