#include "../include/arap_precompute.h"
#include <igl/min_quad_with_fixed.h>
#include <igl/arap_linear_block.h>
#include <igl/cotmatrix.h>
#include <iostream>

void arap_precompute(
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  const Eigen::VectorXi & b,
  igl::min_quad_with_fixed_data<double> & data,
  Eigen::SparseMatrix<double> & K)
{
// REPLACE WITH YOUR CODE
	//number of vertex
	unsigned int numVertex = V.rows();
	unsigned int numFaces = F.rows();
	//Cotagent Stiffnes Matrix L
	Eigen::SparseMatrix<double> L;
	igl::cotmatrix(V,F, L);
	//precompute data
	igl::min_quad_with_fixed_precompute(L, b,Eigen::SparseMatrix<double>(), false, data);



	//***********  Compute K***************
	K = Eigen::SparseMatrix<double>(numVertex, numVertex * 3);

	//To fill up a Sparse matrix, it is recomented to fill it up from a list of Triplets
	std::vector<Eigen::Triplet<double>> vectorOfTriplets;
	//	Row represents a triangle, where each entry in the row is the 1/2 * cotagents corresponding angles   
	//  for triangles, columns correspond to edges [1,2],[2,0],[0,1]
	Eigen::MatrixXd C;
	igl::cotmatrix_entries(V, F, C);


	//loop through faces
	for (int f = 0; f < numFaces; f++) {
		//loop though half edges (i,j) 
		for (int oppositeVertex = 0; oppositeVertex < 3; oppositeVertex++) {

			int i = (oppositeVertex + 1) % 3;
			int j = (oppositeVertex + 2) % 3;
			//vertex index
			int v_i = F(f, i);
			int v_j = F(f, j);

			// 1/2 * cotagent angles
			double cot= C(f, oppositeVertex);//edge[1,2]
	
			//weighted edge difference vector e_ij=c_ij * (v_i-v_j)
			Eigen::Vector3d e;
			e = cot * (V.row(v_i) - V.row(v_j));
		
			//looping though betha
			for (int betha = 0; betha < 3; betha++) {
				//looping through each vertex in face
				for (int k = 0; k < 3; k++) {
					int v_k = F(f, k);
					vectorOfTriplets.push_back(Eigen::Triplet<double>(v_i, 3 * v_k + betha, e[betha]));
					vectorOfTriplets.push_back(Eigen::Triplet<double>(v_j, 3 * v_k + betha, -e[betha]));
				}
			}

		}

		
	}

	//Fill Sparse Matrix
	K.setFromTriplets(vectorOfTriplets.begin(), vectorOfTriplets.end());
	K /= 3;
	std::cout << "Number of non zeros" << K.nonZeros() << "\n";
}
