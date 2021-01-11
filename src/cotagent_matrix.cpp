#include "cotagent.h"
#include "cotagent_matrix.h"
#include <iostream>


template<typename DerivedV, typename DerivedF, typename Scalar>
void cotagent_matrix(
	const Eigen::MatrixBase<DerivedV> & V,
	const Eigen::MatrixBase<DerivedF> & F,
	Eigen::SparseMatrix<Scalar> & L)
{
	unsigned int numVertices = V.rows();
	unsigned int numFaces = F.rows();
	L = Eigen::SparseMatrix<Scalar>(numVertices, numVertices);


	//To fill up a Sparse matrix, it is recomented to fill it up from a list of Triplets
	std::vector < Eigen::Triplet < Scalar >> vectorOfTriplets;
	//w_i,j
	for (int f = 0; f < numFaces; f++) {
		for (int oppositeVertex= 0; oppositeVertex < 3; oppositeVertex++) {
			int i = (oppositeVertex + 1) % 3;
			int j = (oppositeVertex + 2) % 3;
			
			//vertices 
			int v_i = F(f,i);
			int v_j = F(f, j);
			int v_opposite = F(f, oppositeVertex);


			//position of points
			
			Eigen::Vector3d p_i= V.row(v_i);
			Eigen::Vector3d p_j = V.row(v_j);
			Eigen::Vector3d p_oppositeVertex = V.row(v_opposite);


			//cotagent at oppositeVertex 
			double cot = cotagent(p_i,p_j,p_oppositeVertex);


			//per edge weight contribution
			vectorOfTriplets.push_back(Eigen::Triplet<Scalar>(v_i, v_j, .5* cot));
			vectorOfTriplets.push_back(Eigen::Triplet<Scalar>(v_j, v_i, .5 * cot));


			vectorOfTriplets.push_back(Eigen::Triplet<Scalar>(v_i, v_i, .5 * cot));
			vectorOfTriplets.push_back(Eigen::Triplet<Scalar>(v_j, v_j, .5 * cot));
		}
	}
	////per cell weights w_i
	//for (int i = 0; i < numVertices; i++) {
	//	vectorOfTriplets.push_back(Eigen::Triplet<Scalar>(i,i, 1));
	//}
	L.setFromTriplets(vectorOfTriplets.begin(), vectorOfTriplets.end());
};

template void cotagent_matrix<Eigen::Matrix<double, -1, -1, 1, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, double>(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 1, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::SparseMatrix<double, 0, int>&);
template void cotagent_matrix<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, double>(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::SparseMatrix<double, 0, int>&);


