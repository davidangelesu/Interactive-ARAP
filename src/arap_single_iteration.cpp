#include "arap_single_iteration.h"
#include <igl/min_quad_with_fixed.h>
#include <libigl/include/igl/adjacency_list.h>
#include <cotagent_matrix.h>


void arap_single_iteration(
  const std::vector<Eigen::Matrix<double, 3, -1>>& K,
  const Eigen::MatrixXd& constraints,
  const Eigen::VectorXi& constraintsIndices,
  const Eigen::MatrixXd& V,
  const Eigen::MatrixXi& F,
  Eigen::MatrixXd& U,
  Eigen::SparseMatrix<double>& m_systemMatrix)
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
    Eigen::SparseMatrix<double> systemMatrix = m_systemMatrix;
    Eigen::VectorXd m_rhsX(numVertex);
    Eigen::VectorXd m_rhsY(numVertex);
    Eigen::VectorXd m_rhsZ(numVertex);

    // Right hand side
    m_rhsX.setZero();
    m_rhsY.setZero();
    m_rhsZ.setZero();

    // Compute right hand side
    for (unsigned int i = 0; i < numVertex; ++i) {
        std::vector<std::vector<double>> neighbors(numVertex);
        igl::adjacency_list(F, neighbors);

        Eigen::Vector3d sum(0.0, 0.0, 0.0);
        // Loop over one-ring neighborhood
        for (unsigned int j: neighbors[i]) {
            double w_ij = 1.0;
            sum += w_ij / 2.0 * (R[i] + R[j]) * (V.row(i) - V.row(j)).transpose();
        }
        m_rhsX(i) = sum.x();
        m_rhsY(i) = sum.y();
        m_rhsZ(i) = sum.z();
    }

    for (int constraint = 0; constraint < constraints.rows(); constraint++) {
        Eigen::Vector3d c = constraints.row(constraint);
        int c_index = constraintsIndices(constraint);
        // adapt rhs
        for (unsigned int i = 0; i < numVertex; ++i) {
            if (systemMatrix.coeff(i, c_index) != 0.0) {
                m_rhsX(i) -= systemMatrix.coeff(i, c_index) * c.x();
                m_rhsY(i) -= systemMatrix.coeff(i, c_index) * c.y();
                m_rhsZ(i) -= systemMatrix.coeff(i, c_index) * c.z();
            }
        }
        m_rhsX(c_index) = c.x();
        m_rhsY(c_index) = c.y();
        m_rhsZ(c_index) = c.z();

        // Delete rows and columns
        for (unsigned int i = 0; i < numVertex ; ++i) {
            if(systemMatrix.coeff(c_index, i) != 0.0) {
                systemMatrix.coeffRef(c_index, i) = systemMatrix.coeffRef(i, c_index) = 0.0;
                systemMatrix.coeffRef(c_index, c_index) = 1.0;
            }
        }
    }
    /*static*/ Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> svd(systemMatrix);
    Eigen::VectorXd x = svd.solve(m_rhsX);
    Eigen::VectorXd y = svd.solve(m_rhsY);
    Eigen::VectorXd z = svd.solve(m_rhsZ);

    for (unsigned int i = 0; i < numVertex; ++i) {
        U.row(i) = Eigen::Vector3d(x(i), y(i), z(i));
    }

    std::cout << "U has been updated\n";
}
