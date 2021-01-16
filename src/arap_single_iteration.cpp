#include "arap_single_iteration.h"
#include "cotagent_matrix.h"
#include <igl/min_quad_with_fixed.h>
#include <igl/adjacency_list.h>


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

    std::vector<std::vector<double>> neighbors(numVertex);
    igl::adjacency_list(F, neighbors);

    for (unsigned int i = 0; i < numVertex; ++i) {
        for (unsigned int j : neighbors[i]) {
            Eigen::Vector3d e_prime_ij = U.row(i) - U.row(j);
            P_prime[i].conservativeResize(P_prime[i].rows(), P_prime[i].cols() + 1);
            P_prime[i].col(P_prime[i].cols() - 1) = e_prime_ij;
        }
    }

	//solve for each rotation matrix
	for (int i = 0; i < numVertex; i++) {
		//covariance matrix
		Eigen::Matrix3d S_i = K[i] * P_prime[i].transpose();

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(S_i,Eigen::ComputeFullU|Eigen::ComputeFullV);
		R[i] = svd.matrixV() * svd.matrixU().transpose();
	}
//	std::cout << "ROTATIONS COMPUTED\n";

	//**********Global Optimization
    Eigen::SparseMatrix<double> systemMatrix = m_systemMatrix;
    Eigen::VectorXd m_rhsX(numVertex);
    Eigen::VectorXd m_rhsY(numVertex);
    Eigen::VectorXd m_rhsZ(numVertex);

    // Right hand side
    m_rhsX.setZero();
    m_rhsY.setZero();
    m_rhsZ.setZero();

    //calculate cotagent matrix (with weights) w_i,j
    Eigen::SparseMatrix<double> L;
    cotagent_matrix(V, F, L);

    // Compute right hand side
    for (unsigned int i = 0; i < numVertex; ++i) {
        Eigen::Vector3d sum(0.0, 0.0, 0.0);
        // Loop over one-ring neighborhood
        for (unsigned int j: neighbors[i]) {
            double w_ij = L.coeff(i, j);
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

//    std::cout << "U has been updated\n";
}
