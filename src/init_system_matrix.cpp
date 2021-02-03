#include "init_system_matrix.h"
#include "cotagent_matrix.h"
#include <igl/adjacency_list.h>

void init_system_matrix(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::SparseMatrix<double> &systemMatrix, bool use_uniform_weights, double uniform_weight) {
    int numVertices = V.rows();
    systemMatrix = Eigen::SparseMatrix<double>(numVertices, numVertices);

    // Compute weights
    Eigen::SparseMatrix<double> L;
    cotagent_matrix(V, F, L, use_uniform_weights, uniform_weight);

    // Loop over each vertex
    for (unsigned int i = 0; i < numVertices; ++i) {
        std::vector<std::vector<double>> neighbors(numVertices);
        igl::adjacency_list(F, neighbors);

        // Loop over one-ring neighborhood
        for (unsigned int j: neighbors[i]) {
            double w_ij = L.coeff(i, j);
            systemMatrix.coeffRef(i, i) += w_ij;
            systemMatrix.insert(i, j) = -w_ij;
        }
    }
}

