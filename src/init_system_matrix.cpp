#include "init_system_matrix.h"
#include "cotagent_matrix.h"
#include <igl/adjacency_list.h>

void init_system_matrix(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::SparseMatrix<double> &systemMatrix) {
    int numVertices = V.rows();
    systemMatrix = Eigen::SparseMatrix<double>(numVertices, numVertices);

    Eigen::SparseMatrix<double> L;
    cotagent_matrix(V, F, L);

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

