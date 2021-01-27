
#include <igl/unproject_onto_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/project.h>
#include <vector>

class ControlPoints
{
  public:
    bool add(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd V, Eigen::MatrixXi F);
    bool remove(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd V, Eigen::MatrixXi F);
    Eigen::MatrixXd removeAllPoints();
    void setInitialPoints(Eigen::MatrixXd initialPoints);
    bool add(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd V, const std::vector < std::tuple<int, int>>& borderPixelsControlArea);
    bool addSelectedPoints(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd V, const std::vector < std::tuple<int, int>>& borderPixelsControlArea);
    void clearSelectedPoints();
    Eigen::MatrixXd getPoints();
    Eigen::MatrixXd getSelectedPoints();
    Eigen::VectorXi getPointsVertex();
    Eigen::SparseMatrix<double> getPointsAsMatrix(int numRows);

    inline Eigen::RowVector3d getPoint(int index) { return m_points[index]; }
    inline void updatePoint(int index, Eigen::RowVector3d newPoint) { m_points[index] = newPoint; }
  private:
    //Values of all control Points
    std::vector<Eigen::RowVector3d> m_points;
    //indices of vertices of all control points
    std::vector<unsigned int> m_pointsVertexIndex;
    //indices of all Selected control Points (NOTE: indices of m_points vector)
    std::vector<unsigned int> m_selectedPointsIndex;
};
