
#include <igl/unproject_onto_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <vector>

class ControlPoints
{
  public:
    bool add(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd V, Eigen::MatrixXi F);
    void removeAllPoints();

    Eigen::MatrixXd getPoints();
    inline Eigen::RowVector3d getPoint(int index) { return m_points[index]; }
    inline void updatePoint(int index, Eigen::RowVector3d newPoint) { m_points[index] = newPoint; }
  private:
    std::vector<Eigen::RowVector3d> m_points;
};
