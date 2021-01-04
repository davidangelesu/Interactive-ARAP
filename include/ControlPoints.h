
#include <igl/unproject_onto_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <vector>

class ControlPoints
{
  public:
    bool add(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd V, Eigen::MatrixXi F);

    Eigen::MatrixXd getPoints();
  private:
    const Eigen::RowVector3d blue = {0.2,0.3,0.8};
    std::vector<Eigen::RowVector3d> m_points;
};
