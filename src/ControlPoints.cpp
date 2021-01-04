#include "ControlPoints.h"


bool ControlPoints::add(igl::opengl::glfw::Viewer& viewer,Eigen::MatrixXd V, Eigen::MatrixXi F)
{
  // Picked face
  int fid;
  // Barycentric coordinates of picked point.
  Eigen::Vector3f bc;
  // Cast a ray in the view direction starting from the mouse position
  double x = viewer.current_mouse_x;
  double y = viewer.core().viewport(3) - viewer.current_mouse_y;
  if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y), viewer.core().view,
                              viewer.core().proj, viewer.core().viewport, V, F, fid, bc)) {

      long c;
      // Entry with highest value corresponds to the closest vertex in the triangle.
      bc.maxCoeff(&c);
      Eigen::RowVector3d control_point = V.row(F(fid,c));
      m_points.push_back(control_point);
      return true;
  }
  return false;

}

Eigen::MatrixXd ControlPoints::getPoints()
{
  Eigen::MatrixXd result;
  result.conservativeResize(m_points.size(), 3);
  for(int i = 0; i < m_points.size(); i++)
  {
    result.row(i) = m_points[i];
  }
  return result;
}

