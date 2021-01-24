#include "ControlPoints.h"

/* Add Control Point in current location of Mouse*/
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

      // Check if control point not already added
      // If not, add it
      if (std::find(m_pointsVertexIndex.begin(), m_pointsVertexIndex.end(), F(fid, c)) == m_pointsVertexIndex.end())
      {
          m_points.push_back(control_point);
          m_pointsVertexIndex.push_back(F(fid, c));
      }
      return true;
  }
  return false;
}

/*Add control Points to All vertices inside PolYgon*/
bool ControlPoints::add(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd V, Eigen::MatrixXi F, const std::vector < std::tuple<int, int>>& borderPixelsControlArea)
{

};

bool ControlPoints::remove(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd V, Eigen::MatrixXi F)
{
    // Picked face
    int fid;
    // Barycentric coordinates of picked point.
    Eigen::Vector3f bc;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view,
        viewer.core().proj, viewer.core().viewport, V, F, fid, bc)) {

        long c;
        // Entry with highest value corresponds to the closest vertex in the triangle.
        bc.maxCoeff(&c);
        Eigen::RowVector3d control_point = V.row(F(fid, c));

        // Check if control point not already added
        // If iter is
        auto iter = std::find(m_pointsVertexIndex.begin(), m_pointsVertexIndex.end(), F(fid, c));
        if (iter != m_pointsVertexIndex.end())
        {
            int index = iter - m_pointsVertexIndex.begin();
            m_points.erase(m_points.begin()+index);
            m_pointsVertexIndex.erase(m_pointsVertexIndex.begin() + index);
        }
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

Eigen::VectorXi ControlPoints::getPointsVertex()
{
    Eigen::VectorXi result(m_pointsVertexIndex.size());
    for (int i = 0; i < m_pointsVertexIndex.size(); i++) {
        result(i) = m_pointsVertexIndex[i];
    }
    return result;
}
Eigen::SparseMatrix<double> ControlPoints::getPointsAsMatrix(int numRows)
{
    Eigen::SparseMatrix<double> result(numRows,3);
    std::vector< Eigen::Triplet<double> > tripletList;
    tripletList.reserve(m_points.size()*3);

    for (int i = 0; i < m_pointsVertexIndex.size(); i++) {
        tripletList.push_back(Eigen::Triplet<double>(i, 0, m_points[i][0]));
        tripletList.push_back(Eigen::Triplet<double>(i, 1, m_points[i][1]));
        tripletList.push_back(Eigen::Triplet<double>(i, 2, m_points[i][2]));
    }

    result.setFromTriplets(tripletList.begin(),tripletList.end());

    return result;
}   



Eigen::MatrixXd ControlPoints::removeAllPoints()
{
    Eigen::MatrixXd last_points(m_points.size(), 4);
    last_points.block(0,0,m_points.size(),3)= getPoints();
    last_points.block(0, 3, m_points.size(), 1) = getPointsVertex().cast <double>();
    m_points.clear();
    m_pointsVertexIndex.clear();
    return last_points;
}


void ControlPoints::setInitialPoints(Eigen::MatrixXd initialPoints)
{
    for(int i = 0; i < initialPoints.rows(); i++)
    {
        Eigen::RowVector3d control_point = initialPoints.block(i,0,1,3);
        m_points.push_back(control_point);
        m_pointsVertexIndex.push_back(initialPoints(i, 3));
    }
}

