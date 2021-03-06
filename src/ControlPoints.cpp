#include "ControlPoints.h"

/* Add Control Point in current location of Mouse*/
bool ControlPoints::add(igl::opengl::glfw::Viewer& viewer,Eigen::MatrixXd U, Eigen::MatrixXi F)
{
  // Picked face
  int fid;
  // Barycentric coordinates of picked point.
  Eigen::Vector3f bc;
  // Cast a ray in the view direction starting from the mouse position
  double x = viewer.current_mouse_x;
  double y = viewer.core().viewport(3) - viewer.current_mouse_y;
  if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y), viewer.core().view,
                              viewer.core().proj, viewer.core().viewport, U, F, fid, bc)) {

      long c;
      // Entry with highest value corresponds to the closest vertex in the triangle.
      bc.maxCoeff(&c);
      Eigen::RowVector3d control_point = U.row(F(fid,c));

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

/*Add control Points that are inside Control Area.*/
bool ControlPoints::add(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd U,  GUI::Polygon& controlArea)
{
    std::vector<unsigned int> constraintGroup;
    for (int i = 0; i < U.rows(); i++) {
        // Project to Viewport
        Eigen::Matrix<float,4,1> tmp;
        Eigen::RowVector3d control_point = U.row(i);
        tmp << U.row(i).transpose().cast<float>(),1;

        tmp = viewer.core().view * tmp;
        tmp = viewer.core().proj * tmp;
        tmp = tmp.array() / tmp(3);
        tmp = tmp.array() * 0.5f + 0.5f;
        tmp(0) = tmp(0) * viewer.core().viewport(2) + viewer.core().viewport(0);
        tmp(1) = tmp(1) * viewer.core().viewport(3) + viewer.core().viewport(1);

        // Add every vertex inside control area as control point
        if (controlArea.isInside(tmp.x(), tmp.y())) {
            // Check if control point not already added
            // If not, add it
            if (std::find(m_pointsVertexIndex.begin(), m_pointsVertexIndex.end(), i) == m_pointsVertexIndex.end())
            {
                m_points.push_back(control_point);
                m_pointsVertexIndex.push_back(i);
            }
            // Add control point to constraint group
            constraintGroup.push_back(i);
        }
    }
    // Add new constraint group, if not empty
    if (!constraintGroup.empty()) {
        m_constraintGroups.push_back(constraintGroup);
    }
    return false;
};

long ControlPoints::addSelectedPoint(igl::opengl::glfw::Viewer& viewer,Eigen::Vector3f mouseLocation) {
    // If there are no control points
    if (m_points.size() == 0) {
        return -1;
    }
    Eigen::MatrixXf projectedControlPoints;
    igl::project(Eigen::MatrixXf(getPoints().cast<float>()),
        viewer.core().view, viewer.core().proj, viewer.core().viewport, projectedControlPoints);

    // Calculate the norm of the difference of the mouse location and the projected control points
    Eigen::VectorXf D = (projectedControlPoints.rowwise() - mouseLocation.transpose()).rowwise().norm();
    // Obtain index of the selected point if it is close enough
    long selectedPoint = (D.minCoeff(&selectedPoint) < 30) ? selectedPoint : -1;

    if (selectedPoint!=-1) {
        // If selected point is part of a constraint group, add all points of the group
        for (unsigned int i=0; i < m_constraintGroups.size(); i++) {
            std::vector<unsigned int> constraintGroup = m_constraintGroups[i];
            if (std::find(constraintGroup.begin(), constraintGroup.end(), (int) m_pointsVertexIndex[selectedPoint]) != constraintGroup.end()) {
                for (unsigned int j=0; j < constraintGroup.size(); j++) {
                    auto it = find(m_pointsVertexIndex.begin(), m_pointsVertexIndex.end(), (int) constraintGroup[j]);
                    if (it != m_pointsVertexIndex.end()) {
                        int idx = distance(m_pointsVertexIndex.begin(), it);
                        m_selectedPointsIndex.push_back(idx);
                    }
                }
            }
        }
        // Add selected point, if not already added trough constraint group
        if (std::find(m_selectedPointsIndex.begin(), m_selectedPointsIndex.end(),(int)selectedPoint) == m_selectedPointsIndex.end()) {
            m_selectedPointsIndex.push_back(selectedPoint);
        }
    }
    return selectedPoint;
}

bool ControlPoints::remove(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd U, Eigen::MatrixXi F)
{
    // Picked face
    int fid;
    // Barycentric coordinates of picked point.
    Eigen::Vector3f bc;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view,
        viewer.core().proj, viewer.core().viewport, U, F, fid, bc)) {

        long c;
        // Entry with highest value corresponds to the closest vertex in the triangle.
        bc.maxCoeff(&c);
        Eigen::RowVector3d control_point = U.row(F(fid, c));

        int idx = F(fid, c);

        // If control point is part of a constraint group, remove all points of the group and whole group
        std::vector<std::vector<unsigned int>>::iterator constraintGroupIter;
        for(constraintGroupIter = m_constraintGroups.begin(); constraintGroupIter != m_constraintGroups.end(); ){
            if(std::find((*constraintGroupIter).begin(), (*constraintGroupIter).end(), idx) != (*constraintGroupIter).end()) {
                // Remove all points
                for (unsigned int j=0; j < (*constraintGroupIter).size(); j++) {
                    auto it = find(m_pointsVertexIndex.begin(), m_pointsVertexIndex.end(),
                                   (int) (*constraintGroupIter)[j]);
                    if (it != m_pointsVertexIndex.end()) {
                        int index = distance(m_pointsVertexIndex.begin(), it);
                        m_points.erase(m_points.begin() + index);
                        m_pointsVertexIndex.erase(m_pointsVertexIndex.begin() + index);
                    }
                }
                //Remove whole group
                constraintGroupIter = m_constraintGroups.erase(constraintGroupIter);
            } else {
                ++constraintGroupIter;
            }
        }
        // Remove control point, if not already removed trough constraint group
        auto iter = std::find(m_pointsVertexIndex.begin(), m_pointsVertexIndex.end(), idx);
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

void ControlPoints::clearSelectedPoints() {
    m_selectedPointsIndex.clear();
}

Eigen::MatrixXd ControlPoints::getPoints()
{
  Eigen::MatrixXd result(m_points.size(), 3);
  for(int i = 0; i < m_points.size(); i++)
  {
    result.row(i) = m_points[i];
  }
  return result;
}

Eigen::MatrixXd ControlPoints::getSelectedPoints()
{
    Eigen::MatrixXd result(m_selectedPointsIndex.size(),3);
   
    for (int i = 0; i < m_selectedPointsIndex.size(); i++)
    {
        result.row(i) = m_points[m_selectedPointsIndex[i]];
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

std::tuple<Eigen::MatrixXd, std::vector<std::vector<unsigned int>>> ControlPoints::removeAllPoints()
{
    Eigen::MatrixXd last_points(m_points.size(), 4);
    last_points.block(0,0,m_points.size(),3)= getPoints();
    last_points.block(0, 3, m_points.size(), 1) = getPointsVertex().cast <double>();
    m_points.clear();
    m_pointsVertexIndex.clear();
    std::vector<std::vector<unsigned int>> last_groups = m_constraintGroups;
    m_constraintGroups.clear();
    return {last_points, last_groups};
}

void ControlPoints::setInitialPoints(Eigen::MatrixXd initialPoints, std::vector<std::vector<unsigned int>> initialGroups)
{
    for(int i = 0; i < initialPoints.rows(); i++)
    {
        Eigen::RowVector3d control_point = initialPoints.block(i,0,1,3);
        m_points.push_back(control_point);
        m_pointsVertexIndex.push_back(initialPoints(i, 3));
    }
    m_constraintGroups = initialGroups;
}

void ControlPoints::updatePoints(Eigen::RowVector3d translation) {
    for (int i=0; i < m_selectedPointsIndex.size(); i++) {
        updatePoint(m_selectedPointsIndex[i], translation);
    }
};