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

/*Add control Points that are inside Control Area.*/
bool ControlPoints::add(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd V,  GUI::Polygon& controlArea)
{
    //Loop through the whole mesh!!
    //Test each point
    for (int i = 0; i < V.rows(); i++) {
        //Project to Viewport
        Eigen::Matrix<float,4,1> tmp;
        Eigen::RowVector3d control_point = V.row(i);
        tmp << V.row(i).transpose().cast<float>(),1;

        tmp = viewer.core().view * tmp;
        tmp = viewer.core().proj * tmp;
        tmp = tmp.array() / tmp(3);
        tmp = tmp.array() * 0.5f + 0.5f;
        tmp(0) = tmp(0) * viewer.core().viewport(2) + viewer.core().viewport(0);
        tmp(1) = tmp(1) * viewer.core().viewport(3) + viewer.core().viewport(1);

        //TODO: Maybe create a group for all this constraints so that you can move them togetter????
        if (controlArea.isInside(tmp.x(), tmp.y())) {
            // Check if control point not already added
            // If not, add it
            if (std::find(m_pointsVertexIndex.begin(), m_pointsVertexIndex.end(), i) == m_pointsVertexIndex.end())
            {
                m_points.push_back(control_point);
                m_pointsVertexIndex.push_back(i);
            }
        }
    }
    return false;
};

/*Add all control Points that are inside Control Area to list of selected control points.*/
bool ControlPoints::addSelectedPoints(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd V, GUI::Polygon& controlArea)
{
    bool hasNewEntryAdded = false;
    //Loop through all the control points
    for (int i = 0; i < m_points.size(); i++)
    {
        //Project to Viewport
        Eigen::Matrix<float, 4, 1> tmp;
        Eigen::RowVector3d control_point = m_points[i];
        tmp << control_point.transpose().cast<float>(), 1;
        tmp = viewer.core().view * tmp;
        tmp = viewer.core().proj * tmp;
        tmp = tmp.array() / tmp(3);
        tmp = tmp.array() * 0.5f + 0.5f;
        tmp(0) = tmp(0) * viewer.core().viewport(2) + viewer.core().viewport(0);
        tmp(1) = tmp(1) * viewer.core().viewport(3) + viewer.core().viewport(1);
        
        //Check if control Point is inside Control Area and not in list of selectedPoints.
        if (controlArea.isInside( tmp.x(), tmp.y())
            && (std::find(m_selectedPointsIndex.begin(), m_selectedPointsIndex.end(), i)== m_selectedPointsIndex.end())) {
            m_selectedPointsIndex.push_back(i);
            hasNewEntryAdded = true;
        }
    }
    return hasNewEntryAdded;
};

long ControlPoints::addSelectedPoint(igl::opengl::glfw::Viewer& viewer,Eigen::Vector3f mouseLocation) {
    //if there are no control Points
    if (m_points.size() == 0) {
        return -1;
    }
    Eigen::MatrixXf projectedControlPoints;
    igl::project(Eigen::MatrixXf(getPoints().cast<float>()),
        viewer.core().view, viewer.core().proj, viewer.core().viewport, projectedControlPoints);

    //Calculate the norm of the difference of the mouse location and the projected control points
    Eigen::VectorXf D = (projectedControlPoints.rowwise() - mouseLocation.transpose()).rowwise().norm();
    //Obtain index of the selected point if it is close enough
    long selectedPoint = (D.minCoeff(&selectedPoint) < 30) ? selectedPoint : -1;

    
    //if selected point not already added...
    if (selectedPoint!=-1 && std::find(m_selectedPointsIndex.begin(), m_selectedPointsIndex.end(),(int)selectedPoint) == m_selectedPointsIndex.end()) {
        m_selectedPointsIndex.push_back(selectedPoint);
       
    }
    return selectedPoint;
}


bool ControlPoints::removeSelectedPoint(igl::opengl::glfw::Viewer& viewer, Eigen::Vector3f mouseLocation) {
    //if there are no control Points
    if (m_points.size() == 0) {
        return false;
    }
    Eigen::MatrixXf projectedControlPoints;
    igl::project(Eigen::MatrixXf(getPoints().cast<float>()),
        viewer.core().view, viewer.core().proj, viewer.core().viewport, projectedControlPoints);

    //Calculate the norm of the difference of the mouse location and the projected control points
    Eigen::VectorXf D = (projectedControlPoints.rowwise() - mouseLocation.transpose()).rowwise().norm();
    //Obtain index of the selected point if it is close enough
    long selectedPoint = (D.minCoeff(&selectedPoint) < 30) ? selectedPoint : -1;

    //remove selected Point
    if (selectedPoint != -1) {
        std::remove(m_selectedPointsIndex.begin(), m_selectedPointsIndex.end(), (int)selectedPoint);
        return true;
    }
    return false;
}



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


void ControlPoints::updatePoints(Eigen::RowVector3d translation) {
    for (int i=0; i < m_selectedPointsIndex.size(); i++) {
        updatePoint(m_selectedPointsIndex[i], translation);
    }
};