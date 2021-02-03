
#include <igl/unproject_onto_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/project.h>
#include <vector>
#include "Polygon.h"

/**
 * Wrapper class for control points
 */
class ControlPoints
{
  public:
    /**
     * Add new control point
     * @param viewer IGL viewer
     * @param U Deformed vertex locations
     * @param F Faces of the mesh
     * @return if control point was successfully added
     */
    bool add(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd U, Eigen::MatrixXi F);

    /**
     * Add all control points within control area and create group
     * @param viewer IGL viewer
     * @param U Deformed vertex locations
     * @param controlArea Control area to constraint
     * @return if control points were successfully added
     */
    bool add(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd U,  GUI::Polygon& controlArea);

    /**
     * Remove selected control point and all corresponding control points within same constraint area
     * @param viewer IGL viewer
     * @param U Deformed vertex locations
     * @param F Faces of the mesh
     * @return if the control point was successfully removed
     */
    bool remove(igl::opengl::glfw::Viewer& viewer, Eigen::MatrixXd U, Eigen::MatrixXi F);

    /**
     * Remove all control points
     * @return {Position of removed control points, Removed control groups with corresponding control points}
     */
    std::tuple<Eigen::MatrixXd, std::vector<std::vector<unsigned int>>> removeAllPoints();

    /**
     * Initialize with control points
     * @param initialPoints Control points to be added
     * @param initialGroups Control groups with corresponding control points to be added
     */
    void setInitialPoints(Eigen::MatrixXd initialPoints, std::vector<std::vector<unsigned int>> initialGroups);

    /**
     * Select control point and all corresponding control points within same constraint area to deform mesh
     * @param viewer IGL viewer
     * @param mouseLocation Mouse cursor location
     * @return vertex index of selected point or -1 if unsuccessfull
     */
    long addSelectedPoint(igl::opengl::glfw::Viewer& viewer, Eigen::Vector3f mouseLocation);

    /**
     * Unselect all selected control points
     */
    void clearSelectedPoints();

    /**
     * @return positions of all currently placed control points
     */
    Eigen::MatrixXd getPoints();

    /**
     * @return vertex indices of all currently placed control points
     */
    Eigen::VectorXi getPointsVertex();

    /**
     * @return indices of all currently selected control points
     */
    Eigen::MatrixXd getSelectedPoints();

    /**
     * Get control points position trough vertex index
     * @param index Vertex index
     * @return requested control point position
     */
    inline Eigen::RowVector3d getPoint(int index) { return m_points[index]; }

    /**
     * Update control point position
     * @param index Vertex index of control point to update
     * @param translataion Offset between old and new position
     */
    inline void updatePoint(int index, Eigen::RowVector3d translataion) { m_points[index] =  m_points[index] + translataion; }

    /**
     * Update control point position and all corresponding control points within same constraint area
     * @param translation
     */
    void updatePoints(Eigen::RowVector3d translation);
  private:
    //Values of all control Points
    std::vector<Eigen::RowVector3d> m_points;
    //indices of vertices of all control points
    std::vector<unsigned int> m_pointsVertexIndex;
    //indices of all selected control points (NOTE: indices of m_points vector)
    std::vector<unsigned int> m_selectedPointsIndex;
    // Constraint groups with all corresponding vertex indices
    std::vector<std::vector<unsigned int>> m_constraintGroups;
};
