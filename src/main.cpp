// To avoid printing of default keyboard controls
#ifndef IGL_VIEWER_VIEWER_QUIET
#define IGL_VIEWER_VIEWER_QUIET
#endif

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_on_plane.h>
#include "ControlPoints.h"
#include "polygon.h"
#include <iostream>

#include "arap_precompute.h"
#include "arap_single_iteration.h"
#include "init_system_matrix.h"

//Original Vertex Location
Eigen::MatrixXd V;
//Deformed Vertex Location
Eigen::MatrixXd U;
Eigen::MatrixXi F;
//System matrix
Eigen::SparseMatrix<double> m_systemMatrix;
//Control points for constraints
ControlPoints controlpoints;

std::vector<Eigen::Matrix<double, 3, -1>> K;

long selectedPoint = -1;
Eigen::RowVector3f last_mouse;
// Helper variable for 'undo'
Eigen::MatrixXd last_controls;

// defining Control Area:
bool isDefiningControlArea = false;
//defining Dragging Control Area
GUI::Polygon borderPixelsControlArea;

Eigen::Matrix<double, -1, 3> borderPointsControlArea;
Eigen::Matrix<double, -1, 3> tempBorderPoint;

const Eigen::RowVector3d blue = {0.2,0.3,0.8};
const Eigen::RowVector3d green = {0.2,0.6,0.3};
const Eigen::RowVector3d red = { 0.8,0.0,0.1 };

int main(int argc, char *argv[]) {
    igl::opengl::glfw::Viewer viewer;

    // Print keyboard controls
    std::cout<<R"(
[right click]                       Place new control point
[right click] + [ctl]               Remove control point
[right click] + [alt]               Define Control Area
[left click] + [drag]               Pick control point and move it
[right click] + [alt] + [shift]     Define Control Area for Dragging
[left click]+ [drag] +[shift]       Pick point in control Area and move complete control Area.
[drag]                              Rotation
L,l                                 Load a new mesh in OFF format
U,u                                 Undo reset
R,r                                 Reset all control points

)";

    const auto& set_base_mesh = [&](Eigen::MatrixXd vertices, Eigen::MatrixXi faces)
    {
        // Plot the mesh
        viewer.data().set_mesh(vertices, faces);
        viewer.data().face_based = true;

        // Align viewer such that mesh fills entire window
        viewer.core().align_camera_center(vertices, faces);
        arap_precompute(vertices, faces, K);
        // Init system matrix
        init_system_matrix(vertices, faces, m_systemMatrix);
    };

    // This function is called before the draw procedure of Preview3D
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &) -> bool {
        // Clear all points before setting all points again (incl. new points)
        viewer.data().clear_points();
        viewer.data().clear_edges();


        viewer.data().add_points(controlpoints.getSelectedPoints(), red);
        viewer.data().add_points(controlpoints.getPoints(), blue);
        viewer.data().add_points(borderPointsControlArea, green);
        

        // Draw edges between border points
        if (borderPointsControlArea.rows() > 0) {
            for (int i = 0; i < borderPointsControlArea.rows() - 1; ++i) {
                viewer.data().add_edges(borderPointsControlArea.row(i),
                                        borderPointsControlArea.row(i+1),
                                        green);
            };
        }

        // Draw temporary edge between last border point and mouse cursor
        if (borderPointsControlArea.rows() > 0 && tempBorderPoint.rows() > 0) {
            viewer.data().add_edges(borderPointsControlArea.row(borderPointsControlArea.rows() - 1), tempBorderPoint.row(0), green);
        }

        //compute step
        // TODO: Fix arap computation when whole region is selected
        if(controlpoints.getPoints().rows() > 0)
          arap_single_iteration( K, controlpoints.getPoints(), controlpoints.getPointsVertex(), V, F, U, m_systemMatrix);
        viewer.data().set_vertices(U);

      return false;

    };


    // This function is called when a keyboard key is pressed
    viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer &, unsigned int key, int mod) {
        switch(key) {
            case 'L':
            case 'l':
            {
                // Load a new mesh in OFF format

                std::string fname = igl::file_dialog_open();

                if (fname.length() == 0)
                    // 'Cancel' pressed, leave mesh as it is
                    return true;

                size_t last_dot = fname.rfind('.');
                if (last_dot == std::string::npos)
                {
                    std::cerr<<"Error: No file extension found in "<<fname<<std::endl;
                    return true;
                }

                std::string extension = fname.substr(last_dot+1);

                if (extension == "off" || extension =="OFF") {
                    // Clear previous mesh and control points
                    viewer.data().clear();
                    controlpoints.removeAllPoints();
                    // Delete last saved control points to disable 'undo' for new mesh
                    last_controls = Eigen::MatrixXd();

                    igl::readOFF(fname, V, F);
                    U = V;
                    set_base_mesh(U, F);
                    
                } else {
                    printf("Error: %s is not a recognized file type.\n",extension.c_str());
                }
                break;
            }
            case 'R':
            case 'r':
                // Reset control point, if available
                if(controlpoints.getPoints().size() != 0)
                {
                    last_controls = controlpoints.removeAllPoints();
                    // Clear all points
                    viewer.data().clear_points();
                    U = V;
                    //set_base_mesh(U, F);
                }
                break;
            case 'U':
            case 'u':
                // Undo reset, if last control points exists
                if(last_controls.size() != 0)
                {
                    controlpoints.setInitialPoints(last_controls);
                    last_controls = Eigen::MatrixXd();
                    
                }
                break;
            default:
                // Disable default keyboard events
                return true;
        }
        return true;
    };

    // This function is called when the mouse button is pressed
    // Places a new control point when right mouse button is pressed on mesh
    // Picks a control point when left mouse button is pressed
    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& viewer, int one, int two)->bool {
        std::cout << "MOUSE DOWN" << one << " , " << two << "\n";
        last_mouse= Eigen::Vector3f(viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y, 0);
        // Left click
        if(one == 0)
        {
            //No button pressed or Shift pressed
            if (two == 0 ||two==1) {
                selectedPoint=controlpoints.addSelectedPoint(viewer,last_mouse);
               
               
            }
            //if shift +control pressed
            else if (two == 3) {
                controlpoints.removeSelectedPoint(viewer, last_mouse);
            }
        }
        // right click
        else
        {
            //No Control,Shift,Control+Shift
            //Add point
            if (two == 0) {
                
                bool result = controlpoints.add(viewer,U, F);
                return result;
            }
            //Control
            //Remove Point
            else if (two == 2) {
                bool result = controlpoints.remove(viewer, U, F);
                return result;
            }
            //Alt ==4 , Alt+shift=5
            //Start/continue defining control area
            else if (two == 4 ||two==5) {
                isDefiningControlArea = true;
                // Save screen coordinates to compute control points later
                borderPixelsControlArea.addVertex(viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y);

                // Save world coordinates to display control area in GUI
                borderPointsControlArea.conservativeResize(borderPointsControlArea.rows() + 1, borderPointsControlArea.cols());
                borderPointsControlArea.row(borderPointsControlArea.rows() - 1) =
                        igl::unproject(Eigen::Vector3f(viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y, 0),
                                       viewer.core().view, viewer.core().proj, viewer.core().viewport).cast<double>();
                return true;
            }
           

        }
        return false;

    };

    // This function is called when a keyboard key is release
    viewer.callback_key_up = [&](igl::opengl::glfw::Viewer&, int one, int two)->bool {

        std::cout << "Key up\n";
        std::cout << "[ " << one << "," << two << "]\n";
        //Alt has been released
        // User stops pressing alt
        if (one == 342) {
            
            // End of defining control area
            isDefiningControlArea = false;
            borderPointsControlArea = Eigen::Matrix<double, -1, 3>();
            tempBorderPoint = Eigen::Matrix<double, -1, 3>();
            
            //if shift is still mantained pressed
            if (two == 1) {
                controlpoints.addSelectedPoints(viewer, V, borderPixelsControlArea);
                borderPixelsControlArea.clearVertices();
            }
            else {
                controlpoints.add(viewer, V, borderPixelsControlArea);
                borderPixelsControlArea.clearVertices();
                return true;
            }
            
           
        }//if user releases Shift
        else if (one == 340) {
            controlpoints.clearSelectedPoints();
        }
        return false;
    };

    // This function is called every time the mouse is moved
    viewer.callback_mouse_move = [&](igl::opengl::glfw::Viewer&, int one, int two)->bool
    {
      
        if(selectedPoint != -1)
        {
           
            Eigen::RowVector3f drag_mouse(viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y,last_mouse(2));
            Eigen::RowVector3f drag_scene, last_scene;
            igl::unproject(
                    drag_mouse,
                    viewer.core().view, viewer.core().proj,
                    viewer.core().viewport, drag_scene
            );
            igl::unproject(
                    last_mouse,
                    viewer.core().view, viewer.core().proj,
                    viewer.core().viewport, last_scene
            );
                
            auto translation = (drag_scene - last_scene).cast<double>();
          /*  auto oldVal = controlpoints.getPoint(selectedPoint);
            auto newValue = oldVal + (drag_scene-last_scene).cast<double>();*/
            controlpoints.updatePoints(translation);
            last_mouse = drag_mouse;
        
            return true;
        }
        if (isDefiningControlArea) {
            // Compute temporary border point to display on GUI
            if (tempBorderPoint.size() == 0) {
                tempBorderPoint.conservativeResize(1, 3);
            }
            tempBorderPoint.row(0) =
                    igl::unproject(Eigen::Vector3f(viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y, 0),
                                   viewer.core().view, viewer.core().proj, viewer.core().viewport).cast<double>();
            return true;
        }
        return false;
    };

    // This function is called when the mouse button is released
    viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer&, int one, int two)->bool
    {  
        if (two == 0) {
            controlpoints.clearSelectedPoints();
        }
        selectedPoint = -1;
        return false;
    };

    // Load default mesh
    igl::readOFF("../data/armadillo_1k.off", V, F);
    U = V;
    set_base_mesh(U, F);

    viewer.data().point_size = 20;
    viewer.launch();
}
