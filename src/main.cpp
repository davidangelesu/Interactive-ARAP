// To avoid printing of default keyboard controls
#ifndef IGL_VIEWER_VIEWER_QUIET
#define IGL_VIEWER_VIEWER_QUIET
#endif

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include "ControlPoints.h"
#include <iostream>

#include "arap_precompute.h"
#include "arap_single_iteration.h"

//Original Vertex Location
Eigen::MatrixXd V;
//Deformed Vertex Location
Eigen::MatrixXd U;
Eigen::MatrixXi F;


std::vector<Eigen::Matrix<double, 3, -1>> K;

long selectedPoint = -1;
Eigen::RowVector3f last_mouse;
// Helper variable for 'undo'
Eigen::MatrixXd last_controls;

const Eigen::RowVector3d blue = {0.2,0.3,0.8};



int main(int argc, char *argv[]) {
    igl::opengl::glfw::Viewer viewer;
    ControlPoints controlpoints;

    // Print keyboard controls
    std::cout<<R"(
[right click]           Place new control point
[left click] + [drag]   Pick control point and move it
[drag]                  Rotation
L,l                     Load a new mesh in OFF format
N,n                     Update deformation (i.e., run next iteration of solver)
U,u                     Undo reset
R,r                     Reset all control points

)";

    const auto& set_original_mesh = [&](Eigen::MatrixXd vertices, Eigen::MatrixXi faces)
    {
        // Plot the mesh
        viewer.data().set_mesh(vertices, faces);
        viewer.data().face_based = true;

        // Align viewer such that mesh fills entire window
        viewer.core().align_camera_center(vertices, faces);
    };

    const auto& update = [&]()
    {
        // Clear all points before setting all points again (incl. new points)
        viewer.data().clear_points();
        viewer.data().set_points(controlpoints.getPoints(), blue);

        //compute step
        arap_single_iteration( K, controlpoints.getPoints(),F, U);
        std::cout << "Single Step Computed\n";
        viewer.data().set_vertices(U);
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
                    set_original_mesh(U, F);
                } else {
                    printf("Error: %s is not a recognized file type.\n",extension.c_str());
                }
                break;
            }
            case 'N':
            case 'n':
                update();
                break;
            case 'R':
            case 'r':
                // Reset control point, if available
                if(controlpoints.getPoints().size() != 0)
                {
                    last_controls = controlpoints.removeAllPoints();
                    // Clear all points
                    viewer.data().clear_points();
                    U = V;
                    set_original_mesh(U, F);
                }
                break;
            case 'U':
            case 'u':
                // Undo reset, if last control points exists
                if(last_controls.size() != 0)
                {
                    controlpoints.setInitialPoints(last_controls);
                    last_controls = Eigen::MatrixXd();
                    arap_precompute(V, F, K);
                    update();
                }
                break;
            default:
                // Disable default keyboard events
                return true;
        }
        //update();
        return true;
    };

    // This function is called when the mouse button is pressed
    // Places a new control point when right mouse button is pressed on mesh
    // Picks a control point when left mouse button is pressed
    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& viewer, int one, int two)->bool {
        last_mouse = Eigen::Vector3f(viewer.current_mouse_x, viewer.core().viewport(3)-viewer.current_mouse_y, 0);
        // Left click
        if(one == 0)
        {
            // don't crash if no control points are available
            if(controlpoints.getPoints().size() == 0)
                return false;

            Eigen::MatrixXf CP;
            igl::project(Eigen::MatrixXf(controlpoints.getPoints().cast<float>()),
                         viewer.core().view, viewer.core().proj, viewer.core().viewport, CP);
            Eigen::VectorXf D = (CP.rowwise() - last_mouse).rowwise().norm();
            selectedPoint = (D.minCoeff(&selectedPoint) < 30)?selectedPoint: -1;
            if(selectedPoint != -1)
            {
                last_mouse(2) = CP(selectedPoint, 2);
                update();
                return true;
            }
        }
        // right click
        else
        {
            bool result = controlpoints.add(viewer,U, F);
            arap_precompute(V,F,K);
            update();
            return result;
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
            auto oldVal = controlpoints.getPoint(selectedPoint);
            auto newValue = oldVal + (drag_scene-last_scene).cast<double>();
            controlpoints.updatePoint(selectedPoint, newValue);
            last_mouse = drag_mouse;
            arap_precompute(V, F, K);
            update();
            return true;
        }
        return false;
    };

    // This function is called when the mouse button is released
    viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer&, int, int)->bool
    {
        selectedPoint = -1;
        return false;
    };
    // Load default mesh
    igl::readOFF("../data/bunny.off", V, F);
    U = V;
    set_original_mesh(U, F);

    viewer.data().point_size = 20;
    viewer.launch();
}
