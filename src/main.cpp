// To avoid printing of default keyboard controls
#ifndef IGL_VIEWER_VIEWER_QUIET
#define IGL_VIEWER_VIEWER_QUIET
#endif

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include "ControlPoints.h"
#include <iostream>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

long selectedPoint = -1;
Eigen::RowVector3f last_mouse;

int main(int argc, char *argv[]) {
    igl::opengl::glfw::Viewer viewer;
    ControlPoints controlpoints;
    const Eigen::RowVector3d blue = {0.2,0.3,0.8};

    // Print keyboard controls
    std::cout<<R"(
[left click]    To pick new control point
[right click]   Place new control point
[drag]          Now: Rotation, TODO: To move control point
L,l             Load a new mesh in OFF format
U,u             Update deformation (i.e., run another iteration of solver)
R,r             Reset control points
)";

    const auto& update = [&]()
    {
      viewer.data().set_points(controlpoints.getPoints(), blue);
    };

    // This function is called when a keyboard key is pressed.
    viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer &, unsigned int key, int mod) {
        switch(key) {
            case 'L':
            case 'l':
            {
                // Load a new mesh in OFF format

                // Clear previous mesh
                viewer.data().clear();

                std::string fname = igl::file_dialog_open();

                if (fname.length() == 0)
                    return true;

                size_t last_dot = fname.rfind('.');
                if (last_dot == std::string::npos)
                {
                    std::cerr<<"Error: No file extension found in "<<fname<<std::endl;
                    return true;
                }

                std::string extension = fname.substr(last_dot+1);

                if (extension == "off" || extension =="OFF") {
                    igl::readOFF(fname, V, F);

                    // Plot the mesh
                    viewer.data().set_mesh(V, F);
                    viewer.data().face_based = true;

                    // Align viewer such that mesh fills entire window
                    viewer.core().align_camera_center(V, F);
                } else {
                    printf("Error: %s is not a recognized file type.\n",extension.c_str());
                }
                break;
            }
            case 'R':
            case 'r':
                // Reset control point
                viewer.data().clear_points();
                break;
            case 'U':
            case 'u':
                // Trigger an update
                //TODO
                break;
            default:
                // Disable default keyboard events
                return true;
        }
        // update();
        return true;
    };

    // This function is called when the mouse button is pressed
    // and picks a new control point when mouse button is pressed on mesh
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
        bool result = controlpoints.add(viewer,V, F);
        update();
        return result;
      }
      return false;

   };

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
        auto newValue = controlpoints.getPoint(selectedPoint) + (drag_scene-last_scene).cast<double>();
        controlpoints.updatePoint(selectedPoint, newValue);
        last_mouse = drag_mouse;
        update();
        return true;
      }
      return false;
    };

    viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer&, int, int)->bool
    {
      selectedPoint = -1;
      return false;
    };


    // Load default mesh
    igl::readOFF("../data/knight.off", V, F);
    viewer.data().set_mesh(V, F);
    viewer.data().face_based = true;

    viewer.data().point_size = 20;
    viewer.launch();
}
