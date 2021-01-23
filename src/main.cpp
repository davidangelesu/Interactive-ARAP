// To avoid printing of default keyboard controls
#ifndef IGL_VIEWER_VIEWER_QUIET
#define IGL_VIEWER_VIEWER_QUIET
#endif

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include "ControlPoints.h"
#include <iostream>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>


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

std::vector<Eigen::Matrix<double, 3, -1>> K;

long selectedPoint = -1;
Eigen::RowVector3f last_mouse;
// Helper variable for 'undo'
Eigen::MatrixXd last_controls;
bool dragHappend = false;
bool uniform_weights = false;

const Eigen::RowVector3d blue = {0.2,0.3,0.8};



int main(int argc, char *argv[]) {
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);
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
W,w                     Switch between cotangent and uniform weights (w_ij = 1)

)";

    const auto& set_base_mesh = [&](Eigen::MatrixXd vertices, Eigen::MatrixXi faces)
    {
        // Plot the mesh
        viewer.data().set_mesh(vertices, faces);
        viewer.data().face_based = true;
        // Align viewer such that mesh fills entire window
        viewer.core().align_camera_center(vertices, faces);

        // Init system matrix
        init_system_matrix(vertices, faces, m_systemMatrix, uniform_weights);
    };

    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &) -> bool {
        // Clear all points before setting all points again (incl. new points)
        viewer.data().clear_points();
        viewer.data().set_points(controlpoints.getPoints(), blue);

        //compute step
        if(controlpoints.getPoints().rows() > 0 && dragHappend)
          arap_single_iteration( K, controlpoints.getPoints(), controlpoints.getPointsVertex(), V, F, U, m_systemMatrix, uniform_weights);
        viewer.data().set_vertices(U);

      return false;

    };


    menu.callback_draw_custom_window = [&]()
    {
      ImGui::SetNextWindowPos(ImVec2(180.0f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(350, 160), ImGuiCond_FirstUseEver);
      ImGui::Begin("Interactive ARAP", nullptr, ImGuiWindowFlags_NoSavedSettings);
      ImGui::Text("[right click]               Place new control point");
      ImGui::Text("[left click] + [drag]       Pick control point and move it");
      ImGui::Text("[drag]                      Rotation");
      ImGui::Text("L,l                         Load a new mesh in OFF format");
      ImGui::Text("N,n                         Update deformation (i.e., run next iteration of solver)");
      ImGui::Text("U,u                         Undo reset");
      ImGui::Text("R,r                         Reset all control points");
      if (uniform_weights)
        ImGui::Text("W, w                        Switch to cotangent weights");
      else
        ImGui::Text("W,w                         Switch to uniform weights");
      ImGui::End();

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
            case 'N':
            case 'n':
//                update();
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
                    set_base_mesh(U, F);
                }
                break;
            case 'U':
            case 'u':
                // Undo reset, if last control points exists
                if(last_controls.size() != 0)
                {
                    controlpoints.setInitialPoints(last_controls);
                    last_controls = Eigen::MatrixXd();
                    arap_precompute(V, F, K, uniform_weights);
                }
                break;
            case 'W':
            case 'w':
                uniform_weights = !uniform_weights;
                init_system_matrix(V, F, m_systemMatrix, uniform_weights);
                arap_precompute(V, F, K, uniform_weights);

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
                arap_precompute(V,F,K, uniform_weights);
                return true;
            }
        }
        // right click
        else
        {
            bool result = controlpoints.add(viewer,U, F);
            arap_precompute(V,F,K, uniform_weights);
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
            dragHappend = true;
            arap_precompute(V, F, K, uniform_weights);
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
    igl::readOFF("../data/armadillo_1k.off", V, F);
    U = V;
    set_base_mesh(U, F);

    viewer.data().point_size = 20;
    viewer.launch();
}
