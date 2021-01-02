// To avoid printing of default keyboard controls
#ifndef IGL_VIEWER_VIEWER_QUIET
#define IGL_VIEWER_VIEWER_QUIET
#endif

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

const Eigen::RowVector3d blue(0.2,0.3,0.8);

int main(int argc, char *argv[]) {
    igl::opengl::glfw::Viewer viewer;

    // Print keyboard controls
    std::cout<<R"(
[click]         To pick new control point
[drag]          Now: Rotation, TODO: To move control point
L,l             Load a new mesh in OFF format
U,u             Update deformation (i.e., run another iteration of solver)
R,r             Reset control points
)";

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
    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& viewer, int, int)->bool {
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
            viewer.data().set_points(control_point, blue);
            return true;
        }
        return false;
    };


    viewer.data().point_size = 20;
    viewer.launch();
}
