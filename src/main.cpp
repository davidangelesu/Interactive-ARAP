#ifndef IGL_VIEWER_VIEWER_QUIET
#define IGL_VIEWER_VIEWER_QUIET
#endif

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[]) {
    igl::opengl::glfw::Viewer viewer;

    // Print keyboard controls
    std::cout<<R"(
[click]         To place new control point
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
                //TODO
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



    viewer.launch();
}
