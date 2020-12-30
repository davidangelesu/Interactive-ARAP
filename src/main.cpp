#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[])
{
    // Load a mesh in OFF format
    // const std::string file_src = igl::file_dialog_open();
    const std::string file_src = "../data/bunny.off";
    igl::readOFF(file_src, V, F);

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().face_based = true;
    viewer.launch();
}
