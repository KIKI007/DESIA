//
// Created by 汪子琦 on 22.04.22.
//

#include <igl/get_seconds.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <thread>
#include <future>

int main(int argc, char *argv[])
{
    // Load in mesh and initialize viewer
    Eigen::MatrixXd V,U;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh(CRL_DATA_FOLDER "/mesh/sphere.obj",V,F);
    U = V;
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V,F);
    // Computation thread: change the vertices of the mesh but don't touch
    // vertices in the viewer
    auto handle = std::async(std::launch::async,
                             [&V,&U,&viewer]()
                             {
                                 // repeat some phony computation for 5 seconds
                                 double t_start = igl::get_seconds();
                                 while(igl::get_seconds() - t_start < 5)
                                 {
                                     U.col(0) = V.col(0)*cos(igl::get_seconds());
                                 }
                                 // Tell glfw (viewer) to close window: this will cause launch() to
                                 // return.
                                 glfwSetWindowShouldClose(viewer.window, GL_TRUE);
                             }
    );
    // Just before drawing change the vertices of the mesh. Could easily add a
    // flag to only call set_vertices if there's an actually change
    viewer.callback_pre_draw =
            [&U](igl::opengl::glfw::Viewer & v)->bool{ v.data().set_vertices(U);return false;};
    // Set Viewer to tight draw loop
    viewer.core().is_animating = true;
    // Launch viewer in this thread
    viewer.launch();
}