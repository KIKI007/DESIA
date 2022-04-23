#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include "helpers.h"

int main(int argc, char *argv[])
{

    RenderMesh mesh;

    srand(100);
    std::shared_ptr<ColorCoding> colorcoder;
    std::shared_ptr<VoxelizedInterface> voxel_interface;
    std::shared_ptr<VoxelizedPuzzleTree> voxel_tree;
    struct Parameter para;
    initialized(para, colorcoder);

    // Viewer
    igl::opengl::glfw::Viewer viewer;

    //menu
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);
    std::vector<std::thread> threads;
    create_menu(threads, viewer, menu, mesh, colorcoder, para, voxel_interface, voxel_tree);

    viewer.callback_pre_draw = [&mesh](igl::opengl::glfw::Viewer &viewer) -> bool{
        if(mesh.redraw){
            if(mesh.V.rows() >= 3 && mesh.F.rows() >= 1){
                viewer.data().clear();
                viewer.data().set_mesh(mesh.V, mesh.F);
                viewer.data().set_colors(mesh.C);
                viewer.core().align_camera_center(mesh.V, mesh.F);
                viewer.core().camera_zoom = 1.0;
                mesh.redraw = false;
            }
        }
        return false;
    };
    // Set Viewer to tight draw loop
    viewer.core().is_animating = true;

    viewer.launch();

    for(int id = 0; id < threads.size(); id++){
        threads[id].join();
    }
}