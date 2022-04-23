//
// Created by 汪子琦 on 21.04.22.
//

#ifndef CONEJOINT_UTILITY_H
#define CONEJOINT_UTILITY_H
#include "voxel/VoxelizedInterface.h"
#include "voxel/VoxelizedRenderCube.h"
#include "voxel/VoxelizedRenderSphere.h"
#include <igl/opengl/glfw/Viewer.h>
#include "voxel/VoxelRandom.h"
#include "voxel/VoxelizedPuzzleTree.h"
#include "voxel/VoxelizedPuzzle.h"
#include <thread>
#include <future>

using namespace voxel;

struct RenderMesh{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd C;
    bool redraw = false;
};

enum RenderType
{
    RENDER_CUBE,
    RENDER_SPHERE
};

struct Parameter
{
    double voxelizer_precision;
    double voxelizer_size;
    std::vector<Eigen::Vector4d> render_colorTab;
    RenderType render_type;
    bool render_accessibility;
    bool render_mark;
    int render_disassembly;
    int interlock_present_child;

    int interlock_minimum;
    int interlock_maximum;
    int interlock_num_piece;

    int wall_width;

    int wall_height;
};

void initialized(struct Parameter &para,
                std::shared_ptr<ColorCoding> &colorcoder){

    para.render_type = RENDER_CUBE;
    para.voxelizer_precision = 0.01;
    para.voxelizer_size = 0.01;
    para.render_accessibility = false;
    para.render_mark = false;
    para.render_disassembly = 0;
    para.interlock_present_child = 0;
    para.interlock_maximum = 8;
    para.interlock_minimum = 8;
    para.interlock_num_piece = 7;

    para.wall_height = 10;
    para.wall_width = 10;

    Eigen::Vector3d colorTable[10] = {
            Eigen::Vector3d(0.9, 0.2, 0.2),   //  1: Red
            Eigen::Vector3d(0.2, 0.2, 0.9),   //  2: Blue
            Eigen::Vector3d(0.9, 0.9, 0.5),   //  3: Yellow
            Eigen::Vector3d(0.9, 0.5, 0.2),   //  4: Orange
            Eigen::Vector3d(0.7, 0.2, 0.9),   //  5: Purple
            Eigen::Vector3d(0.2, 0.9, 0.9),   //  6: Cyan
            Eigen::Vector3d(0.5, 0.3, 0.2),   //  7: Brown
            Eigen::Vector3d(0.9, 0.2, 0.6),   //  8: Pink
            Eigen::Vector3d(0.6, 0.6, 0.7),   //  9: Gray
            Eigen::Vector3d(0.9, 0.6, 0.5)}; // 10:LightSalmon


    for (int id = 0; id < 10; id++)
    {
        Eigen::Vector4d pColor = Eigen::Vector4d(255 * colorTable[id][0],255 * colorTable[id][1], 255 * colorTable[id][2], 255);
        para.render_colorTab.push_back(pColor);
    }

    colorcoder = std::make_shared<ColorCoding>(ColorCoding(para.render_colorTab));

    //voxel_tree = std::make_shared<VoxelizedPuzzleTree>(VoxelizedPuzzleTree(para.interlock_minimum, para.interlock_maximum));
}


void compute_puzzle_mesh(RenderMesh &mesh,
                 std::shared_ptr<VoxelizedInterface> interface,
                 const struct Parameter &para)
{
    if(interface)
    {
        interface->is_show_extra_value_color = para.render_accessibility;
        interface->is_show_extra_mark_color = para.render_mark;
        switch(para.render_type)
        {
            case RENDER_CUBE:
                interface = std::make_shared<VoxelizedRenderCube>(VoxelizedRenderCube(*interface));
                break;
            case RENDER_SPHERE:
                interface = std::make_shared<VoxelizedRenderSphere>(VoxelizedRenderSphere(*interface));
                break;
        }
        Eigen::MatrixXd V;
        Eigen::MatrixXd C;
        Eigen::MatrixXi F;
        interface->rendering(mesh.V, mesh.F, mesh.C);
        mesh.C /= 256;
        mesh.redraw = true;
    }
}

void random_cube(std::shared_ptr<VoxelizedInterface> &voxel_interface,
                  Parameter &para,
                 std::shared_ptr<ColorCoding> colorcoder)
{
    int Nx, Ny, Nz;
    Nx = Ny = Nz = 4;
    voxel::VoxelRandom randomer(Nx, Ny, Nz, colorcoder);
    std::shared_ptr<boost::multi_array<bool, 3>> voxel;
    voxel = std::make_shared<boost::multi_array<bool, 3>>(boost::multi_array<bool, 3>(boost::extents[Nx][Ny][Nz]));
    for(int ix = 0; ix < Nx;ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
                (*voxel)[ix][iy][iz] = true;
        }
    }
    voxel_interface = randomer.BFS_floodfill(voxel, 7, Nx*Ny*Nz);
}

void read_puzzle_file(RenderMesh &mesh,
                      const struct Parameter &para,
                      std::shared_ptr<ColorCoding> colorcoder,
                      std::shared_ptr<VoxelizedInterface> &voxel_interface,
                      std::shared_ptr<VoxelizedPuzzleTree> &voxel_tree)
{
    std::string file_name =  igl::file_dialog_open();
    if(file_name != "")
    {
        std::ifstream fin;
        fin.open(file_name);
        if(fin.fail()) return;
        int Nx, Ny, Nz;
        fin >> Nx >> Ny >> Nz;

        voxel_interface.reset();

        boost::multi_array<int, 3> array(boost::extents[Nx][Ny][Nz]);
        for(int iz = 0; iz < Nz; iz ++)
        {
            for(int iy = 0; iy < Ny; iy ++)
            {
                for(int ix = 0; ix < Nx; ix ++)
                {
                    int index = 0;
                    fin >> index; index --;
                    array[ix][iy][iz] = index;
                }
            }
        }

        std::shared_ptr<VoxelizedPuzzle> puzzle = std::make_shared<VoxelizedPuzzle>(VoxelizedPuzzle(Nx, Ny, Nz, array, colorcoder));

        voxel_tree = std::make_shared<VoxelizedPuzzleTree>(VoxelizedPuzzleTree(para.interlock_num_piece, puzzle->get_num_voxels()));
        voxel_tree->set_root(puzzle);
        voxel_tree->present_nodes_ = voxel_tree->root();
        voxel_interface = puzzle->output_assembly_interface(true, true);
        for(int id = 0; id < puzzle->parts_.size(); id++)
        {
            std::cout << "part:\t" << id << ", voxel:\t" << puzzle->parts_[id]->elist_.data_.size() << std::endl;
        }
        std::cout << "total voxels:\t" << puzzle->voxel_.size() << std::endl;

        compute_puzzle_mesh(mesh, voxel_interface, para);
    }
}

void generate_children(RenderMesh &mesh,
                       std::shared_ptr<VoxelizedInterface> &voxel_interface,
                       std::shared_ptr<VoxelizedPuzzleTree> &voxel_tree,
                       struct Parameter &para)
{
    if(voxel_tree != nullptr && voxel_tree->present_nodes_ != nullptr)
    {
        voxel_tree->set_traget_part(para.interlock_num_piece, voxel_tree->present_nodes_->puzzle_->get_num_voxels());
        voxel_tree->create_children(voxel_tree->present_nodes_);
        if(voxel_tree->present_nodes_->child.size() > 0)
        {
            para.interlock_present_child = 0;
            voxel_tree->present_nodes_ = voxel_tree->present_nodes_->child[0].get();
            voxel_interface = voxel_tree->present_nodes_->puzzle_->output_assembly_interface(true, true);
            compute_puzzle_mesh(mesh, voxel_interface, para);
        }
    }
}

void automatic_search(RenderMesh &mesh,
                      std::shared_ptr<VoxelizedInterface> &voxel_interface,
                      std::shared_ptr<VoxelizedPuzzleTree> &voxel_tree,
                      struct Parameter &para)
{

    int present_num_part = 3;
    std::shared_ptr<Timer> tot_timer = std::make_shared<Timer>(           Timer("Final Total time    "));
    tot_timer->start();
    if(voxel_tree != nullptr && voxel_tree->present_nodes_ != nullptr)
    {
        std::srand(time(NULL));
        voxel_tree->set_traget_part(para.interlock_num_piece, voxel_tree->present_nodes_->puzzle_->get_num_voxels());
        int it_times = 0;
        while(voxel_tree->present_nodes_ && voxel_tree->present_nodes_->part_num < para.interlock_num_piece)
        {
            if(voxel_tree->present_nodes_ != nullptr && voxel_tree->present_nodes_->part_num >= present_num_part)
            {
                present_num_part = voxel_tree->present_nodes_->part_num;
                voxel_interface = voxel_tree->present_nodes_->puzzle_->output_assembly_interface(true, true);
                compute_puzzle_mesh(mesh, voxel_interface, para);
            }

            voxel_tree->create_children(voxel_tree->present_nodes_);
            if(voxel_tree->present_nodes_->child.size() > 0)
            {
                para.interlock_present_child = 0;
                voxel_tree->present_nodes_ = voxel_tree->present_nodes_->child[0].get();
            }
            else
            {
                if(voxel_tree->present_nodes_->brother != nullptr)
                {
                    voxel_tree->present_nodes_ = voxel_tree->present_nodes_->brother;
                }
                else
                {
                    VPTreeNode *parent = voxel_tree->present_nodes_;
                    do {
                        parent = parent->parent;
                        if(parent)
                        {
                            voxel_tree->present_nodes_ = parent->brother;
                        }
                        else
                        {
                            voxel_tree->present_nodes_ = nullptr;
                            break;
                        }
                    }while(voxel_tree->present_nodes_ == nullptr);
                    if(parent) parent->child.clear();
                }
            }

            if(voxel_tree->present_nodes_)
                std::cout << it_times ++ << "\t, part \t" << voxel_tree->present_nodes_->part_num << "\tFinished!!" << std::endl;
        }


        if(voxel_tree->present_nodes_ && voxel_tree->present_nodes_->part_num == para.interlock_num_piece)
        {
            tot_timer->end();
            tot_timer->print();
            voxel_interface = voxel_tree->present_nodes_->puzzle_->output_assembly_interface(true, true);
            compute_puzzle_mesh(mesh, voxel_interface, para);
        }
    }
}


void go_back(RenderMesh &mesh,
             std::shared_ptr<VoxelizedInterface> &voxel_interface,
             std::shared_ptr<VoxelizedPuzzleTree> &voxel_tree,
             struct Parameter &para)
{
    if(voxel_tree != nullptr && voxel_tree->present_nodes_ != nullptr)
    {
        if(voxel_tree->present_nodes_->parent != nullptr)
        {
            voxel_tree->present_nodes_ = voxel_tree->present_nodes_->parent;
            voxel_interface = voxel_tree->present_nodes_->puzzle_->output_assembly_interface(true, true);
            compute_puzzle_mesh(mesh, voxel_interface, para);
        }
    }
}

void choose_Kth_child(RenderMesh &mesh,
                      std::shared_ptr<VoxelizedInterface> &voxel_interface,
                      std::shared_ptr<VoxelizedPuzzleTree> &voxel_tree,
                      int child_index,
                      Parameter &para)
{
    if(voxel_tree && voxel_tree->present_nodes_ && voxel_tree->present_nodes_->parent)
    {
        VPTreeNode *parent = voxel_tree->present_nodes_->parent;
        child_index = std::clamp(child_index, (int)0, (int)parent->child.size() - 1);
        para.interlock_present_child = child_index;
        voxel_tree->present_nodes_ = parent->child[child_index].get();
        voxel_interface = voxel_tree->present_nodes_->puzzle_->output_assembly_interface(true, true);
        compute_puzzle_mesh(mesh, voxel_interface, para);
    }
}

void create_menu(       std::vector<std::thread> &threads,
                        igl::opengl::glfw::Viewer &viewer,
                        igl::opengl::glfw::imgui::ImGuiMenu &menu,
                        RenderMesh &mesh,
                        std::shared_ptr<ColorCoding> colorcoder,
                        Parameter &para,
                        std::shared_ptr<VoxelizedInterface> &voxel_interface,
                        std::shared_ptr<VoxelizedPuzzleTree> &voxel_tree)
                 {

    // Add content to the default menu window
    menu.callback_draw_viewer_menu = [&]()
    {
        // Draw parent menu content
        //menu.draw_viewer_menu();
        if (ImGui::CollapsingHeader("IO", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if(ImGui::Button("Read .puz")){
                read_puzzle_file(mesh, para, colorcoder, voxel_interface, voxel_tree);
            }
            if(ImGui::Button("Write .puz")){

            }
        }

        if (ImGui::CollapsingHeader("Interlocking", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::InputInt("Number of pieces", &para.interlock_num_piece);
            if(ImGui::Button("Generate Next Part")){
                generate_children(mesh, voxel_interface, voxel_tree, para);
            }
            if(ImGui::Button("Automatic Search"))
            {
                auto thread_f = [](RenderMesh *mesh,
                        std::shared_ptr<VoxelizedInterface> *voxel_interface,
                        std::shared_ptr<VoxelizedPuzzleTree> *voxel_tree,
                        Parameter *para
                        ){
                    automatic_search(*mesh, *voxel_interface, *voxel_tree, *para);
                };

                threads.emplace_back(std::bind(thread_f, &mesh, &voxel_interface, &voxel_tree, &para));
            }
            if(ImGui::Button("Go Back")){
                go_back(mesh, voxel_interface, voxel_tree, para);
            }
            if(ImGui::InputInt("Child Index", &para.interlock_present_child)){
                choose_Kth_child(mesh, voxel_interface, voxel_tree, para.interlock_present_child, para);
            }
        }

        if(ImGui::CollapsingHeader("Rendering Settings", ImGuiTreeNodeFlags_DefaultOpen)){
            static int select_render_type = -1;
            if(ImGui::RadioButton("Cube", &select_render_type, 0)){
                para.render_type = (RenderType)select_render_type;
                compute_puzzle_mesh(mesh, voxel_interface, para);
                viewer.data().show_lines = 1;
            };
            ImGui::SameLine();
            if(ImGui::RadioButton("Sphere", &select_render_type, 1)){
                para.render_type = (RenderType)select_render_type;
                compute_puzzle_mesh(mesh, voxel_interface, para);
                viewer.data().show_lines = 0;
            }

            if(ImGui::Checkbox("Accessbility", &para.render_accessibility)){
                std::cout << para.render_accessibility << std::endl;
                compute_puzzle_mesh(mesh, voxel_interface, para);
            }
        }

        if(ImGui::CollapsingHeader("Disassembly", ImGuiTreeNodeFlags_DefaultOpen)){
            if(ImGui::Button("reset")){
                compute_puzzle_mesh(mesh, voxel_interface, para);
            }
            if(ImGui::InputInt("Step", &para.render_disassembly)){
                int value = para.render_disassembly;
                if(value >= 0 && voxel_interface) {
                    std::shared_ptr<VoxelizedInterface> tmp = voxel_interface->output_partial_part(value);
                    compute_puzzle_mesh(mesh, tmp, para);
                }
            }
        }
    };
}


#endif //CONEJOINT_UTILITY_H
