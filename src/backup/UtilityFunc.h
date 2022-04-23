//
// Created by *** on 18.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_UTILITYFUNC_H
#define UNDERSTAND_INTERLOCK_UTILITYFUNC_H

#include <string>

#include "igl/viewer/Viewer.h"
#include "igl/file_dialog_save.h"
#include "nanogui/slider.h"

#include "ColorCoding.h"
#include "InterlockSDChecking.h"
#include "Voxel/VoxelizedPuzzleTree.h"

/***********************************/
/*             Voxel               */
/***********************************/
#include "Voxel/VoxelizedInterface.h"
#include "Voxel/MeshVoxelizer.h"
#include "Voxel/VoxelizedRenderCube.h"
#include "Voxel/VoxelRandom.h"
#include "Voxel/VoxelizedPuzzle.h"
#include "Voxel/VoxelizedPuzzleTree.h"

enum RenderType
{
    RENDER_CUBE,
    RENDER_SPHERE
};

struct Parameter
{
    double voxelizer_precision;
    double voxelizer_size;
    vector<std::shared_ptr<nanogui::Color>> render_colorTab;
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

extern std::shared_ptr<ColorCoding> colorcoder;
extern std::shared_ptr<VoxelizedInterface> voxel_interface;
extern std::shared_ptr<VoxelizedRenderUnit> voxel_surface;
extern std::shared_ptr<VoxelizedPuzzleTree> voxel_tree;
extern Eigen::MatrixXd V;
extern Eigen::MatrixXi F;
extern Eigen::MatrixXd C;
extern igl::viewer::Viewer viewer;

void initialized(struct Parameter &para) {

    voxel_interface = nullptr;
    voxel_surface = nullptr;

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

    Vector3d colorTable[10] = {
            Vector3d(0.9, 0.2, 0.2),   //  1: Red
            Vector3d(0.2, 0.2, 0.9),   //  2: Blue
            Vector3d(0.9, 0.9, 0.5),   //  3: Yellow
            Vector3d(0.9, 0.5, 0.2),   //  4: Orange
            Vector3d(0.7, 0.2, 0.9),   //  5: Purple
            Vector3d(0.2, 0.9, 0.9),   //  6: Cyan
            Vector3d(0.5, 0.3, 0.2),   //  7: Brown
            Vector3d(0.9, 0.2, 0.6),   //  8: Pink
            Vector3d(0.6, 0.6, 0.7),   //  9: Gray
            Vector3d(0.9, 0.6, 0.5)}; // 10:LightSalmon


    for (int id = 0; id < 10; id++)
    {
        std::shared_ptr<nanogui::Color> pColor = std::make_shared<nanogui::Color>(nanogui::Color(255 * colorTable[id][0],255 * colorTable[id][1], 255 * colorTable[id][2], 255));
        para.render_colorTab.push_back(pColor);
    }

    colorcoder = std::make_shared<ColorCoding>(ColorCoding(para.render_colorTab));

    voxel_tree = std::make_shared<VoxelizedPuzzleTree>(VoxelizedPuzzleTree(para.interlock_minimum, para.interlock_maximum));
}

void write_show_puzzle_blocking_graph(std::shared_ptr<VoxelizedInterface> interface)
{
    if(interface)
    {
        time_t start = clock();
        std::shared_ptr<Assembly> assembly = interface->output_assembly();
        std::cout << (double)(clock() - start) / CLOCKS_PER_SEC << std::endl;
        InterlockSDChecking checker;
        Vector3d vec[3] = {
                        Vector3d(1, 0, 0),
                        Vector3d(0, 1, 0),
                        Vector3d(0, 0, 1)};
        string caption[3] = {"X Direction", "Y Direction", "Z Direction"};
        string openfile;
        for(int id = 0; id < 3; id++)
        {
            start = clock();
            std::shared_ptr<DirectedGraph> graph =  checker.init_sd_graph(vec[id], *assembly);
            std::cout << (double)(clock() - start) / CLOCKS_PER_SEC << std::endl;
            string dot_file = "tmp_" + std::to_string(id) + ".dot";
            string png_file = "tmp_" + std::to_string(id) + ".png";
            graph->output_dot(dot_file, caption[id]);
            string command = "dot -Tpng " + dot_file + " -o " + png_file;
            system(command.c_str());
            openfile += " " + png_file;
        }

        string command = "open" + openfile;
        system(command.c_str());
    }
    return;
}

void create_wall(const struct Parameter &para)
{
    voxel_interface.reset();
    voxel_surface.reset();

    array_3i array(boost::extents[para.wall_width][para.wall_height][3]);
    for(int iz = 0; iz < 3; iz ++)
    {
        for(int iy = 0; iy < para.wall_height; iy ++)
        {
            for(int ix = 0; ix < para.wall_width; ix ++)
            {
                array[ix][iy][iz] = 0;
            }
        }
    }


    voxel_tree.reset();
    std::shared_ptr<VoxelizedPuzzle> puzzle = std::make_shared<VoxelizedPuzzle>(VoxelizedPuzzle(para.wall_width, para.wall_height, 3, array, colorcoder));

    voxel_tree = std::make_shared<VoxelizedPuzzleTree>(VoxelizedPuzzleTree(para.interlock_num_piece, puzzle->get_num_voxels()));
    voxel_tree->set_root(puzzle);
    voxel_tree->present_nodes_ = voxel_tree->root();
    voxel_interface = puzzle->output_assembly_interface(true, true);
    for(int id = 0; id < puzzle->parts_.size(); id++)
    {
        std::cout << "part:\t" << id << ", voxel:\t" << puzzle->parts_[id]->elist_.data_.size() << std::endl;
    }
    std::cout << "total voxels:\t" << puzzle->voxel_.size() << std::endl;

    draw_puzzle(voxel_interface, para);

    viewer.data.clear();
    viewer.data.set_mesh(V, F);
    viewer.data.set_colors(C);
    viewer.core.align_camera_center(V, F);
    viewer.core.camera_zoom = 1;
}



void write_show_puzzle_blocking_graph_simplified(std::shared_ptr<VoxelizedInterface> interface)
{
    if(interface)
    {
        time_t start = clock();
        std::shared_ptr<Assembly> assembly = interface->output_assembly();
        std::cout << (double)(clock() - start) / CLOCKS_PER_SEC << std::endl;
        InterlockSDChecking checker;
        Vector3d vec[3] = {
                Vector3d(1, 0, 0),
                Vector3d(0, 1, 0),
                Vector3d(0, 0, 1)};
        string caption[3] = {"X Direction", "Y Direction", "Z Direction"};
        string openfile;
        for(int id = 0; id < 3; id++)
        {
            start = clock();
            std::shared_ptr<DirectedGraph> graph =  checker.simplified_sd_graph(vec[id], *assembly);
            std::cout << (double)(clock() - start) / CLOCKS_PER_SEC << std::endl;
            string dot_file = "tmp_" + std::to_string(id) + ".dot";
            string png_file = "tmp_" + std::to_string(id) + ".png";
            graph->output_dot(dot_file, caption[id]);
            string command = "dot -Tpng " + dot_file + " -o " + png_file;
            system(command.c_str());
            openfile += " " + png_file;
        }

        string command = "open" + openfile;
        system(command.c_str());
    }
    return;
}


void write_puzzle_file()
{
    if(voxel_interface)
    {
        string file_name =  igl::file_dialog_save();
        if(file_name != "")
            voxel_interface->output_file(file_name);
    }
    return;
}

void read_surface_file(struct Parameter &para)
{
    string file_name =  igl::file_dialog_open();
    if(file_name != "")
    {
        if(igl::readOBJ(file_name, V, F))
        {
            voxel_surface.reset();
            voxel_interface.reset();

            voxel_surface = std::make_shared<VoxelizedRenderUnit>(VoxelizedRenderUnit());
            voxel_surface->V = V;
            voxel_surface->F = F;

            Eigen::Vector3d m = V.colwise().minCoeff();
            Eigen::Vector3d M = V.colwise().maxCoeff();
            para.voxelizer_precision = (M - m).norm() / 100;
            para.voxelizer_size = (M - m).norm() / 50;

            viewer.data.clear();
            viewer.data.set_mesh(V, F);
            viewer.core.align_camera_center(V, F);
            viewer.core.camera_zoom = 1;
        }
    }
}

void voxelize_mesh(const struct Parameter &para)
{
    if(voxel_surface)
    {
        MeshVoxelizer voxelizer;
        voxelizer.set_mesh(voxel_surface->V, voxel_surface->F);
        voxelizer.set_parameter(para.voxelizer_precision,
                                para.voxelizer_size,
                                para.voxelizer_size,
                                para.voxelizer_size);
        auto array = voxelizer.voxelized();
        VoxelizedPuzzle puzzle(voxelizer.nx(), voxelizer.ny(), voxelizer.nz(), *array, colorcoder);
        voxel_interface = puzzle.output_assembly_interface(true, true);
        draw_puzzle(voxel_interface, para);
        return;
    }
}

void generate_children(struct Parameter &para)
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
            draw_puzzle(voxel_interface, para);
        }
    }
}



#endif //UNDERSTAND_INTERLOCK_UTILITYFUNC