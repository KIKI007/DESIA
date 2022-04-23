#include "UtilityFunc.h"
#include <random>
#include <stdlib.h>
using namespace std;

using std::string;
using nanogui::Window;
using nanogui::Widget;
using nanogui::Button;
using nanogui::Label;
using nanogui::Widget;
using nanogui::Slider;


Eigen::MatrixXd V;
Eigen::MatrixXi F;
Eigen::MatrixXd C;


igl::viewer::Viewer viewer;

std::shared_ptr<ColorCoding> colorcoder;
std::shared_ptr<VoxelizedInterface> voxel_interface;
std::shared_ptr<VoxelizedRenderUnit> voxel_surface;
std::shared_ptr<VoxelizedPuzzleTree> voxel_tree;

nanogui::detail::FormWidget<RenderType> * pItem;
void random_cube(struct Parameter &para)
{
    int Nx, Ny, Nz;
    Nx = Ny = Nz = 4;
    VoxelRandom randomer(Nx, Ny, Nz, colorcoder);
    std::shared_ptr<array_3b> voxel;
    voxel = std::make_shared<array_3b>(array_3b(boost::extents[Nx][Ny][Nz]));
    for(int ix = 0; ix < Nx;ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
                (*voxel)[ix][iy][iz] = true;
        }
    }
    voxel_interface.reset();
    voxel_interface = std::make_shared<VoxelizedInterface>(VoxelizedInterface(Nx, Ny, Nz, colorcoder));
    (*voxel_interface) = *randomer.BFS_floodfill(voxel, 7, Nx*Ny*Nz);
    draw_puzzle(voxel_interface, para);
}

int test()
{
    VPuzFilter<int> filter;
    vector<VPuzFE <int>> list;
    for(int id = 0; id < 10; id++)
    {
        VPuzFE <int> ve;
        ve.weight = id * id - 20;
        ve.data_ = id;
        list.push_back(ve);
    }

    filter.max_candidates_num_ = 5;
    filter.insert(list);

    for(auto em: filter.candidate_)
    {
        std::cout << em.data_ << "\t" << em.rand << std::endl;
    }

    return 0;
}

int main(int argc, char** argv)
{
    srand(100);
    struct Parameter para;
    initialized(para);

    nanogui::init();
    viewer.callback_init = [&](igl::viewer::Viewer &viewer) {
        Button *b;
        Window *window;
        Label *label;
        
        window = viewer.ngui->addWindow(Eigen::Vector2i(250, 10), "Processing");
        viewer.ngui->addVariable<int>("width", [&](int w){
            para.wall_width = w;
        }, [&](){return para.wall_width;});
        viewer.ngui->addVariable<int>("height", [&](int h){
            para.wall_height = h;
        }, [&](){return para.wall_height;});
        viewer.ngui->addGroup("I/O");
        b = viewer.ngui->addButton("Create Wall", [&](){
            create_wall(para);
        });
        b->setTooltip("create an nxnx3 wall");

        b = viewer.ngui->addButton("Read .puz", [&](){read_puzzle_file(para);});
        b->setTooltip("Read an interlocking file and render it");

        b = viewer.ngui->addButton("Write .puz", write_puzzle_file);
        b->setTooltip("Write an interlocking file");

        label = viewer.ngui->addGroup("Interlocking");
        label->setFontSize(15);
        viewer.ngui->addVariable("Num of pieces", para.interlock_num_piece);
        b = viewer.ngui->addButton("Generate Next Part", [&](){generate_children(para);});
        b = viewer.ngui->addButton("Automatic Search", [&](){automatic_search(para);});
        b = viewer.ngui->addButton("Go Back", [&](){go_back(para);});
        auto pChild = viewer.ngui->addVariable<int>("Choose Nth child", [&](int a)
        {
            para.interlock_present_child = a;
            if(voxel_tree && voxel_tree->present_nodes_ && voxel_tree->present_nodes_->parent)
            {
                VPTreeNode *parent = voxel_tree->present_nodes_->parent;
                if(a < 0)
                {
                    a = 0;
                }
                else if(a >= parent->child.size())
                {
                    a = parent->child.size() - 1;
                }
                para.interlock_present_child = a;
                voxel_tree->present_nodes_ = parent->child[a].get();
                voxel_interface = voxel_tree->present_nodes_->puzzle_->output_assembly_interface(true, true);
                draw_puzzle(voxel_interface, para);
            }
        },[&](){return para.interlock_present_child;});
        pChild->setMinValue(0);
        pChild->setSpinnable(true);

        viewer.ngui->addButton("DBG", [&](){
            if(voxel_interface) write_show_puzzle_blocking_graph_simplified(voxel_interface);
        });

        viewer.ngui->addButton("Check Interlocking", [&](){
           if(voxel_interface){
               string log;
               shared_ptr<ContactGraph> graph = voxel_interface->output_contactgraph();
               graph->initialize();
               std::cout << graph->isRotationalInterlocking(log) << std::endl;
           }
        });

        window = viewer.ngui->addWindow(Eigen::Vector2i(500, 10), "Rendering");
        viewer.ngui->addGroup("Settings");
        pItem = viewer.ngui->addVariable<RenderType>("render type",
                                                          [&](RenderType type){
                                                              para.render_type = type;
                                                              if(para.render_type == RENDER_SPHERE)
                                                              {
                                                                  viewer.core.show_lines = false;
                                                              }
                                                              else if(para.render_type ==RENDER_CUBE)
                                                              {
                                                                  viewer.core.show_lines = true;
                                                              }
                                                              draw_puzzle(voxel_interface, para);
                                                          },
                                                          [&](){return para.render_type;});
        pItem->setItems({"Voxel", "Sphere"});
        pItem->setFixedWidth(80);

        auto pBox = viewer.ngui->addVariable<bool>("accessibility",
                                                   [&](bool value){
                                                       para.render_accessibility = value;
                                                       if(voxel_interface && voxel_interface->voxel_value_)
                                                       {
                                                           draw_puzzle(voxel_interface, para);
                                                       }
                                                   }, [&](){ return para.render_accessibility;});


        viewer.ngui->addGroup("ColorTable");
        for(int id = 0; id < 10; id++)
        {
            auto p = viewer.ngui->addVariable<Color>(std::to_string(id), *(para.render_colorTab[id]));
            nanogui::Color color = *(para.render_colorTab[id]);
            p->setValue(color);
            p->setFixedHeight(20);
            p->setFixedWidth(70);
            //p->setEditable(false);
        }

        viewer.ngui->addGroup("Tools");
        viewer.ngui->addButton("reDraw", [&](){draw_puzzle(voxel_interface, para);});
        label = viewer.ngui->addGroup("Display Disassembly");
        label->setFontSize(15);
        auto pDis = viewer.ngui->addVariable<int>("Step", [&](int value)
        {
            para.render_disassembly = value;
            if(value >= 0 && voxel_interface) {
                std::shared_ptr<VoxelizedInterface> tmp = voxel_interface->output_partial_part(value);
                draw_puzzle(tmp, para);
            }
        }, [&](){return para.render_disassembly;});
        pDis->setSpinnable(true);
        pDis->setMinValue(0);
        pDis->setValue(0);
        viewer.screen->performLayout();
        return false;
    };
    viewer.launch(true, false);
    return 0;
}
