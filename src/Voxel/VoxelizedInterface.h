//
// Created by *** on 11.03.18.
//

#ifndef TI_STABLE_VOXELIZEDASSEMBLY_H
#define TI_STABLE_VOXELIZEDASSEMBLY_H

#include "../Assembly.h"
#include "voxelizer.h"
#include "../ContactGraph/ContactGraph.h"
#include "../ContactGraph/ContactGraphMosekSolver.h"
#include "../ColorCoding.h"
#include <Eigen/Dense>
#include <boost/multi_array.hpp>
#include <string>
#include <fstream>
#include "igl/jet.h"
using std::string;

using Eigen::MatrixXi;
using Eigen::MatrixXd;
typedef boost::multi_array<int, 3> array_3i;
typedef boost::multi_array<double, 3> array_3d;

using Eigen::Vector3i;

//VoxelziedAssembly
    //For converting an voxel model into general assembly class
    //the part index is starting from 0. -1 represent void space.
    //Usage:
    //VoxelizedAssembly assemblyvoxel(10, 10, 10); //Initial a 10 * 10 * 10 voxel model
    //assemblyvoxel.set_grid_part_index(Eigen::Vector3i(0, 0, 0), 1); //set the (0, 0, 0) to be part 1
    //std::shared_ptr<Assembly> assembly = assemblyvoxel.output_assembly; //get the general assembly.

class VoxelizedInterface {
public:

    VoxelizedInterface(int nx, int ny, int nz, std::shared_ptr<ColorCoding> coder);

    ~VoxelizedInterface()
    {
        if(mesh_) vx_mesh_free(mesh_);
    }

public:

    /************************************/
    /*                                  */
    /* Func for constructing this class */
    /*                                  */
    /************************************/

    //pos : the voxel's position in the model
    //index: the part index of voxel
    void set_grid_part_index(Vector3i pos, int index);

    int get_grid_part_index(Vector3i pos);

    void use_extra_value_color(bool option);

    void set_grid_extra_value(Vector3i pos, double value);

    double get_grid_extra_value(Vector3i pos);

    void use_extra_mark_color(bool option);

    void set_grid_extra_mark(Vector3i pos, int mark);

    int get_grid_extra_mark(Vector3i pos);

    void init(int nx, int ny, int nz);

    void reverse_YZ();

    void merge_into_one_part();

public:

    /*************************************************/
    /*                                               */
    /* Func for converting into the General Assembly */
    /*                                               */
    /*************************************************/

    //output the general assembly class
    std::shared_ptr<Assembly> output_assembly();

    std::shared_ptr<array_3i> output_array(){return voxel_;}

    std::shared_ptr<ContactGraph> output_contactgraph();

public:

    /************************************/
    /*                                  */
    /* Func for outputing partial model */
    /*                                  */
    /************************************/

    std::shared_ptr<VoxelizedInterface> output_partial_part(int min_part_index);

    std::shared_ptr<VoxelizedInterface> output_section_by_Z(int Zlayer);

public:

    /*****************************/
    /*                           */
    /* Func for rendering result */
    /*                           */
    /*****************************/

    //output the triangualte mesh soup (V-vertices, F-faces, C-colors)
    virtual void rendering(MatrixXd &V, MatrixXi &F, MatrixXd &C){}
    
public:

    /**************************/
    /*                        */
    /* Func for saving result */
    /*                        */
    /**************************/

    void output_file(string file_name);

protected:

    /****************/
    /*              */
    /* Utility Func */
    /*              */
    /****************/

    int compute_voxels_num();

    int compute_parts_num();

    //Necessary variables during initialization
public:

    int Nx, Ny, Nz;

    std::shared_ptr<array_3i> voxel_;

    std::shared_ptr<array_3d> voxel_value_;

    std::shared_ptr<array_3i> voxel_mark_;

    std::shared_ptr<ColorCoding> colorcoder_;

public:

    int nvoxels_;

    int part_num_;

    bool is_show_extra_value_color;

    bool is_show_extra_mark_color;

protected:

    std::shared_ptr<Assembly> assembly_;

    vx_mesh_t *mesh_;
};


#endif //TI_STABLE_VOXELIZEDASSEMBLY_H
