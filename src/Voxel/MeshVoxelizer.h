//
// Created by *** on 08.03.18.
//

#ifndef TI_STABLE_MESHVOXELIZER_H
#define TI_STABLE_MESHVOXELIZER_H

#define VOXELIZER_IMPLEMENTATION
#include "voxelizer.h"
#include <boost/multi_array.hpp>
#include <Eigen/Dense>
#include <cassert>
#include <vector>
#include <memory>
#include <queue>

using Eigen::MatrixXi;
using Eigen::MatrixXd;
using Eigen::Vector3i;
typedef std::vector<Vector3i,Eigen::aligned_allocator<Vector3i> > vecVector3i;
typedef std::queue<Vector3i> queueVector3i;
typedef boost::multi_array<bool, 3> array_3b;
//MeshVoxelizer
    //Goal: Convert a 2D-manifold (Triangular mesh) into a 3d grids voxel model.
    //input: V, F
            // V is the vertices matrix: n*3
            // F is the face matrix: m*3

class MeshVoxelizer {

public:

    //3-dim array for storing the 3d grids.


public:

    MeshVoxelizer();

    ~MeshVoxelizer()
    {
        if(mesh_ != nullptr) vx_mesh_free(mesh_);
        if(result_mesh_!= nullptr) vx_mesh_free(result_mesh_);
    }

public:

    void set_mesh(MatrixXd &V, MatrixXi &F);

    void set_parameter(float precision, float voxelsizex, float voxelsizey, float voxelsizez);

public:

    std::shared_ptr<array_3b> voxelized();

    void output_meshVF(MatrixXd &V, MatrixXi &F);

    inline int nx(){return Nx;}

    inline int ny(){return Ny;}

    inline int nz(){return Nz;}

private:

    vx_mesh_t* output_mesh();

public:

    vx_mesh_t* mesh_, *result_mesh_;

    float precision_, voxelsizex_, voxelsizey_, voxelsizez_;

protected:

    std::shared_ptr<array_3b> voxel_;

    int Nx, Ny, Nz;

    size_t nvoxels_;
};


#endif //TI_STABLE_MESHVOXELIZER_H
