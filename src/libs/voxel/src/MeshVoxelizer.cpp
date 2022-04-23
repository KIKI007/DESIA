//
// Created by *** on 08.03.18.
//

#include "voxel/MeshVoxelizer.h"
#include <iostream>

namespace voxel{
    void MeshVoxelizer::set_mesh(MatrixXd &V, MatrixXi &F)
//input a mesh(V, F)
    {
        int nvertices = V.rows(); // number of vertices
        int nfaces = F.rows();    // number of faces

        mesh_ = vx_mesh_alloc(nvertices, 3 * nfaces);   //allocate space for mesh_

        /*          add face            */
        for(int id = 0; id < nfaces; id++)
        {
            mesh_->indices[id * 3] = F(id, 0);
            mesh_->indices[id * 3 + 1] = F(id, 1);
            mesh_->indices[id * 3 + 2] = F(id, 2);
        }

        /*          add vertex          */
        for(int id = 0; id < nvertices; id++)
        {
            mesh_->vertices[id].x = V(id, 0);
            mesh_->vertices[id].y = V(id, 1);
            mesh_->vertices[id].z = V(id, 2);
        }
    }

    std::shared_ptr<boost::multi_array<bool, 3>> MeshVoxelizer::voxelized()
//voxelized the input mesh
    {

        vx_point_cloud_t* vx_pc = nullptr;

        //use the library to create an point cloud
        //each element in the cloud represents the center point of each grid
        vx_pc = vx_voxelize_pc(mesh_, voxelsizex_, voxelsizey_, voxelsizez_, precision_);

        std::cout << vx_pc->nvertices << std::endl;
        if(vx_pc->nvertices > 0)
        {

            /* calculate the bounding box of the point cloud */
            Eigen::Vector3d coord_min(vx_pc->vertices[0].x, vx_pc->vertices[0].y, vx_pc->vertices[0].z);
            Eigen::Vector3d coord_max(vx_pc->vertices[0].x, vx_pc->vertices[0].y, vx_pc->vertices[0].z);
            for(int id = 0; id < vx_pc->nvertices; id++)
            {
                if(coord_min[0] > vx_pc->vertices[id].x) coord_min[0] = vx_pc->vertices[id].x;
                if(coord_min[1] > vx_pc->vertices[id].y) coord_min[1] = vx_pc->vertices[id].y;
                if(coord_min[2] > vx_pc->vertices[id].z) coord_min[2] = vx_pc->vertices[id].z;

                if(coord_max[0] < vx_pc->vertices[id].x) coord_max[0] = vx_pc->vertices[id].x;
                if(coord_max[1] < vx_pc->vertices[id].y) coord_max[1] = vx_pc->vertices[id].y;
                if(coord_max[2] < vx_pc->vertices[id].z) coord_max[2] = vx_pc->vertices[id].z;
            }

            /* calculate the grid's X,Y,Z size */
            Nx = std::round((coord_max[0] - coord_min[0]) / voxelsizex_) + 1;
            Ny = std::round((coord_max[1] - coord_min[1]) / voxelsizey_) + 1;
            Nz = std::round((coord_max[2] - coord_min[2]) / voxelsizez_) + 1;

            std::cout << Nx << Ny << Nz << std::endl;

            /* allocate space for grid */
            voxel_.reset();
            voxel_ = std::make_shared<array_3b>(array_3b(boost::extents[Nx][Ny][Nz]));
            for(int ix = 0; ix < Nx;ix++)
            {
                for(int iy = 0; iy < Ny; iy++)
                {
                    for(int iz = 0; iz < Nz; iz++)
                        (*voxel_)[ix][iy][iz] = false;
                }
            }

            /* set value for grid. !!The library only create a shell of grids. */
            for(int id = 0; id < vx_pc ->nvertices; id++)
            {
                int iX, iY, iZ;
                iX = std::round((vx_pc->vertices[id].x - coord_min[0]) / voxelsizex_);
                iY = std::round((vx_pc->vertices[id].y - coord_min[1]) / voxelsizey_);
                iZ = std::round((vx_pc->vertices[id].z - coord_min[2]) / voxelsizez_);
                (*voxel_)[iX][iY][iZ] = true;
            }

            /* infill the inner part of the model */
            // Algorithm:
            // if a grid cannot reach the boundary by using bfs and is not filled, the grid must be filled.
            // visited prevent dead loop, every grid only travels once.
            array_3b visited(boost::extents[Nx][Ny][Nz]);
            for(int ix = 0; ix < Nx; ix++) {
                for (int iy = 0; iy < Ny; iy++) {
                    for (int iz = 0; iz < Nz; iz++) {
                        visited[ix][iy][iz] = (*voxel_)[ix][iy][iz];
                    }
                }
            }

            int dX[6] = {1, -1,  0,  0, 0,  0};
            int dY[6] = {0,  0,  1, -1, 0,  0};
            int dZ[6] = {0,  0,  0,  0, 1, -1};
            for(int ix = 0; ix < Nx; ix++)
            {
                for(int iy = 0; iy < Ny; iy++)
                {
                    for(int iz = 0; iz < Nz; iz++)
                    {
                        if(!visited[ix][iy][iz])
                        {
                            std::queue<Eigen::Vector3i> queue;
                            std::vector<Eigen::Vector3i> list;

                            queue.push(Vector3i(ix, iy, iz));
                            bool touch_boundary = false;
                            visited[ix][iy][iz] = true;

                            while(!queue.empty())
                            {
                                Vector3i u = queue.front();
                                list.push_back(u);
                                queue.pop();

                                for(int id = 0; id < 6; id++)
                                {
                                    int niX = u[0] + dX[id];
                                    int niY = u[1] + dY[id];
                                    int niZ = u[2] + dZ[id];

                                    if(niX < 0 || niY < 0 || niZ < 0 || niX >= Nx || niY >= Ny || niZ >= Nz)
                                    {
                                        touch_boundary = true;
                                        continue;
                                    }

                                    if(!visited[niX][niY][niZ])
                                    {
                                        queue.push(Vector3i(niX, niY, niZ));
                                        visited[niX][niY][niZ] = true;
                                    }
                                }
                            }

                            if(!touch_boundary)
                            {
                                for(auto u : list)
                                    (*voxel_)[u[0]][u[1]][u[2]] = true;
                            }
                        }

                        if((*voxel_)[ix][iy][iz]) nvoxels_ ++; //count the total number of voxels.
                    }
                }
            }

        }

        if(vx_pc)
            vx_point_cloud_free(vx_pc);

        return voxel_;
    }

    vx_mesh_t *MeshVoxelizer::output_mesh() {

        if(result_mesh_ != NULL) vx_mesh_free(result_mesh_);

        result_mesh_ = VX_MALLOC(vx_mesh_t, 1);
        size_t nvertices = nvoxels_ * 8;
        size_t nindices = nvoxels_ * 36;
        result_mesh_->nnormals = 6;
        result_mesh_->vertices = VX_CALLOC(vx_vertex_t, nvertices);
        result_mesh_->normals = VX_CALLOC(vx_vec3_t, 6);
        result_mesh_->colors = NULL;
        result_mesh_->indices = VX_CALLOC(unsigned int, nindices);
        result_mesh_->normalindices = VX_CALLOC(unsigned int, nindices);
        result_mesh_->nindices = 0;
        result_mesh_->nvertices = 0;

        vx_vertex_t hvs = {0.1, 0.1, 0.1};

        float vertices[24] = {
                -hvs.x,  hvs.y,  hvs.z,
                -hvs.x, -hvs.y,  hvs.z,
                hvs.x, -hvs.y,  hvs.z,
                hvs.x,  hvs.y,  hvs.z,
                -hvs.x,  hvs.y, -hvs.z,
                -hvs.x, -hvs.y, -hvs.z,
                hvs.x, -hvs.y, -hvs.z,
                hvs.x,  hvs.y, -hvs.z,
        };

        for(int i = 0; i < Nx; i++) {
            for (int j = 0; j < Ny; j++) {
                for (int k = 0; k < Nz; k++) {
                    if((*voxel_)[i][j][k])
                    {
                        vx_vertex_t position = {2 * i * hvs.x, 2 * j * hvs.y, 2 * k * hvs.z};
                        vx_vertex_t color = {0, 0, 0};
                        vx__add_voxel(result_mesh_, &position, color, vertices);
                    }
                }
            }
        }
        return result_mesh_;
    }

    MeshVoxelizer::MeshVoxelizer()
    {
        Nx = Ny = Nz = 0;
        precision_ = 0.01;
        voxelsizex_ = 0.01;
        voxelsizey_ = 0.01;
        voxelsizez_ = 0.01;
        nvoxels_ = 0;
        mesh_ = nullptr;
        result_mesh_ = nullptr;
    }

    void MeshVoxelizer::set_parameter(float precision, float voxelsizex, float voxelsizey, float voxelsizez)
    {
        precision_ = precision;
        voxelsizex_ = voxelsizex;
        voxelsizey_ = voxelsizey;
        voxelsizez_ = voxelsizez;
    }

    void MeshVoxelizer::output_meshVF(MatrixXd &V, MatrixXi &F)
    {
        V.setZero();
        F.setZero();

        output_mesh();
        if(result_mesh_ != nullptr)
        {
            V = MatrixXd(result_mesh_->nvertices, 3);
            F = MatrixXi(result_mesh_->nindices / 3, 3);
            for(int id = 0; id < result_mesh_->nindices; id++)
            {
                F(id / 3, id % 3) = result_mesh_->indices[id];
            }
            for(int id = 0; id < result_mesh_->nvertices; id++)
            {
                V(id, 0) = result_mesh_->vertices[id].x;
                V(id, 1) = result_mesh_->vertices[id].y;
                V(id, 2) = result_mesh_->vertices[id].z;
            }
        }
        return;
    }

}
