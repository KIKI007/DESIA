//
// Created by *** on 17.03.18.
//

#include "voxel/VoxelizedRenderCube.h"
#include "iostream"

namespace voxel{
    void VoxelizedRenderCube::rendering(MatrixXd &V, MatrixXi &F, MatrixXd &C)
    {

        compute_voxels_num();

        //create a single cube
        MatrixXd cubeV(8, 3);
        cubeV <<    0, 0, 0,
                0, 0, hz,
                0, hy, 0,
                0, hy, hz,
                hx, 0, 0,
                hx, 0, hz,
                hx, hy, 0,
                hx, hy, hz;

        MatrixXi cubeF(12, 3);
        cubeF <<1, 7, 5,
                1, 3, 7,
                1, 4, 3,
                1, 2, 4,
                3, 8, 7,
                3, 4, 8,
                5, 7, 8,
                5, 8, 6,
                1, 5, 6,
                1, 6, 2,
                2, 6, 8,
                2, 8, 4;

        //create all cubes;
        V = MatrixXd(nvoxels_ * 8, 3);
        F = MatrixXi(nvoxels_ * 12, 3);

        int prev_voxel = 0;
        for(int ix = 0; ix < Nx; ix++)
        {
            for(int iy = 0; iy < Ny; iy++)
            {
                for(int iz = 0; iz < Nz; iz++)
                {
                    if((*voxel_)[ix][iy][iz] != -1)
                    {
                        MatrixXd tV = cubeV;
                        Vector3d trans(ix * hx, iy * hy, iz * hz);
                        for(int id = 0; id < 8; id++)
                            tV.row(id) += trans;
                        MatrixXi tF = cubeF;
                        for(int id = 0; id < 12; id++)
                        {
                            for(int jd = 0; jd < 3; jd++)
                                tF(id, jd) = cubeF(id, jd) - 1 + 8 * prev_voxel;
                        }
                        V.block(8 * prev_voxel, 0, 8, 3) = tV;
                        F.block(12 * prev_voxel, 0, 12, 3) = tF;
                        prev_voxel ++;
                    }
                }
            }
        }

        C = MatrixXd(nvoxels_ * 12, 3);
        if(is_show_extra_value_color && voxel_value_) add_color_by_accessibility(C);
        else add_color_by_groupID(C);

        return;
    }


    void VoxelizedRenderCube::add_color_by_groupID(MatrixXd &C)
    {
        compute_parts_num();
        colorcoder_->request(part_num_);

        int id = 0;
        for(int i = 0; i < Nx; i++) {
            for (int j = 0; j < Ny; j++) {
                for (int k = 0; k < Nz; k++) {
                    if ((*voxel_)[i][j][k] != -1) {
                        for(int kd = 0; kd < 12; kd++)
                            C.row(id ++) = colorcoder_->get((*voxel_)[i][j][k]);
                    }
                }
            }
        }
    }

    void VoxelizedRenderCube::add_color_by_accessibility(MatrixXd &C)
    {
        double maxV = 0;
        double minV = std::numeric_limits<double>::max();
        //get maximum color;
        for(int i = 0; i < Nx; i++) {
            for (int j = 0; j < Ny; j++) {
                for (int k = 0; k < Nz; k++) {
                    if((*voxel_value_)[i][j][k] > maxV)
                    {
                        maxV = (*voxel_value_)[i][j][k];
                    }
                    if((*voxel_value_)[i][j][k] > 0 && minV > (*voxel_value_)[i][j][k])
                        minV = (*voxel_value_)[i][j][k];
                }
            }
        }

        Eigen::VectorXd value(nvoxels_);

        int pid = 0;
        for(int i = 0; i < Nx; i++) {
            for (int j = 0; j < Ny; j++) {
                for (int k = 0; k < Nz; k++) {
                    if((*voxel_value_)[i][j][k] > 0)
                    {
                        value(pid++) = (*voxel_value_)[i][j][k];
                    }
                }
            }
        }

        MatrixXd Color;
        colorcoder_->jet(value, minV, maxV, Color);
        Color *= 256;

        for(int id = 0; id < nvoxels_; id++)
        {
            for(int jd = 0; jd < 12; jd++)
            {
                C.row(id * 12 + jd) = Color.row(id);
            }
        }
        return;
    }
}
