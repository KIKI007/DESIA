//
// Created by *** on 17.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_VOXELIZEDRENDERCUBE_H
#define UNDERSTAND_INTERLOCK_VOXELIZEDRENDERCUBE_H

#include "VoxelizedInterface.h"

namespace voxel{
    class VoxelizedRenderCube : public VoxelizedInterface {

    public:
        VoxelizedRenderCube(int nx, int ny, int nz, std::shared_ptr<ColorCoding> coder): VoxelizedInterface(nx, ny, nz, coder)
        {
            hx = hy = hz = 1;
        }

    public:

        VoxelizedRenderCube(const VoxelizedInterface &interface): VoxelizedInterface(interface)
        {
            hx = hy = hz = 1;
        }

    public:

        void rendering(MatrixXd &V, MatrixXi &F, MatrixXd &C);

        void add_color_by_groupID(MatrixXd &C);

        void add_color_by_accessibility(MatrixXd &C);

    public:

        double hx, hy, hz;
    };
}


#endif //UNDERSTAND_INTERLOCK_VOXELIZEDRENDERCUBE_H
