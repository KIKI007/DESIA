//
// Created by *** on 15.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_VOXELRANDOM_H
#define UNDERSTAND_INTERLOCK_VOXELRANDOM_H

#include "VoxelizedInterface.h"
#include "MeshVoxelizer.h"

namespace voxel{
    class VoxelRandom{
    public:

        typedef boost::multi_array<bool, 3> array_3b;

        typedef std::vector<Eigen::Vector3i> vecVector3i;

        typedef Eigen::Vector3i Vector3i;

    public:

        VoxelRandom(int nx, int ny, int nz, std::shared_ptr<ColorCoding> coder);

    public:

        std::shared_ptr<VoxelizedInterface> BFS_floodfill(std::shared_ptr<array_3b> voxel, int part_num, int nvoxels);

        void random_seeds(std::shared_ptr<array_3b> voxel, int part_num);

    private:

        vecVector3i seeds_;

        std::shared_ptr<VoxelizedInterface> assembly_;

    private:
        int Nx, Ny, Nz;

        std::shared_ptr<ColorCoding> colorcoder_;
    };

}


#endif //UNDERSTAND_INTERLOCK_VOXELRANDOM_H
