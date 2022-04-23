//
// Created by *** on 15.03.18.
//

#include "voxel/VoxelRandom.h"
#include <iostream>
namespace voxel
{
    VoxelRandom::VoxelRandom(int nx, int ny, int nz, std::shared_ptr<ColorCoding> coder)
    {
        Nx = nx;
        Ny = ny;
        Nz = nz;
        colorcoder_ = coder;
    }

    std::shared_ptr<VoxelizedInterface> VoxelRandom::BFS_floodfill(std::shared_ptr<array_3b> voxel, int part_num, int voxels_num)
    {
        assembly_.reset();
        assembly_ = std::make_shared<VoxelizedInterface>(VoxelizedInterface(Nx, Ny, Nz, colorcoder_));

        random_seeds(voxel, part_num);
        std::queue<Eigen::Vector3i> queue;
        int num_allocate_voxel = 0;
        std::vector<int> group_num;
        for(int id = 0; id < part_num; id++)
        {
            queue.push(seeds_[id]);
            assembly_->set_grid_part_index(seeds_[id], id);
            num_allocate_voxel ++;
            group_num.push_back(1);
        }

        int dX[6] = {1, -1,  0,  0, 0,  0};
        int dY[6] = {0,  0,  1, -1, 0,  0};
        int dZ[6] = {0,  0,  0,  0, 1, -1};
        while(num_allocate_voxel <= voxels_num && !queue.empty())
        {
            Eigen::Vector3i u = queue.front(); queue.pop();
            int IDu = assembly_->get_grid_part_index(u);
            if(group_num[IDu] > voxels_num / part_num + 2) continue;
            if(u == seeds_[0]) continue;

            std::vector<int> order_list;
            for(int id = 0; id < 6; id++)
            {
                order_list.push_back(id);
            }
            for(int id = 5; id > 0; id--)
            {
                int rnd = rand() % (id + 1);
                int tmp = order_list[id];
                order_list[id] = order_list[rnd];
                order_list[rnd] = tmp;
            }


            for(int id = 0; id < 6; id++)
            {
                int niX = u[0] + dX[order_list[id]];
                int niY = u[1] + dY[order_list[id]];
                int niZ = u[2] + dZ[order_list[id]];
                Eigen::Vector3i v(niX, niY, niZ);
                if(niX < 0 || niY < 0 || niZ < 0 || niX >= Nx || niY >= Ny || niZ >= Nz)
                {
                    continue;
                }
                if((*voxel)[niX][niY][niZ])
                {
                    int IDv = assembly_->get_grid_part_index(v);
                    if(IDv == -1)
                    {
                        assembly_->set_grid_part_index(v, IDu);
                        queue.push(v);
                        num_allocate_voxel ++;
                        group_num[IDu]++;
                    }
                }

            }
        }

        std::cout << num_allocate_voxel << std::endl;

        return assembly_;
    }

    void VoxelRandom::random_seeds(std::shared_ptr<array_3b> voxel, int part_num)
    {
        Vector3i pos;
        seeds_.push_back(Vector3i(1, 1, 1));
        for (int id = 1; id < part_num; id++) {
            while (true) {
                pos = Vector3i(rand() % Nx, rand() % Ny, rand() % Nz);
                if(!(*voxel)[pos[0]][pos[1]][pos[2]]) continue;
                int jd;
                for(jd = 0; jd < id; jd++)
                {
//                Vector3i dvec = pos - seeds_[jd];
//                double dist =std::sqrt((double)dvec[0]*dvec[0] + dvec[1]*dvec[1] + dvec[2]*dvec[2]);
//                std::cout << id << "th\t" << dist << "\t" << min_dist << std::endl;
                    if(pos == seeds_[jd])
                        break;
                }
                if(jd == id)
                    break;
            }
            seeds_.push_back(pos);
        }
    }

}
