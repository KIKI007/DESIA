//
// Created by *** on 16.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_VOXELELEMENT_H
#define UNDERSTAND_INTERLOCK_VOXELELEMENT_H
#include <Eigen/Dense>
#include <vector>

namespace voxel{
    class VoxelElement
    {
    public:

        typedef Eigen::Vector3i Vector3i;

    public:

        VoxelElement(const VoxelElement &A)
        {
            pos_ = A.pos_;
            order_ = A.order_;
            gid_ = A.gid_;
            tmp_ = A.tmp_;
        }

        VoxelElement(Vector3i pos, int order)
        {
            pos_ = pos;
            order_ = order;
            tmp_ = 0;
            gid_ = -1;
        }

    public:


        Vector3i pos_;// save location in (x,y,z)

        int order_; //Nx * Ny * pos[2] + Nx * pos[1] + pos[0], same as pos_ only for convenience

        int gid_; //belong to which group

        double tmp_; //for temporarily saving data;
    };

//For storing voxel in order
//could speed up operations during searching
    typedef std::shared_ptr<VoxelElement> shared_pEmt;
    typedef VoxelElement* pEmt;
    using std::vector;
    class OrderedVElemList
    {

    public:

        //push assuming the order
        void force_push(pEmt voxel)
        {
            data_.push_back(voxel);
        }

        //insert element into right position by using binary sort;
        void push(pEmt voxel)
        {

            vector<pEmt>::iterator find_up_it = std::upper_bound(data_.begin(), data_.end(), voxel, [&](const pEmt &a, const pEmt &b)
            {
                return a->order_ < b->order_;
            });

            if(find_up_it == data_.end())
            {
                data_.push_back(voxel);
            }
            else
            {
                int index = find_up_it - data_.begin();
                data_.push_back(nullptr);
                for(int id = data_.size() - 1; id > index; id--)
                {
                    data_[id] = data_[id - 1];
                }
                data_[index] = voxel;
            }
            return;
        }

    public:
        vector<pEmt> data_;
    };
}



#endif //UNDERSTAND_INTERLOCK_VOXELELEMENT_H
