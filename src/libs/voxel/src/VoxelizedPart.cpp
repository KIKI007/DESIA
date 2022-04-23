//
// Created by *** on 16.03.18.
//

#include "voxel/VoxelizedPart.h"

namespace voxel{
    VoxelizedPart::VoxelizedPart(const OrderedVElemList &part_voxels_list,
                                 std::unordered_map<int, pEmt> *map_ip,
                                 int part_id,
                                 Vector3i size)
    {
        part_id_= part_id;
        elist_ = part_voxels_list;
        map_ip_ = map_ip;
        size_ = size;
        map_access_ = nullptr;
    }

    bool VoxelizedPart::in_part(pEmt p) {
        auto iter = (*map_ip_).find(V2I(p->pos_));
        if(iter == (*map_ip_).end() || iter->second->gid_ != part_id_) return false;
        else return true;
    }

    double VoxelizedPart::get_accessibilty(pEmt p)
    {
        if(!in_part(p)) return -1;
        return (*map_access_)[V2I(p->pos_)];
    }

    void VoxelizedPart::compute_access_map()
    {
        if(map_access_ == nullptr) return;

        std::vector<double> access_values;
        for(pEmt p: elist_.data_)
        {
            p->tmp_ = compute_num_neighbor(p);
        }

        access_values.resize(elist_.data_.size(), 0);
        for(int id = 1; id <= 3; id++)
        {
            for(int jd = 0; jd < elist_.data_.size(); jd++)
            {
                pEmt p = elist_.data_[jd];
                access_values[jd] = p->tmp_;

                //neighbor
                for(int kd = 0; kd < 6; kd++)
                {
                    pEmt q = neighbor((1 << kd), p);
                    if(q) access_values[jd] += std::pow(0.1, (double)id) * q->tmp_;
                }
            }

            for(int jd = 0; jd < elist_.data_.size(); jd++)
            {
                pEmt p = elist_.data_[jd];
                p->tmp_ = access_values[jd];
            }
        }

        double values;
        maximum_access_value_ = -1;
        sum_access_value_ = 0;
        for(pEmt p : elist_.data_)
        {
            values += p->tmp_;
            sum_access_value_ += p->tmp_;
            if(p->tmp_ > maximum_access_value_) maximum_access_value_ = p->tmp_;
            auto find_it = map_access_->find(p->order_);
            if(find_it == map_access_->end())
            {
                (*map_access_).insert(std::make_pair(V2I(p->pos_), p->tmp_));
            }
            else
            {
                (*map_access_)[p->order_] = p->tmp_;
            }
            p->tmp_ = 0;
        }
    }

    int VoxelizedPart::compute_num_neighbor(pEmt p) {

        int num_neighbor = 0;
        for(int id = 0; id < 6; id++)
        {
            pEmt q = neighbor((1<<id), p);
            if(q) num_neighbor++;
        }
        return num_neighbor;
    }

    void VoxelizedPart::compute_access_map(std::unordered_map<int, double> *map_access)
    {
        if(map_access != nullptr)
        {
            map_access_ = map_access;
            compute_access_map();
        }
    }

    VoxelizedPart::VoxelizedPart(const VoxelizedPart &a)
    {
        elist_ = a.elist_;
        size_ = a.size_;
        part_id_ = a.part_id_;
        map_ip_ = a.map_ip_;
        map_access_ = a.map_access_;
        maximum_access_value_ = a.maximum_access_value_;
        neighbor_[0] = a.neighbor_[0];
        neighbor_[1] = a.neighbor_[1];
        neighbor_[2] = a.neighbor_[2];
    }

    OrderedVElemList::pEmt VoxelizedPart::neighbor(int nrm, pEmt p) {
        Vector3i dX(0, 0, 0);
        switch(nrm)
        {
            case 1:
                dX = Vector3i(1, 0, 0);
                break;
            case 2:
                dX = Vector3i(-1, 0, 0);
                break;
            case 4:
                dX = Vector3i(0, 1, 0);
                break;
            case 8:
                dX = Vector3i(0, -1, 0);
                break;
            case 16:
                dX = Vector3i(0, 0, 1);
                break;
            case 32:
                dX = Vector3i(0, 0, -1);
                break;
            default:
                dX = Vector3i(0, 0, 0);
                break;
        }

        if(dX == Vector3i(0, 0, 0)) return nullptr;

        Vector3i nPos = p->pos_ + dX;
        if(nPos[0] < 0 || nPos[1] < 0 || nPos[2] < 0)
            return nullptr;

        if(nPos[0] >= size_[0] || nPos[1] >= size_[1] || nPos[2] >= size_[2])
            return nullptr;

        auto find_it = (*map_ip_).find(V2I(nPos));
        if(find_it != (*map_ip_).end() &&
           find_it->second->gid_ == part_id_)
        {
            return find_it->second;
        }
        else
        {
            return nullptr;
        }

    }

    OrderedVElemList::pEmt VoxelizedPart::in_part(Vector3i pos) {
        if(pos[0] < 0 || pos[1] < 0 || pos[2] < 0) return nullptr;
        if(pos[0] >= size_[0] || pos[1] >= size_[1] || pos[2] >= size_[2]) return nullptr;
        auto iter = (*map_ip_).find(V2I(pos));
        if(iter == (*map_ip_).end() || iter->second->gid_ != part_id_)
            return nullptr;
        else
            return iter->second;
    }

    void VoxelizedPart::remove_voxels() {
        OrderedVElemList nelist;
        for (pEmt voxel: elist_.data_) {
            if (voxel->gid_ == part_id_) {
                nelist.force_push(voxel);
            }
        }
        elist_ = nelist;
    }
}