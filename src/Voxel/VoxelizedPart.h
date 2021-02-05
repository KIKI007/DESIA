//
// Created by *** on 16.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_VOXELIZEDPART_H
#define UNDERSTAND_INTERLOCK_VOXELIZEDPART_H

#include "VoxelElement.h"
#include <memory>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cmath>

class VoxelizedPart;
typedef VoxelizedPart* pPart;

using std::vector;
using std::unordered_map;

/********************************************/
/*             VPartNeighbor                */
/********************************************/

class VoxelizedPart;
class VPartNeighbor
{
public:
    VPartNeighbor(){}
public:

    vector<pPart> in_;

    vector<pPart> out_;

    vector<std::shared_ptr<OrderedVElemList>> in_blocking_;

    vector<std::shared_ptr<OrderedVElemList>> out_blocking_;

    void clear()
    {
        in_.clear();
        out_.clear();
        in_blocking_.clear();
        out_blocking_.clear();
    }
};


class VoxelizedPart {
public:

    VoxelizedPart(const OrderedVElemList &part_voxels_list,  // all voxel in this part
                  unordered_map<int, pEmt> *map_ip,          // map from (x,y,z) to VoxelElements *
                  int part_id,
                  Vector3i size);                            // size = (Nx, Ny, Nz)

    VoxelizedPart(const VoxelizedPart &a);                   // copy and construction

public:

    // the puzzle define the map_access,
    // and here only for filling the voxel's accessibility which belong to this part
    void compute_access_map(unordered_map<int, double> *map_access);

    //compute the accessibility according to Peng 2012 paper.
    void compute_access_map();

    void remove_voxels();

    double get_maximum_access_value(){ return maximum_access_value_;}

    double get_accessibilty(pEmt p);

    //check whether p in this part
    bool in_part(pEmt p);

    pEmt in_part(Vector3i pos);

public:

    //nrm = 000001b, +X
    //nrm = 000010b, -X
    //...
    //nrm = 100000b, -Z
    //if in this direction, there is no neighbor, return empty;
    pEmt neighbor(int nrm, pEmt p);

    int compute_num_neighbor(pEmt p);

    int V2I(Vector3i pos)
    {
        if(pos[0] < 0 || pos[1] < 0 || pos[2] < 0) return -1;
        if(pos[0] >= size_[0] || pos[1] >= size_[1] || pos[2] >= size_[2]) return -1;
        return pos[2] * size_[0] * size_[1] + pos[1] * size_[0] + pos[0];
    }

public:

    OrderedVElemList elist_;

    Vector3i size_;

    int part_id_;

public:

    unordered_map<int, pEmt> *map_ip_;

    unordered_map<int, double> *map_access_;

    VPartNeighbor neighbor_[3];

    double maximum_access_value_;

    double sum_access_value_;
};

#endif //UNDERSTAND_INTERLOCK_VOXELIZEDPART_H
