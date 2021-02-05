//
// Created by *** on 01.03.18.
//

#ifndef TI_STABLE_PART_H
#define TI_STABLE_PART_H


#include <memory>
#include <Eigen/Dense>
#include <vector>

using std::vector;
using Eigen::Vector3d;
typedef vector<Vector3d,Eigen::aligned_allocator<Vector3d> > vecVector3d;

//Part
    //PartNeighborData save the contact information with its neighbor

class Part;
struct PartNeighborData
{
    std::weak_ptr<Part> node;
    vecVector3d normals;
};

class Part {
public:
    Part(){index_ = -1;}

    Part(int id)
    {
        set_part_index(id);
    }

    ~Part(){}

public:
    void set_part_index(int id){index_ = id;}

    void add_neighbor(PartNeighborData data){neighborLists_.push_back(data);}

    bool valid(){return index_ != -1;}

public:

    vector<PartNeighborData> neighborLists_;

    int index_;

};


#endif //TI_STABLE_PART_H
