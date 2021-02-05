//
// Created by *** on 22.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_PUZZLECANDIDATEFILTER_H
#define UNDERSTAND_INTERLOCK_PUZZLECANDIDATEFILTER_H
#include "VoxelElement.h"
#include "algorithm"

enum VPuzConnType
{
    PuzzleConnection_IOIO = 0,
    PuzzleConnection_IN_OUT = 1,
    PuzzleConnection_OUT_IN = 2,
    PuzzleConnection_NULL = 3,
};

template <typename ExtraData>
struct VPuzFilterElement
{
    int weight;

    int rand;

    ExtraData data_;
};

struct VPuzCycleDat
{
    //type 0: A.A1/A.A2    B.B1/B.B2
    // type 1: A.A1      -> B.B1
    // type 2: A.A1      <- B.B1
    // type 3: A          = B
    vector<OrderedVElemList *> anchor_voxels_list;

    VPuzConnType type;
};
template <typename ExtraData>
using VPuzFE =  VPuzFilterElement<ExtraData>;
typedef VPuzFilterElement<VPuzCycleDat> VPuzFECycle;

struct VPuzCycleXYZDat
{
    //cycle_XYZ[XYZ] save all possible voxel in each direction that could create cycles with previous part
    VPuzFECycle cycle_XYZ[3];

    //6 digits binary number
    //00 00 00
    //Z  Y  X
    //00 means no joint between two
    //01 means A - > B
    //10 means B - > A
    //11 means A  =  B
    int relation;
};
typedef VPuzFilterElement<VPuzCycleXYZDat> VPuzFECycleXYZ;

struct VPuzRemainVolumePartitionDat
{
    vector<pEmt> groupA; // final distribution of voxel in group A

    vector<pEmt> groupB; // final distribution of voxel in group B

    int relation; //6 digits binary number
    //00 00 00
    //Z  Y  X
    //00 means no joint between two
    //01 means A - > B
    //10 means B - > A
    //11 means A  =  B
};
typedef VPuzFilterElement<VPuzRemainVolumePartitionDat> VPuzFERemainVolumePartition;

template <typename ExtraData>
class VPuzFilter
{
public:

    VPuzFilter(int max_candidates_num = std::numeric_limits<int>::max())
    {
        max_candidates_num_ = max_candidates_num;
        base_weight_ = 5;
    }

public:

    void insert(const vector<VPuzFE<ExtraData>> &list)
    {
        if(list.size() <= max_candidates_num_)
        {
            candidate_ = list;
        }
        else
        {
            candidate_.clear();

            candidate_ = list;

            compute_rand_according_to_weight(candidate_);

            keep_K_elements(candidate_, max_candidates_num_);
        }

        return;
    }

    void random_choose(vector<VPuzFE<ExtraData>> &output, int num_pick)
    {
        if(num_pick >= candidate_.size())
        {
            output = candidate_;
            return;;
        }
        else
        {
            output = candidate_;
            keep_K_elements(output, num_pick);
            return;
        }
    }

    VPuzFE<ExtraData> pick()
    {
        if(candidate_.size() < 1)
            return VPuzFE<ExtraData>();
        else
        {
            vector<VPuzFE<ExtraData>> output;
            random_choose(output, 1);
            return output.front();
        }
    }

protected:

    void compute_rand_according_to_weight(vector<VPuzFE<ExtraData>> &list)
    {
        int minimum_weight = list.front().weight;
        for(VPuzFE<ExtraData> &em : list)
        {
            if(minimum_weight > em.weight)
            {
                minimum_weight = em.weight;
            }
        }

        for(VPuzFE<ExtraData> &em: list)
        {
            if(minimum_weight < base_weight_)
                em.weight = em.weight - minimum_weight + base_weight_;
            em.rand = rand() % em.weight;
        }
    }

    void keep_K_elements(vector<VPuzFE <ExtraData>> &list, int num_keep)
    {
        vector<VPuzFE<ExtraData>> heap;
        for(int id = 0; id < num_keep; id++)
        {
            heap.push_back(list[id]);
        }
        std::make_heap(heap.begin(), heap.end(), [](const VPuzFE<ExtraData> &A, const VPuzFE<ExtraData>&B)
        {
            return A.rand > B.rand;
        });

        for(int id = num_keep; id < list.size(); id++)
        {
            if(heap.front().rand < list[id].rand)
            {
                heap.push_back(list[id]);
                std::pop_heap(heap.begin(), heap.end(),
                              [](const VPuzFE<ExtraData> &A, const VPuzFE<ExtraData> &B) {
                                  return A.rand > B.rand;
                              });
                heap.pop_back();
            }
        }

        list = heap;
    }

public:

    vector<VPuzFE<ExtraData>> candidate_;

    int max_candidates_num_;

    int base_weight_;

private:
};


typedef VPuzFilter<VPuzCycleDat> VPuzFilterCycle;
typedef VPuzFilter<VPuzCycleXYZDat> VPuzFilterCycleXYZ;
typedef VPuzFilter<VPuzRemainVolumePartitionDat> VPuzFilterRemainVolumePartition;

#endif //UNDERSTAND_INTERLOCK_PUZZLECANDIDATEFILTER_H
