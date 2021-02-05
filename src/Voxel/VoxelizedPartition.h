//
// Created by *** on 06.04.18.
//

#ifndef UNDERSTAND_INTERLOCK_VOXELIZEDPARTITION_H
#define UNDERSTAND_INTERLOCK_VOXELIZEDPARTITION_H

#include "VoxelizedPuzzle.h"
#include "VPuzzleGraph.h"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <stdlib.h>
#include <iostream>
#include <map>

struct VoxelBFSNode
{
public:

    VoxelElement *em;
    double weight;
};

struct VPuzInnerPartitionDat
{
    pEmt anchor;
    vector<pEmt> fixs;
};


class VoxelizedPartition {
public:
    VoxelizedPartition(int minimum, int maximum);
public:

    void input(VoxelizedPuzzle *puzzle, VPuzRemainVolumePartitionDat &outside);

    void output(vector<vector<pEmt>>  &remain_volume_voxel_lists);

public:

    void find_inner_partition(int relation);

    void merge_inside_outside_partition_dat(VPuzInnerPartitionDat &inner_dat, VPuzRemainVolumePartitionDat &full_partition_dat);

    bool connect_group_A_anchor_voxels(VPuzRemainVolumePartitionDat &full_partition_dat);

    bool maintain_disassemblability(int relation, VPuzRemainVolumePartitionDat &full_partition_dat, int &disassembled_direction);

    bool expand_group_A_to_minimum_pieces(VPuzRemainVolumePartitionDat &full_partition_dat, int &disassembled_direction);

    bool compute_remaining_volume(vector<pEmt> &group_B);

    bool is_remaining_volume_connected(const vector<pEmt> &vlist);

public:

    bool insert_into_group_A(pEmt em);

    bool is_in_group_A(pEmt em);

private:

    VoxelizedPuzzle *puzzle_;

    VPuzRemainVolumePartitionDat outside_partition_dat_;

    vector<VPuzInnerPartitionDat> inside_partition_dats_;

public:

    std::unordered_map<int, bool> in_group_A_;

    vector<pEmt> group_A_;

    int minimum_voxel_in_one_piece;

    int maximum_voxel_in_one_piece;

    int maximum_inner_partition_plan_;

    int maximum_final_plan;

    int maximum_failure_time_;
};


#endif //UNDERSTAND_INTERLOCK_VOXELIZEDPARTITION_H
