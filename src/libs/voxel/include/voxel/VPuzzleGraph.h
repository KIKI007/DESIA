//
// Created by *** on 19.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_VPUZZLEGRAPH_H
#define UNDERSTAND_INTERLOCK_VPUZZLEGRAPH_H

#include "VoxelizedPart.h"
#include "PuzzleCandidateFilter.h"

#include <iostream>
#include "stdlib.h"
#include <queue>
#include <stack>
#include <map>
#include <unordered_map>

//#define VoxelPartitionSearchExhaustion
#define VoxelPartitiomSearchRandom

namespace voxel
{
    class VPuzzleGraph {
    public:

        typedef Eigen::MatrixXi MatrixXi;

        typedef VoxelizedPart* pPart;

    public:

        VPuzzleGraph(){
            max_double_loops_partition_plan = 10000;
            max_single_loop_partition_plan = 10000;
            max_joint_combination = 10000;
            max_voxel_combination = 10;
            max_failure_time = 10;
        }

    public:

        void set_parts(const vector<pPart> &parts) {parts_ = parts; }

        void set_size(Eigen::Vector3i size){size_ = size;}

    public:

        void compute_distance_graph(int XYZ);

        void compute_one_loop_candidate_voxels(int XYZ, VPuzFilterCycle &filter);

        void compute_two_loops_candidate_voxels(int XYZ, VPuzFilterCycle &filter);

        void compute_combination_of_cycleXYZ(vector<VPuzCycleXYZDat> &plans_cycleXYZ);

        void compute_remain_volume_partition_plans(vector<VPuzRemainVolumePartitionDat> &voxel_partition_plan,
                                                   const vector<VPuzCycleXYZDat> &combination_graph_partition_plan);

        void do_separation(vector<VPuzRemainVolumePartitionDat> &output);

    public:

        bool random_compute_voxel_partition_plan(VPuzRemainVolumePartitionDat &plan,
                                                 vector<vector<VPuzFE<VoxelElement *>>> *gr,
                                                 vector<int> &require_blocking);

        void random_pick_voxels_in_a_group(vector<vector<VPuzFE<VoxelElement *>>> &gr,
                                           vector<VoxelElement *> &gvoxel);

    public:

        void exhaust_compute_voxel_partition_plan(vector<VPuzRemainVolumePartitionDat> &plans,
                                                  vector<vector<VPuzFE<VoxelElement *>>> *gr,
                                                  int relation);

        void dfs_compute_voxel_partition_plan(vector<VPuzRemainVolumePartitionDat> &plans,
                                              vector<vector<VPuzFE<VoxelElement *>>> *gr,
                                              int relation);

    public:

        void remove_duplicate(vector<VoxelElement *> &gvoxel, vector<VoxelElement *> &group);

    public:

        vector<pPart> parts_;

        MatrixXi dist_graph[3];

        VPuzFilterCycle graph_partition_plan[6];

        Eigen::Vector3i size_;

    private:

        int max_double_loops_partition_plan;

        int max_single_loop_partition_plan;

        int max_joint_combination;

        int max_voxel_combination;

        int max_failure_time;
    };
}
#endif //UNDERSTAND_INTERLOCK_VPUZZLEGRAPH_H
