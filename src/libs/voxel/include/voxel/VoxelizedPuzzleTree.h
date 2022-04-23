//
// Created by *** on 22.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_VOXELIZEDPUZZLETREE_H
#define UNDERSTAND_INTERLOCK_VOXELIZEDPUZZLETREE_H

#include "VoxelizedPuzzle.h"
#include "VPuzzleGraph.h"
#include <queue>
#include <stdlib.h>
#include "VoxelizedPartition.h"
#include "VoxelizedPuzzleDisassembly.h"
//#include "VoxelizedPartitionExhaust.h"

namespace voxel{
    struct VPTreeNode
    {

    public:
        //key data

        std::shared_ptr<VoxelizedPuzzle> puzzle_;

        VPTreeNode *parent;

        VPTreeNode *brother;

        int part_num;

        std::vector<std::shared_ptr<VPTreeNode>> child;

        std::vector<std::string> sequences;

    public:
        //Auxiliary data

        std::shared_ptr<VPuzzleGraph> graph_;

        std::shared_ptr<std::vector<VPuzRemainVolumePartitionDat>> graph_partition_plan;

        void clear_auxiliary()
        {
            graph_.reset();
            graph_partition_plan.reset();
        }
    };

    typedef std::shared_ptr<VPTreeNode> pVPTNode;


    class Timer
    {
    public:

        Timer(std::string name) {name_ = name;clear();}

    public:

        void start(){timer_ = clock();}

        void end(){tot_time += (double)(clock() - timer_)/(CLOCKS_PER_SEC);}

        void clear(){tot_time = 0;}

        double present_time(){return (double)(clock() - timer_)/(CLOCKS_PER_SEC);}

        void print(){std::cout << name_ << ":\t\t" << tot_time << std::endl;}

    public:
        std::string name_;
        clock_t timer_;
        double tot_time;
    };

    class VoxelizedPuzzleTree
    {
    public:

        typedef VoxelizedPart* pPart;
        typedef Eigen::Vector3i Vector3i;
        typedef VoxelElement* pEmt;
        typedef std::shared_ptr<VoxelElement> shared_pEmt;

    public:

        VoxelizedPuzzleTree(int num_parts, int tot_voxel) { set_traget_part(num_parts, tot_voxel); }

    public:

        void set_traget_part(int num_parts, int tot_voxel);

        void set_root(std::shared_ptr<VoxelizedPuzzle> puzzle);

        inline VPTreeNode *root(){ return nodes_.front().get(); }

    public:

        bool create_children(VPTreeNode *node);

        void compute_node_auxiliary_data(VPTreeNode *node);

        void create_key_part(VPTreeNode *node);

        void filter_remaining_volume_partition_plan(VPTreeNode *node);

        void main_partition_remaining_volume_program(VPTreeNode *node, int num_requried_children);

        void sort_candidate_puzzles(VPTreeNode *node);

    public:

        bool is_puzzle_duplicate(VoxelizedPuzzle *child_puzzle, VPTreeNode *node);

    public:

        std::vector<pVPTNode> nodes_;

        VPTreeNode* present_nodes_;

    public:

        int num_parts_required_;

        std::vector<int> each_part_num_voxels_;

        std::shared_ptr<Timer> partioner_timer;
        std::shared_ptr<Timer> disassembly_timer;
    };
}



#endif //UNDERSTAND_INTERLOCK_VOXELIZEDPUZZLETREE_H
