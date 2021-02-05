//
// Created by *** on 22.03.18.
//

#include "VoxelizedPuzzleTree.h"
#include "../InterlockSDChecking.h"
#include <cmath>
bool VoxelizedPuzzleTree::create_children(VPTreeNode *node)
{
    std::cout << std::endl << std::endl;
    std::cout << "Begin to create children..." << std::endl;
    std::shared_ptr<Timer> tot_timer = std::make_shared<Timer>(           Timer("Total time    "));
    std::shared_ptr<Timer> auxiliary_data_timer = std::make_shared<Timer>(Timer("Auxiliary data"));
    std::shared_ptr<Timer> main_partition_timer = std::make_shared<Timer>(Timer("Main partition"));
    std::shared_ptr<Timer> sort_candidate_timer = std::make_shared<Timer>(Timer("Sort candidate"));
    partioner_timer = std::make_shared<Timer>(Timer("Partioner"));
    disassembly_timer = std::make_shared<Timer>(Timer("Disassembly"));

    tot_timer->start();
    if(node->puzzle_->get_part_num() == 1)
    {
        create_key_part(node);
    }
    else
    {
        //compute auxiliary data
        auxiliary_data_timer->start();
        compute_node_auxiliary_data(node);
        auxiliary_data_timer->end();

        //filter remaining volume partition plan
        filter_remaining_volume_partition_plan(node);

        //decided the size of candidate puzzle
        int num_requried_children = 0;
        {
            //for bunny
//            if(node->part_num < (int)(num_parts_required_ * 0.15)) num_requried_children = 50;
//            else if(node->part_num > (int)(num_parts_required_ * 0.9)) num_requried_children = 50;
//            else num_requried_children = 5;

            //for cube
            num_requried_children = 20;
        }

        //main program
        main_partition_timer->start();
        main_partition_remaining_volume_program(node, num_requried_children);
        main_partition_timer->end();

        sort_candidate_timer->start();
        sort_candidate_puzzles(node);
        sort_candidate_timer->end();
    }
    tot_timer->end();

    std::cout << "*****************Statistics*****************" << std::endl;
    std::cout << "Part number    :\t\t" << node->part_num << std::endl;
    std::cout << "Children number:\t\t" << node->child.size() << std::endl;
    auxiliary_data_timer->print();
    main_partition_timer->print();
    sort_candidate_timer->print();
    partioner_timer->print();
    disassembly_timer->print();
    tot_timer->print();
    std::cout << "********************************************" << std::endl;
    if(node->child.empty())
        return false;
    return true;
}

void VoxelizedPuzzleTree::main_partition_remaining_volume_program(VPTreeNode *node, int num_requried_children)
{


    auto print_percentage = [&](int times)
    {
        printf("\r");
        printf("finish %.2f %%", (double)times * 100.0 / num_requried_children);
    };

    print_percentage(0);
    int run_times = 0;
    for(VPuzRemainVolumePartitionDat plan : *node->graph_partition_plan)
    {
        if(node->child.size() > num_requried_children) break;

        //get voxel range
        int part_num_voxels = each_part_num_voxels_[node->part_num];

        //set up partioner
        partioner_timer->start();
        VoxelizedPartition partioner(part_num_voxels, part_num_voxels);
        partioner.input(node->puzzle_.get(), plan);

        //get remaining_volume_voxel_lists;
        vector<vector<pEmt>> remaining_volume_voxel_lists;
        partioner.output(remaining_volume_voxel_lists);
        partioner_timer->end();

        //sort
//        std::sort(remaining_volume_voxel_lists.begin(), remaining_volume_voxel_lists.end(), [&](vector<pEmt> &A, vector<pEmt> &B)
//        {
//            double access_a = 0, access_b = 0;
//            for(pEmt v : A) access_a += node->puzzle_->get_accessibility(v->order_);
//            for(pEmt v : B) access_b += node->puzzle_->get_accessibility(v->order_);
//            return access_a > access_b;
//        });

        //check each candidate
        int maximum_candidate_num = 0;
        for(int jd = 0; jd < remaining_volume_voxel_lists.size(); jd++)
        {
            run_times ++;
            if(run_times > 1000) return;
            //have multistep disassembly
            disassembly_timer->start();
            VoxelizedPuzzleDisassembly disassembly;
            disassembly.set_puzzle(node->puzzle_.get(), remaining_volume_voxel_lists[jd]);
            vector<string> sequences;
            if(!disassembly.bfs_check_disassembly(sequences))
                continue;
            disassembly_timer->end();

            //final accepted

            //create node
            std::shared_ptr<VPTreeNode> child_node = std::make_shared<VPTreeNode>();
            std::shared_ptr<VoxelizedPuzzle> child_puzzle = std::make_shared<VoxelizedPuzzle>(VoxelizedPuzzle(*node->puzzle_));
            pPart part = child_puzzle->parts_.back().get();
            child_puzzle->partition_part(part, remaining_volume_voxel_lists[jd]);

            //check duplicate
            if(!is_puzzle_duplicate(child_puzzle.get(), node))
            {
                //push child
                child_node->puzzle_ = child_puzzle;
                child_node->parent = node;
                child_node->part_num = node->part_num + 1;
                child_node->brother = nullptr;
                child_node->sequences = sequences;
                node->child.push_back(child_node);

                //statistic
                maximum_candidate_num ++;
                print_percentage(node->child.size());
                if(maximum_candidate_num > 5)
                    break;
                if(node->child.size() > num_requried_children) break;
            }
        }
    }

    std::cout << std::endl;
}

void VoxelizedPuzzleTree::sort_candidate_puzzles(VPTreeNode *node) {

    std::sort(node->child.begin(), node->child.end(), [](pVPTNode A, pVPTNode B)
    {
        return A->puzzle_->parts_.back()->sum_access_value_ > B->puzzle_->parts_.back()->sum_access_value_;
    });

    if(node->child.size() > 0)
    {
        for(int id = 0; id < node->child.size() - 1; id++)
        {
            node->child[id]->brother = node->child[id + 1].get();
        }
        node->child.back()->brother = nullptr;

    }
    return;
}

void VoxelizedPuzzleTree::create_key_part(VPTreeNode *node)
{

    node->child.clear();

    //Y+
    VoxelizedPuzzle *puzzle = node->puzzle_.get();
    VPuzRemainVolumePartitionDat plan;

    //plan.relation = 0b110111; //bunny
    plan.relation = 0b110111;

    pEmt top_voxel = node->puzzle_->voxel_.front().get();
    for(shared_pEmt voxel : node->puzzle_->voxel_)
    {
        //if(voxel->pos_[0] + -voxel->pos_[2] + voxel->pos_[1] > top_voxel->pos_[0] - top_voxel->pos_[2] + top_voxel->pos_[1])
        if(voxel->pos_[2] + voxel->pos_[1] > top_voxel->pos_[2] + top_voxel->pos_[1])
            top_voxel = voxel.get();
    }

    plan.groupA.clear();
    plan.groupA.push_back(top_voxel);
    vector<vector<pEmt>> remaining_volume_voxel_lists;
    //remaining_volume_voxel_lists.push_back(plan.groupA);

    int part_num_voxels = each_part_num_voxels_[node->part_num];
    VoxelizedPartition partioner(part_num_voxels, part_num_voxels);
    partioner.maximum_inner_partition_plan_ = 1000;

    partioner.input(node->puzzle_.get(), plan);

    partioner.output(remaining_volume_voxel_lists);
    for(int id = 0; id < remaining_volume_voxel_lists.size(); id ++)
    {
        std::shared_ptr<VPTreeNode> child_node = std::make_shared<VPTreeNode>();
        std::shared_ptr<VoxelizedPuzzle> child_puzzle = std::make_shared<VoxelizedPuzzle>(VoxelizedPuzzle(*node->puzzle_));
        pPart part = child_puzzle->parts_.back().get();
        child_puzzle->partition_part(part, remaining_volume_voxel_lists[id]);

        child_node->puzzle_ = child_puzzle;
        child_node->parent = node;
        child_node->part_num = node->part_num + 1;
        child_node->brother = nullptr;
        node->child.push_back(child_node);
    }

    std::sort(node->child.begin(), node->child.end(), [](pVPTNode A, pVPTNode B)
    {
            pPart partA = A->puzzle_->parts_.back().get();
            pPart partB = B->puzzle_->parts_.back().get();
            int YA = 0, YB = 0;
            for(pEmt em : partA->elist_.data_) YA += em->pos_[1];
            for(pEmt em : partB->elist_.data_) YB += em->pos_[1];
            return YA < YB;
    });

    if(node->child.size() > 0)
    {
        for(int id = 0; id < node->child.size() - 1; id++)
        {
            node->child[id]->brother = node->child[id + 1].get();
        }
        node->child.back()->brother = nullptr;
    }
}

void VoxelizedPuzzleTree::compute_node_auxiliary_data(VPTreeNode *node)
{
    //allocate memory the puzzle graph
    node->graph_ = std::make_shared<VPuzzleGraph>(VPuzzleGraph());

    //add parts
    vector<pPart> part_list;
    int num_parts = node->puzzle_->get_part_num();
    for (int id = 0; id < num_parts; id++) {
        part_list.push_back(node->puzzle_->parts_[id].get());
    }

    //set parts
    node->graph_->set_parts(part_list);
    node->graph_->set_size(Vector3i(node->puzzle_->Nx, node->puzzle_->Ny, node->puzzle_->Nz));
    node->part_num = num_parts;

    //clear child
    node->child.clear();

    //generate partition plan
    node->graph_partition_plan = std::make_shared<vector<VPuzRemainVolumePartitionDat>>();
    node->graph_->do_separation(*node->graph_partition_plan);

    return;
}

void VoxelizedPuzzleTree::set_traget_part(int num_parts, int tot_voxel)
{
    each_part_num_voxels_.clear();
    num_parts_required_ = num_parts;
    for(int id = 0; id < num_parts; id++)
    {
        each_part_num_voxels_.push_back(tot_voxel/num_parts);
    }
    int remain_voxel = tot_voxel % num_parts;
    for(int id = num_parts - remain_voxel; id < num_parts; id++)
        each_part_num_voxels_[id] ++;
}

void VoxelizedPuzzleTree::set_root(std::shared_ptr<VoxelizedPuzzle> puzzle)
{
    pVPTNode root = std::make_shared<VPTreeNode>(VPTreeNode());
    root->puzzle_ = puzzle;
    root->parent = nullptr;
    root->graph_ = nullptr;
    root->brother = nullptr;
    root->part_num = puzzle->get_part_num();
    nodes_.push_back(root);
}

bool VoxelizedPuzzleTree::is_puzzle_duplicate(VoxelizedPuzzle *child_puzzle, VPTreeNode *node) {
    for(int kd = 0; kd < node->child.size(); kd++)
    {
        if(child_puzzle->is_same(node->child[kd]->puzzle_.get()))
        {
            return true;
        }
    }
    return false;
}

void VoxelizedPuzzleTree::filter_remaining_volume_partition_plan(VPTreeNode *node) {

}



