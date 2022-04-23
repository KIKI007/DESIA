//
// Created by *** on 19.03.18.
//

#include "voxel/VPuzzleGraph.h"
#include <functional>
#include <cmath>

namespace voxel{
    void VPuzzleGraph::compute_distance_graph(int XYZ)
    {
        // This graph only contains all parts except the last one
        // Help to construct cycles in a graph
        // Graph[XYZ](id, jd) > 0 means that there is a path from id to jd in direction XYZ

        //XYZ = 0, X direction
        //XYZ = 1, Y direction
        //XYZ = 2, Z direction

        int num_part = parts_.size() - 1;

        //Initialized dist = -1 means not connected
        dist_graph[XYZ] = MatrixXi::Zero(num_part, num_part);
        for(int id = 0; id < num_part; id++)
        {
            for(int jd = 0; jd < num_part; jd++)
            {
                dist_graph[XYZ](id, jd) = -1;
            }
        }

        //Add the 1-ring neighbouring information
        //Only use out direction
        for(int id = 0; id < num_part; id++)
        {
            dist_graph[XYZ](id, id) = 0;
            for(VoxelizedPart* part : parts_[id]->neighbor_[XYZ].out_)
            {
                if(part->part_id_ != num_part)
                    dist_graph[XYZ](id, part->part_id_) = 1;
            }
        }

        //Floyd-Warshall
        for(int v = 0; v < num_part; v++)
        {
            for(int i = 0; i < num_part ;i++)
            {
                for(int j = 0; j < num_part; j++)
                {
                    if(i == v || j == v || i == j) continue;
                    if(dist_graph[XYZ](i, v) != -1 && dist_graph[XYZ](v, j) != -1)
                    {
                        if(dist_graph[XYZ](i, j) == -1 || dist_graph[XYZ](i, j) > dist_graph[XYZ](i, v) + dist_graph[XYZ](v, j))
                            dist_graph[XYZ](i, j) = dist_graph[XYZ](i, v) + dist_graph[XYZ](v, j);
                    }
                }
            }
        }
        return;
    }

    void VPuzzleGraph::compute_two_loops_candidate_voxels(int XYZ, VPuzFilterCycle &filter)
    {
        //compute_two_loops_candidate_voxels
        //P is the remain_part Rpart, Then
        //P0 -> P -> P1 -> P0
        //P2 -> P -> P3 -> P2
        //!! P1 -> P2, P3 -> P0

        pPart Rpart = parts_.back();
        vector<VPuzFECycle> candidate_list;

        for(int i0 = 0; i0 < Rpart->neighbor_[XYZ].in_.size(); i0++)
        {
            VoxelizedPart *P0 = Rpart->neighbor_[XYZ].in_[i0];
            for(int i1 = 0; i1 < Rpart->neighbor_[XYZ].out_.size(); i1++)
            {
                VoxelizedPart *P1 = Rpart->neighbor_[XYZ].out_[i1];
                if(dist_graph[XYZ](P1->part_id_, P0->part_id_) != -1)
                {
                    for(int i2 = 0; i2 < Rpart->neighbor_[XYZ].in_.size(); i2++)
                    {
                        VoxelizedPart *P2 = Rpart->neighbor_[XYZ].in_[i2];
                        for (int i3 = 0; i3 < Rpart->neighbor_[XYZ].out_.size(); i3++)
                        {
                            VoxelizedPart *P3 = Rpart->neighbor_[XYZ].out_[i3];
                            if(dist_graph[XYZ](P3->part_id_, P2->part_id_) != -1)
                            {

                                if(dist_graph[XYZ](P1->part_id_, P2->part_id_ ) != -1 &&
                                   dist_graph[XYZ](P3->part_id_, P0->part_id_) != -1)
                                    //requirement for making (P0, P1, P, P2, P3) in a strong connected component.
                                {
                                    VPuzCycleDat cycle_dat;
                                    VPuzFECycle element;

                                    //joint data
                                    cycle_dat.anchor_voxels_list.push_back(Rpart->neighbor_[XYZ].in_blocking_ [i0].get());
                                    cycle_dat.anchor_voxels_list.push_back(Rpart->neighbor_[XYZ].out_blocking_[i1].get());
                                    cycle_dat.anchor_voxels_list.push_back(Rpart->neighbor_[XYZ].in_blocking_ [i2].get());
                                    cycle_dat.anchor_voxels_list.push_back(Rpart->neighbor_[XYZ].out_blocking_[i3].get());
                                    cycle_dat.type = PuzzleConnection_IOIO;

                                    element.data_ = cycle_dat;


                                    //******************* Weight Calculation *************************//
                                    element.weight = 0;
                                    element.weight +=    std::pow(Rpart->neighbor_[XYZ].in_blocking_   [i0]->data_.size(), 2)
                                                         + std::pow(Rpart->neighbor_[XYZ].out_blocking_  [i1]->data_.size(), 2)
                                                         + std::pow(Rpart->neighbor_[XYZ].in_blocking_   [i2]->data_.size(), 2)
                                                         + std::pow(Rpart->neighbor_[XYZ].out_blocking_  [i3]->data_.size(), 2);
                                    element.weight += dist_graph[XYZ](P1->part_id_, P0->part_id_) + dist_graph[XYZ](P3->part_id_, P2->part_id_);
                                    //******************* Weight Calculation *************************//


                                    candidate_list.push_back(element);
                                }
                            }
                        }
                    }
                }
            }
        }
        filter.insert(candidate_list);
        //std::cout << "XYZ " << XYZ << ": \t" << candidate_list.size() <<", " << filter.candidate_.size() <<"\n";
    }

    void VPuzzleGraph::compute_one_loop_candidate_voxels(int XYZ, VPuzFilterCycle &filter) {
        //compute_one_loop_candidate_voxels
        //P1 -> P -> P2 -> P1

        pPart Rpart = parts_.back();
        vector<VPuzFECycle> list;

        //Single joint
        //P0 -> P -> P1 -> P0
        for(int i0 = 0; i0 < Rpart->neighbor_[XYZ].in_.size(); i0++)
        {
            VoxelizedPart *P0 = Rpart->neighbor_[XYZ].in_[i0];
            for (int i1 = 0; i1 < Rpart->neighbor_[XYZ].out_.size(); i1++)
            {
                VoxelizedPart *P1 = Rpart->neighbor_[XYZ].out_[i1];
                if(dist_graph[XYZ](P1->part_id_, P0->part_id_) != -1)
                {
                    VPuzCycleDat joint;
                    VPuzFECycle element;

                    //joint
                    joint.anchor_voxels_list.push_back(Rpart->neighbor_[XYZ].in_blocking_ [i0].get());
                    joint.anchor_voxels_list.push_back(Rpart->neighbor_[XYZ].out_blocking_[i1].get());
                    joint.type = PuzzleConnection_IN_OUT;

                    element.data_ = joint;

                    //******************* Weight Calculation *************************//
                    element.weight = 0;
                    element.weight +=  std::pow(Rpart->neighbor_[XYZ].in_blocking_ [i0]->data_.size(), 2)
                                       +   std::pow(Rpart->neighbor_[XYZ].out_blocking_[i1]->data_.size(), 2);
                    element.weight += dist_graph[XYZ](P1->part_id_, P0->part_id_);
                    //******************* Weight Calculation *************************//

                    list.push_back(element);
                }
            }
        }

        filter.insert(list);
        //std::cout << "XYZ " << XYZ << ": \t" << list.size() << ", " << filter.candidate_.size() <<"\n";
    }

    void VPuzzleGraph::do_separation(vector<VPuzRemainVolumePartitionDat> &output)
    {
        // build dist grap
        for(int XYZ = 0 ;XYZ < 3; XYZ++)
        {
            compute_distance_graph(XYZ);
        }


        //compute all possible candidates for generating new loops in following order
        //X_one_loop //X_two_loops//Y_one_loop //Y_two_loops //Z_one_loop //Z_two_loops
        for(int id = 0; id < 6; id ++)
        {
            if(id % 2 == 0)
            {
                //compute all possible single direction joint candidates
                graph_partition_plan[id].max_candidates_num_ = max_single_loop_partition_plan;
                compute_one_loop_candidate_voxels(id / 2, graph_partition_plan[id]);
            }
            else
            {
                //compute all possible double directions joint candidates
                graph_partition_plan[id].max_candidates_num_ = max_double_loops_partition_plan;
                compute_two_loops_candidate_voxels(id / 2, graph_partition_plan[id]);
            }
        }

        //do combination between XYZ joints
        vector<VPuzCycleXYZDat> plans_cycleXYZ;
        compute_combination_of_cycleXYZ(plans_cycleXYZ);
        std::cout << "CycleXYZ plan:\t" << plans_cycleXYZ.size() << std::endl;

        //pickup voxels
        compute_remain_volume_partition_plans(output, plans_cycleXYZ);
        std::cout << "Remain Volume plan:\t" << output.size() << std::endl;
    }

    void VPuzzleGraph::compute_combination_of_cycleXYZ(
            vector<VPuzCycleXYZDat> &plans_cycleXYZ) {

        //63 type
        //00(Z) 00(Y) 00(X)
        //00 : no edge in blocking graph
        //01 : in->out edge in blocking graph
        //10 : out <- in edge in blocking graph
        //11 : A = B, double edge in blocking graph

        for(int id = 0; id < 63; id++)
        {
            int binary_state = id;
            //debug
            if((binary_state & 0b11) == 0b10)
            {
                //avoiding symmetric
                //Since 101010 = 010101
                continue;
            }

            int max_joint_each_direction = 0;
            {
                //the total combination is max_joint_combination
                //if in one direction is double edges in blocking graph, then this direction only have one possible choice
                //therefore, we could accept more choice on the other direction
                int power = 0;
                if((binary_state & 0b11) != 0b11) power++;
                if((binary_state & 0b1100) != 0b1100) power++;
                if((binary_state & 0b110000) != 0b110000) power++;
                max_joint_each_direction = pow(max_joint_combination, 1.0 / power);
            }

            vector<VPuzFECycle> plan[3]; //On each direction, choose K direction and do combination

            for(int XYZ = 0; XYZ < 3; XYZ ++)
            {
                switch (binary_state & 0b11)
                {
                    case 0:
                    {
                        //00 : no edge in blocking graph
                        graph_partition_plan[2 * XYZ + 1].random_choose(plan[XYZ], max_joint_each_direction);
                        break;
                    }
                    case 0b01:
                    {
                        //01 : in->out edge in blocking graph
                        graph_partition_plan[2 * XYZ].random_choose(plan[XYZ], max_joint_each_direction);
                        break;
                    }
                    case 0b10:
                    {
                        //10 : out <- in edge in blocking graph
                        graph_partition_plan[2 * XYZ].random_choose(plan[XYZ], max_joint_each_direction);
                        for(VPuzFECycle &em: plan[XYZ])
                        {
                            OrderedVElemList *tmp = em.data_.anchor_voxels_list[0];
                            em.data_.anchor_voxels_list[0] = em.data_.anchor_voxels_list[1];
                            em.data_.anchor_voxels_list[1] = tmp;
                            em.data_.type = PuzzleConnection_OUT_IN;
                        }
                        break;
                    }
                    default:
                    {
                        VPuzFECycle empty;
                        empty.data_.type = PuzzleConnection_NULL;
                        plan[XYZ].push_back(empty);
                        break;
                    }
                }

                binary_state = binary_state >> 2;
            }

            vector<VPuzFE<VPuzCycleXYZDat>> canididate_list;
            //compute all combination
            for(int i = 0; i < plan[0].size(); i++)
            {
                for(int j = 0; j < plan[1].size(); j++)
                {
                    for(int k = 0; k < plan[2].size(); k++)
                    {

                        VPuzCycleXYZDat cycleXYZ_dat;
                        cycleXYZ_dat.cycle_XYZ[0] = plan[0][i];
                        cycleXYZ_dat.cycle_XYZ[1] = plan[1][j];
                        cycleXYZ_dat.cycle_XYZ[2] = plan[2][k];
                        cycleXYZ_dat.relation = id;

                        VPuzFECycleXYZ FEcycleXYZ;
                        FEcycleXYZ.data_ = cycleXYZ_dat;

                        /******             weight                          ***/

                        FEcycleXYZ.weight = 0;
                        for(int XYZ = 0; XYZ < 3; XYZ++)
                        {
                            for(int id = 0; id < cycleXYZ_dat.cycle_XYZ[XYZ].data_.anchor_voxels_list.size(); id++)
                            {
                                FEcycleXYZ.weight += cycleXYZ_dat.cycle_XYZ[XYZ].data_.anchor_voxels_list[id]->data_.size();
                            }
                        }

                        /******             weight                          ***/

                        canididate_list.push_back(FEcycleXYZ);
                    }
                }
            }
            VPuzFilter<VPuzCycleXYZDat> filter;
            filter.max_candidates_num_ = max_joint_combination / 64;
            filter.insert(canididate_list);
            for(int id = 0; id < filter.candidate_.size(); id++)
                plans_cycleXYZ.push_back(filter.candidate_[id].data_);
        }
        return;
    }

    void VPuzzleGraph::compute_remain_volume_partition_plans(vector<VPuzRemainVolumePartitionDat> &output,
                                                             const vector<VPuzCycleXYZDat> &plans_cycleXYZ)
    {
        for(int gp_plan_id = 0; gp_plan_id < plans_cycleXYZ.size(); gp_plan_id ++) {
            VPuzCycleXYZDat plan_cycleXYZ = plans_cycleXYZ[gp_plan_id];

            vector<vector<VPuzFE<VoxelElement *>>> gr[2];
            //gr[1][3] means : groupB 's Z direction partition plan

            vector<int> require_blocking;
            for (int XYZ = 0; XYZ < 3; XYZ++)
            {
                switch (plan_cycleXYZ.cycle_XYZ[XYZ].data_.type)
                {
                    case PuzzleConnection_IN_OUT: {
                        for (int id = 0; id < 2; id++)
                        {
                            gr[id].push_back(vector<VPuzFE<VoxelElement *>>());
                            for (pEmt v: plan_cycleXYZ.cycle_XYZ[XYZ].data_.anchor_voxels_list[id]->data_)
                            {
                                VPuzFE<VoxelElement *> vpfe;
                                vpfe.data_ = v;
                                vpfe.weight = 1;
                                gr[id].back().push_back(vpfe);
                            }
                        }
                        require_blocking.push_back(1 << (2 * XYZ));
                        break;
                    }
                    case PuzzleConnection_OUT_IN: {
                        for (int id = 0; id < 2; id++)
                        {
                            gr[id].push_back(vector<VPuzFE<VoxelElement *>>());
                            for (pEmt v: plan_cycleXYZ.cycle_XYZ[XYZ].data_.anchor_voxels_list[id]->data_)
                            {
                                VPuzFE<VoxelElement *> vpfe;
                                vpfe.data_ = v;
                                vpfe.weight = 1;
                                gr[id].back().push_back(vpfe);
                            }
                        }
                        require_blocking.push_back(1 << (2 * XYZ + 1));
                        break;
                    }
                    case PuzzleConnection_IOIO:
                    {
                        int gid[4] = {0, 0, 1, 1};
                        int jid[4] = {0, 1, 2, 3};
                        for (int id = 0; id < 4; id++)
                        {
                            gr[gid[id]].push_back(vector<VPuzFE<VoxelElement *>>());
                            for (pEmt v: plan_cycleXYZ.cycle_XYZ[XYZ].data_.anchor_voxels_list[jid[id]]->data_)
                            {
                                VPuzFE<VoxelElement *> vpfe;
                                vpfe.data_ = v;
                                vpfe.weight = 1;
                                gr[gid[id]].back().push_back(vpfe);
                            }
                        }
                        require_blocking.push_back(0);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }

            VPuzRemainVolumePartitionDat plan;
            int time = max_failure_time;
            while(!random_compute_voxel_partition_plan(plan, gr, require_blocking))
            {
                time--;
                if(time == 0) break;
            }
            if(time)
            {
                plan.relation = plan_cycleXYZ.relation;
                output.push_back(plan);
            }
        }
    }

    bool VPuzzleGraph::random_compute_voxel_partition_plan(VPuzRemainVolumePartitionDat &plan,
                                                           vector<vector<VPuzFE<VoxelElement *>>> *gr,
                                                           vector<int> &require_blocking)
    {
        struct GroupVoxels
        {
            vector<pEmt> candidates_;

            int group_id; //A:0 or B:1
        };

        vector<GroupVoxels> task_list;

        std::map<int, bool> voxel_used;
        std::map<int, double> voxel_weight[2];

        for(int kd = 0; kd < 2; kd++)
        {
            for(int id = 0; id < gr[kd].size(); id++)
            {
                GroupVoxels group;
                group.group_id = kd;
                for(int jd = 0; jd < gr[kd][id].size(); jd++) {
                    pEmt  voxel = gr[kd][id][jd].data_;
                    group.candidates_.push_back(voxel);
                }
                task_list.push_back(group);
            }
        }

        auto pop_smallest_group = [&](vector<GroupVoxels> &task_list) -> GroupVoxels
        {
            int minimum_group_num = task_list.front().candidates_.size();
            vector<vector<GroupVoxels>::iterator> candidate_list;
            for(int id = 0; id < task_list.size(); id++)
            {
                if(task_list[id].candidates_.size() < minimum_group_num)
                {
                    minimum_group_num = task_list[id].candidates_.size();
                    candidate_list.clear();
                    candidate_list.push_back(task_list.begin() + id);
                }
                else if(task_list[id].candidates_.size() == minimum_group_num)
                {
                    candidate_list.push_back(task_list.begin() + id);
                }
            }

            int rand_index =rand() % candidate_list.size();

            GroupVoxels group_picked = *candidate_list[rand_index];

            task_list.erase(candidate_list[rand_index]);

            return group_picked;
        };

        auto random_pick_voxel = [&](vector<pEmt> &candidate) -> pEmt
        {
            return candidate[rand() % candidate.size()];
        };

        auto erase_used_voxel_from_list = [&](vector<GroupVoxels> &list, pEmt voxel, int gid) -> bool
        {
            for(int id = 0; id < list.size(); id++)
            {
                GroupVoxels &u = list[id];
                bool delete_u = false;

                for(int jd = 0; jd < u.candidates_.size(); jd++)
                {
                    if(u.candidates_[jd] == voxel)
                    {
                        if(gid != u.group_id)
                        {
                            u.candidates_.erase(jd + u.candidates_.begin());
                            jd --;
                        }
                        else
                        {
                            delete_u = true;
                            break;
                        }
                    }
                }

                if(delete_u)
                {
                    list.erase(id + list.begin());
                    id--;
                }
                else if(u.candidates_.empty())
                {
                    return false;
                }
            }

            return true;
        };

        while(!task_list.empty())
        {
            GroupVoxels u = pop_smallest_group(task_list);
            pEmt voxel = random_pick_voxel(u.candidates_);
            voxel_used.insert(std::make_pair(voxel->order_, true));

            if(u.group_id)
            {
                plan.groupB.push_back(voxel);
            }
            else
            {
                plan.groupA.push_back(voxel);
            }

            if(!erase_used_voxel_from_list(task_list, voxel, u.group_id))
                return false;
        }

        return true;
    }

    void VPuzzleGraph::remove_duplicate(vector<VoxelElement *> &gvoxel, vector<VoxelElement *> &group)
    {
        for(int id = 0; id < gvoxel.size(); id++)
        {
            bool is_duplicate = false;
            for(int jd = 0; jd < id; jd++) {
                if (gvoxel[id] == gvoxel[jd])
                {
                    is_duplicate = true;
                    break;
                }
            }
            if(!is_duplicate) group.push_back(gvoxel[id]);
        }
    }



//void VPuzzleGraph::dfs_compute_voxel_partition_plan(vector<VPuzVoxelPartitionDat> &plans,
//                                                    vector<vector<VPuzFE<VoxelElement *>>> *gr, int relation)
//{
//    struct DFSNode
//    {
//        VPuzVoxelPartitionDat plan;
//        std::unordered_map<int, int> group_satisified[2];
//        int num_satisified;
//        int max_voxel_index;
//    };
//
//    vector<pEmt >voxel_lists;
//    vector<std::unordered_map<int, bool>> group_map[2];
//
//    for(int kd = 0; kd < 2; kd++)
//    {
//        for(int id =0; id < gr[kd].size(); id++)
//        {
//            group_map[kd].push_back(std::unordered_map<int, bool>());
//            std::cout << gr[kd][id].size() << std::endl;
//            for(int jd = 0; jd < gr[kd][id].size(); jd++) {
//                pEmt  voxel = gr[kd][id][jd].data_;
//                voxel_lists.push_back(voxel);
//                group_map[kd][id].insert(std::make_pair(voxel->order_, true));
//            }
//        }
//    }
//
//    auto compare_order = [&](pEmt a, pEmt b) -> bool
//    {
//        return (a->order_ < b->order_);
//    };
//
//    int TOTAL_NUM_SATISIFIED = gr[0].size() + gr[1].size();
//
//    std::sort(voxel_lists.begin(), voxel_lists.end(), compare_order);
//    voxel_lists.erase(std::unique(voxel_lists.begin(), voxel_lists.end()), voxel_lists.end());
//    std::cout << voxel_lists.size() << std::endl;
//
//    std::function<bool(DFSNode &)> dfs = [&](DFSNode &u) -> bool
//    {
//        //std::cout << u.num_satisified << "\t" << u.max_voxel_index << std::endl;
//        if(u.num_satisified == TOTAL_NUM_SATISIFIED)
//        {
//            u.plan.relation = relation;
//            std::sort(u.plan.groupA.begin(), u.plan.groupA.end(), compare_order);
//            std::sort(u.plan.groupB.begin(), u.plan.groupB.end(), compare_order);
//            plans.push_back(u.plan);
//            return true;
//        }
//
//        for(int id = u.max_voxel_index; id < voxel_lists.size(); id++)
//        {
//            pEmt voxel = voxel_lists[id];
//            for(int kd = 0; kd < 2; kd++)
//            {
//                DFSNode v = u;
//                for(int jd = 0; jd < group_map[kd].size(); jd++)
//                {
//                    if(u.group_satisified[kd].find(jd) == u.group_satisified[kd].end()
//                       && group_map[kd][jd].find(voxel->order_) != group_map[kd][jd].end())
//                    {
//                        v.num_satisified++;
//                        v.group_satisified[kd].insert(std::make_pair(jd, true));
//                    }
//                }
//                if(v.num_satisified > u.num_satisified)
//                {
//                    v.max_voxel_index = id + 1;
//                    if(kd == 0){v.plan.groupA.push_back(voxel);}
//                    else {v.plan.groupB.push_back(voxel);}
//                    if(dfs(v)) return true;
//                }
//
//            }
//        }
//        return false;
//    };
//
//    DFSNode u;
//    u.num_satisified = 0;
//    u.max_voxel_index = 0;
//
//    dfs(u);
//
//    return;
//}
//

//bool VPuzzleGraph::random_compute_voxel_partition_plan(VPuzVoxelPartitionDat &plan,
//                                                       vector<vector<VPuzFE<VoxelElement *>>> *gr,
//                                                       vector<int> &require_blocking)
//{
//    vector<VoxelElement *> gvoxel[2];
//    random_pick_voxels_in_a_group(gr[0], gvoxel[0]);
//
//    //erase same voxel
//    bool is_empty = false;
//    for(int id = 0; id < gr[1].size(); id++)
//    {
//        for(int jd = 0; jd < gr[1][id].size(); jd++)
//        {
//            VoxelElement *B = gr[1][id][jd].data_;
//            bool is_same = false;
//            for(int kd = 0; kd < gvoxel[0].size(); kd++)
//            {
//                VoxelElement *A = gvoxel[0][kd];
//                if(A == B)
//                {
//                    is_same = true;
//                    break;
//                }
//                gr[1][id][jd].weight += (B->pos_ - A->pos_).norm();
//            }
//            if(is_same)
//            {
//                gr[1][id].erase(gr[1][id].begin() + jd);
//                jd--;
//            }
//        }
//        if(gr[1][id].size() == 0)
//        {
//            is_empty = true;
//            break;
//        }
//    }
//
//    if(is_empty)
//        return false;
//
//    //rewrite the weight
//    for(int id = 0; id < gr[1].size(); id++)
//    {
//        VoxelElement *A = gvoxel[0][id];
//        for(int jd = 0; jd < gr[1][id].size(); jd++)
//        {
//            VoxelElement *B = gr[1][id][jd].data_;
//            int state = require_blocking[id];
//            for(int XYZ = 0; XYZ < 3; XYZ ++)
//            {
//                switch(state & 0b11)
//                {
//                    case 0b01:
//                    {
//                        //A >= B
//                        gr[1][id][jd].weight += A->pos_[XYZ] - B->pos_[XYZ] + size_[XYZ];
//                        break;
//                    }
//                    case 0b10:
//                    {
//                        //B >= A
//                        gr[1][id][jd].weight += B->pos_[XYZ] - A->pos_[XYZ] + size_[XYZ];
//                        break;
//                    }
//                    default:
//                    {
//                        break;
//                    }
//                }
//                state = state >> 2;
//            }
//        }
//    }
//
//    random_pick_voxels_in_a_group(gr[1], gvoxel[1]);
//
//    plan.groupA.clear();
//    plan.groupB.clear();
//    remove_duplicate(gvoxel[0], plan.groupA);
//    remove_duplicate(gvoxel[1], plan.groupB);
//
//    return true;
//}
//
//
//void VPuzzleGraph::random_pick_voxels_in_a_group(vector<vector<VPuzFE<VoxelElement *>>> &gr,
//                                                 vector<VoxelElement *> &gvoxel)
//{
//    //choose groupA's elements
//    int rand_grAID = rand() % gr.size();
//    int grAID = rand_grAID;
//    vector<bool> visited;
//    visited.resize(gr.size(), false);;
//    gvoxel.resize(gr.size(), nullptr);
//    do
//    {
//        if(visited[grAID])
//        {
//            grAID = (grAID + 1) % gr.size();
//            continue;
//        }
//
//        VPuzFilter<VoxelElement *> filter;
//        filter.insert(gr[grAID]);
//        VPuzFE<VoxelElement *> A0 = filter.random_choose();
//        gvoxel[grAID] = A0.data_;
//        visited[grAID] = true;
//
//        //rewrite the weight of other in group A
//        for(int id = 0; id < gr.size(); id++)
//        {
//            if(!visited[id])
//            {
//                for(int jd = 0; jd < gr[id].size(); jd++)
//                {
//                    VPuzFE<VoxelElement *> &A = gr[id][jd];
//                    if(A.data_ == A0.data_)
//                    {
//                        gvoxel[id] = A0.data_;
//                        visited[id] = true;
//                        break;
//                    }
//                    else
//                    {
//                        Vector3i dV(A.data_->pos_ - A0.data_->pos_);
//                        dV[0] = size_[0] - std::abs(dV[0]);
//                        dV[1] = size_[1] - std::abs(dV[1]);
//                        dV[2] = size_[2] - std::abs(dV[2]);
//                        A.weight += dV.norm();
//                    }
//                }
//            }
//        }
//        grAID = (grAID + 1) % gr.size();
//    }while(grAID != rand_grAID);
//}
}
