//
// Created by *** on 14.04.18.
//

#include "voxel/VoxelizedPuzzleDisassembly.h"

namespace voxel{
    void VoxelizedPuzzleDisassembly::init(VoxelizedPuzzle *puzzle, const vector<pEmt> &remains)
    {
        puzzle_ = puzzle;
        pPart part = puzzle->parts_.back().get();
        for(pEmt em: remains)
        {
            map_remains_.insert(std::make_pair(em->order_, em));
        }
        for(pEmt em: part->elist_.data_)
        {
            if(map_remains_.find(em->order_) == map_remains_.end())
            {
                npart_.push_back(em);
            }
        }

        return;
    }

    bool VoxelizedPuzzleDisassembly::check_is_collision(Vector3i move)
    {
        for(pEmt em : npart_)
        {
            Vector3i pos = em->pos_ + move;

            //out of range
            int order = puzzle_->V2I(pos);
            if(order == -1) continue;

            auto find_it = map_remains_.find(order);

            if(find_it == map_remains_.end()) continue;
            else return true;
        }
        return false;
    }

    bool VoxelizedPuzzleDisassembly::check_disassembly_in_one_direction(Vector3i move, Vector3i direct)
    {
        std::map<int, bool> visited_map;
        for(pEmt em : npart_)
        {
            Vector3i pos = em->pos_ + move;
            while(true)
            {
                pos = pos + direct;

                //stop searching
                //out of range
                int order = puzzle_->V2I(pos);
                if(order == -1) break;
                if(visited_map.find(order) != visited_map.end()) break;
                else visited_map.insert(std::make_pair(order, true));

                //collision
                if(map_remains_.find(order) != map_remains_.end())
                    return false;
            }
        }
        return true;
    }

    bool VoxelizedPuzzleDisassembly::bfs_check_disassembly(vector<std::string> &sequences)
    {
        DisassemblyBFSNode u;
        u.move = Vector3i(0, 0, 0);

        std::queue<DisassemblyBFSNode> queue;
        queue.push(u);

        std::unordered_map<int, bool> visited;
        visited.insert(std::make_pair(0, true));

        auto moveV2I = [&](Vector3i move) -> int
        {
            int X = move[0] + puzzle_->Nx;
            int Y = move[1] + puzzle_->Ny;

            int Z = move[2] + puzzle_->Nz;
            int order = Z * puzzle_->Nx * puzzle_->Ny + Y * puzzle_->Nx + X;
            return order;
        };

        int dX[6] = {1, -1, 0, 0, 0, 0};
        int dY[6] = {0, 0, 1, -1, 0, 0};
        int dZ[6] = {0, 0, 0, 0, 1, -1};
        auto get_direction = [&](int nrm)->Vector3i
        {
            return Vector3i(dX[nrm], dY[nrm], dZ[nrm]);
        };

        auto is_visited = [&](Vector3i pos) -> bool
        {
            auto find_it = visited.find(moveV2I(pos));
            if(find_it == visited.end())
                return false;
            else
                return true;
        };

        auto set_visited = [&](Vector3i pos)
        {
            visited.insert(std::make_pair(moveV2I(pos), true));
        };

        while(!queue.empty())
        {
            DisassemblyBFSNode u = queue.front(); queue.pop();
            for(int nrm = 0; nrm < 6; nrm++)
            {
                if(check_disassembly_in_one_direction(u.move, get_direction(nrm)))
                {
                    u.move_and_disassembly(get_direction(nrm));
                    sequences = u.sequences;
                    return true;
                }
            }

            Vector3i pos;
            for(int nrm = 0; nrm < 6; nrm++)
            {
                pos = u.move + get_direction(nrm);
                if(!is_visited(pos) && !check_is_collision(pos))
                {
                    set_visited(pos);
                    DisassemblyBFSNode v;
                    v = u;
                    v.move = pos;
                    v.move_one_step(get_direction(nrm));
                    queue.push(v);
                }
            }
        }

        return false;
    }
}
