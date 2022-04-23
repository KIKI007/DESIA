//
// Created by *** on 14.04.18.
//

#ifndef UNDERSTAND_INTERLOCK_VOXELIZEDPUZZLEDISASSEMBLY_H
#define UNDERSTAND_INTERLOCK_VOXELIZEDPUZZLEDISASSEMBLY_H

#include "VoxelizedPuzzle.h"
#include "VoxelizedInterface.h"
#include <cstring>
#include <queue>
#include <map>

namespace voxel{
    struct DisassemblyBFSNode
    {
    public:

        typedef Eigen::Vector3i Vector3i;

    public:

        vector<std::string> sequences;
        Vector3i move;

        void move_one_step(Vector3i direct)
        {
            if     (direct[0] ==  1) sequences.push_back("+x");
            else if(direct[0] == -1) sequences.push_back("-x");
            else if(direct[1] ==  1) sequences.push_back("+y");
            else if(direct[1] == -1) sequences.push_back("-y");
            else if(direct[2] ==  1) sequences.push_back("+z");
            else if(direct[2] == -1) sequences.push_back("-z");
            return;
        }

        void move_and_disassembly(Vector3i direct)
        {
            if     (direct[0] ==  1) sequences.push_back("+X");
            else if(direct[0] == -1) sequences.push_back("-X");
            else if(direct[1] ==  1) sequences.push_back("+Y");
            else if(direct[1] == -1) sequences.push_back("-Y");
            else if(direct[2] ==  1) sequences.push_back("+Z");
            else if(direct[2] == -1) sequences.push_back("-Z");
            return;
        }
    };

    class VoxelizedPuzzleDisassembly{
    public:

        typedef Eigen::Vector3i Vector3i;

        typedef VoxelizedPart* pPart;

    public:

        void set_puzzle(VoxelizedPuzzle* puzzle, const vector<pEmt>& remains)
        {
            init(puzzle, remains);
        }

    public:

        void init(VoxelizedPuzzle* puzzle, const vector<pEmt>& remains);

    public:

        bool check_is_collision(Vector3i move);

        bool check_disassembly_in_one_direction(Vector3i move, Vector3i direct);

        bool bfs_check_disassembly(vector<std::string> &sequences);

    public:

        VoxelizedPuzzle* puzzle_;

        std::unordered_map<int, pEmt> map_remains_;

        std::vector<pEmt> npart_;
    };

}


#endif //UNDERSTAND_INTERLOCK_VOXELIZEDPUZZLEDISASSEMBLY_H
