//
// Created by *** on 16.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_VOXELIZEDPUZZLE_H
#define UNDERSTAND_INTERLOCK_VOXELIZEDPUZZLE_H
#include "VoxelizedPart.h"
#include "VoxelElement.h"
#include "VoxelizedInterface.h"
#include "VoxelizedRenderCube.h"
#include "VoxelizedRenderSphere.h"
#include <boost/multi_array.hpp>

namespace voxel
{
    class VoxelizedPuzzle
    {
    public:
        typedef boost::multi_array<bool, 3> array_3b;
        typedef boost::multi_array<int, 3> array_3i;
        typedef std::shared_ptr<VoxelizedPart> shared_pPart;
        typedef std::shared_ptr<VoxelizedInterface> shared_pInterface;
        typedef Eigen::MatrixXi MatrixXi;
        typedef VoxelizedPart* pPart;
        typedef Eigen::Vector3i Vector3i;

    public:

        VoxelizedPuzzle();

        //all voxel of nx * ny * nz belongs one part
        VoxelizedPuzzle(int nx, int ny, int nz, std::shared_ptr<ColorCoding> coder);

        //array[x][y][z] = true means (x, y, z) is a valid voxel.
        //All valid voxels belong to one part
        VoxelizedPuzzle(int nx, int ny, int nz, const array_3b &array, std::shared_ptr<ColorCoding> coder);

        //array[x][y][z] represents voxel (x, y, z)'s part id
        //-1 means a non-valid voxel
        VoxelizedPuzzle(int nx, int ny, int nz, const array_3i &array, std::shared_ptr<ColorCoding> coder);

        VoxelizedPuzzle(const VoxelizedPuzzle &);

        //initilized everything according to the array
        //array[x][y][z] represents voxel (x, y, z)'s part id
        //-1 means a non-valid voxel
        void init(const array_3i &array);

    public: //IO

        //output the result into the central class : VoxeliedInterface
        //VoxelizedInterface can be used as rendering, checking, etc
        //bool accessbility = true means output the accessibility of each voxel
        //bool vertex_mark = true means output the mark of each voxel
        shared_pInterface output_assembly_interface(bool accessibility, bool vertex_mark);

    public:

        int get_part_id(Eigen::Vector3i pos);

        bool is_same(VoxelizedPuzzle *A);

        int get_part_num(){return parts_.size();}

        // build the connection between different part depending on their contacting relation
        // Edge from A to B in X direction means X_A >= X_B
        // Edge also have the information of the voxel which produces this relation
        void build_part_connection(pPart part);

        void partition_part(pPart part, vector<pEmt> remove_portion);

        void update_part_connection(pPart part);

        int get_num_voxels(){return voxel_.size();}

    public:

        void set_voxel_mark(int order, int mark)
        {
            map_mark.insert(std::make_pair(order, mark));
        }

        double get_accessibility(int order)
        {
            auto find_it = map_access_.find(order);
            if(find_it == map_access_.end())
                return  -1;
            return find_it->second;
        }

        int V2I(Vector3i pos){
            if(pos[0] < 0 || pos[1] < 0 || pos[2] < 0) return -1;
            if(pos[0] >= Nx || pos[1] >= Ny || pos[2] >= Nz) return -1;
            return pos[2] * Nx * Ny + pos[1] * Nx + pos[0];
        }

    public:

        int Nx, Ny, Nz;

        vector<shared_pPart> parts_;

        vector<shared_pEmt> voxel_;

    public:

        std::unordered_map<int, pEmt> map_ip_;

        std::unordered_map<int, double> map_access_;

        std::unordered_map<int, int> map_mark;

    public:

        std::shared_ptr<ColorCoding> colorcoder_;

    };
}


#endif //UNDERSTAND_INTERLOCK_VOXELIZEDPUZZLE_H
