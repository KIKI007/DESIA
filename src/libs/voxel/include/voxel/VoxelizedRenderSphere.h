//
// Created by *** on 18.03.18.
//

#ifndef UNDERSTAND_INTERLOCK_VOXELIZEDRENDERSPHERE_H
#define UNDERSTAND_INTERLOCK_VOXELIZEDRENDERSPHERE_H

#include "VoxelizedInterface.h"
#include "igl/readOBJ.h"

namespace voxel {

    class VoxelizedRenderSphere :public VoxelizedInterface
    {

    public:

        typedef Eigen::Vector3i Vector3i;

        typedef Eigen::MatrixXd MatrixXd;

        typedef Eigen::MatrixXi MatrixXi;

        typedef Eigen::Vector3d Vector3d;

    public:

        struct VoxelizedRenderUnit
        {
            MatrixXd V;
            MatrixXi F;
            Vector3d dV;
            Vector3i dF;
            int gid;
            int mark;
        };

    public:
        VoxelizedRenderSphere(int nx, int ny, int nz, std::shared_ptr<ColorCoding> coder): VoxelizedInterface(nx, ny, nz, coder)
        {
            sphere_radius = 0.7;
            cylinder_length = 3;
            hwdith = 3;
        }

        VoxelizedRenderSphere(const VoxelizedInterface &interface): VoxelizedInterface(interface)
        {
            sphere_radius = 0.7;
            cylinder_length = 3;
            hwdith = 3;
        }

    public:

        void rendering(MatrixXd &V, MatrixXi &F, MatrixXd &C);

        void add_color_by_groupID(MatrixXd &C);

        void add_to_render_list(std::shared_ptr<VoxelizedRenderUnit> unit);

    public:

        bool loadSphere(VoxelizedRenderUnit &sphere);

        bool loadCylinderX(VoxelizedRenderUnit &cylinderX);

        bool loadCylinderY(VoxelizedRenderUnit &cylinderY);

        bool loadCylinderZ(VoxelizedRenderUnit &cylinderZ);

    public:

        double sphere_radius;

        double cylinder_length;

        double hwdith;

    public:

        int nvertices = 0;

        int nfaces = 0;

        int present_vertices_num = 0;

    public:
        std::vector<std::shared_ptr<VoxelizedRenderUnit>> render_lists;
    };
}




#endif //UNDERSTAND_INTERLOCK_VOXELIZEDRENDERSPHERE_H
