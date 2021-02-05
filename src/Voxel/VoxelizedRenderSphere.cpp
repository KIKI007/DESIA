//
// Created by *** on 18.03.18.
//

#include "VoxelizedRenderSphere.h"

void VoxelizedRenderSphere::rendering(MatrixXd &V, MatrixXi &F, MatrixXd &C)
{
    VoxelizedRenderUnit sphere, cylinderX, cylinderY, cylinderZ;
    loadSphere(sphere);
    loadCylinderX(cylinderX);
    loadCylinderY(cylinderY);
    loadCylinderZ(cylinderZ);


    nvertices = nfaces = present_vertices_num = 0;

    for(int ix = 0; ix < Nx; ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
            {
                if((*voxel_)[ix][iy][iz] != -1)
                {
                    //Self
                    {
                        std::shared_ptr<VoxelizedRenderUnit> unit = std::make_shared<VoxelizedRenderUnit>(sphere);
                        unit->dV = Vector3d(ix * hwdith, iy * hwdith, iz * hwdith);
                        unit->dF = Vector3i(present_vertices_num, present_vertices_num, present_vertices_num);
                        unit->gid = (*voxel_)[ix][iy][iz];
                        if(voxel_mark_) unit->mark = (*voxel_mark_)[ix][iy][iz];
                        else unit->mark = 0;
                        add_to_render_list(unit);

                    }

                    //+X
                    {
                        int nix = ix + 1, niy = iy, niz = iz;
                        if(nix < Nx && (*voxel_)[nix][niy][niz] == (*voxel_)[ix][iy][iz])
                        {
                            std::shared_ptr<VoxelizedRenderUnit> unit = std::make_shared<VoxelizedRenderUnit>(cylinderX);
                            unit->dV = Vector3d((ix + 0.5) * hwdith, (iy) * hwdith, (iz) * hwdith);
                            unit->dF = Vector3i(present_vertices_num, present_vertices_num, present_vertices_num);
                            unit->gid = (*voxel_)[ix][iy][iz];
                            unit->mark = 0;
                            add_to_render_list(unit);
                        }
                    }

                    //+Y
                    {
                        int nix = ix, niy = iy + 1, niz = iz;
                        if(niy < Ny && (*voxel_)[nix][niy][niz] == (*voxel_)[ix][iy][iz])
                        {
                            std::shared_ptr<VoxelizedRenderUnit> unit = std::make_shared<VoxelizedRenderUnit>(cylinderY);
                            unit->dV = Vector3d((ix) * hwdith, (iy + 0.5) * hwdith, (iz) * hwdith);
                            unit->dF = Vector3i(present_vertices_num, present_vertices_num, present_vertices_num);
                            unit->gid = (*voxel_)[ix][iy][iz];
                            unit->mark = 0;
                            add_to_render_list(unit);
                        }
                    }

                    //+Z
                    {
                        int nix = ix, niy = iy, niz = iz + 1;
                        if(niz < Nz && (*voxel_)[nix][niy][niz] == (*voxel_)[ix][iy][iz])
                        {
                            std::shared_ptr<VoxelizedRenderUnit> unit = std::make_shared<VoxelizedRenderUnit>(cylinderZ);
                            unit->dV = Vector3d((ix) * hwdith, (iy) * hwdith, (iz + 0.5) * hwdith);
                            unit->dF = Vector3i(present_vertices_num, present_vertices_num, present_vertices_num);
                            unit->gid = (*voxel_)[ix][iy][iz];
                            unit->mark = 0;
                            add_to_render_list(unit);
                        }
                    }
                }
            }
        }
    }

    V = MatrixXd(nvertices, 3);
    F = MatrixXi(nfaces, 3);
    C = MatrixXd(nfaces, 3);

    int iV = 0, iF = 0;
    for(std::shared_ptr<VoxelizedRenderUnit> unit : render_lists)
    {
        //debug
        if(is_show_extra_mark_color && unit->mark == 1)
            unit->V *= (1 + unit->mark);
        if(is_show_extra_mark_color && unit->mark == 2)
        {
            unit->gid += 1;
            unit->V *= 1.5;
        }
        if(is_show_extra_mark_color && unit->mark == 3)
        {
            unit->gid += 2;
            unit->V *= 1.5;
        }


        for(int id = 0; id < unit->V.rows(); id++) unit->V.row(id) += unit->dV;
        for(int id = 0; id < unit->F.rows(); id++) unit->F.row(id) += unit->dF;

        V.block(iV, 0, unit->V.rows(), 3) = unit->V;
        F.block(iF, 0, unit->F.rows(), 3) = unit->F;
        iV += unit->V.rows();
        iF += unit->F.rows();
    }

    add_color_by_groupID(C);

    render_lists.clear();
}

void VoxelizedRenderSphere::add_color_by_groupID(MatrixXd &C) {

    compute_parts_num();
    colorcoder_->request(part_num_ + 2);

    int iC = 0;
    for(auto unit : render_lists)
    {
        int nF = unit->F.rows();
        for(int id = iC; id < iC + nF; id++)
        {
            C.row(id) = colorcoder_->get(unit->gid);
        }
        iC += nF;
    }
    return;
}

bool VoxelizedRenderSphere::loadSphere(VoxelizedRenderUnit &sphere)
{
    string path = LIBIGL_PATH;
    path += "/tutorial/shared/sphere.obj";
    if(igl::readOBJ(path, sphere.V, sphere.F))
    {
        double scale = sphere_radius / 0.7;
        sphere.V *= scale;
        return true;
    }
    else
    {
        return false;
    }
}

bool VoxelizedRenderSphere::loadCylinderX(VoxelizedRenderUnit &cylinderX)
{
    string path = LIBIGL_PATH;
    path += "/tutorial/shared/xcylinder.obj";
    if(igl::readOBJ(path, cylinderX.V, cylinderX.F))
    {
        double scale = cylinder_length / 1.6;
        for(int id = 0; id < cylinderX.V.rows(); id++)
        {
            cylinderX.V(id, 0) *= scale;
            cylinderX.V(id, 1) *= 0.5;
            cylinderX.V(id, 2) *= 0.5;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool VoxelizedRenderSphere::loadCylinderY(VoxelizedRenderUnit &cylinderY)
{
    string path = LIBIGL_PATH;
    path += "/tutorial/shared/ycylinder.obj";
    if(igl::readOBJ(path, cylinderY.V, cylinderY.F))
    {
        double scale = cylinder_length / 1.6;
        for(int id = 0; id < cylinderY.V.rows(); id++)
        {
            cylinderY.V(id, 1) *= scale;
            cylinderY.V(id, 2) *= 0.5;
            cylinderY.V(id, 0) *= 0.5;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool VoxelizedRenderSphere::loadCylinderZ(VoxelizedRenderUnit &cylinderZ)
{
    string path = LIBIGL_PATH;
    path += "/tutorial/shared/zcylinder.obj";
    if(igl::readOBJ(path, cylinderZ.V, cylinderZ.F))
    {
        double scale = cylinder_length / 1.6;
        for(int id = 0; id < cylinderZ.V.rows(); id++)
        {
            cylinderZ.V(id, 2) *= scale;
            cylinderZ.V(id, 1) *= 0.5;
            cylinderZ.V(id, 0) *= 0.5;
        }
        return true;
    }
    else
    {
        return false;
    }
}

void VoxelizedRenderSphere::add_to_render_list(std::shared_ptr<VoxelizedRenderUnit> unit)
{
    render_lists.push_back(unit);
    present_vertices_num += unit->V.rows();
    nvertices += unit->V.rows();
    nfaces += unit->F.rows();
}
