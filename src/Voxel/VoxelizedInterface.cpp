//
// Created by *** on 11.03.18.
//

#include "VoxelizedInterface.h"
#include <iostream>
VoxelizedInterface::VoxelizedInterface(int nx, int ny, int nz, std::shared_ptr<ColorCoding> coder) {
    init(nx, ny, nz);
    colorcoder_ = coder;
    is_show_extra_mark_color = is_show_extra_value_color = false;
}

void VoxelizedInterface::init(int nx, int ny, int nz)
{

    mesh_ = nullptr; //this pointer should be set null manually

    Nx = nx; Ny = ny; Nz = nz;

    //allocate nx*ny*nz space for voxel class
    voxel_.reset();
    voxel_ = std::make_shared<array_3i>(array_3i(boost::extents[Nx][Ny][Nz]));


    for(int ix = 0; ix < Nx;ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
            {
                (*voxel_)[ix][iy][iz] = -1;
            }
        }
    }
}

void VoxelizedInterface::set_grid_part_index(Vector3i pos, int index)
{

    assert(0 <= pos[0] && pos[0] < Nx);
    assert(0 <= pos[1] && pos[1] < Ny);
    assert(0 <= pos[2] && pos[2] < Nz);

    if(index >= 0) (*voxel_)[pos[0]][pos[1]][pos[2]] = index;
}

int VoxelizedInterface::compute_voxels_num() {
/* calculate voxel size */
    nvoxels_ = 0;
    for(int ix = 0; ix < Nx; ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
            {
                if((*voxel_)[ix][iy][iz] != -1)
                {
                    nvoxels_++;
                }
            }
        }
    }
    return nvoxels_;
}

int VoxelizedInterface::compute_parts_num()
{
/* calculate how many part does the assembly has? */
    part_num_ = -1;
    for(int ix = 0; ix < Nx; ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
            {
                if(part_num_ < (*voxel_)[ix][iy][iz])
                    part_num_ = (*voxel_)[ix][iy][iz];
            }
        }
    }

    part_num_ = part_num_ + 1; //starting from 0;

    return part_num_;
}

int VoxelizedInterface::get_grid_part_index(Vector3i pos) {
    return (*voxel_)[pos[0]][pos[1]][pos[2]];
}

void VoxelizedInterface::output_file(string file_name)
{
    std::ofstream fout(file_name);
    if(fout.fail()) return;
    fout << Nx << " " << Ny << " " << Nz << std::endl;

    for(int iz = 0; iz < Nz; iz ++)
    {
        for(int iy = 0; iy < Ny; iy ++)
        {
            for(int ix = 0; ix < Nx; ix ++)
            {
                int index = 0;
                fout << (*voxel_)[ix][iy][iz] + 1 << " ";
            }
            fout << std::endl;
        }
        fout << std::endl;
    }

}

std::shared_ptr<Assembly> VoxelizedInterface::output_assembly() {

    if(compute_parts_num() == 0)
        return nullptr;

    int ix, iy, iz;
    int nix, niy, niz;
    int ID, nID;


    /* allocate space for assembly class */
    assembly_.reset();
    assembly_ = std::make_shared<Assembly>(Assembly(part_num_));


    /* Matrix for recording joints between different part */
    //voxel model only have 6 normal directions:
    //000001b: ( 1,  0,  0)
    //000010b: (-1,  0,  0)
    //000100b: ( 0,  1,  0)
    //001000b: ( 0, -1,  0)
    //010000b: ( 0,  0,  1)
    //100000b: ( 0,  0, -1)
    MatrixXi Joint(part_num_, part_num_);
    Joint.setZero();

    /* compute joint between different part */
    // Going through all voxels.
    // Checking its up, down, left, right, front, back.
    int dX[6] = {1, -1,  0,  0, 0,  0};
    int dY[6] = {0,  0,  1, -1, 0,  0};
    int dZ[6] = {0,  0,  0,  0, 1, -1};

    for(ix = 0; ix < Nx; ix++)
    {
        for(iy = 0; iy < Ny; iy++)
        {
            for(iz = 0; iz < Nz; iz++)
            {
                ID = (*voxel_)[ix][iy][iz];
                if(ID != -1)
                {
                    for(int id = 0; id < 6; id++)
                    {
                        //(nix, niy, niz) is adjacent voxel of (ix, iy, iz)
                        nix = ix + dX[id];
                        niy = iy + dY[id];
                        niz = iz + dZ[id];

                        //check whether is outside of the model
                        if(nix < 0 || niy < 0 || niz < 0) continue;
                        if(nix >= Nx || niy >= Ny || niz >= Nz) continue;

                        //create joint in the matrix
                        nID = (*voxel_)[nix][niy][niz];
                        if(nID != -1 && ID < nID)
                        {
                            Joint(ID, nID) |= (1 << id);
                        }
                    }
                }
            }
        }
    }

    //create joint in the assembly.
    for(int id = 0; id < part_num_; id++)
    {
        for(int jd = id + 1; jd < part_num_; jd++)
        {
            vecVector3d nrms;
            if(Joint(id, jd) & 1) nrms.push_back(Vector3d(1, 0, 0));
            if(Joint(id, jd) & 2) nrms.push_back(Vector3d(-1, 0, 0));
            if(Joint(id, jd) & 4) nrms.push_back(Vector3d(0, 1, 0));
            if(Joint(id, jd) & 8) nrms.push_back(Vector3d(0, -1, 0));
            if(Joint(id, jd) & 16) nrms.push_back(Vector3d(0, 0, 1));
            if(Joint(id, jd) & 32) nrms.push_back(Vector3d(0, 0, -1));
            if(nrms.size() > 0) assembly_->add_contact(id, jd, nrms);
        }
    }

    return assembly_;
}


std::shared_ptr<ContactGraph> VoxelizedInterface::output_contactgraph()
{
    if(compute_parts_num() == 0)
        return nullptr;

    int ix, iy, iz;
    int nix, niy, niz;
    int ID, nID;


    /* allocate space for assembly class */
    shared_ptr<ContactGraph> graph = make_shared<ContactGraphMosekSolver>(1e-7);

    /*
     * 1. Create each part
     */
    compute_parts_num();
    vecVector3d centers; centers.resize(part_num_, Eigen::Vector3d(0, 0, 0));
    vector<int> num_voxels; num_voxels.resize(part_num_, 0);
    for(ix = 0; ix < Nx; ix++) {
        for (iy = 0; iy < Ny; iy++) {
            for (iz = 0; iz < Nz; iz++) {
                ID = (*voxel_)[ix][iy][iz];
                if (ID != -1) {
                    centers[ID] += Vector3d(ix + 0.5, iy + 0.5, iz + 0.5);
                    num_voxels[ID] += 1;
                }
            }
        }
    }

    for(int id = 0; id < part_num_; id++){
        centers[id] /= num_voxels[id];
        shared_ptr<ContactGraphNode> node;
        if(id < 2) node = make_shared<ContactGraphNode>(true, centers[id], centers[id]);
        else node = make_shared<ContactGraphNode>(false, centers[id], centers[id]);
        graph->addNode(node);
    }

    /* compute contact between different part */
    // Going through all voxels.
    // Checking its up, down, left, right, front, back.
    int dX[6] = {1, -1,  0,  0, 0,  0};
    int dY[6] = {0,  0,  1, -1, 0,  0};
    int dZ[6] = {0,  0,  0,  0, 1, -1};
    for(ix = 0; ix < Nx; ix++) {
        for (iy = 0; iy < Ny; iy++) {
            for (iz = 0; iz < Nz; iz++) {
                ID = (*voxel_)[ix][iy][iz];
                if (ID != -1) {
                    for(int id = 0; id < 6; id++)
                    {
                        //(nix, niy, niz) is adjacent voxel of (ix, iy, iz)
                        nix = ix + dX[id];
                        niy = iy + dY[id];
                        niz = iz + dZ[id];

                        //check whether is outside of the model
                        if(nix < 0 || niy < 0 || niz < 0) continue;
                        if(nix >= Nx || niy >= Ny || niz >= Nz) continue;

                        //neighbor exist
                        nID = (*voxel_)[nix][niy][niz];
                        if(nID != -1 && ID < nID)
                        {
                            /*
                             * 1. Compute Contact Polygon
                             */
                            ContactPolygon poly;
                            EigenPoint nrm;
                            EigenPoint pt(ix, iy, iz);
                            switch (id){
                                case 0: {
                                    nrm = EigenPoint(1, 0, 0);
                                    poly.points.push_back(EigenPoint(1, 0, 0) + pt);
                                    poly.points.push_back(EigenPoint(1, 1, 0) + pt);
                                    poly.points.push_back(EigenPoint(1, 1, 1) + pt);
                                    poly.points.push_back(EigenPoint(1, 0, 1) + pt);
                                    poly.center = EigenPoint(1, 0.5, 0.5) + pt;
                                    break;
                                }
                                case 1: {
                                    nrm = EigenPoint(-1, 0, 0);
                                    poly.points.push_back(EigenPoint(0, 0, 0) + pt);
                                    poly.points.push_back(EigenPoint(0, 0, 1) + pt);
                                    poly.points.push_back(EigenPoint(0, 1, 1) + pt);
                                    poly.points.push_back(EigenPoint(0, 1, 0) + pt);
                                    poly.center = EigenPoint(0, 0.5, 0.5) + pt;
                                    break;
                                }
                                case 2: {
                                    nrm = EigenPoint(0, 1, 0);
                                    poly.points.push_back(EigenPoint(1, 1, 0) + pt);
                                    poly.points.push_back(EigenPoint(0, 1, 0) + pt);
                                    poly.points.push_back(EigenPoint(0, 1, 1) + pt);
                                    poly.points.push_back(EigenPoint(1, 1, 1) + pt);
                                    poly.center = EigenPoint(0.5, 1, 0.5) + pt;
                                    break;
                                }
                                case 3: {
                                    nrm = EigenPoint(0, -1, 0);
                                    poly.points.push_back(EigenPoint(0, 0, 0) + pt);
                                    poly.points.push_back(EigenPoint(1, 0, 0) + pt);
                                    poly.points.push_back(EigenPoint(1, 0, 1) + pt);
                                    poly.points.push_back(EigenPoint(0, 0, 1) + pt);
                                    poly.center = EigenPoint(0.5, 0, 0.5) + pt;
                                    break;
                                }
                                case 4: {
                                    nrm = EigenPoint(0, 0, 1);
                                    poly.points.push_back(EigenPoint(0, 0, 1) + pt);
                                    poly.points.push_back(EigenPoint(1, 0, 1) + pt);
                                    poly.points.push_back(EigenPoint(1, 1, 1) + pt);
                                    poly.points.push_back(EigenPoint(0, 1, 1) + pt);
                                    poly.center = EigenPoint(0.5, 0.5, 1) + pt;
                                    break;
                                }
                                case 5: {
                                    nrm = EigenPoint(0, 0, -1);
                                    poly.points.push_back(EigenPoint(0, 0, 0) + pt);
                                    poly.points.push_back(EigenPoint(0, 1, 0) + pt);
                                    poly.points.push_back(EigenPoint(1, 1, 0) + pt);
                                    poly.points.push_back(EigenPoint(1, 0, 0) + pt);
                                    poly.center = EigenPoint(0.5, 0.5, 0) + pt;
                                    break;
                                }
                            }

                            shared_ptr<ContactGraphNode> nodeA, nodeB;
                            nodeA = graph->nodes[ID];
                            nodeB = graph->nodes[nID];
                            shared_ptr<ContactGraphEdge> edge;
                            graph->findEdge(nodeA, nodeB, edge);
                            if(edge == nullptr){
                                edge = make_shared<ContactGraphEdge>(poly, nrm);
                                graph->addContact(nodeA, nodeB, edge);
                            }
                            else{
                                edge->polygons.push_back(poly);
                                edge->normals.push_back(nrm);
                            }
                        }
                    }
                }
            }
        }
    }

    return graph;
}

std::shared_ptr<VoxelizedInterface> VoxelizedInterface::output_section_by_Z(int Zlayer) {
    int nx = Nx;
    int ny = Ny;
    int nz = Zlayer > 0 ? Zlayer : Nz / 2;

    std::shared_ptr<VoxelizedInterface> p = std::make_shared<VoxelizedInterface>(VoxelizedInterface(nx, ny, nz, colorcoder_));

    if(voxel_value_) p->use_extra_value_color(true);
    if(voxel_mark_) p->use_extra_mark_color(true);

    for(int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                p->set_grid_part_index(Vector3i(i, j, k), (*voxel_)[i][j][k]);
                if(voxel_value_) p->set_grid_extra_value(Vector3i(i, j, k), (*voxel_value_)[i][j][k]);
                if(voxel_mark_) p->set_grid_extra_mark(Vector3i(i, j, k), (*voxel_value_)[i][j][k]);
            }
        }
    }

    return p;
}

std::shared_ptr<VoxelizedInterface> VoxelizedInterface::output_partial_part(int min_part_index) {

    std::shared_ptr<VoxelizedInterface> p = std::make_shared<VoxelizedInterface>(VoxelizedInterface(Nx, Ny, Nz, colorcoder_));
    if(voxel_value_) p->use_extra_value_color(true);
    if(voxel_mark_) p->use_extra_mark_color(true);

    for(int i = 0; i < Nx; i++) {
        for (int j = 0; j < Ny; j++) {
            for (int k = 0; k < Nz; k++) {
                if((*voxel_)[i][j][k] < min_part_index)
                {
                    p->set_grid_part_index(Vector3i(i, j, k), -1);
                    if(voxel_value_) p->set_grid_extra_value(Vector3i(i, j, k), -1);
                    if(voxel_mark_)p->set_grid_extra_mark(Vector3i(i, j, k), -1);
                }
                else if((*voxel_)[i][j][k] == min_part_index)
                {
                    p->set_grid_part_index(Vector3i(i, j, k), 0);
                    if(voxel_value_) p->set_grid_extra_value(Vector3i(i, j, k), (*voxel_value_)[i][j][k]);
                    if(voxel_mark_)  p->set_grid_extra_mark(Vector3i(i, j, k), (*voxel_mark_)[i][j][k]);
                }
                else
                {
                    p->set_grid_part_index(Vector3i(i, j, k), 1);
                    if(voxel_value_) p->set_grid_extra_value(Vector3i(i, j, k), (*voxel_value_)[i][j][k]);
                    if(voxel_mark_)  p->set_grid_extra_mark(Vector3i(i, j, k), (*voxel_mark_)[i][j][k]);
                }
            }
        }
    }

    return p;
}

void VoxelizedInterface::use_extra_value_color(bool option)
{
    if(option)
    {
        voxel_value_.reset();
        voxel_value_ = std::make_shared<array_3d>(array_3d(boost::extents[Nx][Ny][Nz]));
        for(int ix = 0; ix < Nx;ix++)
        {
            for(int iy = 0; iy < Ny; iy++)
            {
                for(int iz = 0; iz < Nz; iz++)
                {
                    (*voxel_value_)[ix][iy][iz] = -1;
                }

            }
        }
    }
}

void VoxelizedInterface::set_grid_extra_value(Vector3i pos, double value)
{
    assert(0 <= pos[0] && pos[0] < Nx);
    assert(0 <= pos[1] && pos[1] < Ny);
    assert(0 <= pos[2] && pos[2] < Nz);

    (*voxel_value_)[pos[0]][pos[1]][pos[2]] = value;
}

double VoxelizedInterface::get_grid_extra_value(Vector3i pos) {

    assert(0 <= pos[0] && pos[0] < Nx);
    assert(0 <= pos[1] && pos[1] < Ny);
    assert(0 <= pos[2] && pos[2] < Nz);
    return (*voxel_value_)[pos[0]][pos[1]][pos[2]];
}

void VoxelizedInterface::set_grid_extra_mark(Vector3i pos, int mark) {

    assert(0 <= pos[0] && pos[0] < Nx);
    assert(0 <= pos[1] && pos[1] < Ny);
    assert(0 <= pos[2] && pos[2] < Nz);

    (*voxel_mark_)[pos[0]][pos[1]][pos[2]] = mark;
}

int VoxelizedInterface::get_grid_extra_mark(Vector3i pos) {

    assert(0 <= pos[0] && pos[0] < Nx);
    assert(0 <= pos[1] && pos[1] < Ny);
    assert(0 <= pos[2] && pos[2] < Nz);
    return (*voxel_mark_)[pos[0]][pos[1]][pos[2]];
}

void VoxelizedInterface::use_extra_mark_color(bool option)
{
    if(option)
    {
        voxel_mark_.reset();
        voxel_mark_ = std::make_shared<array_3i>(array_3i(boost::extents[Nx][Ny][Nz]));
        for(int ix = 0; ix < Nx;ix++)
        {
            for(int iy = 0; iy < Ny; iy++)
            {
                for(int iz = 0; iz < Nz; iz++)
                {
                    (*voxel_mark_)[ix][iy][iz] = 0;
                }

            }
        }
    }
}

void VoxelizedInterface::reverse_YZ() {

    std::shared_ptr<array_3i> new_voxel_ = std::make_shared<array_3i>(array_3i(boost::extents[Nx][Ny][Nz]));
    for(int ix = 0; ix < Nx;ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
            {
                (*new_voxel_)[ix][iy][iz] = (*voxel_)[ix][Ny - iy - 1][Nz - iz - 1];
            }
        }
    }
    voxel_ = new_voxel_;
    voxel_mark_.reset();
    voxel_value_.reset();
    is_show_extra_mark_color = false;
    is_show_extra_value_color = false;
}

void VoxelizedInterface::merge_into_one_part() {
    std::shared_ptr<array_3i> new_voxel_ = std::make_shared<array_3i>(array_3i(boost::extents[Nx][Ny][Nz]));
    for(int ix = 0; ix < Nx;ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
            {
                if((*voxel_)[ix][iy][iz] != -1)
                    (*new_voxel_)[ix][iy][iz] = 0;
                else
                    (*new_voxel_)[ix][iy][iz] = -1;
            }
        }
    }
    voxel_ = new_voxel_;
    voxel_mark_.reset();
    voxel_value_.reset();
    is_show_extra_mark_color = false;
    is_show_extra_value_color = false;
}
