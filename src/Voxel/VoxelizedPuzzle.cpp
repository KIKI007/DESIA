 //
// Created by *** on 16.03.18.
//
#include "VoxelizedPuzzle.h"
VoxelizedPuzzle::VoxelizedPuzzle(int nx, int ny, int nz, std::shared_ptr<ColorCoding> coder)
{
    Nx = nx;
    Ny = ny;
    Nz = nz;
    array_3i A(boost::extents[Nx][Ny][Nz]);
    for(int ix = 0; ix < Nx; ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
            {
                A[ix][iy][iz] = -1;
            }
        }
    }
    colorcoder_ = coder;
    init(A);
}

VoxelizedPuzzle::VoxelizedPuzzle(int nx, int ny, int nz, const array_3b &array, std::shared_ptr<ColorCoding> coder) {
    Nx = nx;
    Ny = ny;
    Nz = nz;
    array_3i A(boost::extents[Nx][Ny][Nz]);
    for(int ix = 0; ix < Nx; ix++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int iz = 0; iz < Nz; iz++)
            {
                if(array[ix][iy][iz] == true)
                {
                    A[ix][iy][iz] = 0;
                }
                else
                {
                    A[ix][iy][iz] = -1;
                }
            }
        }
    }
    colorcoder_ = coder;
    init(A);
}

VoxelizedPuzzle::VoxelizedPuzzle(int nx, int ny, int nz, const array_3i &array, std::shared_ptr<ColorCoding> coder)
{
     Nx = nx;
     Ny = ny;
     Nz = nz;
     colorcoder_ = coder;
     init(array);
}

void VoxelizedPuzzle::init(const array_3i &array)
{
    int num_part = -1;
    for(int iz = 0; iz < Nz; iz++)
    {
        for(int iy = 0; iy < Ny; iy++)
        {
            for(int ix = 0; ix < Nx; ix++)
            {
                if(array[ix][iy][iz] != -1)
                {
                    shared_pEmt p = std::make_shared<VoxelElement>(VoxelElement(Vector3i(ix, iy, iz), V2I(Vector3i(ix, iy, iz))));
                    p->gid_ = array[ix][iy][iz];
                    map_ip_.insert(std::make_pair(p->order_, p.get()));
                    voxel_.push_back(p);
                    if(num_part < array[ix][iy][iz]) num_part = array[ix][iy][iz];
                }
            }
        }
    }

    num_part += 1;

    vector<OrderedVElemList> vlist;
    vlist.resize(num_part);
    for(shared_pEmt voxel : voxel_)
    {
        vlist[voxel->gid_].force_push(voxel.get());
    }

    parts_.resize(num_part);
    Vector3i size(Nx, Ny, Nz);
    for(int id = 0; id < num_part; id++)
    {
        parts_[id] = std::make_shared<VoxelizedPart>(VoxelizedPart(vlist[id], &map_ip_, id, size));
        parts_[id]->compute_access_map(&map_access_);
    }

    for(shared_pPart part: parts_)
        build_part_connection(part.get());

    return;
}

void VoxelizedPuzzle::build_part_connection(VoxelizedPart *vpart)
//build connection
{
    int num_part = parts_.size();
    int pid = vpart->part_id_;

    //for three direction
    int dX[3] = {1, 0, 0};
    int dY[3] = {0, 1, 0};
    int dZ[3] = {0, 0, 1};

    for(int XYZ = 0; XYZ < 3; XYZ++)
    {
        MatrixXi Graph = MatrixXi::Zero(num_part, num_part);
        vpart->neighbor_[XYZ].clear();
        VPartNeighbor &neighbor = vpart->neighbor_[XYZ];
        Vector3i npos;
        for(pEmt voxel : vpart->elist_.data_)
        {
            //X/Y/Z + positive
            //X/Y/Z_npos >= X/Y/Z_pos
            npos = voxel->pos_ + Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
            int npid = get_part_id(npos);
            if(npid != -1 && npid != pid)
            {
                if (Graph(npid, pid) == 0)
                {
                    neighbor.in_.push_back(parts_[npid].get());
                    std::shared_ptr<OrderedVElemList> pOVEL = std::make_shared<OrderedVElemList>(OrderedVElemList());
                    neighbor.in_blocking_.push_back(pOVEL);
                    Graph(npid, pid) = neighbor.in_blocking_.size();
                }
                OrderedVElemList* pOVEL = neighbor.in_blocking_[Graph(npid, pid) - 1].get();
                if(pOVEL) pOVEL->force_push(voxel);
            }

            //X/Y/Z - negative
            //X/Y/Z_npos <= X/Y/Z_pos
            npos = voxel->pos_ - Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
            npid = get_part_id(npos);
            if(npid != -1 && npid != pid)
            {
                if (Graph(pid, npid) == 0)
                {
                    neighbor.out_.push_back(parts_[npid].get());
                    std::shared_ptr<OrderedVElemList> pOVEL = std::make_shared<OrderedVElemList>(OrderedVElemList());
                    neighbor.out_blocking_.push_back(pOVEL);
                    Graph(pid, npid) = neighbor.out_blocking_.size();
                }
                OrderedVElemList* pOVEL = neighbor.out_blocking_[Graph(pid, npid) - 1].get();
                if(pOVEL) pOVEL->force_push(voxel);
            }

        }
    }
    return;
}

int VoxelizedPuzzle::get_part_id(Vector3i pos)
{
     unordered_map<int, pEmt>::iterator find_it = map_ip_.find(V2I(pos));
     if(find_it == map_ip_.end())
         return -1;
     else
         return find_it->second->gid_;
}

std::shared_ptr<VoxelizedInterface> VoxelizedPuzzle::output_assembly_interface(bool accessibility = true, bool vertex_mark = true)
{

    std::shared_ptr<VoxelizedInterface> interface = std::make_shared<VoxelizedInterface>(VoxelizedInterface(Nx, Ny, Nz, colorcoder_));

    //set grid
    for(shared_pEmt voxel : voxel_)
    {
        interface->set_grid_part_index(voxel->pos_, voxel->gid_);
    }

    //set accessibility
    if(accessibility)
    {
        interface->use_extra_value_color(true);
        for(shared_pPart part: parts_)
        {
            for(pEmt voxel : part->elist_.data_)
            {
                interface->set_grid_extra_value(voxel->pos_, map_access_[voxel->order_]);
            }
        }
    }

    //set mark
    //for debuging
    if(vertex_mark)
    {
        interface->use_extra_mark_color(true);
        for(auto it = map_mark.begin(); it != map_mark.end(); it++)
        {
            interface->set_grid_extra_mark(map_ip_[it->first]->pos_, it->second);
        }
    }

    return interface;
}

 VoxelizedPuzzle::VoxelizedPuzzle(const VoxelizedPuzzle &A) {
     Nx = A.Nx;
     Ny = A.Ny;
     Nz = A.Nz;

     map_access_ = A.map_access_;
     map_mark = A.map_mark;

     voxel_.clear();
     map_ip_.clear();
     for (int id = 0; id < A.voxel_.size(); id++) {
         shared_pEmt p = std::make_shared<VoxelElement>(VoxelElement(*A.voxel_[id]));
         voxel_.push_back(p);
         map_ip_.insert(std::make_pair(p->order_, p.get()));
     }

     parts_.clear();
     for (int id = 0; id < A.parts_.size(); id++)
     {
         OrderedVElemList part_voxel_list;
         for(pEmt em : A.parts_[id]->elist_.data_)
         {
             pEmt voxel = map_ip_[em->order_];
             part_voxel_list.force_push(voxel);
         }

         shared_pPart part = std::make_shared<VoxelizedPart>(VoxelizedPart(part_voxel_list, &map_ip_, id, A.parts_[id]->size_));
         part->map_access_ = &map_access_;
         parts_.push_back(part);
     }

     //neighboring relation
     for(shared_pPart partA: A.parts_)
     {
         pPart part = parts_[partA->part_id_].get();
         for(int XYZ = 0; XYZ < 3; XYZ++)
         {
             //in
             for(pPart n_partA: partA->neighbor_[XYZ].in_)
             {
                 part->neighbor_[XYZ].in_.push_back(parts_[n_partA->part_id_].get());
             }

             //in_blocking
             for(int id = 0; id < partA->neighbor_[XYZ].in_blocking_.size(); id++)
             {
                 std::shared_ptr<OrderedVElemList> shared_list = std::make_shared<OrderedVElemList>();
                 part->neighbor_[XYZ].in_blocking_.push_back(shared_list);
                 for(pEmt vA: partA->neighbor_[XYZ].in_blocking_[id]->data_)
                 {
                     pEmt voxel = map_ip_[vA->order_];
                     shared_list->force_push(voxel);
                 }
             }

             //out
             for(pPart n_partA: partA->neighbor_[XYZ].out_)
             {
                 part->neighbor_[XYZ].out_.push_back(parts_[n_partA->part_id_].get());
             }

             //out_blocking
             for(int id = 0; id < partA->neighbor_[XYZ].out_blocking_.size(); id++)
             {
                 std::shared_ptr<OrderedVElemList> shared_list = std::make_shared<OrderedVElemList>();
                 part->neighbor_[XYZ].out_blocking_.push_back(shared_list);
                 for(pEmt vA: partA->neighbor_[XYZ].out_blocking_[id]->data_)
                 {
                     pEmt voxel = map_ip_[vA->order_];
                     shared_list->force_push(voxel);
                 }
             }
         }
     }

     colorcoder_ = A.colorcoder_;
 }

 void VoxelizedPuzzle::partition_part(pPart part, vector<pEmt> remove_portion)
 {
     //update
     vector<pPart> update_list;
     vector<bool> visited;
     visited.resize(parts_.size(), false);
     for(int XYZ = 0; XYZ < 3; XYZ++)
     {
         for(pPart Neighbor_part : part->neighbor_[XYZ].in_)
         {
             if(!visited[Neighbor_part->part_id_])
             {
                 visited[Neighbor_part->part_id_] = true;
                 update_list.push_back(Neighbor_part);
             }
         }
         for(pPart Neighbor_part : part->neighbor_[XYZ].out_)
         {
             if(!visited[Neighbor_part->part_id_])
             {
                 visited[Neighbor_part->part_id_] = true;
                 update_list.push_back(Neighbor_part);
             }
         }
     }

     //change id
     OrderedVElemList list;
     for(pEmt v: remove_portion)
     {
         pEmt u = map_ip_[v->order_];
         u->gid_ += 1;
         list.push(u);
     }

     part->remove_voxels();
     shared_pPart new_part = std::make_shared<VoxelizedPart>(VoxelizedPart(list, &map_ip_, part->part_id_+1, part->size_));
     new_part->compute_access_map(&map_access_);
     parts_.push_back(new_part);

     //build
     build_part_connection(part);
     build_part_connection(new_part.get());

     //update
     for(int id = 0; id < update_list.size(); id++)
     {
         build_part_connection(update_list[id]);
     }
//     for(int id = 0; id < parts_.size(); id++)
//     {
//         build_part_connection(parts_[id].get());
//     }
 }

 VoxelizedPuzzle::VoxelizedPuzzle()
 {
     Nx = Ny = Nz = 0;
 }

 void VoxelizedPuzzle::update_part_connection(pPart part)
 {
     VPartNeighbor new_neighbor[3];
     for(int XYZ = 0; XYZ < 3; XYZ++)
     {
         //in_blocking
         std::shared_ptr<OrderedVElemList> remain_list = std::make_shared<OrderedVElemList>();

         //for three direction
         int dX[3] = {1, 0, 0};
         int dY[3] = {0, 1, 0};
         int dZ[3] = {0, 0, 1};
         for(int id = 0; id < part->neighbor_[XYZ].in_blocking_.size(); id++)
         {
             pPart part_Neighbor = part->neighbor_[XYZ].in_[id];
             std::shared_ptr<OrderedVElemList> vlist = std::make_shared<OrderedVElemList>();
             for(pEmt voxel: part->neighbor_[XYZ].in_blocking_[id]->data_)
             {

                 Vector3i npos = voxel->pos_ + Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
                 int npid = get_part_id(npos);

                 if(npid != part_Neighbor->part_id_)
                 {
                     remain_list->force_push(voxel);
                 }
                 else
                 {
                     vlist->force_push(voxel);
                 }

                 if(!vlist->data_.empty())
                 {
                     new_neighbor[XYZ].in_blocking_.push_back(vlist);
                     new_neighbor[XYZ].in_.push_back(part_Neighbor);
                 }

             }
         }
         if(!remain_list->data_.empty())
         {
             new_neighbor->in_.push_back(parts_.back().get());
             new_neighbor->in_blocking_.push_back(remain_list);
         }

         remain_list.reset();

         //out_blocking
         remain_list = std::make_shared<OrderedVElemList>();
         for(int id = 0; id < part->neighbor_[XYZ].out_blocking_.size(); id++)
         {
             pPart part_Neighbor = part->neighbor_[XYZ].out_[id];
             std::shared_ptr<OrderedVElemList> vlist = std::make_shared<OrderedVElemList>();
             for(pEmt voxel: part->neighbor_[XYZ].out_blocking_[id]->data_)
             {
                 Vector3i npos = voxel->pos_ - Vector3i(dX[XYZ], dY[XYZ], dZ[XYZ]);
                 int npid = get_part_id(npos);
                 if(npid != part_Neighbor->part_id_)
                 {
                     remain_list->force_push(voxel);
                 }
                 else
                 {
                     vlist->force_push(voxel);
                 }

                 if(!vlist->data_.empty())
                 {
                     new_neighbor[XYZ].out_blocking_.push_back(vlist);
                     new_neighbor[XYZ].out_.push_back(part_Neighbor);
                 }

             }
         }
         if(!remain_list->data_.empty())
         {
             new_neighbor->out_.push_back(parts_.back().get());
             new_neighbor->out_blocking_.push_back(remain_list);
         }

         part->neighbor_[XYZ] = new_neighbor[XYZ];
     }
 }

 bool VoxelizedPuzzle::is_same(VoxelizedPuzzle *A) {
     for(int id = 0; id < Nx * Ny * Nz; id++)
     {
         auto find_it = map_ip_.find(id);
         if(find_it == map_ip_.end()) continue;
         if(map_ip_[id]->gid_ != A->map_ip_[id]->gid_)
             return false;
     }
     return true;
 }


