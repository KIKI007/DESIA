//
// Created by *** on 01.03.18.
//

#include "Assembly.h"

Assembly::Assembly() {

}

Assembly::~Assembly() {

}

void Assembly::set_part(int num){
    partLists_.clear();
    for(int id = 0; id < num; id++)
    {
        std::shared_ptr<Part> p = std::make_shared<Part>(partLists_.size());
        partLists_.push_back(p);
    }

    parts_fixed_.clear();
    parts_fixed_.resize(num, false);

    return;
}

void Assembly::add_contact(int idA, int idB, vecVector3d &nrms) {
    assert(idA >= 0 && idB >= 0);
    assert(idA < partLists_.size());
    assert(idB < partLists_.size());

    //
    std::shared_ptr<Part> p, q;
    p = partLists_[idA];
    q = partLists_[idB];

    //for p
    PartNeighborData data;
    data.normals = nrms;
    data.node = q;
    p->neighborLists_.push_back(data);

    //for q
    data.node = p;
    for(auto &x : data.normals) x = x * (-1);
    q->neighborLists_.push_back(data);

    return;
}
