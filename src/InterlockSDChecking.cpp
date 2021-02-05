//
// Created by *** on 01.03.18.
//

#include "InterlockSDChecking.h"

InterlockSDData InterlockSDChecking::check_interlock_sd(Vector3d dv, const Assembly& assembly)
{
    init_sd_graph(dv, assembly);

    DirectedGraphSSC ssc(*blocking_graph_);
    ssc.tarjan_SSC();

    InterlockSDData data;
    data.ssc_groups = ssc.ssc_group_;
    switch (data.ssc_groups.size())
    {
        case 1:
            data.status = HAS_NO_KEY;
            break;
        case 2:
            data.status = HAS_KEY;
            break;
        default:
            data.status = NOT_INTERLOCKING;
            break;
    }
    return data;
}

std::shared_ptr<DirectedGraph> InterlockSDChecking::init_sd_graph(Vector3d dv, const Assembly&assembly)
{
    int pID, qID;
    std::shared_ptr<Part> p, q;

    blocking_graph_.reset();
    blocking_graph_ = std::make_shared<DirectedGraph>(DirectedGraph(assembly.partLists_.size()));

    for(int id = 0; id < assembly.partLists_.size(); id++)
    {
        p = assembly.partLists_[id]; pID = p->index_;
        for(int jd = 0; jd < p->neighborLists_.size(); jd++)
        {
            q = p->neighborLists_[jd].node.lock(); qID = q->index_;
            if(pID < qID)
            {
                for(int kd = 0; kd < p->neighborLists_[jd].normals.size(); kd++)
                {
                    Vector3d nrm = p->neighborLists_[jd].normals[kd];
                    double sgn = nrm.dot(dv);
                    if(sgn > eps_) blocking_graph_->add_edge(qID, pID);
                    else if(sgn < -eps_) blocking_graph_->add_edge(pID, qID);
                }
            }
        }
    }

    return blocking_graph_;
}

std::shared_ptr<DirectedGraph> InterlockSDChecking::simplified_sd_graph(Vector3d dv, const Assembly &assembly) {

    InterlockSDData data = check_interlock_sd(dv, assembly);

    int part_num = data.ssc_groups.size();
    int node_num = blocking_graph_->nodeLists_.size();

    vector<int> new_index;
    new_index.resize(node_num);
    for(int id = 0; id < data.ssc_groups.size(); id++)
    {
        for(int jd = 0; jd < data.ssc_groups[id].size(); jd++)
        {
            new_index[data.ssc_groups[id][jd]] = id;
        }
    }

    MatrixXi EdgeMat(part_num, part_num);
    EdgeMat.setZero();
    std::shared_ptr<DirectedGraphNode> p, q;
    int pID, qID;
    for(int id = 0; id < node_num; id++)
    {
        p = blocking_graph_->nodeLists_[id];
        pID = new_index[p->index_];
        for(int jd = 0; jd < p->neighborList_.size(); jd++)
        {
            q = p->neighborList_[jd].node.lock();
            qID = new_index[q->index_];
            if(pID == qID) continue;
            else EdgeMat(pID, qID) = 1;
        }
    }

    simpified_graph_ = std::make_shared<DirectedGraph>(DirectedGraph(part_num));
    for(int id = 0; id < part_num; id++)
    {
        simpified_graph_->nodeLists_[id]->label = simpified_graph_->to_string(data.ssc_groups[id]);
    }

    for(int id = 0; id < part_num; id++)
    {
        p = simpified_graph_->nodeLists_[id];
        for(int jd = 0; jd < part_num; jd++)
        {
            if(EdgeMat(id, jd))
            {
                DirectedGraphEdge edge;
                edge.node = simpified_graph_->nodeLists_[jd];
                edge.weight = 1;
                p->neighborList_.push_back(edge);
            }
        }
    }

    return simpified_graph_;
}
