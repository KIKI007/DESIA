//
// Created by ziqwang on 14.01.19.
//

#include "ContactGraph.h"

ContactGraph::ContactGraph()
{

}


ContactGraph::~ContactGraph()
{
    nodes.clear();
    edges.clear();
}


void ContactGraph::addNode(shared_ptr<ContactGraphNode> _node)
{
    _node->staticID = nodes.size();

    nodes.push_back(_node);

    return;
}

void ContactGraph::addContact(shared_ptr<ContactGraphNode> _nodeA, shared_ptr<ContactGraphNode> _nodeB, shared_ptr<ContactGraphEdge> _edge)
{

    if(_nodeA->isBoundary && _nodeB->isBoundary)
        return;

    _edge->partIDA = _nodeA->staticID;
    _edge->partIDB = _nodeB->staticID;

    edges.push_back(_edge);

    pair<wpContactGraphNode, wpContactGraphEdge> contactNeighbor;

    contactNeighbor.first = _nodeB;
    contactNeighbor.second = _edge;
    _nodeA->neighbors.push_back(contactNeighbor);

    contactNeighbor.first = _nodeA;
    contactNeighbor.second = _edge;
    _nodeB->neighbors.push_back(contactNeighbor);

    return;
}

bool ContactGraph::findEdge(shared_ptr<ContactGraphNode> _nodeA, shared_ptr<ContactGraphNode> _nodeB, shared_ptr<ContactGraphEdge> &_edge)
{
    _edge = nullptr;
    for(auto neighbor : _nodeA->neighbors){
        if(neighbor.first.lock() == _nodeB){
            _edge = neighbor.second.lock();
            return true;
        }
    }
    return false;
}

void ContactGraph::initialize()
{
    int dynamicID = 0;
    for(shared_ptr<ContactGraphNode> node: nodes){
        if(!node->isBoundary){
            node->dynamicID = dynamicID ++;
            dynamic_nodes.push_back(node);
        }
        else{
            node->dynamicID = -1;
        }
    }
}


void ContactGraph::computeTranslationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int rowID = 0;
    tri.clear();
    for(shared_ptr<ContactGraphEdge> edge: edges) {

        int iA = nodes[edge->partIDA]->dynamicID;

        int iB = nodes[edge->partIDB]->dynamicID;

        for (EigenPoint nrm : edge->normals) {
            if (iA != -1) {
                tri.push_back(EigenTriple(rowID, 3 * iA, -nrm[0]));
                tri.push_back(EigenTriple(rowID, 3 * iA + 1, -nrm[1]));
                tri.push_back(EigenTriple(rowID, 3 * iA + 2, -nrm[2]));
            }

            if (iB != -1) {
                tri.push_back(EigenTriple(rowID, 3 * iB, nrm[0]));
                tri.push_back(EigenTriple(rowID, 3 * iB + 1, nrm[1]));
                tri.push_back(EigenTriple(rowID, 3 * iB + 2, nrm[2]));
            }
            rowID++;
        }
    }
    size = Eigen::Vector2i(rowID, 3 * dynamic_nodes.size());
}

void ContactGraph::computeRotationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int rowID = 0;
    tri.clear();
    for(shared_ptr<ContactGraphEdge> edge: edges)
    {
        shared_ptr<ContactGraphNode> nodeA = nodes[edge->partIDA];
        shared_ptr<ContactGraphNode> nodeB = nodes[edge->partIDB];

        int iA = nodeA->dynamicID;
        int iB = nodeB->dynamicID;

        EigenPoint ctA = nodeA->centroid;
        EigenPoint ctB = nodeB->centroid;

        for(int id = 0; id < edge->normals.size(); id++)
        {
            ContactPolygon poly = edge->polygons[id];
            EigenPoint nrm = edge->normals[id];
            for(EigenPoint pt: poly.points)
            {
                if (iA != -1)
                {
                    EigenPoint mt = (pt - ctA).cross(nrm);
                    tri.push_back(EigenTriple(rowID, 6 * iA, -nrm[0]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 1, -nrm[1]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 2, -nrm[2]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 3, -mt[0]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 4, -mt[1]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 5, -mt[2]));
                }

                if (iB != -1) {
                    EigenPoint mt = (pt - ctB).cross(nrm);
                    tri.push_back(EigenTriple(rowID, 6 * iB, nrm[0]));
                    tri.push_back(EigenTriple(rowID, 6 * iB + 1, nrm[1]));
                    tri.push_back(EigenTriple(rowID, 6 * iB + 2, nrm[2]));
                    tri.push_back(EigenTriple(rowID, 6 * iB + 3, mt[0]));
                    tri.push_back(EigenTriple(rowID, 6 * iB + 4, mt[1]));
                    tri.push_back(EigenTriple(rowID, 6 * iB + 5, mt[2]));
                }
                rowID++;
            }
        }
    }
    size = Eigen::Vector2i(rowID, 6 * dynamic_nodes.size());
}


