//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPH_H
#define TOPOLOCKCREATOR_CONTACTGRAPH_H

#include "ContactGraphNode.h"
#include <Eigen/SparseCore>
#include <string>

using std::string;

typedef Eigen::SparseMatrix<double> EigenSpMat;

typedef Eigen::Triplet<double> EigenTriple;

typedef shared_ptr<ContactGraphNode> pContactGraphNode;
typedef weak_ptr<ContactGraphNode> wpContactGraphNode;
typedef shared_ptr<ContactGraphEdge> pContactGraphEdge;
typedef weak_ptr<ContactGraphEdge> wpContactGraphEdge;

/*!
 * \brief An graph describe an rigid body system
 */
class ContactGraph {

public:

    vector<shared_ptr<ContactGraphNode>> nodes;

    vector<shared_ptr<ContactGraphEdge>> edges;

public: //automatic generate

    vector<weak_ptr<ContactGraphNode>> dynamic_nodes;


public:

    ContactGraph();

    ~ContactGraph();

public:

    void addNode(pContactGraphNode _node);

    void addContact(pContactGraphNode _nodeA, pContactGraphNode _nodeB, pContactGraphEdge _edge);

public:

    void initialize();

    bool findEdge(pContactGraphNode _nodeA, pContactGraphNode _nodeB, pContactGraphEdge &_edge);

public:

    void computeTranslationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size);

    void computeRotationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size);

    void computeEquilibriumMatrix(EigenSpMat &mat){}

public:

    virtual bool isTranslationalInterlocking(string &log){ return  true;}

    virtual bool isRotationalInterlocking(string &log){ return  true;}

    virtual bool isEquilibrium(double friction_coef, EigenPoint gravity, string &log){ return  true;}
};


#endif //TOPOLOCKCREATOR_CONTACTGRAPH_H
