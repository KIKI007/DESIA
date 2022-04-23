//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPHMOSEKSOLVER_H
#define TOPOLOCKCREATOR_CONTACTGRAPHMOSEKSOLVER_H

#include "ContactGraph.h"
#include "fusion.h"

class ContactGraphMosekSolver : public ContactGraph
{

public:

    double zero_eps;

public:

    ContactGraphMosekSolver(double eps);

public:

    void computeTranslationalInterlockingMatrix(mosek::fusion::Matrix::t &mat, Eigen::Vector2i& size);

    void computeRotationalInterlockingMatrix(mosek::fusion::Matrix::t &mat, Eigen::Vector2i& size);

public:

    bool isTranslationalInterlocking(string &log);

    bool isRotationalInterlocking(string &log);

    bool isEquilibrium(double friction_coef, EigenPoint gravity, string &log){return true;}
};


#endif //TOPOLOCKCREATOR_CONTACTGRAPHMOSEKSOLVER_H
