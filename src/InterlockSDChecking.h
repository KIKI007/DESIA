//
// Created by *** on 01.03.18.
//

#ifndef TI_STABLE_ASSEMBLYINTERLOCK_H
#define TI_STABLE_ASSEMBLYINTERLOCK_H

#include "Assembly.h"
#include <Eigen/Dense>
#include "DirectedGraphSSC.h"
using Eigen::MatrixXi;
enum InterlockingStatus
{
    NOT_INTERLOCKING = 0,
    HAS_KEY = 1,
    HAS_NO_KEY = 2
};

struct InterlockSDData
{
    InterlockingStatus status;
    vector< vector<int> > ssc_groups;
};

//InterlockingSDChecking
    //construct a directed graph when checking in single direction
    //the edge i->j means in this direction, the translation value has: V_i >= V_j

class InterlockSDChecking{


public:

    InterlockSDChecking()
    {
        eps_ = 1e-5;
    }


/* single direction checking */
public:

    InterlockSDData check_interlock_sd(Vector3d dv, const Assembly &assembly);

    std::shared_ptr<DirectedGraph> init_sd_graph(Vector3d dv, const Assembly &assembly);

public:

    std::shared_ptr<DirectedGraph> simplified_sd_graph(Vector3d dv, const Assembly &assembly);

public:

    std::shared_ptr<DirectedGraph> blocking_graph_, simpified_graph_;

private:
    double eps_;
};


#endif //TI_STABLE_ASSEMBLYINTERLOCK_H
