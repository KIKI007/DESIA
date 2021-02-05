//
// Created by *** on 01.03.18.
//

#ifndef TI_STABLE_DIRECTEDGRAPHSSC_H
#define TI_STABLE_DIRECTEDGRAPHSSC_H

#include "DirectedGraph.h"
#include <stack>
using std::stack;
class DirectedGraphSSC: public DirectedGraph {

public:
    DirectedGraphSSC(DirectedGraph graph):DirectedGraph(graph)
    {
        init();
    }

public:

    void init();

    void tarjan_SSC();

    void tarjan_SSC_dfs(int id);

private:

    vector<int> DFN_;

    vector<int> LOW_;

    vector<bool> in_stack_;

    stack<int> stack_;

    int Dindex;

public:
    vector<vector<int>> ssc_group_;
};


#endif //TI_STABLE_DIRECTEDGRAPHSSC_H
