//
// Created by *** on 01.03.18.
//

#ifndef TI_STABLE_DIRECTEDGRAPHSSC_H
#define TI_STABLE_DIRECTEDGRAPHSSC_H

#include "DirectedGraph.h"
#include <stack>

namespace voxel{
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

        std::vector<int> DFN_;

        std::vector<int> LOW_;

        std::vector<bool> in_stack_;

        std::stack<int> stack_;

        int Dindex;

    public:
        std::vector<std::vector<int>> ssc_group_;
    };
}



#endif //TI_STABLE_DIRECTEDGRAPHSSC_H
