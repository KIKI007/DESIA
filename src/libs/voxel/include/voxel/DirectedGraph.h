//
// Created by *** on 01.03.18.
//

#ifndef TI_STABLE_DIRECTEDGRAPH_H
#define TI_STABLE_DIRECTEDGRAPH_H

#include <memory>
#include <vector>
#include <fstream>
#include <string>

namespace voxel{
    struct DirectedGraphNode;
    struct DirectedGraphEdge
    {
        double weight;
        std::weak_ptr<DirectedGraphNode> node;
    };

    struct DirectedGraphNode
    {
        int index_;
        std::string label;
        std::vector<DirectedGraphEdge> neighborList_;
    };

    class DirectedGraph {
    public:
        DirectedGraph(int num = 0)
        {
            init(num);
        }

    public:

        int size(){return nodeLists_.size();}

        void init(int num)
        {
            for(int id = 0; id < num; id++)
            {
                std::shared_ptr<DirectedGraphNode> p = std::make_shared<DirectedGraphNode>();
                p->index_ = id;
                p->label = std::to_string(id);
                nodeLists_.push_back(p);
            }
        }

    public:

        void add_edge(int idA, int idB, double weight = 0)
        {
            DirectedGraphEdge edge;
            edge.weight = weight;
            edge.node = nodeLists_[idB];
            nodeLists_[idA]->neighborList_.push_back(edge);
        }

        void output_dot(std::string filename, std::string caption)
        {
            std::ofstream fout(filename);
            fout << "digraph {\n";
            if(caption != "") fout << "label=\"" << caption << "\"";
            std::shared_ptr<DirectedGraphNode> p, q;
            if(nodeLists_.size() == 1)
            {
                p = nodeLists_[0];
                fout << p->label << "\n}";
            }
            else
            {
                for(int id = 0; id < nodeLists_.size(); id++)
                {
                    p = nodeLists_[id];
                    for(int jd = 0; jd < p->neighborList_.size(); jd++)
                    {
                        q = p->neighborList_[jd].node.lock();
                        fout << p->label << " -> " << q->label << "\n";
                    }
                }
                fout << "}";
            }
        }

    public:
        std::string to_string(std::vector<int> indexs)
        {
            if(indexs.size() == 1) return std::to_string(indexs.back());
            std::string str = "\"";
            for(int id = 0; id < indexs.size() - 1; id++)
                str += std::to_string(indexs[id]) + ", ";
            str+= std::to_string(indexs.back()) + "\"";
            return str;
        }

    public:
        std::vector< std::shared_ptr<DirectedGraphNode> > nodeLists_;
    };
}




#endif //TI_STABLE_DIRECTEDGRAPH_H
