//
// Created by *** on 01.03.18.
//

#include "DirectedGraphSSC.h"
#include <iostream>
using std::cout;

void DirectedGraphSSC::tarjan_SSC()
{
    init();
    for(int id = 0; id < size(); id++)
    {
        if(!DFN_[id])
            tarjan_SSC_dfs(id);
    }
}

void DirectedGraphSSC::init()
{
    int num = nodeLists_.size();

    DFN_.resize(num, 0);
    LOW_.resize(num, 0);
    in_stack_.resize(num, false);

    Dindex = 0;
    ssc_group_.clear();
}

void DirectedGraphSSC::tarjan_SSC_dfs(int pID)
{
    std::shared_ptr<DirectedGraphNode> p, q;

    DFN_[pID] = LOW_[pID] = ++Dindex;

    in_stack_[pID] = true; stack_.push(pID);

    p = nodeLists_[pID];

    for(int jd = 0; jd < p->neighborList_.size(); jd++)
    {
        q = p->neighborList_[jd].node.lock();
        int qID = q->index_;
        if(!DFN_[qID])
        {
            //un-visited node
            tarjan_SSC_dfs(qID);
            LOW_[pID] = std::min(LOW_[pID], LOW_[qID]);
        }
        else if(in_stack_[qID])
        {
            LOW_[pID] = std::min(LOW_[pID], LOW_[qID]);
        }

    }

    if (DFN_[pID] == LOW_[pID])
    {
        //cout << "{";
        int jd = 0;
        vector<int> group_id;
        do{
            jd = stack_.top();
            //cout << stack_.top() << ", ";
            stack_.pop();
            in_stack_[jd] = false;
            group_id.push_back(jd);
        }while(jd != pID);
        //cout << "\b\b}, ";
        ssc_group_.push_back(group_id);
    }
}
