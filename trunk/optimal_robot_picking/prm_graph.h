#ifndef PRM_GRAPH_H
#define PRM_GRAPH_H

#include <vector>
#include "node.h"
#include "edge.h"

using namespace std;

class PRM_Graph
{
public:
    PRM_Graph();
    bool hasEdge(Node *n1, Node *n2);

    vector<Node*> V;
    vector<Edge*> E;
};

#endif // GRAPH_H
