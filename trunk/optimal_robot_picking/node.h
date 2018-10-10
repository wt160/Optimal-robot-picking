#ifndef NODE_H
#define NODE_H

#include <vector>

using namespace std;

class PRM_Graph;
class Edge;

class Node
{
public:
    Node();
    Node(int id);
    vector<Node*> sortedClosest(PRM_Graph *graph, int numberClosestNeighbors);
    double distance(Node *node);
    std::pair<double,double> pose;
    vector<Edge*> neighbors;
    int mId;
	int cid;
	bool visited;
private:
    Edge** merge(int start,int finish);
    Edge **edges;

};

#include "prm_graph.h"

#endif // NODE_H
