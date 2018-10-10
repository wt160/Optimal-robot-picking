#ifndef PRM_H
#define PRM_H

#include <time.h>
#include "system_single_integrator.h"
#include "prm_graph.h"
#include "astar.h"
#include <iostream>
using namespace SingleIntegrator;

class PRM
{
public:
    PRM();
    PRM_Graph* createGraph(int numberOfNodes, int numberClosestNeighbors);
    vector<Edge*> query(int numberClosestNeighbors, double& distance);
    // int delta(Node *n1, Node *n2);
    // bool checkNodeAreaIsEmpty(Node *n1);
    // bool checkNodeAreaIsEmpty(int x,int y);
    // void render();
    // void drawBox(double xi,double yi,double xf,double yf,QColor color);
    void setInitNode(double x, double y);
    void setGoalNode(double x, double y);
    int setSystem (System& system);
    void sampleInRegion(region target_region ,int numberOfNodes, int numberClosestNeighbors);
    void restore_temp();
	void DFSUtil(Node* v, int cid);
	~PRM();

    PRM_Graph *mGraph;
    Node *mGoalNode, *mInitNode;
    double **mMap;
    int mMapSize, celRange;
    vector<Edge*> path;
    System * system;    
    int numDimensions;
    std::vector<Node*> temp_V;
    std::vector<Edge*> temp_E;
};

#endif // PRM_H
