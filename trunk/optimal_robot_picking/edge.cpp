#include "edge.h"

Edge::Edge(Node *nodeLeft, Node *nodeRight)
{
    mNodeL = nodeLeft;
    mNodeR = nodeRight;

    distance = sqrt(pow(mNodeR->pose.first - mNodeL->pose.first,2)+pow(mNodeR->pose.second - mNodeL->pose.second,2));
    //qDebug() << distance;
}
