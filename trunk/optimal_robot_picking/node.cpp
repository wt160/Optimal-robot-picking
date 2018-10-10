#include "node.h"

Node::Node() : mId(-1)
{
}

Node::Node(int id)
{
    mId = id;
}

vector<Node*> Node::sortedClosest(PRM_Graph *graph, int numberClosestNeighbors)
{
	vector<Node*> sorted_list;
	Node* temp = NULL;
	bool found = false;
	for (int j = 0; j < numberClosestNeighbors; j++) {
		double max_dist = 1000000000;
		found = false;
		for (int i = 0; i < graph->V.size(); i++)
		{
			bool is_in = false;
			for (int k = 0; k < sorted_list.size(); k++) {
				if (graph->V.at(i) == sorted_list[k]) {
					is_in = true;
					break;
				}
			}
			
			if (graph->V.at(i) != this && !is_in)
			{
				double dist = this->distance(graph->V.at(i));
				if (dist < max_dist) {
					max_dist = dist;
					temp = graph->V.at(i);
					found = true;
				}
			}
		}
		if (found) {
			sorted_list.push_back(temp);
		}
	}
	return sorted_list;
	/*  
    edges = new Edge*[graph->V.size()-1];
    for(int i=0, n=0; i<graph->V.size();i++)
    {
        if(graph->V.at(i) != this)
        {
            edges[n] = new Edge(this,graph->V.at(i));
            n++;
        }
    }

    //qDebug() << "Sorting";
    edges = merge(0,graph->V.size()-2);
    //qDebug() << "Sorted";

    vector<Node*> *ret = new vector<Node*>();
    for(int i =0;i<numberClosestNeighbors;i++)
    {
        ret->push_back(edges[i]->mNodeR);
    }
    for(int i =numberClosestNeighbors;i<graph->V.size()-1;i++)
    {
        delete edges[i];
    }
    delete edges;
    return *ret;
	*/
}

double Node::distance(Node *node)
{
    return sqrt(pow(this->pose.first - node->pose.first,2)+pow(this->pose.second - node->pose.second,2));
}

Edge** Node::merge(int start, int finish)
{
    Edge** ret = new Edge*[finish-start+1];
    if(start != finish)
    {
        int m = (start+finish)/2;
        Edge **left = merge(start,m);
        Edge **right = merge(m+1,finish);
        int a,b;
        a = b = 0;
        //qDebug() << "merging ";
        for(int i=0;i<=finish-start;i++)
        {
            if((a<=m-start) && ((b>finish-m-1) || (left[a]->distance < right[b]->distance)))
            {
                ret[i] = left[a];
                a++;
            }
            else
            {
                ret[i] = right[b];
                b++;
            }
            //qDebug() << i << "/" << finish-start << ": " << ret[i]->mNodeR->mId;
        }
        delete left;
        delete right;
        //qDebug() << "." << endl;
    }
    else
    {
        //qDebug() <<  edges[start]->mNodeR->mId;
        ret[0] = edges[start];
    }
    return ret;
}
