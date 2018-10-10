#include "prm.h"





PRM::PRM(): mGraph(NULL), celRange(100)
{
	system = NULL;
    mGoalNode = mInitNode = NULL;
    mGraph = NULL;
    //createGraph(1000,4);
    //regionOperating.setNumDimensions(2);
}

PRM::~PRM() {
	for (int i = 0; i < mGraph->V.size(); i++) {
		for (int j = 0; j < mGraph->V.at(i)->neighbors.size(); j++) {
			if (mGraph->V.at(i)->neighbors.at(j) != NULL) {
				delete mGraph->V.at(i)->neighbors.at(j);
				mGraph->V.at(i)->neighbors.at(j) = NULL;
			}
		}
		if (mGraph->V.at(i) != NULL) {
			delete mGraph->V.at(i);
			mGraph->V.at(i) = NULL;
		}
	}

	//for (int j = 0; j < mGraph->E.size(); j++) {
	//	if (mGraph->E.at(j) != NULL) {
			//delete mGraph->E.at(j);
	//	}
	//}
}

int PRM::setSystem (System& systemIn) {
    
    if (system)
        delete system;
    
    system = &systemIn;
    
    numDimensions = system->getNumDimensions();
    

    
    // Initialize the root vertex

    
    return 1;
}


void PRM::sampleInRegion(region target_region ,int numberOfNodes, int numberClosestNeighbors)
{
    int sample_num = 0;
    vector<Node*> new_added_nodes;
    while(sample_num < numberOfNodes)
    {
        Node *n;
        do
        {
            State stateRandom;

            if(system->sampleStateInRegion(stateRandom,target_region) > 0){     
                n = new Node(mGraph->V.size());                
                n->pose = std::make_pair(stateRandom[0], stateRandom[1]);
                sample_num ++;
                break;
            }
        } while(1); //Verificar se é ponto livre no mapa.
        mGraph->V.push_back(n);
        new_added_nodes.push_back(n);
        temp_V.push_back(n);
    }
//  bool System::IsInCollision (double *stateIn) {
    for(int i=0;i<new_added_nodes.size();i++)
    {
        vector<Node*> Nq = new_added_nodes.at(i)->sortedClosest(mGraph,numberClosestNeighbors);

        //qDebug() << "Near: " << Nq.size() ;
        for(int j=0; j<Nq.size();j++)
        {
            if( !mGraph->hasEdge(new_added_nodes.at(i),Nq.at(j)))
            {
                double start[2] = {(new_added_nodes.at(i)->pose).first, (new_added_nodes.at(i)->pose).second};
                double end[2] = {(Nq.at(j)->pose).first, (Nq.at(j)->pose).second};
                if(system->isLineInCollision(start, end)){
                    continue;
                }

                //qDebug() << "Add Edge";
				new_added_nodes.at(i)->cid = Nq.at(j)->cid;
                Edge *edge = new Edge(new_added_nodes.at(i),Nq.at(j));
                new_added_nodes.at(i)->neighbors.push_back(edge);
                Nq.at(j)->neighbors.push_back(new Edge(Nq.at(j),new_added_nodes.at(i)));
                mGraph->E.push_back(edge);
                temp_E.push_back(edge);
            }
        }
    }
}

void PRM::restore_temp()
{
    for(int i = 0; i < mGraph->V.size(); i ++){
        for(int j = 0; j < temp_V.size(); j ++){
            if((mGraph->V)[i] == temp_V[j]){
				delete temp_V[j];
				(mGraph->V).erase((mGraph->V).begin()+i);

            }
        }
    }
    for(int i = 0; i < mGraph->E.size(); i ++){
        for(int j = 0; j < temp_E.size(); j ++){
            if(((mGraph->E)[i]->mNodeL) == ((temp_E)[j]->mNodeL)  &&  ((mGraph->E)[i]->mNodeR) == ((temp_E)[j]->mNodeR)){
				delete (temp_E)[j];
				(mGraph->E).erase((mGraph->E).begin()+i);
            }
        }
    }
    temp_V.clear();
    temp_E.clear();  
}

PRM_Graph* PRM::createGraph(int numberOfNodes, int numberClosestNeighbors)
{
   // struct timeval begin,end;
   // gettimeofday(&begin,NULL);
    //qDebug() << "Criando o grafo";
    if(mGraph)
        delete mGraph;
    mGraph = new PRM_Graph();

    while(mGraph->V.size() < numberOfNodes)
    {
        Node *n;
        do
        {
            State stateRandom;
            if(system->sampleState (stateRandom) > 0){     
                n = new Node(mGraph->V.size());                
                n->pose = std::make_pair(stateRandom[0], stateRandom[1]);
                break;
            }
        } while(1); 
        mGraph->V.push_back(n);
    }
//  bool System::IsInCollision (double *stateIn) {
    for(int i=0;i<mGraph->V.size();i++)
    {
        vector<Node*> Nq = mGraph->V.at(i)->sortedClosest(mGraph,numberClosestNeighbors);
		mGraph->V.at(i)->visited = false;
        //qDebug() << "Near: " << Nq.size() ;
        for(int j=0; j<Nq.size();j++)
        {
            if( !mGraph->hasEdge(mGraph->V.at(i),Nq.at(j)))
            {
                double start[2] = {(mGraph->V.at(i)->pose).first, (mGraph->V.at(i)->pose).second};
                double end[2] = {(Nq.at(j)->pose).first, (Nq.at(j)->pose).second};
                if(system->isLineInCollision(start, end)){
                    continue;
                }

                //qDebug() << "Add Edge";
                Edge *edge = new Edge(mGraph->V.at(i),Nq.at(j));
				bool is_edge_exist = false;
				for (int k = 0; k < mGraph->V.at(i)->neighbors.size(); k++) {
					if (mGraph->V.at(i)->neighbors[k]->mNodeL == edge->mNodeL && mGraph->V.at(i)->neighbors[k]->mNodeR == edge->mNodeR) {
						is_edge_exist = true;
						break;
					}
				}
				if (!is_edge_exist) {
					mGraph->E.push_back(edge);
					mGraph->V.at(i)->neighbors.push_back(edge);
				}
				else {
					delete edge;
				}
				is_edge_exist = false;
				for (int k = 0; k < Nq.at(j)->neighbors.size(); k++) {
					if (Nq.at(j)->neighbors[k]->mNodeL == Nq.at(j) && Nq.at(j)->neighbors[k]->mNodeR == mGraph->V.at(i)) {
						is_edge_exist = true;
						break;
					}
				}
				if (!is_edge_exist) {
					Nq.at(j)->neighbors.push_back(new Edge(Nq.at(j), mGraph->V.at(i)));
				}
				
               
            }
        }
    }
	int cid = 0;
	for (int i = 0; i < mGraph->V.size(); i++)
	{
		if (mGraph->V.at(i)->visited == false) {
			DFSUtil(mGraph->V.at(i), cid);
			cid++;
		}
	}
   // gettimeofday(&end,NULL);
	return mGraph;
    //std::cout << "Time used to create graph: " << ((double)(end.tv_usec-begin.tv_usec))/1000000 + (double)(end.tv_sec-begin.tv_sec);
    //std::cout << "Vertex: " << mGraph->V.size();
    //std::cout << "Edges: " << mGraph->E.size();

}

void PRM::DFSUtil(Node* v, int cid) { 
	v->visited = true;
	v->cid = cid;
	std::cout << cid << " ";
	for (int k = 0; k < v->neighbors.size(); k++) {
		if (v->neighbors[k]->mNodeR->visited == false) {
			DFSUtil(v->neighbors[k]->mNodeR, cid);
		}
	}
}

vector<Edge*> PRM::query(int numberClosestNeighbors, double& total_distance)
{
	bool init_no_connection = false;
	bool goal_no_connection = false;
    path.erase(path.begin(),path.end());

    //struct timeval begin,end;
    //gettimeofday(&begin,NULL);
    //qDebug() << "Query começando";
    mGraph->V.push_back(mGoalNode);
    mGraph->V.push_back(mInitNode);


    vector<Node*> Nq = mInitNode->sortedClosest(mGraph,numberClosestNeighbors);
    Edge *edge = NULL;
	bool no_collision = false;
    do
    {   
        double start[2] = {(mInitNode->pose).first, (mInitNode->pose).second};
        double end[2] = {(Nq.at(0)->pose).first, (Nq.at(0)->pose).second};
        if(!(system->isLineInCollision(start, end))){
            edge = new Edge(mInitNode,Nq.at(0));
			bool edge_exist = false;
			no_collision = true;
			for (int i = 0; i < Nq.at(0)->neighbors.size(); i++) {
				if (Nq.at(0)->neighbors.at(i)->mNodeR == edge->mNodeL && Nq.at(0)->neighbors.at(i)->mNodeL == edge->mNodeR) {
					edge_exist = true;
					break;
				}
			}
			if (!edge_exist) {
				mInitNode->neighbors.push_back(edge);
				//Nq.at(0)->neighbors.push_back(edge);
				mGraph->E.push_back(edge);
			}
			else {
				delete edge;
				edge = NULL;
			}




            //mInitNode->neighbors.push_back(edge);
            //mGraph->E.push_back(edge);
			mInitNode->cid = Nq.at(0)->cid;
            //Nq.erase(Nq.begin());
        }
        else
        {
            Nq.erase(Nq.begin());
        }
    }while(Nq.size() && (no_collision == false));
	if (Nq.size() == 0) {
		std::cout << "initnode has no neighbor" << std::endl;
		init_no_connection = true;
	}

    edge = NULL;
    Nq = mGoalNode->sortedClosest(mGraph,numberClosestNeighbors);
	no_collision = false;
    do
    {
        double start[2] = {(mGoalNode->pose).first, (mGoalNode->pose).second};
        double end[2] = {(Nq.at(0)->pose).first, (Nq.at(0)->pose).second};
        if(!(system->isLineInCollision(start, end))){
            edge = new Edge(Nq.at(0),mGoalNode);
			no_collision = true;
			bool edge_exist = false;
			for (int i = 0; i < Nq.at(0)->neighbors.size(); i++) {
				if (Nq.at(0)->neighbors.at(i)->mNodeR == edge->mNodeR && Nq.at(0)->neighbors.at(i)->mNodeL == edge->mNodeL) {
					//std::cout << "edge existed";
					edge_exist = true;
					break;
				}
			}
			if (!edge_exist) {
				Nq.at(0)->neighbors.push_back(edge);
				mGraph->E.push_back(edge);
			}
			else {
				delete edge;
				edge = NULL;
			}
			mGoalNode->cid = Nq.at(0)->cid;
        }
        else
        {
            Nq.erase(Nq.begin());
        }
    }while(Nq.size() && (no_collision == false));
	if (Nq.size() == 0) {
		std::cout << "goalnode has no neighbor" << std::endl;
		goal_no_connection = true;
	}
	
    AStar search;
	if(goal_no_connection || init_no_connection || mGoalNode->cid != mInitNode->cid){
		
	}
	else {

		path = search.shortestPath(mInitNode, mGoalNode); //Run shortest path
	}
    if(path.size() == 0)
    {
        delete mInitNode;
        delete mGoalNode;
        mInitNode = mGoalNode = NULL;
    }

    //Nq.at(0)->neighbors.pop_back();
    mGraph->E.pop_back();
    mGraph->E.pop_back();
    mGraph->V.pop_back();
    mGraph->V.pop_back();
   // gettimeofday(&end,NULL);

    total_distance = 0;
    for(int i = 0; i < path.size(); i++){
        total_distance += path[i]->distance;
    }
    // qDebug() << "Path acabando!";
    // qDebug() << "Tempo busca: " << ((double)(end.tv_usec-begin.tv_usec))/1000000 + (double)(end.tv_sec-begin.tv_sec);
    
    return path;
}

// int PRM::delta(Node *n1, Node *n2)
// {
//     double deltaX = n2->mPose->getX() - n1->mPose->getX(), deltaY = n2->mPose->getY() - n1->mPose->getY();
//     double m;
//     if(fabs(deltaX) > fabs(deltaY))
//         m = deltaX;
//     else
//         m = deltaY;
//     double step = fabs(1/m);
//     for (double percent = 0.0;percent <= 1.0;percent += step)
//     {
//         int x = (int)round((n2->mPose->getX() - n1->mPose->getX())*percent+n1->mPose->getX()),
//                 y = (int)round((n2->mPose->getY() - n1->mPose->getY())*percent+n1->mPose->getY());

//         if(mMap[x][y] == 0.0)
//             return NULL;
//     }
//     return 1;
// }

// bool PRM::checkNodeAreaIsEmpty(Node *n1)
// {
//     return checkNodeAreaIsEmpty((int)n1->mPose->getX(),(int)n1->mPose->getY());
// }

// bool PRM::checkNodeAreaIsEmpty(int x, int y)
// {
//     for(int nx =max(x-2,0);nx <= min(x+2,mMapSize-1);nx++)
//     {
//         for(int ny =max(y-2,0);ny <= min(y+2,mMapSize-1);ny++)
//         {
//             if(mMap[nx][ny] == 0.0)
//             {
//                 return false;
//             }
//         }
//     }
//     return true;
// }

// void PRM::render()
// {
//     //Desenhando o mapa em si
//     glBegin(GL_QUADS);

//     for(int x=0;x<mMapSize;x++)
//     {
//         for(int y=0;y<mMapSize;y++)
//         {
//             if(mMap[x][y] != 1.0)
//             {
//                 int value = 255*mMap[x][y];
//                 drawBox(
//                             (x-1-mMapSize/2)*celRange,
//                             (mMapSize/2 - y+1)*celRange,
//                             celRange,
//                             celRange,
//                             QColor(value,value,value)
//                             );
//             }
//         }
//     }
//     glEnd();

//     if(mGraph)
//     {
//         //Desenhando os Arcos
//         glColor3f(0.0f,0.0f,1.0f);
//         glBegin(GL_LINES);

//         for(int i =0; i<mGraph->E.size();i++)
//         {
//             Pose *pL = mGraph->E.at(i)->mNodeL->mPose, *pR = mGraph->E.at(i)->mNodeR->mPose;
//             glVertex2f((pL->getX()-mMapSize/2)*celRange,(mMapSize/2 - pL->getY())*celRange);
//             glVertex2f((pR->getX()-mMapSize/2)*celRange,(mMapSize/2 - pR->getY())*celRange);
//         }
//         glEnd();

//         //Desenhando os  nodos
//         glBegin(GL_QUADS);
//         for(int i =0;i<mGraph->V.size();i++)
//         {
//             Pose *p = mGraph->V.at(i)->mPose;

//             drawBox(
//                 (p->getX() - 1 - mMapSize/2)*celRange,
//                 (mMapSize/2 - 1 - p->getY())*celRange,
//                 celRange*2,
//                 celRange*2,
//                 QColor(0,255,255)
//                 );
//         }
//         glEnd();
//     }


//     if(mInitNode && mGoalNode && path.size())
//     {
//         //Desenhando os Arcos
//         glColor3f(1.0f,0.0f,1.0f);
//         glLineWidth(3.0f);
//         glBegin(GL_LINES);

//         for(int i =0; i<path.size();i++)
//         {
//             Pose *pL = path.at(i)->mNodeL->mPose, *pR = path.at(i)->mNodeR->mPose;
//             glVertex2f((pL->getX()-mMapSize/2)*celRange,(mMapSize/2 - pL->getY())*celRange);
//             glVertex2f((pR->getX()-mMapSize/2)*celRange,(mMapSize/2 - pR->getY())*celRange);
//         }
//         glEnd();
//         glLineWidth(1.0f);
//     }

//     //Desenhando os objetivo e posicao inicial
//     if(mInitNode)
//     {
//         glBegin(GL_QUADS);
//         drawBox(
//             (mInitNode->mPose->getX() - 2 - mMapSize/2)*celRange,
//             (mMapSize/2 - 1 - mInitNode->mPose->getY())*celRange,
//             celRange*3,
//             celRange*3,
//             QColor(0,150,0)
//             );
//         glEnd();
//     }
//     if(mGoalNode)
//     {
//         glBegin(GL_QUADS);
//         drawBox(
//             (mGoalNode->mPose->getX() - 2 - mMapSize/2)*celRange,
//             (mMapSize/2 - 1 - mGoalNode->mPose->getY())*celRange,
//             celRange*3,
//             celRange*3,
//             QColor(255,0,0)
//             );
//         glEnd();
//     }
// }

// void PRM::drawBox(double x, double y, double width, double height,QColor color)
// {
//     glColor3f(color.redF(),color.greenF(),color.blueF());

//     glVertex2f(x,y);
//     glVertex2f(x+width,y);
//     glVertex2f(x+width,y+height);
//     glVertex2f(x,y+height);
// }

void PRM::setGoalNode(double x, double y)
{

    if(mGoalNode)
        delete mGoalNode;
    mGoalNode= new Node();
    mGoalNode->pose = std::make_pair(x, y);
        //mGoalNodepose->mPose->setPose(mMapSize-20,20);
    
}

void PRM::setInitNode(double x, double y)
{
    
    if(mInitNode)
        delete mInitNode;
    mInitNode= new Node();
    mInitNode->pose = std::make_pair(x, y);
        //mInitNode->mPose->setPose(20,mMapSize-20);
    
}
