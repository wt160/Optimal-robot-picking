/*! 
 * \file rrts.hpp 
 */ 

#ifndef __RRTS_HPP_
#define __RRTS_HPP_

#include <iostream>
#include <cfloat>
#include <cmath>
#include <algorithm>


#include "rrts.h"



template<class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>
::Vertex () {
    
    state = NULL;
    parent = NULL;
    trajFromParent = NULL;
    
    costFromParent = 0.0;
    costFromRoot = 0.0;
}


template<class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>
::~Vertex () {
    
    if (state)
        delete state;
    parent = NULL;
	if (trajFromParent) {
		delete trajFromParent;
		trajFromParent = NULL;
	}
    children.clear();
}


template<class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>
::Vertex(const Vertex<State, Trajectory, System>& vertexIn) {
    
    if (vertexIn.state)
        state = new State (vertexIn.getState());
    else 
        state = NULL;
    parent = vertexIn.parent;
    for (typename std::set< Vertex<State,Trajectory,System> * >::const_iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++)
        children.insert (*iter);
    costFromParent = vertexIn.costFromParent;
    costFromRoot = vertexIn.costFromRoot;
    if (vertexIn.trajFromParent)
        trajFromParent = new Trajectory (*(vertexIn.trajFromParent));
    else 
        trajFromParent = NULL;
}


// int Vertex::setState (const State &stateIn) {
//   *state = stateIn;
//   return 1;
// }


template<class State, class Trajectory, class System>
RRTstar::Planner<State, Trajectory, System>
::Planner () {
    
    gamma = 1.0;
    
    lowerBoundCost = DBL_MAX;
    lowerBoundVertex = NULL;
    
    kdtree = NULL; 
    
    root = NULL;
	current_index_ = 0;
    numVertices = 0;
    
    system = NULL;
	debug_time_1 = 0;
	debug_time_2 = 0;
	debug_time_3 = 0;
}


/* 
template<class State, class Trajectory, class System>
RRTstar::Planner<State, Trajectory, System>
::Planner(const Planner& planner) {

	gamma = planner.gamma;

	lowerBoundCost = planner.lowerBoundCost;
	lowerBoundVertex = planner.lowerBoundVertex;

	kdtree = new KdTree(*(planner.kdtree));

	root = new vertex_t(*(planner.root));

	numVertices = planner.numVertices;

	system = new System(*(planner.system));

}
*/

template<class State, class Trajectory, class System>
RRTstar::Planner<State, Trajectory, System>
::~Planner () {
    
    // Delete the kdtree structure
    if (kdtree) {
        kd_clear (kdtree);
        kd_free (kdtree);
    }
    
    // Delete all the vertices
    for (typename std::list<Vertex <State,Trajectory,System> * >::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++) 
        delete *iter;
    
}

template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::temp_insertIntoKdtree(Vertex<State, Trajectory, System>& vertexIn) {

	double *stateKey = new double[numDimensions];
	system->getStateKey(*(vertexIn.state), stateKey);
	temp_kd_insert(kdtree, stateKey, &vertexIn);
	delete[] stateKey;

	return 1;
}

template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::insertIntoKdtree (Vertex<State,Trajectory,System>& vertexIn) {
    
    double *stateKey = new double[numDimensions];
    system->getStateKey ( *(vertexIn.state), stateKey);
    kd_insert (kdtree, stateKey, &vertexIn);
    delete [] stateKey;
    
    return 1;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State,Trajectory,System>
::getNearestVertex (State& stateIn, Vertex<State,Trajectory,System>*& vertexPointerOut) {
    
    // Get the state key for the query state
    double *stateKey = new double[numDimensions];
    system->getStateKey (stateIn, stateKey);
    
    // Search the kdtree for the nearest vertex
    KdRes *kdres = kd_nearest (kdtree, stateKey);
    if (kd_res_end (kdres))  
        vertexPointerOut = NULL;
    vertexPointerOut = (Vertex<State,Trajectory,System>*) kd_res_item_data (kdres);
    
    // Clear up the memory
    delete [] stateKey;
    kd_res_free (kdres);
    
    // Return a non-positive number if any errors
    if (vertexPointerOut == NULL)
        return 0;
    
    return 1;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::getNearVertices (State& stateIn, std::vector< Vertex<State,Trajectory,System>* >& vectorNearVerticesOut) {
    
    // Get the state key for the query state
    double *stateKey = new double[numDimensions];
    system->getStateKey (stateIn, stateKey);
    
    // Compute the ball radius
    double ballRadius = gamma * pow( log((double)(numVertices + 1.0))/((double)(numVertices + 1.0)), 1.0/((double)numDimensions) );
    
    // Search kdtree for the set of near vertices
    KdRes *kdres = kd_nearest_range (kdtree, stateKey, ballRadius);
    delete [] stateKey;
    
    // Create the vector data structure for storing the results
    int numNearVertices = kd_res_size (kdres);
    if (numNearVertices == 0) {
        vectorNearVerticesOut.clear();
        return 1;
    }
    vectorNearVerticesOut.resize(numNearVertices);
    
    // Place pointers to the near vertices into the vector 
    int i = 0;
    kd_res_rewind (kdres);
    while (!kd_res_end(kdres)) {
        Vertex<State,Trajectory,System> *vertexCurr = (Vertex<State,Trajectory,System> *) kd_res_item_data (kdres);
        vectorNearVerticesOut[i] = vertexCurr;
        kd_res_next (kdres);
        i++;
    }
    
    // Free temporary memory
    kd_res_free (kdres);
    
    return 1;
}

template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::getNearVerticesBig(State& stateIn, std::vector< Vertex<State, Trajectory, System>* >& vectorNearVerticesOut) {

	// Get the state key for the query state
	double *stateKey = new double[numDimensions];
	system->getStateKey(stateIn, stateKey);

	// Compute the ball radius
	double ballRadius = gamma * pow(log((double)(numVertices + 1.0)) / ((double)(numVertices + 1.0)), 1.0 / ((double)numDimensions));
	ballRadius = ballRadius * 3;
	// Search kdtree for the set of near vertices
	KdRes *kdres = kd_nearest_range(kdtree, stateKey, ballRadius);
	delete[] stateKey;

	// Create the vector data structure for storing the results
	int numNearVertices = kd_res_size(kdres);
	if (numNearVertices == 0) {
		vectorNearVerticesOut.clear();
		return 1;
	}
	vectorNearVerticesOut.resize(numNearVertices);

	// Place pointers to the near vertices into the vector 
	int i = 0;
	kd_res_rewind(kdres);
	while (!kd_res_end(kdres)) {
		Vertex<State, Trajectory, System> *vertexCurr = (Vertex<State, Trajectory, System> *) kd_res_item_data(kdres);
		vectorNearVerticesOut[i] = vertexCurr;
		kd_res_next(kdres);
		i++;
	}

	// Free temporary memory
	kd_res_free(kdres);

	return 1;
}

template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::checkUpdateBestVertex (Vertex<State,Trajectory,System>& vertexIn) {
    
    if (system->isReachingTarget(vertexIn.getState())){
        
        
        double costCurr = vertexIn.getCost();
        if ( (lowerBoundVertex == NULL) || ( (lowerBoundVertex != NULL) && (costCurr < lowerBoundCost)) ) {
            
            lowerBoundVertex = &vertexIn;
            lowerBoundCost = costCurr;
        }
    }
    
    return 1;
}


template<class State, class Trajectory, class System>
RRTstar::Vertex<State,Trajectory,System>*
RRTstar::Planner<State, Trajectory, System>
::insertTrajectory (Vertex<State,Trajectory,System>& vertexStartIn, Trajectory& trajectoryIn) {
    
    // Check for admissible cost-to-go
    if (lowerBoundVertex != NULL) {
        double costToGo = system->evaluateCostToGo (trajectoryIn.getEndState());
        if (costToGo >= 0.0) 
            if (lowerBoundCost < vertexStartIn.getCost() + costToGo) 
                return NULL;
    }
    
    // Create a new end vertex
    Vertex<State,Trajectory,System>* vertexNew = new Vertex<State,Trajectory,System>;
    vertexNew->state = new State;
    vertexNew->parent = NULL;
    vertexNew->getState() = trajectoryIn.getEndState();
	vertexNew->setIndex(current_index_);
	current_index_++;
    insertIntoKdtree (*vertexNew);  
    this->listVertices.push_front (vertexNew);
    this->numVertices++;
    
    // Insert the trajectory between the start and end vertices
    insertTrajectory (vertexStartIn, trajectoryIn, *vertexNew);
    
    return vertexNew;
}

template<class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>*
RRTstar::Planner<State, Trajectory, System>
::temp_insertTrajectory(Vertex<State, Trajectory, System>& vertexStartIn, Trajectory& trajectoryIn) {

	// Check for admissible cost-to-go
	if (lowerBoundVertex != NULL) {
		double costToGo = system->evaluateCostToGo(trajectoryIn.getEndState());
		if (costToGo >= 0.0)
			if (lowerBoundCost < vertexStartIn.getCost() + costToGo)
				return NULL;
	}

	// Create a new end vertex
	Vertex<State, Trajectory, System>* vertexNew = new Vertex<State, Trajectory, System>;
	vertexNew->state = new State;
	vertexNew->parent = NULL;
	vertexNew->getState() = trajectoryIn.getEndState();
	temp_insertIntoKdtree(*vertexNew);
	this->listVertices.push_front(vertexNew);
	this->numVertices++;
	this->temp_add_vertices.insert(vertexNew);
	this->temp_add_num++;
	// Insert the trajectory between the start and end vertices
	temp_insertTrajectory(vertexStartIn, trajectoryIn, *vertexNew);

	return vertexNew;
}

template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::temp_insertTrajectory(Vertex<State, Trajectory, System>& vertexStartIn, Trajectory& trajectoryIn, Vertex<State, Trajectory, System>& vertexEndIn) {

	//if the trajectory is from new vertex to existing parent, ex, in the rewire case
	if ((temp_add_vertices.find(&vertexStartIn) != temp_add_vertices.end()) && (temp_add_vertices.find(&vertexEndIn) == temp_add_vertices.end())) {
		if (temp_rewired_vertices.find(&vertexEndIn) == temp_rewired_vertices.end()) {
			temp_rewired_vertices.insert(&vertexEndIn);
			temp_rewired_old_parent[&vertexEndIn] = vertexEndIn.parent;
			temp_rewired_old_cost[&vertexEndIn] = vertexEndIn.costFromRoot;
			temp_rewired_old_costfromparent[&vertexEndIn] = vertexEndIn.costFromParent;
		}
	}

	//if ((temp_add_vertices.find(&vertexStartIn) == temp_add_vertices.end())) {
	//	if (temp_rewired_vertices.find(&vertexEndIn) == temp_rewired_vertices.end()) {
	//		temp_rewired_vertices.insert(&vertexEndIn);
	//		temp_rewired_old_parent[&vertexEndIn] = vertexEndIn.parent;
	//		temp_rewired_old_cost[&vertexEndIn] = vertexEndIn.costFromRoot;
	//		temp_rewired_old_costfromparent[&vertexEndIn] = vertexEndIn.costFromParent;
	//	}
	//}
	// Update the costs
	vertexEndIn.costFromParent = trajectoryIn.evaluateCost();
	vertexEndIn.costFromRoot = vertexStartIn.costFromRoot + vertexEndIn.costFromParent;
	checkUpdateBestVertex(vertexEndIn);

	// store those parent vertexs belong to the original set, so that they can delete temp child in the future
	if (temp_add_vertices.find(&vertexStartIn) == temp_add_vertices.end()) {
		new_added_parent.insert(&vertexStartIn);
	}


	// Update the trajectory between the two vertices
	if (vertexEndIn.trajFromParent) {
		delete vertexEndIn.trajFromParent;
		vertexEndIn.trajFromParent = NULL;
	}
	vertexEndIn.trajFromParent = new Trajectory(trajectoryIn);

	// Update the parent to the end vertex
	if (vertexEndIn.parent)
		vertexEndIn.parent->children.erase(&vertexEndIn);
	vertexEndIn.parent = &vertexStartIn;
	
	
	// Add the end vertex to the set of chilren
	vertexStartIn.children.insert(&vertexEndIn);
	if (vertexEndIn.children.find(&vertexStartIn) != vertexEndIn.children.end()) {
		vertexEndIn.children.erase(&vertexStartIn);
		temp_rewired_old_children[&vertexEndIn] = &vertexStartIn;
	}

	return 1;
}

template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::insertTrajectory (Vertex<State,Trajectory,System>& vertexStartIn, Trajectory& trajectoryIn, Vertex<State,Trajectory,System>& vertexEndIn) {
    
    // Update the costs
    vertexEndIn.costFromParent = trajectoryIn.evaluateCost();
    vertexEndIn.costFromRoot = vertexStartIn.costFromRoot + vertexEndIn.costFromParent;
    checkUpdateBestVertex (vertexEndIn);
    
    // Update the trajectory between the two vertices
	if (vertexEndIn.trajFromParent) {
		delete vertexEndIn.trajFromParent;
		
	}
    vertexEndIn.trajFromParent = new Trajectory (trajectoryIn);
    
    // Update the parent to the end vertex
    if (vertexEndIn.parent)
        vertexEndIn.parent->children.erase (&vertexEndIn);
    vertexEndIn.parent = &vertexStartIn;
    
    // Add the end vertex to the set of chilren
    vertexStartIn.children.insert (&vertexEndIn);
    
    return 1;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::setSystem (System& systemIn) {
    
    if (system)
        delete system;
    
    system = &systemIn;
    
    numDimensions = system->getNumDimensions ();
    
    // Delete all the vertices
    for (typename std::list< Vertex<State,Trajectory,System>* >::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++)
        delete *iter;
    numVertices = 0;
	temp_add_num = 0;
    lowerBoundCost = DBL_MAX;
    lowerBoundVertex = NULL;
    
    // Clear the kdtree
    if (kdtree) {
        kd_clear (kdtree);
        kd_free (kdtree);
    }
    kdtree = kd_create (numDimensions);
    
    // Initialize the root vertex
    root = new Vertex<State,Trajectory,System>;
    root->state = new State (system->getRootState());
    root->costFromParent = 0.0;
    root->costFromRoot = 0.0;
    root->trajFromParent = NULL;
    
    return 1;
}



template<class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>& 
RRTstar::Planner<State, Trajectory, System>
::getRootVertex () {
    
    return *root;
}



template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::initialize () {
    
    // If there is no system, then return failure
    if (!system)
        return 0;
    
    // Backup the root
    Vertex<State,Trajectory,System> *rootBackup = NULL;
    if (root)
        rootBackup = new Vertex<State,Trajectory,System> (*root);
    
    // Delete all the vertices
    for (typename std::list< Vertex<State,Trajectory,System>* >::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++)
        delete *iter;
    listVertices.clear();
    numVertices = 0;
    lowerBoundCost = DBL_MAX;
    lowerBoundVertex = NULL;
    
    // Clear the kdtree
    if (kdtree) {
        kd_clear (kdtree);
        kd_free (kdtree);
    }
    kdtree = kd_create (system->getNumDimensions());
    
    // Initialize the variables
    numDimensions = system->getNumDimensions();
    root = rootBackup;
    if (root){
        listVertices.push_back(root);
        insertIntoKdtree (*root);
        numVertices++;
    }
    lowerBoundCost = DBL_MAX;
    lowerBoundVertex = NULL;
    
    return 1;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::setGamma (double gammaIn) {
    
    if (gammaIn < 0.0)
        return 0;
    
    gamma = gammaIn;
    
    return 1;
}




template <class State,class Trajectory, class System>
int compareVertexCostPairs (std::pair<RRTstar::Vertex<State,Trajectory,System>*,double> i, std::pair<RRTstar::Vertex<State,Trajectory,System>*,double> j) {
    
    return (i.second < j.second);
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::findBestParent (State& stateIn, std::vector< Vertex<State,Trajectory,System>* >& vectorNearVerticesIn, Vertex<State,Trajectory,System>*& vertexBest, Trajectory& trajectoryOut, bool& exactConnection) {
    
	clock_t t1, t2;
	float diff;
	float seconds;
    // Compute the cost of extension for each near vertex
    int numNearVertices = vectorNearVerticesIn.size();
    
    std::vector< std::pair<Vertex<State,Trajectory,System>*,double> > vectorVertexCostPairs(numNearVertices);
    
    int i = 0;
	
    for (typename std::vector< Vertex<State,Trajectory,System>* >::iterator iter = vectorNearVerticesIn.begin(); iter != vectorNearVerticesIn.end(); iter++) {
        
        vectorVertexCostPairs[i].first = *iter;
        exactConnection = false;
        double trajCost = system->evaluateExtensionCost ( *((*iter)->state), stateIn, exactConnection);
        vectorVertexCostPairs[i].second = (*iter)->costFromRoot + trajCost;
        i++;
    }

    // Sort vertices according to cost
    std::sort (vectorVertexCostPairs.begin(), vectorVertexCostPairs.end(), compareVertexCostPairs<State,Trajectory,System>);
    
    // Try out each extension according to increasing cost
    i = 0;
	t1 = clock();
    bool connectionEstablished = false;
    for (typename std::vector< std::pair<Vertex<State,Trajectory,System>*,double> >::iterator iter = vectorVertexCostPairs.begin(); 
         iter != vectorVertexCostPairs.end(); iter++) {
        
        Vertex<State,Trajectory,System>* vertexCurr = iter->first;
        
        // Extend the current vertex towards stateIn (and this time check for collision with obstacles)
        exactConnection = false;
        if (system->extendTo(*(vertexCurr->state), stateIn, trajectoryOut, exactConnection) > 0) {
            vertexBest = vertexCurr;
            connectionEstablished = true;
            break;
        }
    }
	t2 = clock();
	diff = ((float)t2 - (float)t1);
	seconds = diff / CLOCKS_PER_SEC;
	debug_time_2 += seconds;
    // Return success if a connection was established
    if (connectionEstablished)
        return 1;
    
    // If the connection could not be established then return zero
    return 0;
}


template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::updateBranchCost(Vertex<State, Trajectory, System>& vertexIn, int depth) {

	if(depth == 0){
		updated_vertex_list.clear();
	}
	if (updated_vertex_list.find(&vertexIn) != updated_vertex_list.end()) {
		return 1;
	}
	else {
		updated_vertex_list.insert(&vertexIn);
		//std::cout << "index:" << vertexIn.getIndex() << std::endl;
		if (vertexIn.getIndex() < 0 || depth > 1000)return 1;
		//std::cout << "depth:" << depth;
		//std::cout << " branch:" << vertexIn.children.size() << std::endl;
		// Update the cost for each children
		if (vertexIn.children.size() > 0) {
			for (typename std::set< Vertex<State, Trajectory, System>* >::iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++) {

				Vertex<State, Trajectory, System>* vertex = *iter;
				//State s = vertex.getState();
				//std::cout << s[0] << "," << s[1] << std::endl;
				if (vertex != NULL) {
					vertex->costFromRoot = vertexIn.costFromRoot + vertex->costFromParent;

					//checkUpdateBestVertex(*vertex);

					updateBranchCost(*vertex, depth + 1);
				}
			}
		}

		if (depth == 0) {
			//std::cout << "_____________________________________________________" << std::endl;
			//std::cout << "_____________________________________________________" << std::endl;
		}

		return 1;
	}
}

/* 

template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::temp_updateBranchCost (Vertex<State,Trajectory,System>& vertexIn, int depth) {

	//std::cout << "index:" << vertexIn.getIndex() << std::endl;
	if (vertexIn.getIndex() < 0 || depth > 1000)return 1;
	//std::cout << "depth:" << depth << std::endl;
    // Update the cost for each children
    for (typename std::set< Vertex<State,Trajectory,System>* >::iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++) {
        
        Vertex<State,Trajectory,System>& vertex = **iter;
		//State s = vertex.getState();
		//std::cout << s[0] << "," << s[1] << std::endl;
        vertex.costFromRoot = vertexIn.costFromRoot + vertex.costFromParent;
        
        checkUpdateBestVertex (vertex);
        
        temp_updateBranchCost (vertex, depth + 1);
    }
    
	if (depth == 0) {
		//std::cout << "_____________________________________________________" << std::endl;
		//std::cout << "_____________________________________________________" << std::endl;
	}
    
    return 1;
}
*/
template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::temp_rewireVertices(Vertex<State, Trajectory, System>& vertexNew, std::vector< Vertex<State, Trajectory, System>* >& vectorNearVertices) {

	int rewire_num = 0;
	// Repeat for all vertices in the set of near vertices
	for (typename std::vector< Vertex<State, Trajectory, System>* >::iterator iter = vectorNearVertices.begin(); iter != vectorNearVertices.end(); iter++) {

		Vertex<State, Trajectory, System>& vertexCurr = **iter;

		// Check whether the extension results in an exact connection
		bool exactConnection = false;
		double costCurr = system->evaluateExtensionCost(*(vertexNew.state), *(vertexCurr.state), exactConnection);
		if ((exactConnection == false) || (costCurr < 0))
			continue;

		// Check whether the cost of the extension is smaller than current cost
		double totalCost = vertexNew.costFromRoot + costCurr;
		if (totalCost < vertexCurr.costFromRoot - 0.001) {
			rewire_num++;
			// Compute the extension (checking for collision)
			Trajectory trajectory;
			if (system->extendTo(*(vertexNew.state), *(vertexCurr.state), trajectory, exactConnection) <= 0)
				continue;

			// Insert the new trajectory to the tree by rewiring
			temp_insertTrajectory(vertexNew, trajectory, vertexCurr);

			// Update the cost of all vertices in the rewired branch
			updateBranchCost(vertexCurr, 0);
		}
	}
	//std::cout << "rewire_num:" << rewire_num << std::endl;
	return 1;
}

template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::temp_rewireVerticesRecursive(Vertex<State, Trajectory, System>& vertexNew, std::vector< Vertex<State, Trajectory, System>* >& vectorNearVertices) {

	int rewire_num = 0;
	int recursive_branch_num = 1;
	std::map<Vertex<State, Trajectory, System>*, double> recursive_map;
	std::vector<Vertex<State, Trajectory, System>*> final_recursive_list;

	// Repeat for all vertices in the set of near vertices
	for (typename std::vector< Vertex<State, Trajectory, System>* >::iterator iter = vectorNearVertices.begin(); iter != vectorNearVertices.end(); iter++) {

		Vertex<State, Trajectory, System>& vertexCurr = **iter;

		// Check whether the extension results in an exact connection
		bool exactConnection = false;
		double costCurr = system->evaluateExtensionCost(*(vertexNew.state), *(vertexCurr.state), exactConnection);
		if ((exactConnection == false) || (costCurr < 0))
			continue;

		// Check whether the cost of the extension is smaller than current cost
		double totalCost = vertexNew.costFromRoot + costCurr;
		if (totalCost < vertexCurr.costFromRoot - 0.001) {
			rewire_num++;
			recursive_map[*iter] = costCurr;
			// Compute the extension (checking for collision)
			Trajectory trajectory;
			if (system->extendTo(*(vertexNew.state), *(vertexCurr.state), trajectory, exactConnection) <= 0)
				continue;

			// Insert the new trajectory to the tree by rewiring
			temp_insertTrajectory(vertexNew, trajectory, vertexCurr);
			//std::cout << "rewire!";
			// Update the cost of all vertices in the rewired branch
			updateBranchCost(vertexCurr, 0);
		}

	}
	//std::cout << "recursive_rewire" << " ";
	//std::cout << "recursive_size:" << recursive_map.size() << std::endl;
	if (recursive_map.size() < REWIRE_THRESHOLD) {
		return 1;
	}
	for (int j = 0; j < recursive_branch_num; j++) {
		double max_dist = -1;
		Vertex<State, Trajectory, System>* recursive_vertex;
		for (auto i = recursive_map.begin(); i != recursive_map.end(); i++) {
			if (i->second > max_dist) {
				max_dist = i->second;
				recursive_vertex = i->first;
			}

		}
		recursive_map.erase(recursive_vertex);
		final_recursive_list.push_back(recursive_vertex);
	}
	//std::cout << "final_recursive" << std::endl;
	for(int k = 0; k < final_recursive_list.size(); k++){
		std::vector< Vertex<State, Trajectory, System>* > vectorNearVertices;
		State *new_state = final_recursive_list[k]->state;
		Trajectory trajectory;
		bool exactConnection = false;
		if (system->extendTo(*(vertexNew.state), *new_state, trajectory, exactConnection) <= 0)
			continue;
		Vertex<State, Trajectory, System>* vertexNext = temp_insertTrajectory(vertexNew, trajectory);
		if (vertexNext == NULL)
			continue;

		
		getNearVerticesBig(*(vertexNext->state), vectorNearVertices);
		//std::cout << "near vertex size:" << vectorNearVertices.size() << std::endl;
		temp_rewireVerticesRecursive(*vertexNext, vectorNearVertices);
	}
	//std::cout << "rewire_num:" << rewire_num << std::endl;
	return 1;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::rewireVertices (Vertex<State,Trajectory,System>& vertexNew, std::vector< Vertex<State,Trajectory,System>* >& vectorNearVertices) {
    
    
    // Repeat for all vertices in the set of near vertices
    for (typename std::vector< Vertex<State,Trajectory,System>* >::iterator iter = vectorNearVertices.begin(); iter != vectorNearVertices.end(); iter++) {
        
        Vertex<State,Trajectory,System>& vertexCurr = **iter; 
        
        // Check whether the extension results in an exact connection
        bool exactConnection = false;
        double costCurr = system->evaluateExtensionCost (*(vertexNew.state), *(vertexCurr.state), exactConnection);
        if ( (exactConnection == false) || (costCurr < 0) )
            continue;
        
        // Check whether the cost of the extension is smaller than current cost
        double totalCost = vertexNew.costFromRoot + costCurr;
        if (totalCost < vertexCurr.costFromRoot - 0.001) {
            
            // Compute the extension (checking for collision)
            Trajectory trajectory;
            if (system->extendTo (*(vertexNew.state), *(vertexCurr.state), trajectory, exactConnection) <= 0 ) 
                continue;
            
            // Insert the new trajectory to the tree by rewiring
            insertTrajectory (vertexNew, trajectory, vertexCurr);
            
            // Update the cost of all vertices in the rewired branch
            updateBranchCost (vertexCurr, 0);
        }
    }
    
    return 1;
}

template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::connect(State target, double& dist, std::vector<std::pair<double, double>>& path) {
	std::vector< Vertex<State, Trajectory, System>* > vectorNearVertices;
	getNearVertices(target, vectorNearVertices);
	
	// 3. Find the best parent and extend from that parent
	Vertex<State, Trajectory, System>* vertexParent = NULL;
	Trajectory trajectory;
	bool exactConnection = false;

	if (vectorNearVertices.size() == 0) {

		// 3.a Extend the nearest
		if (getNearestVertex(target, vertexParent) <= 0)
			return 0;
		if (system->extendTo(vertexParent->getState(), target, trajectory, exactConnection) <= 0)
			return 0;
	}
	else {

		// 3.b Extend the best parent within the near vertices
		if (findBestParent(target, vectorNearVertices, vertexParent, trajectory, exactConnection) <= 0) {
			return 0;
		}
	}

	dist = trajectory.evaluateCost() + vertexParent->getCost();
	path.push_back(std::make_pair<double, double>(vertexParent->getState()[0], vertexParent->getState()[1]));
	while (vertexParent->parent != NULL) {
		vertexParent = vertexParent->parent;
		path.push_back(std::make_pair<double, double>(vertexParent->getState()[0], vertexParent->getState()[1]));
	}
	return 1;
}

template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::iteration () {
    
	clock_t t1, t2;
	float diff;
	float seconds;
	t1 = clock();
	
    // 1. Sample a new state
    State stateRandom;
	if (abs(system->sampleState(stateRandom) - 0.0)<0.001) {
		std::cout << "In Collision !!!!!" << std::endl;
		return 0;
	}
    
    // 2. Compute the set of all near vertices
    std::vector< Vertex<State,Trajectory,System>* > vectorNearVertices;
    getNearVertices (stateRandom, vectorNearVertices);
	t2 = clock();
	diff = ((float)t2 - (float)t1);
	seconds = diff / CLOCKS_PER_SEC;
	debug_time_1 += seconds;
    
    // 3. Find the best parent and extend from that parent
    Vertex<State,Trajectory,System>* vertexParent = NULL;  
    Trajectory trajectory;
    bool exactConnection = false;

	//std::cout << "nearvertices:" << vectorNearVertices.size() << std::endl;
    if (vectorNearVertices.size() == 0) {
        
        // 3.a Extend the nearest
        if (getNearestVertex (stateRandom, vertexParent) <= 0) 
            return 0;


		int r = system->extendTo(vertexParent->getState(), stateRandom, trajectory, exactConnection);
		
		if(r <=0)
			return 0;
		
		
    }
    else {
        // 3.b Extend the best parent within the near vertices
		int r = findBestParent(stateRandom, vectorNearVertices, vertexParent, trajectory, exactConnection);
		
		if (r <= 0)
			return 0;
    }
    // 3.c add the trajectory from the best parent to the tree
    Vertex<State,Trajectory,System>* vertexNew = insertTrajectory (*vertexParent, trajectory);
    if (vertexNew == NULL) 
        return 0;
    
	

	t1 = clock();
    // 4. Rewire the tree  
    if (vectorNearVertices.size() > 0) 
        rewireVertices (*vertexNew, vectorNearVertices);
    
	t2 = clock();
	diff = ((float)t2 - (float)t1);
	seconds = diff / CLOCKS_PER_SEC;
	debug_time_3 += seconds;
    return 1;
}

template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::temp_iteration(Polygon_2 *add_region) {

	
	// 1. Sample a new state
	State stateRandom;
	//system->sampleState(stateRandom);
	system->sampleStateInRegion(stateRandom,add_region);

	// 2. Compute the set of all near vertices
	std::vector< Vertex<State, Trajectory, System>* > vectorNearVertices;
	getNearVertices(stateRandom, vectorNearVertices);


	// 3. Find the best parent and extend from that parent
	Vertex<State, Trajectory, System>* vertexParent = NULL;
	Trajectory trajectory;
	bool exactConnection = false;

	if (vectorNearVertices.size() == 0) {

		// 3.a Extend the nearest
		if (getNearestVertex(stateRandom, vertexParent) <= 0)
			return 0; 
		if (system->extendTo(vertexParent->getState(), stateRandom, trajectory, exactConnection) <= 0)
			return 0;
	}
	else {

		// 3.b Extend the best parent within the near vertices
		if (findBestParent(stateRandom, vectorNearVertices, vertexParent, trajectory, exactConnection) <= 0) {
			//std::cout << "can not connect to any neighbors without collision" << std::endl;
			return 0;
		}
		else {
			//std::cout << "connect to some neighbors" << std::endl;
		}
	}

	// 3.c add the trajectory from the best parent to the tree
	Vertex<State, Trajectory, System>* vertexNew = temp_insertTrajectory(*vertexParent, trajectory);
	if (vertexNew == NULL)
		return 0;


	// 4. Rewire the tree  
	if (vectorNearVertices.size() > 0)
		temp_rewireVertices(*vertexNew, vectorNearVertices);


	return 1;
}

template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::full_temp_iteration(Polygon_2 *add_region) {


	// 1. Sample a new state
	State stateRandom;
	//system->sampleState(stateRandom);
	system->sampleStateInRegion(stateRandom, add_region);

	// 2. Compute the set of all near vertices
	std::vector< Vertex<State, Trajectory, System>* > vectorNearVertices;
	getNearVerticesBig(stateRandom, vectorNearVertices);

	//std::cout << "Big near vertex:" << vectorNearVertices.size() << std::endl;
	// 3. Find the best parent and extend from that parent
	Vertex<State, Trajectory, System>* vertexParent = NULL;
	Trajectory trajectory;
	bool exactConnection = false;

	if (vectorNearVertices.size() == 0) {

		// 3.a Extend the nearest
		if (getNearestVertex(stateRandom, vertexParent) <= 0)
			return 0;
		if (system->extendTo(vertexParent->getState(), stateRandom, trajectory, exactConnection) <= 0)
			return 0;
	}
	else {

		// 3.b Extend the best parent within the near vertices
		if (findBestParent(stateRandom, vectorNearVertices, vertexParent, trajectory, exactConnection) <= 0) {
			//std::cout << "can not connect to any neighbors without collision" << std::endl;
			return 0;
		}
		else {
			//std::cout << "connect to some neighbors" << std::endl;
		}
	}

	// 3.c add the trajectory from the best parent to the tree
	Vertex<State, Trajectory, System>* vertexNew = temp_insertTrajectory(*vertexParent, trajectory);
	if (vertexNew == NULL)
		return 0;


	// 4. Rewire the tree  
	if (vectorNearVertices.size() > 0)
		temp_rewireVerticesRecursive(*vertexNew, vectorNearVertices);


	return 1;
}

template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::temp_restore() {
/*  
	std::list<vertex_t*> listVertices;
	std::set<vertex_t*> temp_add_vertices;
	std::set<vertex_t*> new_added_parent;
	std::set<vertex_t*> temp_rewired_vertices;
	std::map<vertex_t*, vertex_t*> temp_rewired_old_parent;
	std::map<vertex_t*, double> temp_rewired_old_cost;


	KdTree *kdtree;

	vertex_t *root;
*/
	//delete new added temp vertex
	auto v = listVertices.begin();
	while (v != listVertices.end()) {
		if (temp_add_vertices.find(*v) != temp_add_vertices.end()) {
			listVertices.erase(v++);
		}
		else {
			break;
		}
	}
	numVertices -= temp_add_num;

	for (auto i = new_added_parent.begin(); i != new_added_parent.end(); i++) {
		//std::set<vertex_t*> child_list = (*i)->children;
		auto j = ((*i)->children).begin();
		while (j != ((*i)->children).end()) {
			if (temp_add_vertices.find(*j) != temp_add_vertices.end()) {
				((*i)->children).erase(j++);
			}
			else {
				j++;
			}
		}
	}
	for (auto i = temp_add_vertices.begin(); i != temp_add_vertices.end(); i++) {
		current_index_--;
		delete *i;
	}
	int q = 0;
	//std::cout << "restore_rewired" << std::endl;
	for (auto i = temp_rewired_vertices.begin(); i != temp_rewired_vertices.end(); i++) {
		if (temp_rewired_old_parent.find(*i) != temp_rewired_old_parent.end()) {
			(*i)->parent = temp_rewired_old_parent[*i];
			if ((*i)->parent != NULL) {
				if ((*i)->parent->children.find(*i) == (*i)->parent->children.end())
					(*i)->parent->children.insert(*i);
			}
		}

		(*i)->costFromRoot = temp_rewired_old_cost[*i];
		if ((*i)->trajFromParent && (*i)->parent) {
			delete (*i)->trajFromParent;
		}
		if (temp_rewired_old_children.find(*i) != temp_rewired_old_children.end()) {
			(*i)->children.insert(temp_rewired_old_children[*i]);
		}
		(*i)->trajFromParent = new Trajectory(temp_rewired_old_costfromparent[*i], (*i)->getState());
		(*i)->costFromParent = temp_rewired_old_costfromparent[*i];
		updateBranchCost(*(*i), 0);
		//std::cout << "q:" << q << std::endl;
		q++;

	}

	restore_tree(kdtree);

	temp_add_vertices.clear();
	new_added_parent.clear();
	temp_rewired_vertices.clear();
	temp_rewired_old_parent.clear();
	temp_rewired_old_cost.clear();
	temp_rewired_old_costfromparent.clear();
	temp_add_num = 0;
	// restore: 1, delete those temp_add_vertices
	//			2, remove these vertices from new_added_parent 's children list
	//			3, restore temp_rewired_vertices 's parent to original parent, restore original cost, from temp_rewired_old_parent
	//             and temp_rewired_old_cost 
	//          4, for the kdtree, remove those new added temp_added_nodes, restore the original rect using rect_min, rect_max
	//double rect_min[2];
	//double rect_max[2];

	

	return 1;
}

template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::getBestTrajectory (std::list<double*>& trajectoryOut) {
    
    if (lowerBoundVertex == NULL)
        return 0;
    
    Vertex<State,Trajectory,System>* vertexCurr = lowerBoundVertex;
    
    
    while (vertexCurr) {
        
        State& stateCurr = vertexCurr->getState();
        
        double *stateArrCurr = new double[2]; 
        stateArrCurr[0] = stateCurr[0];
        stateArrCurr[1] = stateCurr[1];
        
        trajectoryOut.push_front (stateArrCurr);
        
        Vertex<State,Trajectory,System>& vertexParent = vertexCurr->getParent(); 
        
        if (&vertexParent != NULL) {
            
            State& stateParent = vertexParent.getState();
            
            std::list<double*> trajectory;
            system->getTrajectory (stateParent, stateCurr, trajectory);
            
            trajectory.reverse ();
            for (std::list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++) {
                
                double *stateArrFromParentCurr = *iter;
                
                stateArrCurr = new double[2];
                stateArrCurr[0] = stateArrFromParentCurr[0];
                stateArrCurr[1] = stateArrFromParentCurr[1];
                
                trajectoryOut.push_front (stateArrCurr);
                
                delete [] stateArrFromParentCurr;
            }
        }
        
        vertexCurr = &vertexParent;
    }
    
    return 1;
}


#endif
