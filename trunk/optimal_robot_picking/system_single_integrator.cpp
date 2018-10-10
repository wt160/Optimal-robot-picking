#include "system_single_integrator.h"
#include <cmath>
#include <cstdlib>

#include <iostream>

using namespace std;
using namespace SingleIntegrator;

#define DISCRETIZATION_STEP 1


region::region () {
    
    numDimensions = 0;
    
    center = NULL;
    size = NULL;
}


region::~region () {
    
    if (center)
        delete [] center;
    if (size)
        delete [] size;
    
}

region::region(const region& other) {
	numDimensions = other.numDimensions;
	center = new double[numDimensions];
	size = new double[numDimensions];
	for (int i = 0; i < numDimensions; i++) {
		center[i] = other.center[i];
		size[i] = other.size[i];
	}
	
}

int region::setNumDimensions (int numDimensionsIn) {
    
    numDimensions = numDimensionsIn;
    
    if (center)
        delete [] center;
    center = new double[numDimensions];
    
    if (size)
        delete [] size;
    size = new double[numDimensions];
    
    return 1;
    
}


State::State () {
    
    numDimensions = 0;
    
    x = NULL;
}


State::~State () {
    
    if (x)
        delete [] x;
}

State::State(double x_, double y_) {
	numDimensions = 2;
	x = new double[numDimensions];
	x[0] = x_;
	x[1] = y_;
}

State::State (const State &stateIn) {
    
    numDimensions = stateIn.numDimensions;
    
    if (numDimensions > 0) {
        x = new double[numDimensions];
        
        for (int i = 0; i < numDimensions; i++) 
            x[i] = stateIn.x[i];
    }
    else {
        x = NULL;
    }
}


State& State::operator=(const State &stateIn){
    
    if (this == &stateIn)
        return *this;
    
    if (numDimensions != stateIn.numDimensions) {
        if (x) 
            delete [] x;
        numDimensions = stateIn.numDimensions;
        if (numDimensions > 0)
            x = new double[numDimensions];
    }
    
    for (int i = 0; i < numDimensions; i++) 
        x[i] = stateIn.x[i];
    
    return *this;
}


int State::setNumDimensions (int numDimensionsIn) {
    
    if (x)
        delete [] x;
    
    if (numDimensions < 0)
        return 0;
    
    numDimensions = numDimensionsIn;
    
    if (numDimensions > 0)
        x = new double[numDimensions];
    
    return 1;
}


Trajectory::Trajectory () {
    
    endState = NULL;
}


Trajectory::~Trajectory () {
    
    if (endState)
        delete endState;
}


Trajectory::Trajectory (const Trajectory &trajectoryIn) {
    
    endState = new State (trajectoryIn.getEndState()); 

}

Trajectory::Trajectory(double costFromParent,const  State& s) {
	endState = new State(s);
	totalVariation = costFromParent;
}


Trajectory& Trajectory::operator=(const Trajectory &trajectoryIn) {
    
    if (this == &trajectoryIn)
        return *this;
    
    if (endState)
        delete endState;
    
    
    endState = new State (trajectoryIn.getEndState());
    
    totalVariation = trajectoryIn.totalVariation;
    
    return *this;
}


double Trajectory::evaluateCost () {
    
    return totalVariation;
}




System::System () {
    
    numDimensions = 0;
	hash_unit_size = 200;
}


System::~System () {
	// for (auto i = obstacles.begin(); i != obstacles.end(); i++) {
		
	// 	if(*i != NULL)
	// 		delete *i;
	// }
	//delete hash_map_size;
}


int System::setNumDimensions (int numDimensionsIn) {
    
    if (numDimensions < 0)
        return 0;
    
    numDimensions = numDimensionsIn;
    
    rootState.setNumDimensions (numDimensions);
    
    return 1;
}


int System::getStateKey (State& stateIn, double* stateKey) {
    
    for (int i = 0; i < numDimensions; i++) 
        stateKey[i] =  stateIn.x[i] / regionOperating.size[i];
    
    return 1;
}



bool System::isReachingTarget (State &stateIn) {
    
    
    for (int i = 0; i < numDimensions; i++) {
        
        if (fabs(stateIn.x[i] - regionGoal.center[i]) > regionGoal.size[i]/2.0 ) 
            return false;
    }
    
    return true;
}

void System::generateHashMap() {
	//hash_map_size = new int(numDimensions);
	//for (int i = 0; i < numDimensions; i++) {
	//	hash_map_size[i] = floor(regionOperating.size[i] / hash_unit_size);
	//}

	for (auto ob = obstacles.begin(); ob != obstacles.end(); ob++) {
		std::set<std::pair<int, int>> ob_index_set = getIndexForHash(**ob, hash_unit_size);
		for (auto i = ob_index_set.begin(); i != ob_index_set.end(); i++) {
			if (obs_hash_map.find(*i) == obs_hash_map.end()) {
				std::set<Polygon_2*> temp;
				temp.insert(*ob);
				obs_hash_map[*i] = temp;
			}
			else {
				obs_hash_map[*i].insert(*ob);
			}

		}
	}

}

void System::updateHashMap(Polygon_2 *delete_region) {

	std::set<std::pair<int, int>> delete_index_set = getIndexForHash(*delete_region, hash_unit_size);
		for (auto i = delete_index_set.begin(); i != delete_index_set.end(); i++) {
			if (obs_hash_map.find(*i) == obs_hash_map.end()) {
			}
			else {
				obs_hash_map[*i].erase(delete_region);
			}

		}
	
}


std::set<std::pair<int, int>> System::getIndexForHash(Polygon_2 ob, double unit_size) {
	double upper_left_x, upper_left_y, upper_right_x, upper_right_y, down_left_x, down_left_y, down_right_x, down_right_y;
	std::set<std::pair<int, int>> result;
    Point_2 upper_left, upper_right, down_left, down_right;
    getBoundingPoly(ob, upper_left, upper_right, down_left, down_right, 3);

	// upper_left_x = ob.center[0] - ob.size[0] / 2;
	// upper_left_y = ob.center[1] - ob.size[1] / 2;
	// upper_right_x = ob.center[0] + ob.size[0] / 2;
	// upper_right_y = ob.center[1] - ob.size[1] / 2;
	// down_left_x = ob.center[0] - ob.size[0] / 2;
	// down_left_y = ob.center[1] + ob.size[1] / 2;
	// down_right_x = ob.center[0] + ob.size[0] / 2;
	// down_right_y = ob.center[1] + ob.size[1] / 2;
    upper_left_x = upper_left.get<0>();
    upper_left_y = upper_left.get<1>();
    upper_right_x = upper_right.get<0>();
    upper_right_y = upper_right.get<1>();
    down_left_x = down_left.get<0>();
    down_left_y = down_left.get<1>();
    down_right_x = down_right.get<0>();
    down_right_y = down_right.get<1>();
	std::pair<int, int> upper_left_index;
	upper_left_index.first = floor(upper_left_x / unit_size);
	upper_left_index.second = floor(upper_left_y / unit_size);
	std::pair<int, int> upper_right_index;
	upper_right_index.first = floor(upper_right_x / unit_size);
	upper_right_index.second = floor(upper_right_y / unit_size);
	std::pair<int, int> down_left_index;
	down_left_index.first = floor(down_left_x / unit_size);
	down_left_index.second = floor(down_left_y / unit_size);
	std::pair<int, int> down_right_index;
	down_right_index.first = floor(down_right_x / unit_size);
	down_right_index.second = floor(down_right_y / unit_size);

	result.insert(upper_left_index);
	result.insert(upper_right_index);
	result.insert(down_left_index);
	result.insert(down_right_index);
	return result;

}


bool System::isLineInCollision(double *start, double *end){
     
    double *dists = new double[numDimensions];
    for (int i = 0; i < numDimensions; i++) 
        dists[i] = end[i] - start[i];
    
    double distTotal = 0.0;
    for (int i = 0; i < numDimensions; i++) 
        distTotal += dists[i]*dists[i];
    distTotal = sqrt (distTotal);
    
    double incrementTotal = distTotal/DISCRETIZATION_STEP;
    
    // normalize the distance according to the disretization step
    for (int i = 0; i < numDimensions; i++)
        dists[i] /= incrementTotal;
    
    int numSegments = (int)floor(incrementTotal);
    
    double *stateCurr = new double[numDimensions];
    for (int i = 0; i < numDimensions; i++) 
        stateCurr[i] = start[i];
    
    for (int i = 0; i < numSegments; i++) {
        
        if (IsInCollision (stateCurr))  
            return true;
        
        for (int i = 0; i < numDimensions; i++)
            stateCurr[i] += dists[i];
    }
    
    if (IsInCollision (end))
        return true;

    return false;
}

bool System::IsInCollision (double *stateIn) {
	std::pair<int, int> state_index;
	state_index.first = floor(stateIn[0] / hash_unit_size);
	state_index.second = floor(stateIn[1] / hash_unit_size);
	if (obs_hash_map.find(state_index) == obs_hash_map.end()) {
		return false;
	}
	else {


		std::set<Polygon_2*> related_obs = obs_hash_map[state_index];

		for (std::set<Polygon_2*>::iterator iter = related_obs.begin(); iter != related_obs.end(); iter++) {

			Polygon_2 obstacleCurr = **iter;
			bool collisionFound = false;

			// for (int i = 0; i < numDimensions; i++)
			// 	if (fabs(obstacleCurr->center[i] - stateIn[i]) > obstacleCurr->size[i] / 2.0) {
			// 		collisionFound = false;
			// 		break;
			// 	}
            if(bg::within(Point_2(stateIn[0], stateIn[1]), obstacleCurr)){
                collisionFound = true;
            }

			if (collisionFound) {
				return true;
			}
		}

		return false;
	}
} 


int System::sampleState (State &randomStateOut) {
    
    randomStateOut.setNumDimensions (numDimensions);
    
    for (int i = 0; i < numDimensions; i++) {
        
        randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*regionOperating.size[i] 
        - regionOperating.size[i]/2.0 + regionOperating.center[i];
    }
    
    if (IsInCollision (randomStateOut.x))
        return 0;
    
    return 1;
}

int System::sampleStateInRegion(State &randomStateOut, Polygon_2* sampleRegion) {
	randomStateOut.setNumDimensions(numDimensions);
    double upper_left_x, upper_left_y, upper_right_x, upper_right_y, down_left_x, down_left_y, down_right_x, down_right_y;
    Point_2 upper_left, upper_right, down_left, down_right;
    getBoundingPoly(*sampleRegion, upper_left, upper_right, down_left, down_right, 3);

    // upper_left_x = ob.center[0] - ob.size[0] / 2;
    // upper_left_y = ob.center[1] - ob.size[1] / 2;
    // upper_right_x = ob.center[0] + ob.size[0] / 2;
    // upper_right_y = ob.center[1] - ob.size[1] / 2;
    // down_left_x = ob.center[0] - ob.size[0] / 2;
    // down_left_y = ob.center[1] + ob.size[1] / 2;
    // down_right_x = ob.center[0] + ob.size[0] / 2;
    // down_right_y = ob.center[1] + ob.size[1] / 2;
    upper_left_x = upper_left.get<0>();
    upper_left_y = upper_left.get<1>();
    upper_right_x = upper_right.get<0>();
    upper_right_y = upper_right.get<1>();
    down_left_x = down_left.get<0>();
    down_left_y = down_left.get<1>();
    down_right_x = down_right.get<0>();
    down_right_y = down_right.get<1>();
    randomStateOut.x[0] = (double)rand() / (RAND_MAX + 1.0)*(upper_right_x - upper_left_x)
            - (upper_right_x - upper_left_x) / 2.0 + (upper_right_x + upper_left_x) / 2.0 ;
randomStateOut.x[1] = (double)rand() / (RAND_MAX + 1.0)*(upper_right_y - down_right_y)
            - (upper_right_y - down_right_y) / 2.0 +  (upper_right_y + down_right_y) / 2.0;
	// for (int i = 0; i < numDimensions; i++) {

	// 	randomStateOut.x[i] = (double)rand() / (RAND_MAX + 1.0)*sampleRegion.size[i]
	// 		- sampleRegion.size[i] / 2.0 + sampleRegion.center[i];
	// }

	if (IsInCollision(randomStateOut.x))
		return 0;

	return 1;
}

int System::extendTo (State &stateFromIn, State &stateTowardsIn, Trajectory &trajectoryOut, bool &exactConnectionOut) {
    
    double *dists = new double[numDimensions];
    for (int i = 0; i < numDimensions; i++) 
        dists[i] = stateTowardsIn.x[i] - stateFromIn.x[i];
    
    double distTotal = 0.0;
    for (int i = 0; i < numDimensions; i++) 
        distTotal += dists[i]*dists[i];
    distTotal = sqrt (distTotal);
    
    double incrementTotal = distTotal/DISCRETIZATION_STEP;
    
    // normalize the distance according to the disretization step
    for (int i = 0; i < numDimensions; i++)
        dists[i] /= incrementTotal;
    
    int numSegments = (int)floor(incrementTotal);
    
    double *stateCurr = new double[numDimensions];
    for (int i = 0; i < numDimensions; i++) 
        stateCurr[i] = stateFromIn.x[i];
    
    for (int i = 0; i < numSegments; i++) {
        
        if (IsInCollision (stateCurr))  
            return 0;
        
        for (int i = 0; i < numDimensions; i++)
            stateCurr[i] += dists[i];
    }
    
    if (IsInCollision (stateTowardsIn.x))
        return 0;
    
    trajectoryOut.endState = new State (stateTowardsIn);
    trajectoryOut.totalVariation = distTotal;
    
    delete [] dists;
    delete [] stateCurr;
    
    exactConnectionOut = true;
    
    return 1;
}


double System::evaluateExtensionCost (State& stateFromIn, State& stateTowardsIn, bool &exactConnectionOut) {
    
    
    exactConnectionOut = true;
    
    double distTotal = 0.0;
    for (int i = 0; i < numDimensions; i++) {
        double distCurr = stateTowardsIn.x[i] - stateFromIn.x[i];
        distTotal += distCurr*distCurr;
    }
    
    return sqrt(distTotal);
    
}


int System::getTrajectory (State& stateFromIn, State& stateToIn, list<double*>& trajectoryOut) {
    
    double *stateArr = new double[numDimensions];
    for (int i = 0; i < numDimensions; i++)
        stateArr[i] = stateToIn[i];
    trajectoryOut.push_front (stateArr);
    
    return 1;
    
}


double System::evaluateCostToGo (State& stateIn) {
    
    double radius = 0.0;
    for (int i = 0; i < numDimensions; i++) 
        radius += regionGoal.size[i] * regionGoal.size[i];
    radius = sqrt(radius);
    
    double dist = 0.0;
    for (int i = 0; i < numDimensions; i++) 
        dist += (stateIn[i] - regionGoal.center[i])*(stateIn[0] - regionGoal.center[i]);
    dist = sqrt(dist);
    
    return dist - radius;
}
