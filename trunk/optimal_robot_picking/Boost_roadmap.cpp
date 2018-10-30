/*
 *  The core class (implementation) for roadmap building
 *
 *  Created on: Jan 30, 2015
 *  Author: Jingjin Yu
 */

#include "Boost_roadmap.h"
#include "Boost_helper_functions.h"

#include <utility>   
#include <algorithm> 
#include <vector>
#include <cmath>
#include <limits>
//#include <windows.h>
//#include <CGAL/Qt/Converter.h>
//#include <CGAL/Boolean_set_operations_2.h>

#include <QGraphicsSimpleTextItem>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/lookup_edge.hpp>

//#define _ADD_SPECIAL_CONS
/*------------------------------------------------------------------------------------------
			see resources/lattice-indexing.pptx for the processing logic
--------------------------------------------------------------------------------------------*/

//static const QColor BASIC_QCOLORS8[] = {Qt::red, Qt::blue, Qt::green, Qt::magenta, Qt::cyan, Qt::gray, Qt::black, Qt::yellow};
//static int colorCounter = 0;
//#define SMALL_ENV

#define TOTAL_EXIT_NUM 4
//#define HAVE_OBS





#ifdef BIG_ENV
#define START_X 0
#define START_Y 2500
#define EXIT_0_X 2500
#define EXIT_0_Y 5000
#define EXIT_1_X 2500 
#define EXIT_1_Y 0
#define EXIT_2_X 0 
#define EXIT_2_Y 2500
#define EXIT_3_X 5000
#define EXIT_3_Y 2500
#endif
#ifdef SMALL_ENV
#define START_X 500
#define START_Y 1000
#define EXIT_0_X 500 
#define EXIT_0_Y 1000
#define EXIT_1_X 1000 
#define EXIT_1_Y 500
#define EXIT_2_X 500 
#define EXIT_2_Y 0
#define EXIT_3_X 0 
#define EXIT_3_Y 500
#endif

#define GRASP_CENTER_DIST 5
#define PICK_DIST 48
#define CUBE_WIDTH 30
#define GRIPPER_LENGTH 26
#define GRIPPER_WIDTH 14
#define ROBOT_RADIUS 30

TreeNode::TreeNode():parent(NULL) {};

TreeNode::TreeNode(int index, double dist) :
	index_(index),
	distance(dist),
	parent(NULL)
{}


int TreeNode::countNodesRec(TreeNode *root, int& count)
{
	TreeNode *parent = root;
	TreeNode *child = NULL;

	for (int it = 0; it < parent->childrenNumber(); it++)
	{
		child = parent->getChild(it);
		count++;
		//std::cout<<child->getTextContent()<<" Number : "<<count<<std::endl;
		if (child->childrenNumber() > 0)
		{
			countNodesRec(child, count);
		}
	}

	return count;
}

void TreeNode::appendChild(TreeNode *child)
{
	child->setParent(this);
	children.push_back(child);
}

void TreeNode::setParent(TreeNode *theParent)
{
	parent = theParent;
}

void TreeNode::popBackChild()
{
	children.pop_back();
}

void TreeNode::removeChild(int pos)
{
	if (children.size() > 0) {
		children.erase(children.begin() + pos);
	}
	else {
		children.pop_back();
	}
}

bool TreeNode::hasChildren()
{
	if (children.size() > 0)
		return true;
	else
		return false;
}

bool TreeNode::hasParent()
{
	if (parent != NULL)
		return true;
	else
		return false;
}

TreeNode * TreeNode::getParent()
{
	return parent;
}

std::vector<TreeNode*> TreeNode::getAllParents()
{
	std::vector<TreeNode*> parent_list;
	parent_list.push_back(this);
	TreeNode * temp = getParent();
	while (temp != NULL) {
		parent_list.push_back(temp);
		temp = temp->getParent();
	}
	return parent_list;
}

TreeNode* TreeNode::getChild(int pos)
{
	if (children.size() < pos)
		return NULL;
	else
		return children[pos];
}

int TreeNode::childrenNumber()
{
	return children.size();
}

int TreeNode::grandChildrenNum()
{
	int t = 0;

	if (children.size() < 1)
	{
		return 0;
	}

	countNodesRec(this, t);

	return t;
}

std::string TreeNode::getTextContent()
{
	return textContent;
}

std::string TreeNode::getTagName()
{
	return tagName;
}

double Roadmap::declutterUsingMultipleGreedy(int num_objs) {
	std::vector<int> picking_sequence;
	System robot2d_2;
	planner_t rrts_2;
	//PRM prm_2;
	robot2d_2.setNumDimensions(2);
	double optimal_dist = 0;
	robot2d_2.regionOperating.setNumDimensions(2);
#ifdef BIG_ENV
	robot2d_2.regionOperating.center[0] = 2500.0;
	robot2d_2.regionOperating.center[1] = 2500.0;
	robot2d_2.regionOperating.size[0] = 5000.0;
	robot2d_2.regionOperating.size[1] = 5000.0;
#endif
#ifdef SMALL_ENV
	robot2d_2.regionOperating.center[0] = 500.0;
	robot2d_2.regionOperating.center[1] = 500.0;
	robot2d_2.regionOperating.size[0] = 1000.0;
	robot2d_2.regionOperating.size[1] = 1000.0;
#endif
	robot2d_2.regionGoal.setNumDimensions(2);
	robot2d_2.regionGoal.center[0] = 300000.0;
	robot2d_2.regionGoal.center[1] = 300000.0;
	robot2d_2.regionGoal.size[0] = 2.0;
	robot2d_2.regionGoal.size[1] = 2.0;


	Polygon2_list obs_outer_list = m_objectOuterPolyList;
	Polygon2_list obs_list = m_objectPolyList;

	// insert obstacles into system
	double min_x = 0;
	double max_x = 0;
	double min_y = 0;
	double max_y = 0;
	// if (robot2d_2.obstacles.size() > 0) {
	// 	for (auto i = robot2d_2.obstacles.begin(); i != robot2d_2.obstacles.end(); i++) {
	// 		delete *i;
	// 	}
	// }
	robot2d_2.obstacles.clear();
	for(auto ob = obs_list.begin(); ob != obs_list.end(); ob++) {
		// region *obstacle;
		Polygon_2 * obstacle = new Polygon_2;
		*obstacle = *ob;
		// obstacle = new region;
		// obstacle->setNumDimensions(2);
		//convertPolygon2Region(*ob, obstacle->center[0], obstacle->center[1], obstacle->size[0], obstacle->size[1]);
		robot2d_2.obstacles.push_back(obstacle);  // Add the obstacle to the list
	}
	
	robot2d_2.generateHashMap();
	// Add the system to the planner
	rrts_2.setSystem(robot2d_2);
	//prm_2.setSystem(robot2d_2);
	// Set up the root vertex
	vertex_t &root = rrts_2.getRootVertex();
	State &rootState = root.getState();
	rootState[0] = START_X;
	rootState[1] = START_Y;



	// Initialize the planner
	rrts_2.initialize();

	// This parameter should be larger than 1.5 for asymptotic 
	//   optimality. Larger values will weigh on optimization 
	//   rather than exploration in the RRT* algorithm. Lower 
	//   values, such as 0.1, should recover the RRT.
	rrts_2.setGamma(1.5);
#ifdef BIG_ENV
	for (int i = 0; i < 10000; i++)
		rrts_2.iteration();
	// prm_2.createGraph(8000, 5);
#endif
#ifdef SMALL_ENV
	for (int i = 0; i < 2000; i++)
		rrts_2.iteration();
#endif

	//region* erase_region = robot2d.obstacles.front();
	//robot2d.obstacles.pop_front();
	//for (int i = 0; i < 1000; i++)
	//	rrts.temp_iteration(*erase_region);


	//rrts.temp_restore();
	std::vector<TreeNode*> branch_list;
	TreeNode* origin = new TreeNode(-1, 0.0);
	branch_list.push_back(origin);
	int level = 0;
	int ahead_step = 0;
	int ahead_total_step = 3;
	std::vector<TreeNode*> temp_node_list;
	std::vector<TreeNode*> last_level_list;
	std::list<Polygon_2*> backup_obstacles;
	backup_obstacles = robot2d_2.obstacles;
	std::map<std::pair<int, int>, std::set<Polygon_2*>> back_hashmap = robot2d_2.obs_hash_map;
	bool at_bottom = false;
	while (level < num_objs) {
		TreeNode* least_child;
		ahead_step = 0;
		std::cout << "level " << level << std::endl;
		while (ahead_step < ahead_total_step) {
			//std::cout << "ahead step:" << ahead_step << std::endl;
			if (ahead_step == (ahead_total_step - 1)) {
				double least_dist = 10000000000;
				for (auto heu1 = branch_list.begin(); heu1 != branch_list.end(); heu1++) {
					std::vector<TreeNode*> all_parents = (*heu1)->getAllParents();
					//std::cout << (*heu1)->index_ << " , ";
					std::set<int> parents_index;
					double distance_till = 0;
					for (int j = 0; j < all_parents.size(); j++) {
						parents_index.insert(all_parents[j]->index_);
						distance_till += all_parents[j]->distance;
					}
					(*heu1)->distance_till_now = distance_till;
					(*heu1)->all_parents_index = parents_index;
					
					if (distance_till < least_dist) {
						least_dist = distance_till;
						least_child = *heu1;
					}
				}
				//std::cout << "ahead end:" << least_child->index_ << std::endl;
				if (level == num_objs - 2) {
					last_level_list.push_back(least_child);
					at_bottom = true;
					break;
				}
				for (int k = 0; k < ahead_total_step-2; k++) {

					least_child = least_child->getParent();
				}
				ahead_step = 0;
				break;
			}
			for (auto i = branch_list.begin(); i != branch_list.end(); i++) {
				robot2d_2.obstacles.clear();
				robot2d_2.obstacles = backup_obstacles;
				robot2d_2.obs_hash_map = back_hashmap;
				//std::cout << (*i)->index_ << " , ";
				std::vector<TreeNode*> all_parents = (*i)->getAllParents();
				std::vector<int> parents_index;
				for (int j = 0; j < all_parents.size(); j++) {
					parents_index.push_back(all_parents[j]->index_);
				}
				Polygon2_list temp_obs_list;
				Polygon2_list temp_obs_outer_list;
				int current_object_index = 0;
				Polygon_2 current;
				int obs_index = 0;
				auto delete_obs = robot2d_2.obstacles.begin();
				while (delete_obs != robot2d_2.obstacles.end()) {
					bool is_delete = false;
					for (int k = 0; k < parents_index.size(); k++) {
						if (parents_index[k] == obs_index) {
							is_delete = true;
							Polygon_2 *erase_region = *delete_obs;
							robot2d_2.obstacles.erase(delete_obs++);
							robot2d_2.updateHashMap(erase_region);
#ifdef BIG_ENV
							//prm_2.sampleInRegion(*erase_region, 20, 10);
							for (int i = 0; i < 10; i++) {
								if (i == 9) {
									//std::cout << "full temp_iteration:" << std::endl;
									rrts_2.full_temp_iteration(erase_region);
								}
								else {
									rrts_2.temp_iteration(erase_region);
								}
							}
#endif
#ifdef SMALL_ENV
							for (int i = 0; i < 5; i++)
								rrts_2.temp_iteration(erase_region);
#endif
							break;
						}

					}
					if (!is_delete) {
						delete_obs++;
					}
					obs_index++;
				}
				//find out the original index of current object, based on original global obstaclePolyList
				for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
					bool is_delete = false;
					for (int k = 0; k < parents_index.size(); k++) {
						if (parents_index[k] == current_object_index) {
							is_delete = true;
							break;
						}
					}
					if (!is_delete) {
						temp_obs_list.push_back(*ob);
					}
					current_object_index++;
				}
				current_object_index = 0;
				for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
					bool is_delete = false;
					for (int k = 0; k < parents_index.size(); k++) {
						if (parents_index[k] == current_object_index) {
							is_delete = true;
							break;
						}
					}
					if (!is_delete) {
						temp_obs_outer_list.push_back(*ob);
					}
					current_object_index++;
				}


				std::map<int, graspDist> candidates = getAllObjects(temp_obs_list, temp_obs_outer_list, rrts_2);
				for (auto candi = candidates.begin(); candi != candidates.end(); candi++) {
					TreeNode* child = new TreeNode(candi->first, candi->second.shortestDist);
					//std::cout << "obj " << candi->first << " ";
					(*i)->appendChild(child);
					temp_node_list.push_back(child);
					
				}
				rrts_2.temp_restore();
				//prm_2.restore_temp();
			}
			branch_list.clear();
			branch_list = temp_node_list;
			temp_node_list.clear();
			ahead_step++;
		}
		if (at_bottom) {
			break;
		}
		//std::cout << std::endl;
		branch_list.clear();
		//std::cout << "level " << level << " , index is" << least_child->index_ << std::endl;
		branch_list.push_back(least_child);
		temp_node_list.clear();
		level++;
	}
	TreeNode* optim = last_level_list[0];
	//std::cout << "total sequence num:" << last_level_list.size() << std::endl;
	int optimal_sequence_index = 0;
	double optimalDist = 100000000;
	
	
	optimal_dist = optim->distance_till_now;
	std::cout << "optimal distance:" << optim->distance_till_now << std::endl;
	std::cout << "pick sequence:";
	auto it = picking_sequence.begin();
	while (optim->getParent() != NULL) {
		picking_sequence.push_back(optim->index_);
		std::cout << optim->index_ << ",";
		optim = optim->getParent();
	}
	// for (auto p = backup_obstacles.end(); p != backup_obstacles.end(); p++) {
	// 	delete *p;
	// }
	std::reverse(std::begin(picking_sequence), std::end(picking_sequence));
	return optimal_dist;
}


double Roadmap::declutterUsingMultiExitTruncatedTree(int num_objs) {
	 
	planner_t rrts_exit_0_tree;   
	planner_t rrts_exit_1_tree;
	planner_t rrts_exit_2_tree;
	planner_t rrts_exit_3_tree;
	System robot2d_multiexit_tree;
	std::vector<std::pair<double, double>> exit_list;
	exit_list.push_back(std::make_pair(EXIT_0_X, EXIT_0_Y));
	exit_list.push_back(std::make_pair(EXIT_1_X, EXIT_1_Y));
	//exit_list.push_back(std::make_pair(EXIT_2_X, EXIT_2_Y));
	//exit_list.push_back(std::make_pair(EXIT_3_X, EXIT_3_Y));

	std::vector<planner_t> rrts_list;
	std::vector<int> picking_sequence;
	robot2d_multiexit_tree.setNumDimensions(2);
	robot2d_multiexit_tree.regionOperating.setNumDimensions(2);
#ifdef BIG_ENV
	robot2d_multiexit_tree.regionOperating.center[0] = 2500.0;
	robot2d_multiexit_tree.regionOperating.center[1] = 2500.0;
	robot2d_multiexit_tree.regionOperating.size[0] = 5000.0;
	robot2d_multiexit_tree.regionOperating.size[1] = 5000.0;
#endif
#ifdef SMALL_ENV
	robot2d_multiexit_tree.regionOperating.center[0] = 500.0;
	robot2d_multiexit_tree.regionOperating.center[1] = 500.0;
	robot2d_multiexit_tree.regionOperating.size[0] = 1000.0;
	robot2d_multiexit_tree.regionOperating.size[1] = 1000.0;
#endif
	robot2d_multiexit_tree.regionGoal.setNumDimensions(2);
	robot2d_multiexit_tree.regionGoal.center[0] = 300000.0;
	robot2d_multiexit_tree.regionGoal.center[1] = 300000.0;
	robot2d_multiexit_tree.regionGoal.size[0] = 2.0;
	robot2d_multiexit_tree.regionGoal.size[1] = 2.0;


	Polygon2_list obs_outer_list = m_objectOuterPolyList;
	Polygon2_list obs_list = m_objectPolyList;

	// insert obstacles into system
	double min_x = 0;
	double max_x = 0;
	double min_y = 0;
	double max_y = 0;
	if (robot2d_multiexit_tree.obstacles.size() > 0) {
	 	for (auto i = robot2d_multiexit_tree.obstacles.begin(); i != robot2d_multiexit_tree.obstacles.end(); i++) {
	 		delete *i;
	 	}
	}
	robot2d_multiexit_tree.obstacles.clear();
	for (auto ob = obs_list.begin(); ob != obs_list.end(); ob++) {
		// region *obstacle;
		Polygon_2 * obstacle = new Polygon_2;
		*obstacle = *ob;
		// obstacle = new region;
		// obstacle->setNumDimensions(2);
		//convertPolygon2Region(*ob, obstacle->center[0], obstacle->center[1], obstacle->size[0], obstacle->size[1]);
		robot2d_multiexit_tree.obstacles.push_back(obstacle);  // Add the obstacle to the list
	}
	
	robot2d_multiexit_tree.generateHashMap();
	rrts_list.push_back(rrts_exit_0_tree);
	rrts_list.push_back(rrts_exit_1_tree);
	//rrts_list.push_back(rrts_exit_2_tree);
	//rrts_list.push_back(rrts_exit_3_tree);

	for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
		rrts_list[rrts_index].setSystem(robot2d_multiexit_tree);

		// Set up the root vertex
		vertex_t &root = rrts_list[rrts_index].getRootVertex();
		State &rootState = root.getState();
		rootState[0] = exit_list[rrts_index].first; 
		rootState[1] = exit_list[rrts_index].second;



		// Initialize the planner
		rrts_list[rrts_index].initialize();

		// This parameter should be larger than 1.5 for asymptotic 
		//   optimality. Larger values will weigh on optimization 
		//   rather than exploration in the RRT* algorithm. Lower 
		//   values, such as 0.1, should recover the RRT.
		rrts_list[rrts_index].setGamma(1.5);
#ifdef BIG_ENV
		for (int i = 0; i < 10000; i++)
			rrts_list[rrts_index].iteration(); 
#endif
#ifdef SMALL_ENV
		for (int i = 0; i < 2000; i++)
			rrts_list[rrts_index].iteration();
#endif
	}
	clock_t t1, t2;
	float diff = 0; 
	float seconds = 0;    


	std::vector<TreeNode*> branch_list;
	TreeNode* origin = new TreeNode(-1, 0.0);
	branch_list.push_back(origin);
	int level = 0;
	std::vector<TreeNode*> temp_node_list;
	std::vector<TreeNode*> last_level_list;
	std::list<Polygon_2*> backup_obstacles;
	backup_obstacles = robot2d_multiexit_tree.obstacles;
	std::map<std::pair<int, int>, std::set<Polygon_2*>> back_hashmap = robot2d_multiexit_tree.obs_hash_map;
	while (level < num_objs + num_objs ) {
		std::cout << "level " << level << std::endl;
		for (auto heu1 = branch_list.begin(); heu1 != branch_list.end(); heu1++) {
			std::vector<TreeNode*> all_parents = (*heu1)->getAllParents();
			//std::cout << (*heu1)->index_ << " , ";
			std::set<int> parents_index;
			double distance_till = 0;
			for (int j = 0; j < all_parents.size(); j++) {
				parents_index.insert(all_parents[j]->index_);
				distance_till += all_parents[j]->distance;
			}
			(*heu1)->distance_till_now = distance_till;
			(*heu1)->all_parents_index = parents_index;   
		}
		std::cout << "level done" << std::endl;
		//std::cout << std::endl;
		std::vector<TreeNode*> selected_list;
		if (level > 1) {
			while (branch_list.size() > 0) {
				std::set<int> all_parents = branch_list[0]->all_parents_index;
				std::vector<int> same_parent_list;
				same_parent_list.push_back(0);
				for (int p = 0; p < branch_list.size(); p++) {
					if (p != 0) {
						if (all_parents == branch_list[p]->all_parents_index) {
							same_parent_list.push_back(p);
						}
					}
				}
				int optimal_index = 0;
				double optimal_dist = 10000000;
				for (int k = 0; k < same_parent_list.size(); k++) {
					if (branch_list[same_parent_list[k]]->distance_till_now < optimal_dist) {
						optimal_dist = branch_list[same_parent_list[k]]->distance_till_now;
						optimal_index = same_parent_list[k];
					}
				}
				selected_list.push_back(branch_list[optimal_index]);

				std::vector<TreeNode*> temp_keep_list;
				for (int p = 0; p < branch_list.size(); p++) {
					bool is_keep = true;
					for (int k = 0; k < same_parent_list.size(); k++) {
						if (same_parent_list[k] == p) {
							is_keep = false;
							break;
						}
					}
					if (is_keep) {
						temp_keep_list.push_back(branch_list[p]);
					}
				}
				branch_list.clear();
				branch_list = temp_keep_list;
			}
			branch_list = selected_list;
		}
		//std::cout << "after heu1 truncation" << std::endl;
		for (auto i = branch_list.begin(); i != branch_list.end(); i++) {
			robot2d_multiexit_tree.obstacles.clear();
			robot2d_multiexit_tree.obstacles = backup_obstacles;
			robot2d_multiexit_tree.obs_hash_map = back_hashmap;
			//std::cout << (*i)->index_ << " , " << std::endl;
			std::vector<TreeNode*> all_parents = (*i)->getAllParents();
			std::vector<int> parents_index;
			for (int j = 0; j < all_parents.size(); j++) {
				if (all_parents[j]->node_type == "object") { 
					parents_index.push_back(all_parents[j]->index_);
				}
			}
			Polygon2_list temp_obs_list;
			Polygon2_list temp_obs_outer_list;
			int current_object_index = 0;
			Polygon_2 current;
			int obs_index = 0;
			auto delete_obs = robot2d_multiexit_tree.obstacles.begin();
			while (delete_obs != robot2d_multiexit_tree.obstacles.end()) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == obs_index) {
						is_delete = true;
						Polygon_2* erase_region = *delete_obs;
						robot2d_multiexit_tree.obstacles.erase(delete_obs++);
						robot2d_multiexit_tree.updateHashMap(erase_region);


						for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {


#ifdef BIG_ENV
							for (int i = 0; i < 10; i++) {
								if (i == 9) {
									//std::cout << "full temp_iteration:" << std::endl;
									rrts_list[rrts_index].full_temp_iteration(erase_region);
								}
								else {
									rrts_list[rrts_index].temp_iteration(erase_region);
								}
							}
			
#endif
#ifdef SMALL_ENV
							for (int i = 0; i < 5; i++)
								rrts_list[rrts_index].temp_iteration(*erase_region);
#endif

						}
						break;
					}

				}
				if (!is_delete) {
					delete_obs++;
				}
				obs_index++;
			}
			//find out the original index of current object, based on original global obstaclePolyList
			for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == current_object_index) {
						is_delete = true;
						break;
					}
				}
				if (!is_delete) {
					temp_obs_list.push_back(*ob);
				}
				current_object_index++;
			}
			current_object_index = 0;
			for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == current_object_index) {
						is_delete = true;
						break;
					}
				}
				if (!is_delete) {
					temp_obs_outer_list.push_back(*ob);
				}
				current_object_index++;
			}
			if (level % 2 == 0) {
				std::pair<double, double> temp_exit;
				int exit_index = 0;

				if (level == 0) {
					temp_exit.first = EXIT_0_X;
					temp_exit.second = EXIT_0_Y;
					exit_index = 0;
				}
				else {
					temp_exit.first = exit_list[(*i)->index_].first;
					temp_exit.second = exit_list[(*i)->index_].second;
					exit_index = (*i)->index_;
				}
				std::map<int, graspDist> candidates = getCandidateObjectsFromExit(temp_obs_list, temp_obs_outer_list, temp_exit, rrts_list[exit_index]);
				for (auto candi = candidates.begin(); candi != candidates.end(); candi++) {
					TreeNode* child = new TreeNode(candi->first, candi->second.shortestDist);
					//std::cout << "obj " << candi->first << " ";
					child->node_type = "object";    
					child->coordinate = std::make_pair(candi->second.grasp.get<0>(), candi->second.grasp.get<1>());     
					//std::cout << "obj grasp point:" << child->coordinate.first << "," << child->coordinate.second << std::endl;
					(*i)->appendChild(child);
					temp_node_list.push_back(child);
					if (level == num_objs + num_objs - 1) {
						last_level_list.push_back(child);
					}
				}
				for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
						rrts_list[rrts_index].temp_restore();
				}



			}
			else {
				for (int e = 0; e < exit_list.size(); e++) {
					std::list<Point_2>  path;
					double temp_dist = new_computeShortestPath(exit_list[e].first, exit_list[e].second, (*i)->coordinate.first, (*i)->coordinate.second, path, rrts_list[e] );
					//std::cout << "exit point:" << exit_list[e].first << "," << exit_list[e].second << std::endl;
					//std::cout << "next grasp point:" << (*i)->coordinate.first << "," << (*i)->coordinate.second << std::endl;

					//std::cout << "exit " << e << " ";
					//std::cout << "dist:" << temp_dist << std::endl;
					TreeNode* child = new TreeNode(e, temp_dist);
					child->node_type = "exit";
					child->coordinate = std::make_pair(exit_list[e].first, exit_list[e].second);
					(*i)->appendChild(child);
					temp_node_list.push_back(child);
					if (level == num_objs + num_objs - 1) {
						last_level_list.push_back(child);
					}
				}
				for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
					rrts_list[rrts_index].temp_restore();
				}
			}

			
		}
		std::cout << std::endl;
		branch_list.clear();
		branch_list = temp_node_list;
		temp_node_list.clear();
		level++;
	}
	//std::vector<std::vector<int>> pick_sequence = generateCombUsingTree(origin);
	std::cout << "total sequence num:" << last_level_list.size() << std::endl;
	int optimal_sequence_index = 0;
	double optimalDist = 100000000;
	for (int q = 0; q < last_level_list.size(); q++) {
		double current_dist = 0;
		current_dist += last_level_list[q]->distance;
		TreeNode* temp = last_level_list[q];
		while (temp->getParent() != NULL) {
			temp = temp->getParent();
			current_dist += temp->distance;
		}
		if (current_dist < optimalDist) {
			optimalDist = current_dist;
			optimal_sequence_index = q;
		}
	}
	std::cout << "optimal total dist:" << optimalDist << std::endl;
	TreeNode* optim = last_level_list[optimal_sequence_index];
	std::cout << "pick sequence:";
	auto it = picking_sequence.begin();
	while (optim->getParent() != NULL) {
		picking_sequence.push_back(optim->index_);
		std::cout << optim->index_ << ",";
		optim = optim->getParent();
	}
	for (auto p = backup_obstacles.end(); p != backup_obstacles.end(); p++) {
		delete *p;
	}
	std::reverse(std::begin(picking_sequence), std::end(picking_sequence));
	return optimalDist;
	
}

double Roadmap::declutterUsingTruncatedTree(int num_objs) {
	System robot2d;
	planner_t rrts;
	std::vector<int> picking_sequence;
	robot2d.setNumDimensions(2);
	robot2d.regionOperating.setNumDimensions(2);
#ifdef BIG_ENV
	robot2d.regionOperating.center[0] = 2500.0;
	robot2d.regionOperating.center[1] = 2500.0;
	robot2d.regionOperating.size[0] = 5000.0;
	robot2d.regionOperating.size[1] = 5000.0;
#endif
#ifdef SMALL_ENV
	robot2d.regionOperating.center[0] = 500.0;
	robot2d.regionOperating.center[1] = 500.0;
	robot2d.regionOperating.size[0] = 1000.0;
	robot2d.regionOperating.size[1] = 1000.0;
#endif
	robot2d.regionGoal.setNumDimensions(2);
	robot2d.regionGoal.center[0] = 300000.0;

	robot2d.regionGoal.center[1] = 300000.0;
	robot2d.regionGoal.size[0] = 2.0;
	robot2d.regionGoal.size[1] = 2.0;


	Polygon2_list obs_outer_list = m_objectOuterPolyList;
	Polygon2_list obs_list = m_objectPolyList;
	Polygon2_list obstacle_outer_list = m_obstacleOuterPolyList;
	Polygon2_list obstacle_list = m_obstaclePolyList;
	// insert obstacles into system
	double min_x = 0;
	double max_x = 0;
	double min_y = 0;
	double max_y = 0;
	// if (robot2d.obstacles.size() > 0) {
	// 	for (auto i = robot2d.obstacles.begin(); i != robot2d.obstacles.end(); i++) {
	// 		delete *i;
	// 	}
	// }
	robot2d.obstacles.clear();
	for(auto ob = obs_list.begin(); ob != obs_list.end(); ob++) {
		// region *obstacle;
		Polygon_2 * region = new Polygon_2;
		*region = *ob;
		// obstacle = new region;
		// obstacle->setNumDimensions(2);
		//convertPolygon2Region(*ob, obstacle->center[0], obstacle->center[1], obstacle->size[0], obstacle->size[1]);
		robot2d.obstacles.push_back(region);  // Add the obstacle to the list
	}
	// for (auto ob = obstacle_list.begin(); ob != obstacle_list.end(); ob++) {
	// 	region *obstacle;

	// 	obstacle = new region;
	// 	obstacle->setNumDimensions(2);
	// 	convertPolygon2Region(*ob, obstacle->center[0], obstacle->center[1], obstacle->size[0], obstacle->size[1]);
	// 	robot2d.obstacles.push_back(obstacle);  // Add the obstacle to the list
	// }

	robot2d.generateHashMap();
	// Add the system to the planner
	rrts.setSystem(robot2d);

	// Set up the root vertex
	vertex_t &root = rrts.getRootVertex();
	State &rootState = root.getState();
	rootState[0] = START_X;
	rootState[1] = START_Y;



	// Initialize the planner
	rrts.initialize();

	// This parameter should be larger than 1.5 for asymptotic 
	//   optimality. Larger values will weigh on optimization 
	//   rather than exploration in the RRT* algorithm. Lower 
	//   values, such as 0.1, should recover the RRT.
	rrts.setGamma(1.5);
	clock_t t1, t2;
	t1 = clock();
#ifdef BIG_ENV
	for (int i = 0; i < 10000; i++)
		rrts.iteration();

	//std::cout << rrts.debug_time_1 << "," << rrts.debug_time_2 << "," << rrts.debug_time_3 << std::endl;
#endif
#ifdef SMALL_ENV
	for (int i = 0; i < 2000; i++)
		rrts.iteration();
#endif
	t2 = clock();
	float diff((float)t2 - (float)t1);
	float seconds = diff / CLOCKS_PER_SEC;
	
	//region* erase_region = robot2d.obstacles.front();
	//robot2d.obstacles.pop_front();
	//for (int i = 0; i < 1000; i++)
	//	rrts.temp_iteration(*erase_region);

	//std::cout << "done main iteration, use time " <<seconds<< std::endl;
	//rrts.temp_restore();
	std::vector<TreeNode*> branch_list;
	TreeNode* origin = new TreeNode(-1, 0.0);
	branch_list.push_back(origin);
	int level = 0;
	std::vector<TreeNode*> temp_node_list;
	std::vector<TreeNode*> last_level_list;
	std::list<Polygon_2*> backup_obstacles;
	backup_obstacles = robot2d.obstacles;
	std::map<std::pair<int, int>, std::set<Polygon_2*>> back_hashmap = robot2d.obs_hash_map;
	while (level < num_objs) {
		std::cout << "level " << level << std::endl;
		for (auto heu1 = branch_list.begin(); heu1 != branch_list.end(); heu1++) {
			std::vector<TreeNode*> all_parents = (*heu1)->getAllParents();
			//std::cout << (*heu1)->index_ << " , ";
			std::set<int> parents_index;
			double distance_till = 0;
			for (int j = 0; j < all_parents.size(); j++) {
				parents_index.insert(all_parents[j]->index_);
				distance_till += all_parents[j]->distance;
			}
			(*heu1)->distance_till_now = distance_till;
			(*heu1)->all_parents_index = parents_index;
		}
		//std::cout <<"level done"<< std::endl;
		//std::cout << std::endl;
		// this is to eliminate the redundant ones, change from permutation to combination
		std::vector<TreeNode*> selected_list;
		if (level > 1) {
			while (branch_list.size() > 0) {
				std::set<int> all_parents = branch_list[0]->all_parents_index;
				std::vector<int> same_parent_list;
				same_parent_list.push_back(0);
				for (int p = 0; p < branch_list.size(); p++) {
					if (p != 0) {
						if (all_parents == branch_list[p]->all_parents_index) {
							same_parent_list.push_back(p);
						}
					}
				}
				int optimal_index = 0;
				double optimal_dist = 10000000;
				for (int k = 0; k < same_parent_list.size(); k++) {
					if (branch_list[same_parent_list[k]]->distance_till_now < optimal_dist) {
						optimal_dist = branch_list[same_parent_list[k]]->distance_till_now;
						optimal_index = same_parent_list[k];
					}
				}
				selected_list.push_back(branch_list[optimal_index]);
				
				std::vector<TreeNode*> temp_keep_list;
				for (int p = 0; p < branch_list.size(); p++) {
					bool is_keep = true;
					for (int k = 0; k < same_parent_list.size(); k++) {
						if (same_parent_list[k] == p) {
							is_keep = false;
							break;
						}
					}
					if (is_keep) {
						temp_keep_list.push_back(branch_list[p]);
					}
				}
				branch_list.clear();
				branch_list = temp_keep_list;
			}
			branch_list = selected_list;
		}
		//std::cout << "after heu1 truncation" << std::endl;
		for (auto i = branch_list.begin(); i != branch_list.end(); i++) {
			robot2d.obstacles.clear();
			robot2d.obstacles = backup_obstacles;
			robot2d.obs_hash_map = back_hashmap;
			//std::cout << (*i)->index_ << " , " << std::endl;
			std::vector<TreeNode*> all_parents = (*i)->getAllParents();
			std::vector<int> parents_index;
			for (int j = 0; j < all_parents.size(); j++) {
				parents_index.push_back(all_parents[j]->index_);
			}
			Polygon2_list temp_obs_list;
			Polygon2_list temp_obs_outer_list;
			int current_object_index = 0;
			Polygon_2 current;
			int obs_index = 0;
			auto delete_obs = robot2d.obstacles.begin();
			while (delete_obs != robot2d.obstacles.end()) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == obs_index) {
						is_delete = true;
						Polygon_2 *erase_region = *delete_obs;
						robot2d.obstacles.erase(delete_obs++);
						robot2d.updateHashMap(erase_region);
#ifdef BIG_ENV
						for (int i = 0; i < 10; i++) {
							if (i == 9) {
								//std::cout << "full temp_iteration:" << std::endl;
								rrts.full_temp_iteration(erase_region);
							}
							else {
								rrts.temp_iteration(erase_region);
							}
						}
#endif	
#ifdef SMALL_ENV
						for (int i = 0; i < 5; i++)
							rrts.temp_iteration(erase_region);
#endif
						break;
					}
			
				}
				if (!is_delete) {
					delete_obs++;
				}
				obs_index++;
			}
			//find out the original index of current object, based on original global obstaclePolyList
			for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == current_object_index) {
						is_delete = true;
						break;
					}
				}
				if (!is_delete) {
					temp_obs_list.push_back(*ob);
				}
				current_object_index++;
			}
			current_object_index = 0;
			for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == current_object_index) {
						is_delete = true;
						break;
					}
				}
				if (!is_delete) {
					temp_obs_outer_list.push_back(*ob);
				}
				current_object_index++;
			}


			std::map<int, graspDist> candidates =  getCandidateObjects(temp_obs_list, temp_obs_outer_list, *i, rrts);
			for (auto candi = candidates.begin(); candi != candidates.end(); candi++) {
				TreeNode* child = new TreeNode(candi->first, candi->second.shortestDist); 
				//std::cout << "obj " << candi->first << " ";
				(*i)->appendChild(child);
				temp_node_list.push_back(child);
				if (level == num_objs - 1) {
					last_level_list.push_back(child);
				}
			}
			rrts.temp_restore();
		}
		std::cout << std::endl;
		branch_list.clear();
		branch_list = temp_node_list;
		temp_node_list.clear();
		level++;
	}
	for (auto p = backup_obstacles.end(); p != backup_obstacles.end(); p++) {
	 	delete *p;
	 }
	//std::vector<std::vector<int>> pick_sequence = generateCombUsingTree(origin);
	std::cout << "total sequence num:" << last_level_list.size() << std::endl;
	int optimal_sequence_index = 0;
	double optimalDist = 100000000;
	for (int q = 0; q < last_level_list.size(); q++) {
		double current_dist = 0;
		current_dist += last_level_list[q]->distance;
		TreeNode* temp = last_level_list[q];
		while (temp->getParent() != NULL) {
			temp = temp->getParent();
			current_dist += temp->distance;
		}
		if (current_dist < optimalDist) {
			optimalDist = current_dist;
			optimal_sequence_index = q;
		}
	}
	std::cout << "optimal total dist:" << optimalDist << std::endl;
	TreeNode* optim = last_level_list[optimal_sequence_index];
	std::cout << "pick sequence:";
	auto it = picking_sequence.begin();
	while (optim->getParent() != NULL) {
		picking_sequence.push_back(optim->index_);
		std::cout << optim->index_ << ",";
		optim = optim->getParent();
	}
	std::reverse(std::begin(picking_sequence), std::end(picking_sequence));
	return optimalDist;
}

double Roadmap::declutterUsingMCTS(int num_objs) {
	std::vector<int> picking_sequence;
	System robot2d_4;
	planner_t rrts_4;
	robot2d_4.setNumDimensions(2);
	robot2d_4.regionOperating.setNumDimensions(2);
#ifdef BIG_ENV
	robot2d_4.regionOperating.center[0] = 2500.0;
	robot2d_4.regionOperating.center[1] = 2500.0;
	robot2d_4.regionOperating.size[0] = 5000.0;
	robot2d_4.regionOperating.size[1] = 5000.0;
#endif
#ifdef SMALL_ENV
	robot2d_4.regionOperating.center[0] = 500.0;
	robot2d_4.regionOperating.center[1] = 500.0;
	robot2d_4.regionOperating.size[0] = 1000.0;
	robot2d_4.regionOperating.size[1] = 1000.0;
#endif
	robot2d_4.regionGoal.setNumDimensions(2);
	robot2d_4.regionGoal.center[0] = 300000.0;
	robot2d_4.regionGoal.center[1] = 300000.0;
	robot2d_4.regionGoal.size[0] = 2.0;
	robot2d_4.regionGoal.size[1] = 2.0;


	Polygon2_list obs_outer_list = m_objectOuterPolyList;
	Polygon2_list obs_list = m_objectPolyList;
	Polygon2_list obstacle_outer_list = m_obstacleOuterPolyList;
	Polygon2_list obstacle_list = m_obstaclePolyList;
	// insert obstacles into system
	double min_x = 0;
	double max_x = 0;
	double min_y = 0;
	double max_y = 0;
	if (robot2d_4.obstacles.size() > 0) {
		for (auto i = robot2d_4.obstacles.begin(); i != robot2d_4.obstacles.end(); i++) {
			delete *i;
		}
	}
	robot2d_4.obstacles.clear();
	for (auto ob = obs_list.begin(); ob != obs_list.end(); ob++) {
		// region *obstacle;
		Polygon_2 * region = new Polygon_2;
		*region = *ob;
		// obstacle = new region;
		// obstacle->setNumDimensions(2);
		//convertPolygon2Region(*ob, obstacle->center[0], obstacle->center[1], obstacle->size[0], obstacle->size[1]);
		robot2d_4.obstacles.push_back(region);  // Add the obstacle to the list
	}
	
	robot2d_4.generateHashMap();
	// Add the system to the planner
	rrts_4.setSystem(robot2d_4);

	// Set up the root vertex
	vertex_t &root = rrts_4.getRootVertex();
	State &rootState = root.getState();
	rootState[0] = START_X;
	rootState[1] = START_Y;



	// Initialize the planner
	rrts_4.initialize();

	// This parameter should be larger than 1.5 for asymptotic 
	//   optimality. Larger values will weigh on optimization 
	//   rather than exploration in the RRT* algorithm. Lower 
	//   values, such as 0.1, should recover the RRT.
	rrts_4.setGamma(1.5);
	clock_t t1, t2;
	t1 = clock();
#ifdef BIG_ENV
	for (int i = 0; i < 10000; i++)
		rrts_4.iteration();

	//std::cout << rrts_4.debug_time_1 << "," << rrts_4.debug_time_2 << "," << rrts_4.debug_time_3 << std::endl;
#endif
#ifdef SMALL_ENV
	for (int i = 0; i < 2000; i++)
		rrts_4.iteration();
#endif
	t2 = clock();
	float diff((float)t2 - (float)t1);
	float seconds = diff / CLOCKS_PER_SEC;

	//region* erase_region = robot2d.obstacles.front();
	//robot2d.obstacles.pop_front();
	//for (int i = 0; i < 1000; i++)
	//	rrts.temp_iteration(*erase_region);

	std::cout << "done main iteration, use time " << seconds << std::endl;
	//rrts.temp_restore();
	std::vector<TreeNode*> branch_list;
	TreeNode* origin = new TreeNode(-1, 0.0);
	branch_list.push_back(origin);
	int level = 0;
	std::vector<TreeNode*> temp_node_list;
	std::vector<TreeNode*> last_level_list;
	std::list<Polygon_2*> backup_obstacles;
	backup_obstacles = robot2d_4.obstacles;
	std::map<std::pair<int, int>, std::set<Polygon_2*>> back_hashmap = robot2d_4.obs_hash_map;
	while (level < num_objs) {
		std::cout << "level " << level << std::endl;
		for (auto heu1 = branch_list.begin(); heu1 != branch_list.end(); heu1++) {
			std::vector<TreeNode*> all_parents = (*heu1)->getAllParents();
			//std::cout << (*heu1)->index_ << " , ";
			std::set<int> parents_index;
			double distance_till = 0;
			for (int j = 0; j < all_parents.size(); j++) {
				parents_index.insert(all_parents[j]->index_);
				distance_till += all_parents[j]->distance;
			}
			(*heu1)->distance_till_now = distance_till;
			(*heu1)->all_parents_index = parents_index;
		}
		std::cout << "level done" << std::endl;
		//std::cout << std::endl;
		std::vector<TreeNode*> selected_list;
		if (level > 1) {
			while (branch_list.size() > 0) {
				std::set<int> all_parents = branch_list[0]->all_parents_index;
				std::vector<int> same_parent_list;
				same_parent_list.push_back(0);
				for (int p = 0; p < branch_list.size(); p++) {
					if (p != 0) {
						if (all_parents == branch_list[p]->all_parents_index) {
							same_parent_list.push_back(p);
						}
					}
				}
				int optimal_index = 0;
				double optimal_dist = 10000000;
				for (int k = 0; k < same_parent_list.size(); k++) {
					if (branch_list[same_parent_list[k]]->distance_till_now < optimal_dist) {
						optimal_dist = branch_list[same_parent_list[k]]->distance_till_now;
						optimal_index = same_parent_list[k];
					}
				}
				selected_list.push_back(branch_list[optimal_index]);

				std::vector<TreeNode*> temp_keep_list;
				for (int p = 0; p < branch_list.size(); p++) {
					bool is_keep = true;
					for (int k = 0; k < same_parent_list.size(); k++) {
						if (same_parent_list[k] == p) {
							is_keep = false;
							break;
						}
					}
					if (is_keep) {
						temp_keep_list.push_back(branch_list[p]);
					}
				}
				branch_list.clear();
				branch_list = temp_keep_list;
			}
			branch_list = selected_list;
		}
		//std::cout << "after heu1 truncation" << std::endl;
		for (auto i = branch_list.begin(); i != branch_list.end(); i++) {
			robot2d_4.obstacles.clear();
			robot2d_4.obstacles = backup_obstacles;
			robot2d_4.obs_hash_map = back_hashmap;
			//std::cout << (*i)->index_ << " , " << std::endl;
			std::vector<TreeNode*> all_parents = (*i)->getAllParents();
			std::vector<int> parents_index;
			for (int j = 0; j < all_parents.size(); j++) {
				parents_index.push_back(all_parents[j]->index_);
			}
			Polygon2_list temp_obs_list;
			Polygon2_list temp_obs_outer_list;
			int current_object_index = 0;
			Polygon_2 current;
			int obs_index = 0;
			auto delete_obs = robot2d_4.obstacles.begin();
			while (delete_obs != robot2d_4.obstacles.end()) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == obs_index) {
						is_delete = true;
						Polygon_2* erase_region = *delete_obs;
						robot2d_4.obstacles.erase(delete_obs++);
						robot2d_4.updateHashMap(erase_region);
#ifdef BIG_ENV
						for (int i = 0; i < 10; i++) {
							if (i == 9) {
								rrts_4.full_temp_iteration(erase_region);
							}
							else {
								rrts_4.temp_iteration(erase_region);
							}

						}
#endif	
#ifdef SMALL_ENV
						for (int i = 0; i < 5; i++)
							rrts_4.temp_iteration(*erase_region);
#endif
						break;
					}

				}
				if (!is_delete) {
					delete_obs++;
				}
				obs_index++;
			}
			//find out the original index of current object, based on original global obstaclePolyList
			for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == current_object_index) {
						is_delete = true;
						break;
					}
				}
				if (!is_delete) {
					temp_obs_list.push_back(*ob);
				}
				current_object_index++;
			}
			current_object_index = 0;
			for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == current_object_index) {
						is_delete = true;
						break;
					}
				}
				if (!is_delete) {
					temp_obs_outer_list.push_back(*ob);
				}
				current_object_index++;
			}


			std::map<int, graspDist> candidates = getCandidateObjects(temp_obs_list, temp_obs_outer_list, *i, rrts_4);
			int next_index;
			graspDist next_grasp;
			double shortest_next_dist = 1000000000;
			if (candidates.size() > 1) {
				for (auto candi = candidates.begin(); candi != candidates.end(); candi++) {
					temp_obs_list.clear();
					temp_obs_outer_list.clear();
					current_object_index = 0;
					for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
						bool is_delete = false;
						for (int k = 0; k < parents_index.size(); k++) {
							if (parents_index[k] == current_object_index) {
								is_delete = true;
								break;
							}
						}
						if (candi->first == current_object_index) {
							is_delete = true;
						}
						if (!is_delete) {
							temp_obs_list.push_back(*ob);
						}
						current_object_index++;
					}
					current_object_index = 0;
					for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
						bool is_delete = false;
						for (int k = 0; k < parents_index.size(); k++) {
							if (parents_index[k] == current_object_index) {
								is_delete = true;
								break;
							}
						}
						if (candi->first == current_object_index) {
							is_delete = true;
						}
						if (!is_delete) {
							temp_obs_outer_list.push_back(*ob);
						}
						current_object_index++;
					}



					double greedy_left_dist = declutterUsingLocalGreedy(temp_obs_list, temp_obs_outer_list);
					if (greedy_left_dist + candi->second.shortestDist < shortest_next_dist) {
						shortest_next_dist = greedy_left_dist + candi->second.shortestDist;
						next_index = candi->first;
						next_grasp = candi->second;
					}

				}
			}
			else {
				auto first = candidates.begin();
				next_index = first->first;
				next_grasp = first->second;
			}
			TreeNode* child = new TreeNode(next_index, next_grasp.shortestDist);
			//std::cout << "obj " << candi->first << " ";
			(*i)->appendChild(child);
			temp_node_list.push_back(child);
			if (level == num_objs - 1) {
				last_level_list.push_back(child);
			}
			rrts_4.temp_restore();
		}
		std::cout << std::endl;
		branch_list.clear();
		branch_list = temp_node_list;
		temp_node_list.clear();
		level++;
	}
	//std::vector<std::vector<int>> pick_sequence = generateCombUsingTree(origin);
	std::cout << "total sequence num:" << last_level_list.size() << std::endl;
	int optimal_sequence_index = 0;
	double optimalDist = 100000000;
	for (int q = 0; q < last_level_list.size(); q++) {
		double current_dist = 0;
		current_dist += last_level_list[q]->distance;
		TreeNode* temp = last_level_list[q];
		while (temp->getParent() != NULL) {
			temp = temp->getParent();
			current_dist += temp->distance;
		}
		if (current_dist < optimalDist) {
			optimalDist = current_dist;
			optimal_sequence_index = q;
		}
	}
	for (auto p = backup_obstacles.end(); p != backup_obstacles.end(); p++) {
		delete *p;
	}
	std::cout << "optimal total dist:" << optimalDist << std::endl;
	TreeNode* optim = last_level_list[optimal_sequence_index];
	std::cout << "pick sequence:";
	auto it = picking_sequence.begin();
	while (optim->getParent() != NULL) {
		picking_sequence.push_back(optim->index_);
		//std::cout << optim->index_ << ",";
		optim = optim->getParent();
	}
	std::reverse(std::begin(picking_sequence), std::end(picking_sequence));
	return optimalDist;
}

double Roadmap::declutterUsingParticleGreedy(int num_objs) {
	std::vector<int> picking_sequence;
	System robot2d_3;
	planner_t rrts_3;
	//PRM prm_3;
	robot2d_3.setNumDimensions(2);
	robot2d_3.regionOperating.setNumDimensions(2);
#ifdef BIG_ENV
	robot2d_3.regionOperating.center[0] = 2500.0;
	robot2d_3.regionOperating.center[1] = 2500.0;
	robot2d_3.regionOperating.size[0] = 5000.0;
	robot2d_3.regionOperating.size[1] = 5000.0;
#endif
#ifdef SMALL_ENV
	robot2d_3.regionOperating.center[0] = 500.0;
	robot2d_3.regionOperating.center[1] = 500.0;
	robot2d_3.regionOperating.size[0] = 1000.0;
	robot2d_3.regionOperating.size[1] = 1000.0;
#endif
	robot2d_3.regionGoal.setNumDimensions(2);
	robot2d_3.regionGoal.center[0] = 300000.0;
	robot2d_3.regionGoal.center[1] = 300000.0;
	robot2d_3.regionGoal.size[0] = 2.0;
	robot2d_3.regionGoal.size[1] = 2.0;


	Polygon2_list obs_outer_list = m_objectOuterPolyList;
	Polygon2_list obs_list = m_objectPolyList;

	// insert obstacles into system
	double min_x = 0;
	double max_x = 0;
	double min_y = 0;
	double max_y = 0;
	
	robot2d_3.obstacles.clear();
	for(auto ob = obs_list.begin(); ob != obs_list.end(); ob++) {
		// region *obstacle;
		Polygon_2 * region = new Polygon_2;
		*region = *ob;
		// obstacle = new region;
		// obstacle->setNumDimensions(2);
		//convertPolygon2Region(*ob, obstacle->center[0], obstacle->center[1], obstacle->size[0], obstacle->size[1]);
		robot2d_3.obstacles.push_back(region);  // Add the obstacle to the list
	}

	
	robot2d_3.generateHashMap();
	// Add the system to the planner
	rrts_3.setSystem(robot2d_3);
	
	//prm_3.setSystem(robot2d_3);
	// Set up the root vertex
	vertex_t &root = rrts_3.getRootVertex();
	State &rootState = root.getState();
	rootState[0] = START_X;
	rootState[1] = START_Y;



	// Initialize the planner
	rrts_3.initialize();

	// This parameter should be larger than 1.5 for asymptotic 
	//   optimality. Larger values will weigh on optimization 
	//   rather than exploration in the RRT* algorithm. Lower 
	//   values, such as 0.1, should recover the RRT.
	rrts_3.setGamma(1.5);
#ifdef BIG_ENV
	for (int i = 0; i < 10000; i++)
		rrts_3.iteration();
	//prm_3.createGraph(8000, 5);
#endif
#ifdef SMALL_ENV
	for (int i = 0; i < 2000; i++)
		rrts_3.iteration();
#endif

	//region* erase_region = robot2d.obstacles.front();
	//robot2d.obstacles.pop_front();
	//for (int i = 0; i < 1000; i++)
	//	rrts.temp_iteration(*erase_region);


	//rrts.temp_restore();
	int particle_num = 5;
	std::vector<TreeNode*> branch_list;
	TreeNode* origin = new TreeNode(-1, 0.0);
	branch_list.push_back(origin);
	int level = 0;
	std::vector<TreeNode*> temp_node_list;
	std::vector<TreeNode*> last_level_list;
	std::list<Polygon_2*> backup_obstacles;
	backup_obstacles = robot2d_3.obstacles;
	std::map<std::pair<int, int>, std::set<Polygon_2*>> back_hashmap = robot2d_3.obs_hash_map;
	std::vector<TreeNode*> particle_list;
	while (level < num_objs) {
		particle_list.clear();
		std::cout << "level " << level << std::endl;
		for (auto heu1 = branch_list.begin(); heu1 != branch_list.end(); heu1++) {
			std::vector<TreeNode*> all_parents = (*heu1)->getAllParents();
			//std::cout << (*heu1)->index_ << " , ";
			std::set<int> parents_index;
			double distance_till = 0;
			for (int j = 0; j < all_parents.size(); j++) {
				parents_index.insert(all_parents[j]->index_);
				distance_till += all_parents[j]->distance;
			}
			(*heu1)->distance_till_now = distance_till;
			(*heu1)->all_parents_index = parents_index;

		}
		double current_lowest = 1000000000;
		if (branch_list.size() > particle_num) {
			TreeNode* least_node;
			int least_num = 0;
			int least_index = 0;
			std::vector<TreeNode*> temp_left_list;
			while (least_num < particle_num) {
				for (int h = 0; h < branch_list.size(); h++) {
					if (branch_list[h]->distance_till_now < current_lowest) {
						least_node = branch_list[h];
						current_lowest = branch_list[h]->distance_till_now;
						least_index = h;
					}
				}
				for (int h = 0; h < branch_list.size(); h++) {
					if (h != least_index) {
						temp_left_list.push_back(branch_list[h]);
					}
				}
				branch_list.clear();
				branch_list = temp_left_list;
				temp_left_list.clear();
				particle_list.push_back(least_node);
				least_num++;
			}
			branch_list.clear();
			branch_list = particle_list;
		}
		//std::cout << "after heu1 truncation" << std::endl;
		for (auto i = branch_list.begin(); i != branch_list.end(); i++) {
			robot2d_3.obstacles.clear();
			robot2d_3.obstacles = backup_obstacles;
			robot2d_3.obs_hash_map = back_hashmap;
			//std::cout << (*i)->index_ << " , ";
			std::vector<TreeNode*> all_parents = (*i)->getAllParents();
			std::vector<int> parents_index;
			for (int j = 0; j < all_parents.size(); j++) {
				parents_index.push_back(all_parents[j]->index_);
			}
			Polygon2_list temp_obs_list;
			Polygon2_list temp_obs_outer_list;
			int current_object_index = 0;
			Polygon_2 current;
			int obs_index = 0;
			auto delete_obs = robot2d_3.obstacles.begin();
			while (delete_obs != robot2d_3.obstacles.end()) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == obs_index) {
						is_delete = true;
						Polygon_2 *erase_region = *delete_obs;
						robot2d_3.obstacles.erase(delete_obs++);
						robot2d_3.updateHashMap(erase_region);
#ifdef BIG_ENV
						for (int i = 0; i < 10; i++) {
							if (i == 9) {
								//std::cout << "full temp_iteration:" << std::endl;
								rrts_3.full_temp_iteration(erase_region);
							}
							else {
								rrts_3.temp_iteration(erase_region);
							}
						}
							//prm_3.sampleInRegion(erase_region, 20, 10);
#endif
#ifdef SMALL_ENV
						for (int i = 0; i < 5; i++)
							rrts_3.temp_iteration(erase_region);
#endif
						break;
					}

				}
				if (!is_delete) {
					delete_obs++;
				}
				obs_index++;
			}
			//find out the original index of current object, based on original global obstaclePolyList
			for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == current_object_index) {
						is_delete = true;
						break;
					}
				}
				if (!is_delete) {
					temp_obs_list.push_back(*ob);
				}
				current_object_index++;
			}
			current_object_index = 0;
			for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
				bool is_delete = false;
				for (int k = 0; k < parents_index.size(); k++) {
					if (parents_index[k] == current_object_index) {
						is_delete = true;
						break;
					}
				}
				if (!is_delete) {
					temp_obs_outer_list.push_back(*ob);
				}
				current_object_index++;
			}


			std::map<int, graspDist> candidates = getAllObjects(temp_obs_list, temp_obs_outer_list, rrts_3);
			for (auto candi = candidates.begin(); candi != candidates.end(); candi++) {
				TreeNode* child = new TreeNode(candi->first, candi->second.shortestDist);
				//std::cout << "obj " << candi->first << " ";
				(*i)->appendChild(child);
				temp_node_list.push_back(child);
				if (level == num_objs - 1) {
					last_level_list.push_back(child);
				}
			}
			rrts_3.temp_restore();
			//prm_3.restore_temp();
		}
		std::cout << std::endl;
		branch_list.clear();
		branch_list = temp_node_list;
		temp_node_list.clear();
		level++;
	}
	//std::vector<std::vector<int>> pick_sequence = generateCombUsingTree(origin);
	//std::cout << "total sequence num:" << last_level_list.size() << std::endl;
	int optimal_sequence_index = 0;
	double optimalDist = 100000000;
	for (int q = 0; q < last_level_list.size(); q++) {
		double current_dist = 0;
		current_dist += last_level_list[q]->distance;
		TreeNode* temp = last_level_list[q];
		while (temp->getParent() != NULL) {
			temp = temp->getParent();
			current_dist += temp->distance;
		}
		if (current_dist < optimalDist) {
			optimalDist = current_dist;
			optimal_sequence_index = q;
		}
	}
	// for (auto p = backup_obstacles.end(); p != backup_obstacles.end(); p++) {
	// 	delete *p;
	// }
	std::cout << "optimal total dist:" << optimalDist << std::endl;
	TreeNode* optim = last_level_list[optimal_sequence_index];
	std::cout << "pick sequence:";
	auto it = picking_sequence.begin();
	while (optim->getParent() != NULL) {
		picking_sequence.push_back(optim->index_);
		std::cout << optim->index_ << ",";
		optim = optim->getParent();
	}
	std::reverse(std::begin(picking_sequence), std::end(picking_sequence));
	return optimalDist;
}



/*  
std::vector<std::vector<int>> Roadmap::generateCombUsingTree(TreeNode* origin) {
	std::vector<std::vector<int>> result;

}
*/
void Roadmap::buildRoadmap(Polygon2_list* pObsList, Polygon2_list* pObsInnerList,Polygon_2 *pBoundingRect,Polygon2_list* pEnvVoronoiList,double radius, QGraphicsScene& scene, Polygon2_list* pObsObstacleList, Polygon2_list* pObsObstacleInnerList){
	// Populate some internal variables for use across calls
	m_radius = radius;
	mm_scene = &scene;
	//m_edgeLength = radius/0.43;   // this is for hexgon case
    m_edgeLength = radius * 2.828;
	m_objectPolyList = *pObsInnerList;
	m_objectOuterPolyList = *pObsList;
	if (pObsObstacleList != NULL) {
		m_obstacleOuterPolyList = *pObsObstacleList;
	}
	if (pObsObstacleInnerList != NULL) {
		m_obstaclePolyList = *pObsObstacleInnerList;
	}
	m_pBoundingRect = pBoundingRect;
	m_polyVoronoiList = *pEnvVoronoiList;
	if(m_pVisibilityGraph != 0){
		delete m_pVisibilityGraph;
		m_pVisibilityGraph = 0;
	}
	if(m_pEnvironment != 0){
		delete m_pEnvironment;
		m_pEnvironment = 0;
	}

	Point_2 bottomLeft = (*m_pBoundingRect).outer()[0];
	Point_2 topRight = (*m_pBoundingRect).outer()[2];

	bottomLeftX = bottomLeft.get<0>();
	bottomLeftY = bottomLeft.get<1>();
	width = std::abs(topRight.get<0>() - bottomLeft.get<0>());
	height = std::abs(topRight.get<1>() - bottomLeft.get<1>());
	sqrt3 = std::sqrt(3.0);

	// Compute number of columns and rows
	n_w = (int)(ceil(width/(m_edgeLength*3/2))) + 3;
	n_h = (int)(ceil(height/(m_edgeLength*sqrt3))) + 3;

	// The lattice start x, y
	xs = bottomLeftX - (3/2)*m_edgeLength*1.35;
	ys = bottomLeftY - sqrt3*m_edgeLength*1.4;

	// Clean up from previous build
	m_pointList.clear();
	m_vidPointMap.clear();
	m_pointVidMap.clear();
	m_graph.clear();
	m_finalGraph.clear();
	m_vidFGMap.clear();
	m_vidGFMap.clear();
	m_edgeToBeRemovedSet.clear();
	m_vertexToBeRemovedSet.clear();
	m_boundaryBoundingCycle.clear();
	m_connectingPathMap.clear();
	m_gidGraphMap.clear();
	m_graphGidMap.clear();
	m_isGraphUsedMap.clear();
	for(std::vector<Graph*>::iterator git = m_obsBoundingCycleVec.begin(); git != m_obsBoundingCycleVec.end(); git++){
		delete (*git);
	}
	m_obsBoundingCycleVec.clear();

	findGraspablePoses();
	//buildVisibilityGraph();
	QFont font;
	font.setPointSizeF(5);
	font.setBold(false);
	//for (int i = 0; i < m_objectPolyList.size(); i++) {
	//	std::vector<Point_2> grasp_poses = m_graspCubeToBaseMap[i];
	//	for (int j = 0; j < grasp_poses.size(); j++) {
	//		std::vector<std::pair<double, double> > path;
			//std::cout << "grasp pose:" << grasp_poses[j].get<0>() << "," << grasp_poses[j].get<1>() << std::endl;
		//	double length = computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path);
			//m_graspToDistance[grasp_poses[j]] = length;
			//std::cout << "path: ";
			//for (int k = 0; k < path.size(); k++) {
			//	std::cout << "(" << path[k].first << "," << path[k].second << ")";
			//}
			//std::cout << std::endl;
			//QGraphicsSimpleTextItem *ti = mm_scene->addSimpleText(QString::number(length), font);
			//ti->setPos(grasp_poses[j].get<0>() + ROBOT_RADIUS/2, grasp_poses[j].get<1>() - ROBOT_RADIUS/2);
			//ti->setPen(QPen(QColor(Qt::green), 0.01*ROBOT_RADIUS, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
			//ti->setZValue(0.5);
	//	}
	//}
	/*
	// Build the roadmap, first obtain a lattice that cover the outer boundary
	buildHexgaonLattice();

	// Remove extra edges, at the same time, find smallest cycles enclosing the obstacles
	removeExcessEdges();

	// Preserve connectivity
	checkAndFixConnectivitget<1>();
*/
}



std::map<int, std::vector<Point_2>> Roadmap::new_findGraspablePoses(Polygon2_list obs_list, Polygon2_list obs_outer_list) {
	std::map<int, std::vector<Point_2>> graspCubeToBaseMap;
	std::map<int, std::vector<Point_2>> graspcenterMap;
	int current_object_index = 0;
	Polygon2_list obstacle_list = m_obstaclePolyList;
	Polygon2_list obstacle_outer_list = m_obstacleOuterPolyList;
	QPen vertexPen = QPen(Qt::blue, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	for (auto i = obs_list.begin(); i != obs_list.end(); i++) {
		current_object_index = 0;
		Polygon_2 current = *i;
		//find out the original index of current object, based on original global obstaclePolyList
		for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
			if (bg::equals(current, *ob)) {
				break;
			}
			current_object_index++;
		}
		std::vector<Point_2> grasp_center_pts;
		std::vector<Point_2> current_pts = current.outer();
		Point_2 pt_1 = current_pts[0];
		Point_2 pt_2 = current_pts[1];
		Point_2 pt_3 = current_pts[2];
		Point_2 pt_4 = current_pts[3];
		Point_2 center_pt;
		center_pt.set<0>((pt_1.get<0>() + pt_2.get<0>() + pt_3.get<0>() + pt_4.get<0>()) / 4);
		center_pt.set<1>((pt_1.get<1>() + pt_2.get<1>() + pt_3.get<1>() + pt_4.get<1>()) / 4);
		Point_2 pt_12;
		pt_12.set<0>((pt_1.get<0>() + pt_2.get<0>()) / 2);
		pt_12.set<1>((pt_1.get<1>() + pt_2.get<1>()) / 2);
		Point_2 pt_23;
		pt_23.set<0>((pt_3.get<0>() + pt_2.get<0>()) / 2);
		pt_23.set<1>((pt_3.get<1>() + pt_2.get<1>()) / 2);
		Point_2 pt_34;
		pt_34.set<0>((pt_3.get<0>() + pt_4.get<0>()) / 2);
		pt_34.set<1>((pt_3.get<1>() + pt_4.get<1>()) / 2);
		Point_2 pt_41;
		pt_41.set<0>((pt_1.get<0>() + pt_4.get<0>()) / 2);
		pt_41.set<1>((pt_1.get<1>() + pt_4.get<1>()) / 2);
		double primary_axis_x = 0;
		double primary_axis_y = 0;
		double second_axis_x = 0;
		double second_axis_y = 0;
		double primary_axis_length = 0;
		double second_axis_length = 0;
		double orientation = 0;
		if (bg::distance(pt_12, pt_34) > bg::distance(pt_23, pt_41)) {
			int center_num = bg::distance(pt_12, pt_34) / GRASP_CENTER_DIST;
			double dist_x = (pt_12.get<0>() - pt_34.get<0>()) / center_num;
			double dist_y = (pt_12.get<1>() - pt_34.get<1>()) / center_num;
			primary_axis_length = bg::distance(pt_12, pt_34);
			second_axis_length = bg::distance(pt_23, pt_41);
			//compute the unit vector for primary axis
			if (pt_12.get<0>() > pt_34.get<0>()) {
				primary_axis_x = (pt_12.get<0>() - pt_34.get<0>()) / primary_axis_length;
				primary_axis_y = (pt_12.get<1>() - pt_34.get<1>()) / primary_axis_length;

				orientation = atan2(pt_12.get<1>() - pt_34.get<1>(), pt_12.get<0>() - pt_34.get<0>());
			}
			else if (pt_12.get<0>() < pt_34.get<0>()) {
				primary_axis_x = (pt_34.get<0>() - pt_12.get<0>()) / primary_axis_length;
				primary_axis_y = (pt_34.get<1>() - pt_12.get<1>()) / primary_axis_length;
				orientation = atan2(pt_34.get<1>() - pt_12.get<1>(), pt_34.get<0>() - pt_12.get<0>());
			}
			else if (pt_12.get<0>() == pt_34.get<0>()) {
				primary_axis_x = 0;
				primary_axis_y = 1;
				orientation = 3.14159 / 2;
			}
			//compute the unit vector for secondary axis
			if (pt_23.get<1>() > pt_41.get<1>()) {
				second_axis_x = (pt_23.get<0>() - pt_41.get<0>()) / second_axis_length;
				second_axis_y = (pt_23.get<1>() - pt_41.get<1>()) / second_axis_length;
			}
			else if (pt_23.get<1>() < pt_41.get<1>()) {
				second_axis_x = (pt_41.get<0>() - pt_23.get<0>()) / second_axis_length;
				second_axis_y = (pt_41.get<1>() - pt_23.get<1>()) / second_axis_length;
			}
			else if (pt_23.get<1>() == pt_41.get<1>()) {
				second_axis_x = -1;
				second_axis_y = 0;
			}

			for (int j = 2; j < center_num - 3; j++) {
				Point_2 temp;

				temp.set<0>(pt_34.get<0>() + (j + 1)*dist_x);
				temp.set<1>(pt_34.get<1>() + (j + 1)*dist_y);
				grasp_center_pts.push_back(temp);
			}
		}
		else {
			int center_num = bg::distance(pt_23, pt_41) / GRASP_CENTER_DIST;
			double dist_x = (pt_23.get<0>() - pt_41.get<0>()) / center_num;
			double dist_y = (pt_23.get<1>() - pt_41.get<1>()) / center_num;
			primary_axis_length = bg::distance(pt_23, pt_41);
			second_axis_length = bg::distance(pt_12, pt_34);
			//compute the unit vector for primary axis
			if (pt_23.get<0>() > pt_41.get<0>()) {
				primary_axis_x = (pt_23.get<0>() - pt_41.get<0>()) / primary_axis_length;
				primary_axis_y = (pt_23.get<1>() - pt_41.get<1>()) / primary_axis_length;
				orientation = atan2(pt_23.get<1>() - pt_41.get<1>(), pt_23.get<0>() - pt_41.get<0>());
			}
			else if (pt_23.get<0>() < pt_41.get<0>()) {
				primary_axis_x = (pt_41.get<0>() - pt_23.get<0>()) / primary_axis_length;
				primary_axis_y = (pt_41.get<1>() - pt_23.get<1>()) / primary_axis_length;
				orientation = atan2(pt_41.get<1>() - pt_23.get<1>(), pt_41.get<0>() - pt_23.get<0>());
			}
			else if (pt_23.get<0>() == pt_41.get<0>()) {
				primary_axis_x = 0;
				primary_axis_y = 1;
				orientation = 3.14159 / 2;
			}
			//compute the unit vector for secondary axis
			if (pt_12.get<1>() > pt_34.get<1>()) {
				second_axis_x = (pt_12.get<0>() - pt_34.get<0>()) / second_axis_length;
				second_axis_y = (pt_12.get<1>() - pt_34.get<1>()) / second_axis_length;
			}
			else if (pt_12.get<1>() < pt_34.get<1>()) {
				second_axis_x = (pt_34.get<0>() - pt_12.get<0>()) / second_axis_length;
				second_axis_y = (pt_34.get<1>() - pt_12.get<1>()) / second_axis_length;
			}
			else if (pt_12.get<1>() == pt_34.get<1>()) {
				second_axis_x = -1;
				second_axis_y = 0;
			}
			for (int j = 2; j < center_num - 3; j++) {
				Point_2 temp;

				temp.set<0>(pt_41.get<0>() + (j + 1)*dist_x);
				temp.set<1>(pt_41.get<1>() + (j + 1)*dist_y);
				grasp_center_pts.push_back(temp);
			}
		}

		graspcenterMap[current_object_index] = grasp_center_pts;
		// iterate all the center points in a single cube object, for each center point, get all the potential robot base positions, 
		// and evaluate whether it is in collision
		std::vector<Point_2> graspRobotBase_list;
		for (int k = 0; k < grasp_center_pts.size(); k++) {
			// for each center point, the circle points around it are candidates, check collision, and whether there is enough space o
			// on the other end of the center point
			//mm_scene->addEllipse(grasp_center_pts[k].get<0>(), grasp_center_pts[k].get<1>(), 0.5, 0.5, vertexPen);
			double gripper_space_x_side_1 = 0;
			double gripper_space_y_side_1 = 0;
			double gripper_space_x_side_2 = 0;
			double gripper_space_y_side_2 = 0;
			//side_1 is against the direction of second_axis, side_2 is same with the direction of second axis
			gripper_space_x_side_1 = grasp_center_pts[k].get<0>() - (CUBE_WIDTH / 2 + GRIPPER_WIDTH / 2 + 1) * second_axis_x;
			gripper_space_y_side_1 = grasp_center_pts[k].get<1>() - (CUBE_WIDTH / 2 + GRIPPER_WIDTH / 2 + 1) * second_axis_y;
			gripper_space_x_side_2 = grasp_center_pts[k].get<0>() + (CUBE_WIDTH / 2 + GRIPPER_WIDTH / 2 + 1) * second_axis_x;
			gripper_space_y_side_2 = grasp_center_pts[k].get<1>() + (CUBE_WIDTH / 2 + GRIPPER_WIDTH / 2 + 1) * second_axis_y;
			Polygon_2 gripper_space_side_1, gripper_space_side_2;
			bg::append(gripper_space_side_1.outer(), Point_2(gripper_space_x_side_1 - GRIPPER_LENGTH / 2 * primary_axis_x - GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_1 - GRIPPER_LENGTH / 2 * primary_axis_y - GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_1.outer(), Point_2(gripper_space_x_side_1 - GRIPPER_LENGTH / 2 * primary_axis_x + GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_1 - GRIPPER_LENGTH / 2 * primary_axis_y + GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_1.outer(), Point_2(gripper_space_x_side_1 + GRIPPER_LENGTH / 2 * primary_axis_x + GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_1 + GRIPPER_LENGTH / 2 * primary_axis_y + GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_1.outer(), Point_2(gripper_space_x_side_1 + GRIPPER_LENGTH / 2 * primary_axis_x - GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_1 + GRIPPER_LENGTH / 2 * primary_axis_y - GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_1.outer(), Point_2(gripper_space_x_side_1 - GRIPPER_LENGTH / 2 * primary_axis_x - GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_1 - GRIPPER_LENGTH / 2 * primary_axis_y - GRIPPER_WIDTH / 2 * second_axis_y));
			bg::correct(gripper_space_side_1);

			bg::append(gripper_space_side_2.outer(), Point_2(gripper_space_x_side_2 - GRIPPER_LENGTH / 2 * primary_axis_x - GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_2 - GRIPPER_LENGTH / 2 * primary_axis_y - GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_2.outer(), Point_2(gripper_space_x_side_2 - GRIPPER_LENGTH / 2 * primary_axis_x + GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_2 - GRIPPER_LENGTH / 2 * primary_axis_y + GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_2.outer(), Point_2(gripper_space_x_side_2 + GRIPPER_LENGTH / 2 * primary_axis_x + GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_2 + GRIPPER_LENGTH / 2 * primary_axis_y + GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_2.outer(), Point_2(gripper_space_x_side_2 + GRIPPER_LENGTH / 2 * primary_axis_x - GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_2 + GRIPPER_LENGTH / 2 * primary_axis_y - GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_2.outer(), Point_2(gripper_space_x_side_2 - GRIPPER_LENGTH / 2 * primary_axis_x - GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_2 - GRIPPER_LENGTH / 2 * primary_axis_y - GRIPPER_WIDTH / 2 * second_axis_y));
			bg::correct(gripper_space_side_2);
			bool side_1_ok = true; // against the second axis
			bool side_2_ok = true; // along the second axis
			for (auto p = obs_list.begin(); p != obs_list.end(); p++) {
				if (p != i) {
					if (bg::intersects(*p, gripper_space_side_1)) {
						side_1_ok = false;
						break;
					}
				}
			}
			for (auto p = obstacle_list.begin(); p != obstacle_list.end(); p++) {
				if (p != i) {
					if (bg::intersects(*p, gripper_space_side_1)) {
						side_1_ok = false;
						break;
					}
				}
			}
			for (auto p = obs_list.begin(); p != obs_list.end(); p++) {
				if (p != i) {
					if (bg::intersects(*p, gripper_space_side_2)) {
						side_2_ok = false;
						break;
					}
				}
			}
			for (auto p = obstacle_list.begin(); p != obstacle_list.end(); p++) {
				if (p != i) {
					if (bg::intersects(*p, gripper_space_side_2)) {
						side_2_ok = false;
						break;
					}
				}
			}
			std::vector<Point_2> grasp_pose_pts = getCirclePoints(grasp_center_pts[k], PICK_DIST, orientation);
			bool validGraspPose = true;
			for (int q = 0; q < grasp_pose_pts.size(); q++) {
				validGraspPose = true;
				//mm_scene->addEllipse(grasp_pose_pts[q].get<0>(), grasp_pose_pts[q].get<1>(), 0.5, 0.5, vertexPen);
				double temp_x = grasp_pose_pts[q].get<0>() - grasp_center_pts[k].get<0>();
				double temp_y = grasp_pose_pts[q].get<1>() - grasp_center_pts[k].get<1>();
				double gripper_space_x = 0;
				double gripper_space_y = 0;
				/*
				if (temp_x*second_axis_x + temp_y*second_axis_y > 0) {
				if (!side_1_ok) {
				continue;
				}
				gripper_space_x = grasp_center_pts[k].get<0>() - (CUBE_WIDTH / 2 + GRIPPER_WIDTH/2+1) * second_axis_x;
				gripper_space_y = grasp_center_pts[k].get<1>() - (CUBE_WIDTH / 2 + GRIPPER_WIDTH/2+1) * second_axis_y;

				}
				else {
				if (!side_2_ok) {
				continue;
				}
				gripper_space_x = grasp_center_pts[k].get<0>() + (CUBE_WIDTH / 2 + GRIPPER_WIDTH/2+1) * second_axis_x;
				gripper_space_y = grasp_center_pts[k].get<1>() + (CUBE_WIDTH / 2 + GRIPPER_WIDTH/2+1) * second_axis_y;
				}
				*/
				if (!(side_1_ok && side_2_ok)) {
					continue;
				}
				for (auto x = obs_outer_list.begin(); x != obs_outer_list.end(); x++) {
					if (bg::within(grasp_pose_pts[q], *x)) {
						validGraspPose = false;
						break;
					}
				}
				for (auto x = obstacle_outer_list.begin(); x != obstacle_outer_list.end(); x++) {
					if (bg::within(grasp_pose_pts[q], *x)) {
						validGraspPose = false;
						break;
					}
				}
				if (validGraspPose) {
					graspRobotBase_list.push_back(grasp_pose_pts[q]);
				}
			}
		}
		//after examining a single object, graspRobotBase_list contains the valid grasp pose robot bases for current object 
		// with index i in m_objectPolyList
		graspCubeToBaseMap[current_object_index] = graspRobotBase_list;
		//current_object_index++;

		for (int o = 0; o < graspRobotBase_list.size(); o++) {
			//mm_scene->addEllipse(graspRobotBase_list[o].get<0>() - ROBOT_RADIUS, graspRobotBase_list[o].get<1>() - ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 2, vertexPen);

		}

	}
	return graspCubeToBaseMap;
}

void Roadmap::findGraspablePoses() {
	m_graspCubeToBaseMap.clear();
	int current_object_index = 0;
	QPen vertexPen = QPen(Qt::blue, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	for (auto i = m_objectPolyList.begin(); i != m_objectPolyList.end(); i++) {
		Polygon_2 current = *i;
		std::vector<Point_2> grasp_center_pts;
		std::vector<Point_2> current_pts = current.outer();
		Point_2 pt_1 = current_pts[0];
		Point_2 pt_2 = current_pts[1];
		Point_2 pt_3 = current_pts[2];
		Point_2 pt_4 = current_pts[3];
		Point_2 center_pt;
		center_pt.set<0>((pt_1.get<0>() + pt_2.get<0>() + pt_3.get<0>() + pt_4.get<0>())/4);
		center_pt.set<1>((pt_1.get<1>() + pt_2.get<1>() + pt_3.get<1>() + pt_4.get<1>()) / 4);
		Point_2 pt_12;
		pt_12.set<0>((pt_1.get<0>() + pt_2.get<0>())/2);
		pt_12.set<1>((pt_1.get<1>() + pt_2.get<1>()) / 2);
		Point_2 pt_23;
		pt_23.set<0>((pt_3.get<0>() + pt_2.get<0>()) / 2);
		pt_23.set<1>((pt_3.get<1>() + pt_2.get<1>()) / 2);
		Point_2 pt_34;
		pt_34.set<0>((pt_3.get<0>() + pt_4.get<0>()) / 2);
		pt_34.set<1>((pt_3.get<1>() + pt_4.get<1>()) / 2);
		Point_2 pt_41;
		pt_41.set<0>((pt_1.get<0>() + pt_4.get<0>()) / 2);
		pt_41.set<1>((pt_1.get<1>() + pt_4.get<1>()) / 2);
		double primary_axis_x = 0;
		double primary_axis_y = 0;
		double second_axis_x = 0;
		double second_axis_y = 0;
		double primary_axis_length = 0;
		double second_axis_length = 0;
		double orientation = 0;
		if (bg::distance(pt_12, pt_34) > bg::distance(pt_23, pt_41)) {
			int center_num = bg::distance(pt_12, pt_34) / GRASP_CENTER_DIST;
			double dist_x = (pt_12.get<0>() - pt_34.get<0>()) / center_num;
			double dist_y = (pt_12.get<1>() - pt_34.get<1>()) / center_num;
			primary_axis_length = bg::distance(pt_12, pt_34);
			second_axis_length = bg::distance(pt_23, pt_41);
			//compute the unit vector for primary axis
			if (pt_12.get<0>() > pt_34.get<0>()) {
				primary_axis_x = (pt_12.get<0>() - pt_34.get<0>()) / primary_axis_length;
				primary_axis_y = (pt_12.get<1>() - pt_34.get<1>()) / primary_axis_length;

				orientation = atan2(pt_12.get<1>() - pt_34.get<1>(), pt_12.get<0>() - pt_34.get<0>());
			}else if(pt_12.get<0>() < pt_34.get<0>()){
				primary_axis_x = (pt_34.get<0>() - pt_12.get<0>()) / primary_axis_length;
				primary_axis_y = (pt_34.get<1>() - pt_12.get<1>()) / primary_axis_length;
				orientation = atan2(pt_34.get<1>() - pt_12.get<1>(), pt_34.get<0>() - pt_12.get<0>());
			}
			else if (pt_12.get<0>() == pt_34.get<0>()) {
				primary_axis_x = 0;
				primary_axis_y = 1;
				orientation = 3.14159 / 2;
			}
			//compute the unit vector for secondary axis
			if (pt_23.get<1>() > pt_41.get<1>()) {
				second_axis_x = (pt_23.get<0>() - pt_41.get<0>()) / second_axis_length;
				second_axis_y = (pt_23.get<1>() - pt_41.get<1>()) / second_axis_length;
			}
			else if(pt_23.get<1>() < pt_41.get<1>()){
				second_axis_x = (pt_41.get<0>() - pt_23.get<0>()) / second_axis_length;
				second_axis_y = (pt_41.get<1>() - pt_23.get<1>()) / second_axis_length;
			}
			else if (pt_23.get<1>() == pt_41.get<1>()) {
				second_axis_x = -1;
				second_axis_y = 0;
			}
			
			for (int j = 2; j < center_num - 3; j++) {
				Point_2 temp;

				temp.set<0>(pt_34.get<0>() + (j+1)*dist_x);
				temp.set<1>(pt_34.get<1>() + (j + 1)*dist_y);
				grasp_center_pts.push_back(temp);
			}
		}
		else {
			int center_num = bg::distance(pt_23, pt_41) / GRASP_CENTER_DIST;
			double dist_x = (pt_23.get<0>() - pt_41.get<0>()) / center_num;
			double dist_y = (pt_23.get<1>() - pt_41.get<1>()) / center_num;
			primary_axis_length = bg::distance(pt_23, pt_41);
			second_axis_length = bg::distance(pt_12, pt_34);
			//compute the unit vector for primary axis
			if (pt_23.get<0>() > pt_41.get<0>()) {
				primary_axis_x = (pt_23.get<0>() - pt_41.get<0>()) / primary_axis_length;
				primary_axis_y = (pt_23.get<1>() - pt_41.get<1>()) / primary_axis_length;
				orientation = atan2(pt_23.get<1>() - pt_41.get<1>(), pt_23.get<0>() - pt_41.get<0>());
			}
			else if(pt_23.get<0>() < pt_41.get<0>()){
				primary_axis_x = (pt_41.get<0>() - pt_23.get<0>()) / primary_axis_length;
				primary_axis_y = (pt_41.get<1>() - pt_23.get<1>()) / primary_axis_length;
				orientation = atan2(pt_41.get<1>() - pt_23.get<1>(), pt_41.get<0>() - pt_23.get<0>());
			}
			else if (pt_23.get<0>() == pt_41.get<0>()) {
				primary_axis_x = 0;
				primary_axis_y = 1;
				orientation = 3.14159 / 2;
			}
			//compute the unit vector for secondary axis
			if (pt_12.get<1>() > pt_34.get<1>()) {
				second_axis_x = (pt_12.get<0>() - pt_34.get<0>()) / second_axis_length;
				second_axis_y = (pt_12.get<1>() - pt_34.get<1>()) / second_axis_length;
			}
			else if(pt_12.get<1>() < pt_34.get<1>()){
				second_axis_x = (pt_34.get<0>() - pt_12.get<0>()) / second_axis_length;
				second_axis_y = (pt_34.get<1>() - pt_12.get<1>()) / second_axis_length;
			}
			else if (pt_12.get<1>() == pt_34.get<1>()) {
				second_axis_x = -1;
				second_axis_y = 0;
			}
			for (int j = 2; j < center_num - 3; j++) {
				Point_2 temp;

				temp.set<0>(pt_41.get<0>() + (j + 1)*dist_x);
				temp.set<1>(pt_41.get<1>() + (j + 1)*dist_y);
				grasp_center_pts.push_back(temp);
			}
		}

		m_graspcenterMap[current_object_index] = grasp_center_pts;
		// iterate all the center points in a single cube object, for each center point, get all the potential robot base positions, 
		// and evaluate whether it is in collision
		std::vector<Point_2> graspRobotBase_list;
		for (int k = 0; k < grasp_center_pts.size(); k++) {
			// for each center point, the circle points around it are candidates, check collision, and whether there is enough space o
			// on the other end of the center point
			//mm_scene->addEllipsme(grasp_center_pts[k].get<0>(), grasp_center_pts[k].get<1>(), 0.5, 0.5, vertexPen);
			double gripper_space_x_side_1 = 0;
			double gripper_space_y_side_1 = 0;
			double gripper_space_x_side_2 = 0;
			double gripper_space_y_side_2 = 0;
			//side_1 is against the direction of second_axis, side_2 is same with the direction of second axis
			gripper_space_x_side_1 = grasp_center_pts[k].get<0>() - (CUBE_WIDTH / 2 + GRIPPER_WIDTH / 2 + 1) * second_axis_x;
			gripper_space_y_side_1 = grasp_center_pts[k].get<1>() - (CUBE_WIDTH / 2 + GRIPPER_WIDTH / 2 + 1) * second_axis_y;
			gripper_space_x_side_2 = grasp_center_pts[k].get<0>() + (CUBE_WIDTH / 2 + GRIPPER_WIDTH / 2 + 1) * second_axis_x;
			gripper_space_y_side_2 = grasp_center_pts[k].get<1>() + (CUBE_WIDTH / 2 + GRIPPER_WIDTH / 2 + 1) * second_axis_y;
			Polygon_2 gripper_space_side_1, gripper_space_side_2;
			bg::append(gripper_space_side_1.outer(), Point_2(gripper_space_x_side_1 - GRIPPER_LENGTH/2*primary_axis_x - GRIPPER_WIDTH/2*second_axis_x, gripper_space_y_side_1 - GRIPPER_LENGTH/2*primary_axis_y -  GRIPPER_WIDTH/2*second_axis_y ));
			bg::append(gripper_space_side_1.outer(), Point_2(gripper_space_x_side_1 - GRIPPER_LENGTH / 2 * primary_axis_x + GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_1 - GRIPPER_LENGTH / 2 * primary_axis_y + GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_1.outer(), Point_2(gripper_space_x_side_1 + GRIPPER_LENGTH / 2 * primary_axis_x + GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_1 + GRIPPER_LENGTH / 2 * primary_axis_y + GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_1.outer(), Point_2(gripper_space_x_side_1 + GRIPPER_LENGTH / 2 * primary_axis_x - GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_1 + GRIPPER_LENGTH / 2 * primary_axis_y - GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_1.outer(), Point_2(gripper_space_x_side_1 - GRIPPER_LENGTH / 2 * primary_axis_x - GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_1 - GRIPPER_LENGTH / 2 * primary_axis_y - GRIPPER_WIDTH / 2 * second_axis_y));
			bg::correct(gripper_space_side_1);

			bg::append(gripper_space_side_2.outer(), Point_2(gripper_space_x_side_2 - GRIPPER_LENGTH / 2 * primary_axis_x - GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_2 - GRIPPER_LENGTH / 2 * primary_axis_y - GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_2.outer(), Point_2(gripper_space_x_side_2 - GRIPPER_LENGTH / 2 * primary_axis_x + GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_2 - GRIPPER_LENGTH / 2 * primary_axis_y + GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_2.outer(), Point_2(gripper_space_x_side_2 + GRIPPER_LENGTH / 2 * primary_axis_x + GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_2 + GRIPPER_LENGTH / 2 * primary_axis_y + GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_2.outer(), Point_2(gripper_space_x_side_2 + GRIPPER_LENGTH / 2 * primary_axis_x - GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_2 + GRIPPER_LENGTH / 2 * primary_axis_y - GRIPPER_WIDTH / 2 * second_axis_y));
			bg::append(gripper_space_side_2.outer(), Point_2(gripper_space_x_side_2 - GRIPPER_LENGTH / 2 * primary_axis_x - GRIPPER_WIDTH / 2 * second_axis_x, gripper_space_y_side_2 - GRIPPER_LENGTH / 2 * primary_axis_y - GRIPPER_WIDTH / 2 * second_axis_y));
			bg::correct(gripper_space_side_2);
			bool side_1_ok = true; // against the second axis
			bool side_2_ok = true; // along the second axis
			for (auto p = m_objectPolyList.begin(); p != m_objectPolyList.end(); p++) {
				if (p != i) {
					if (bg::intersects(*p, gripper_space_side_1)) {
						side_1_ok = false;
						break;
					}
				}
			}
			for (auto p = m_objectPolyList.begin(); p != m_objectPolyList.end(); p++) {
				if (p != i) {
					if (bg::intersects(*p, gripper_space_side_2)) {
						side_2_ok = false;
						break;
					}
				}
			}
			std::vector<Point_2> grasp_pose_pts = getCirclePoints( grasp_center_pts[k],PICK_DIST, orientation);
			bool validGraspPose = true;
			for (int q = 0; q < grasp_pose_pts.size(); q++) {
				validGraspPose = true;
				mm_scene->addEllipse(grasp_pose_pts[q].get<0>(), grasp_pose_pts[q].get<1>(), 0.5, 0.5, vertexPen);
				double temp_x = grasp_pose_pts[q].get<0>() - grasp_center_pts[k].get<0>();
				double temp_y = grasp_pose_pts[q].get<1>() - grasp_center_pts[k].get<1>();
				double gripper_space_x = 0;
				double gripper_space_y = 0;
				/*  
				if (temp_x*second_axis_x + temp_y*second_axis_y > 0) {
					if (!side_1_ok) {
						continue;
					}
					gripper_space_x = grasp_center_pts[k].get<0>() - (CUBE_WIDTH / 2 + GRIPPER_WIDTH/2+1) * second_axis_x;
					gripper_space_y = grasp_center_pts[k].get<1>() - (CUBE_WIDTH / 2 + GRIPPER_WIDTH/2+1) * second_axis_y;

				}
				else {
					if (!side_2_ok) {
						continue;
					}
					gripper_space_x = grasp_center_pts[k].get<0>() + (CUBE_WIDTH / 2 + GRIPPER_WIDTH/2+1) * second_axis_x;
					gripper_space_y = grasp_center_pts[k].get<1>() + (CUBE_WIDTH / 2 + GRIPPER_WIDTH/2+1) * second_axis_y;
				}
				*/
				if (!(side_1_ok && side_2_ok)) {
					continue;
				}
				for (auto x = m_objectOuterPolyList.begin(); x != m_objectOuterPolyList.end(); x++) {
					if (bg::within(grasp_pose_pts[q], *x)) {
						validGraspPose = false;
						break;
					}
				}
				if (validGraspPose) {
					graspRobotBase_list.push_back(grasp_pose_pts[q]);
				}
			}
		}
		//after examining a single object, graspRobotBase_list contains the valid grasp pose robot bases for current object 
		// with index i in m_objectPolyList
		m_graspCubeToBaseMap[current_object_index] = graspRobotBase_list;
		current_object_index++;
		
		for (int o = 0; o < graspRobotBase_list.size(); o++) {
			mm_scene->addEllipse(graspRobotBase_list[o].get<0>() - ROBOT_RADIUS, graspRobotBase_list[o].get<1>() - ROBOT_RADIUS, ROBOT_RADIUS*2 , ROBOT_RADIUS*2, vertexPen);
			
		}

	}

}


/* 
void Roadmap::declutterUsingTree() {
	std::vector<std::vector<int>> pick_sequence = generateComb(m_objectPolyList.size());
	std::vector<double> pick_dist;
	for (int i = 0; i < pick_sequence.size(); i++) {
		double dist = declutterUsingSequence(pick_sequence[i]);
		std::cout << "total pick dist: " << dist << std::endl;
		pick_dist.push_back(dist);
	}
	auto result = std::min_element(pick_dist.begin(), pick_dist.end());
	std::cout << "optimal dist:" << *result << std::endl;
	std::vector<int> optimal_seq = pick_sequence[std::distance(pick_dist.begin(), result)];
	for (int j = 0; j < optimal_seq.size(); j++) {
		std::cout << optimal_seq[j] << ",";
	}
	std::cout << std::endl;
}

double Roadmap::declutterUsingSequence(std::vector<int> seq) {
	Polygon2_list obs_outer_list = m_objectOuterPolyList;
	Polygon2_list obs_list = m_objectPolyList;
	double nearest_dist = 100000;
	double total_dist = 0;
	Point_2 closest_grasp;
	std::vector<int> prev_index;
	Polygon2_list::iterator obs_iterator;
	Polygon2_list::iterator obs_outer_iterator;
	obs_iterator = obs_list.begin();
	obs_outer_iterator = obs_outer_list.begin();
	int last_index = 1000;
	for (int p = 0; p < seq.size(); p++) {
		std::cout << seq[p] << ",";
	}
	std::cout << std::endl;
	for (int i = 0; i < seq.size(); i++) {
		obs_iterator = obs_list.begin();
		obs_outer_iterator = obs_outer_list.begin();
		nearest_dist = 100000;
		int obj_index = seq[i];
		Environment  *env;
		Visibility_Graph *v_graph;
		new_buildVisibilityGraph(obs_outer_list, env, v_graph);
		std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(obs_list, obs_outer_list);
		std::vector<Point_2> grasp_poses;
		
		std::cout << "current one:" << seq[i] << std::endl;
		grasp_poses = graspCubeToBaseMap[obj_index];
		if (grasp_poses.size() == 0) {
			std::cout << "no valid sequence, no grasp poses available" << std::endl;
			return 100000;
		}
		for (int j = 0; j < grasp_poses.size(); j++) {
			std::list<Point_2 > path;
			double temp_dist = visi_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, env, v_graph);
			if (temp_dist < nearest_dist) {
				nearest_dist = temp_dist;
				closest_grasp = grasp_poses[j];
			}
		}
		total_dist += nearest_dist;
		int k = 0;
		while (k++ < obj_index) {
			obs_iterator++;
			obs_outer_iterator++;
		}
		
		for (int q = 0; q < prev_index.size(); q++) {
			if (prev_index[q] < obj_index) {
				obs_iterator--;
				obs_outer_iterator--;
			}
		}
			
		
		prev_index.push_back(obj_index);
		obs_list.erase(obs_iterator);
		obs_outer_list.erase(obs_outer_iterator);
		last_index = obj_index;
	}
	return total_dist;
}
*/
// return true:  not in same cluster
// return false: in same cluster
bool Roadmap::checkNotInSameCluster(Polygon_2 collision_obj, Polygon2_list obs_outer_list, Polygon_2 target_obj) {
	std::list<Polygon_2> explore_list;
	std::list<Polygon_2> explored_list;
	explore_list.push_back(target_obj);
	explored_list.push_back(target_obj);
	int loop_num = obs_outer_list.size();
	bool is_connected = false;
	while (loop_num > 0) {
		for (auto i = obs_outer_list.begin(); i != obs_outer_list.end(); i++) {
			bool is_explored = false;
			for (auto explored = explored_list.begin(); explored != explored_list.end(); explored++) {
				if (bg::equals(*explored, *i)) {
					is_explored = true;
					break;
				}
			}

			if (is_explored) {
				continue;
			}
			for (auto j = explore_list.begin(); j != explore_list.end(); j++) {
				if (bg::intersects(*i, *j)) {
					if (bg::equals(*i, collision_obj)) {
						is_connected = true;
						break;
					}
					explore_list.push_back(*i);
					explored_list.push_back(*i);
					break;
				}
			}
			if (is_connected) {
				break;
			}
		}
		if (is_connected) {
			break;
		}
		loop_num--;
	}
	return !is_connected;
}

Point_2 Roadmap::getShortestDistGrasp(std::vector<Point_2> grasp_poses, Polygon2_list obs_outer_list, double & dist, planner_t& planner) {
	Environment  *env;
	Visibility_Graph *v_graph;
	double nearest_dist = 10000000;
	Point_2 closest_grasp;
	//new_buildVisibilityGraph(obs_outer_list, env, v_graph);
	for (int j = 0; j < grasp_poses.size(); j++) {
		std::list<Point_2>  path;


		double temp_dist = new_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, planner);
		if (temp_dist < nearest_dist) {
			nearest_dist = temp_dist;
			closest_grasp = grasp_poses[j];
			dist = nearest_dist;
		}
	}
	return closest_grasp;
}


// Point_2 Roadmap::getShortestDistGrasp(std::vector<Point_2> grasp_poses, Polygon2_list obs_outer_list, double & dist, PRM& planner) {
// 	Environment  *env;
// 	Visibility_Graph *v_graph;
// 	double nearest_dist = 10000000;
// 	Point_2 closest_grasp;
// 	//new_buildVisibilityGraph(obs_outer_list, env, v_graph);
// 	for (int j = 0; j < grasp_poses.size(); j++) {
// 		std::list<Point_2>  path;


// 		double temp_dist = new_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, planner);
// 		if (temp_dist < nearest_dist) {
// 			nearest_dist = temp_dist;
// 			closest_grasp = grasp_poses[j];
// 			dist = nearest_dist;
// 		}
// 	}
// 	return closest_grasp;
// }

Point_2 Roadmap::getShortestDistGrasp(std::vector<Point_2> grasp_poses, Polygon2_list obs_outer_list, double & dist) {
	Environment  *env;
	Visibility_Graph *v_graph;
	double nearest_dist = 10000000;
	Point_2 closest_grasp;
	//new_buildVisibilityGraph(obs_outer_list, env, v_graph);
	for (int j = 0; j < grasp_poses.size(); j++) {
		std::vector<std::pair<double, double> > path;


		double temp_dist = new_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, env, v_graph);
		if (temp_dist < nearest_dist) {
			nearest_dist = temp_dist;
			closest_grasp = grasp_poses[j];
			dist = nearest_dist;
		}
	}
	return closest_grasp;
}
  
std::vector<Polygon_2> Roadmap::checkLineCollision(Linestring_2 shortest_line, Polygon_2 target_obj, Polygon2_list obs_outer_list) {
	std::vector<Polygon_2> collision_objs;
	for (auto i = obs_outer_list.begin(); i != obs_outer_list.end(); i++) { 
		if (bg::equals(target_obj, *i)) {
			continue;
		}
		if (bg::intersects(*i, shortest_line)) {
			collision_objs.push_back(*i);
		}
	}
	return collision_objs;
}

std::map<int, graspDist> Roadmap::getAllObjects(Polygon2_list obs_list, Polygon2_list obs_outer_list , planner_t& planner) {
	std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(obs_list, obs_outer_list);
	std::map<int, graspDist> result;
	for (auto obj = graspCubeToBaseMap.begin(); obj != graspCubeToBaseMap.end(); obj++) {
		if (obj->second.size() == 0) {
			continue;
		}
		graspDist shortest_result;
		Point_2 shortest_grasp = getShortestDistGrasp(obj->second, obs_outer_list, shortest_result.shortestDist, planner);
		shortest_result.grasp = shortest_grasp;
		result[obj->first] = shortest_result;
	}
	return result;
}

std::map<int, graspDist> Roadmap::getCandidateObjectsFromExit(Polygon2_list obs_list, Polygon2_list obs_outer_list, std::pair<double, double> exit_start, planner_t& planner) {
	std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(obs_list, obs_outer_list);
	std::vector<Point_2> grasp_poses;
	std::map<int, graspDist> result;
	//std::cout << "getCandidates" << std::endl;
	//std::cout << "obs_list size:" << obs_list.size() << std::endl;
	for (auto obj = graspCubeToBaseMap.begin(); obj != graspCubeToBaseMap.end(); obj++) {
		// if current object has no availabe grasp poses, then continue to next object
		if (obj->second.size() == 0) {
			continue;
		}
		//std::cout << "obj " << obj->first << std::endl;
		Polygon_2 target_obj;
		Polygon_2 target_obj_inner;
		int target_index = 0;
		for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
			if (target_index == obj->first) {
				target_obj = *ob;
				break;
			}
			target_index++;
		}
		target_index = 0;
		for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
			if (target_index == obj->first) {
				target_obj_inner = *ob;
				break;
			}
			target_index++;
		}

		graspDist shortest_result;
		graspDist realshortest_result;
		Polygon2_list target_list, target_inner_list;
		target_list.push_back(target_obj);
		target_inner_list.push_back(target_obj_inner);
		std::map<int, std::vector<Point_2>> target_grasps = new_findGraspablePoses(target_inner_list, target_list);
		double shortestDist = 1000000;
		Point_2 real_shortest_pt;
		for (int j = 0; j < target_grasps[target_index].size(); j++) {
			if (bg::distance(Point_2(exit_start.first, exit_start.second), target_grasps[target_index][j]) < shortestDist) {
				shortestDist = bg::distance(Point_2(exit_start.first, exit_start.second), target_grasps[target_index][j]);
				real_shortest_pt = target_grasps[target_index][j];
				realshortest_result.shortestDist = shortestDist;

			}
		}
		bool real_rrt_shortest_valid = false;
		for (int p = 0; p < obj->second.size(); p++) {
			if (bg::equals(real_shortest_pt, obj->second[p])) {
				real_rrt_shortest_valid = true;
				break;
			}
		}
		Point_2 shortest_grasp = getShortestDistGrasp(obj->second, obs_outer_list, shortest_result.shortestDist, planner);

		//std::cout << "rrt shortest_grasp:" << shortest_grasp.get<0>() << "," << shortest_grasp.get<1>() << std::endl;
		//std::cout << "real shortest_grasp:" << real_shortest_pt.get<0>() << "," << real_shortest_pt.get<1>() << std::endl;
		shortest_result.grasp = shortest_grasp;
		realshortest_result.grasp = real_shortest_pt;
		Linestring_2 shortest_linestring;
		bg::append(shortest_linestring, shortest_grasp);
		bg::append(shortest_linestring, Point_2(exit_start.first, exit_start.second));
		Segment_2 shortest_line(shortest_grasp, Point_2(exit_start.first, exit_start.second));
		std::vector<Polygon_2> collision_objs = checkLineCollision(shortest_linestring, target_obj, obs_outer_list);
		Linestring_2 real_shortest_linestring;
		bg::append(real_shortest_linestring, real_shortest_pt);
		bg::append(real_shortest_linestring, Point_2(exit_start.first, exit_start.second));
		std::vector<Polygon_2> real_collision_objs = checkLineCollision(real_shortest_linestring, target_obj, obs_outer_list);
		if ((real_collision_objs.size() == 0) && real_rrt_shortest_valid) {

			if (std::abs(exit_start.first - 0) < 0.0001 && std::abs(exit_start.second - 2500) < 0.0001) {
				if (bg::distance(real_shortest_pt, Point_2(exit_start.first, exit_start.second)) < bg::distance(real_shortest_pt, Point_2(5000, 2500))) {
					result.clear();
					result[obj->first] = realshortest_result;
					//std::cout << "fastshortcut:" << obj->first << std::endl;
					break;
				}
			}
			else {
				if (bg::distance(real_shortest_pt, Point_2(exit_start.first, exit_start.second)) < bg::distance(real_shortest_pt, Point_2(0, 2500))) {
					result.clear();
					result[obj->first] = realshortest_result;
					//std::cout << "fastshortcut:" << obj->first << std::endl;
					break;
				}
			}
			//if (bg::distance(real_shortest_pt, Point_2(exit_start.first, exit_start.second)) < bg::distance(real_shortest_pt, Point_2(0, 2500 )) && bg::distance(real_shortest_pt, Point_2(exit_start.first, exit_start.second)) < bg::distance(real_shortest_pt, Point_2(2500, 0)) && bg::distance(real_shortest_pt, Point_2(exit_start.first, exit_start.second)) < bg::distance(real_shortest_pt, Point_2(5000, 2500))) {
				//result.clear();
				//result[obj->first] = realshortest_result;
				//std::cout << "fastshortcut:" << obj->first << std::endl;
				//break;
			//}
		}

		if (collision_objs.size() == 0) {

			result[obj->first] = shortest_result;
			//if (bg::equals(shortest_grasp, real_shortest_pt)) {

				//std::cout << "shortcut:" << obj->first << std::endl;
				//break;
			//}
			//std::cout << "collision_objs num is 0" << std::endl;
			continue;
			//break;
		}
		else
		{
			//std::cout << "in collision" <<obj->first<< std::endl;
			bool is_valid = true;
			for (int j = 0; j < collision_objs.size(); j++) {
				if (checkNotInSameCluster(collision_objs[j], obs_outer_list, target_obj)) {
					//std::cout << "not in same cluster" << std::endl;
					is_valid = false;
					break;
				}
			}
			if (is_valid)
			{
				result[obj->first] = shortest_result;
			}
		}


	}
	return result;
	/*
	if (obj_index > last_index) {
	grasp_poses = graspCubeToBaseMap[obj_index-1];
	}
	else {
	grasp_poses = graspCubeToBaseMap[obj_index];
	}
	*/
	/*
	std::cout << "current one:" << seq[i] << std::endl;
	grasp_poses = graspCubeToBaseMap[obj_index];
	if (grasp_poses.size() == 0) {
	std::cout << "no valid sequence, no grasp poses available" << std::endl;
	return 100000;
	}
	for (int j = 0; j < grasp_poses.size(); j++) {
	std::vector<std::pair<double, double> > path;
	double temp_dist = new_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, env, v_graph);
	if (temp_dist < nearest_dist) {
	nearest_dist = temp_dist;
	closest_grasp = grasp_poses[j];
	}
	}
	*/
}

std::map<int, graspDist> Roadmap::getCandidateObjects(Polygon2_list obs_list, Polygon2_list obs_outer_list, TreeNode* parent_node, planner_t& planner) {
	std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(obs_list, obs_outer_list);
	std::vector<Point_2> grasp_poses;
	std::map<int, graspDist> result;
	bool use_cluster_heu = false;
	if (parent_node->getParent() != NULL) {
		if (parent_node->getParent()->childrenNumber() > 1) {
			use_cluster_heu = true;
		}
	}
	int parent_index = 0;
	Polygon_2 parent_poly;
	for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
		if (parent_index == parent_node->index_) {
			parent_poly = *ob;
			break;
		}
		parent_index++;
	}
	//std::cout << "getCandidates" << std::endl;
	//std::cout << "obs_list size:" << obs_list.size() << std::endl;
	for (auto obj = graspCubeToBaseMap.begin(); obj != graspCubeToBaseMap.end(); obj++) {
		// if current object has no availabe grasp poses, then continue to next object
		if (obj->second.size() == 0) {
			continue;
		}
		//std::cout << "obj " << obj->first << std::endl;
		Polygon_2 target_obj;
		Polygon_2 target_obj_inner;
		int target_index = 0;
		for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
			if (target_index == obj->first) {
				target_obj = *ob;
				break;
			}
			target_index++;
		}
		target_index = 0;
		for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
			if (target_index == obj->first) {
				target_obj_inner = *ob;
				break;
			}
			target_index++;
		}

		graspDist shortest_result;
		graspDist realshortest_result;
		Polygon2_list target_list, target_inner_list;
		target_list.push_back(target_obj);
		target_inner_list.push_back(target_obj_inner);
		std::map<int, std::vector<Point_2>> target_grasps = new_findGraspablePoses(target_inner_list, target_list);
		double shortestDist = 1000000;
		Point_2 real_shortest_pt;
#ifndef HAVE_OBS
		for (int j = 0; j < target_grasps[target_index].size(); j++) {
			if (bg::distance(Point_2(START_X, START_Y), target_grasps[target_index][j]) < shortestDist) {
				shortestDist = bg::distance(Point_2(START_X, START_Y), target_grasps[target_index][j]);
				//std::list<Point_2>  path;
				//shortestDist = new_computeShortestPath(START_X, START_Y, target_grasps[target_index][j].get<0>(), target_grasps[target_index][j].get<1>(), path, planner);
				real_shortest_pt = target_grasps[target_index][j];
				realshortest_result.shortestDist = shortestDist;

			}
		}
#endif
#ifdef HAVE_OBS

		for (int j = 0; j < target_grasps[target_index].size(); j++) {
			std::list<Point_2>  path;
			if (bg::distance(Point_2(START_X, START_Y), target_grasps[target_index][j]) < shortestDist) {
				shortestDist = bg::distance(Point_2(START_X, START_Y), target_grasps[target_index][j]);
				//std::list<Point_2>  path;
				//shortestDist = new_computeShortestPath(START_X, START_Y, target_grasps[target_index][j].get<0>(), target_grasps[target_index][j].get<1>(), path, planner);
				real_shortest_pt = target_grasps[target_index][j];
				realshortest_result.shortestDist = shortestDist;

			}
		}
#endif
		bool real_rrt_shortest_valid = false;
		for (int p = 0; p < obj->second.size(); p++) {
			if (bg::equals(real_shortest_pt, obj->second[p])) {
				real_rrt_shortest_valid = true;
				break;
			}
		}
		Point_2 shortest_grasp = getShortestDistGrasp(obj->second, obs_outer_list, shortest_result.shortestDist, planner);

		//std::cout << "rrt shortest_grasp:" << shortest_grasp.get<0>() << "," << shortest_grasp.get<1>() << std::endl;
		//std::cout << "real shortest_grasp:" << real_shortest_pt.get<0>() << "," << real_shortest_pt.get<1>() << std::endl;
		shortest_result.grasp = shortest_grasp;
		realshortest_result.grasp = real_shortest_pt;
		Linestring_2 shortest_linestring;
		bg::append(shortest_linestring, shortest_grasp);
		bg::append(shortest_linestring, Point_2(START_X, START_Y));
		Segment_2 shortest_line(shortest_grasp, Point_2(START_X, START_Y));
		std::vector<Polygon_2> collision_objs = checkLineCollision(shortest_linestring, target_obj, obs_outer_list);
#ifndef HAVE_OBS
		Linestring_2 real_shortest_linestring;
		bg::append(real_shortest_linestring, real_shortest_pt);
		bg::append(real_shortest_linestring, Point_2(START_X, START_Y));
		std::vector<Polygon_2> real_collision_objs = checkLineCollision(real_shortest_linestring, target_obj, obs_outer_list);
		if ((real_collision_objs.size() == 0) && real_rrt_shortest_valid) {
			result.clear();
			result[obj->first] = realshortest_result;
			//std::cout << "fastshortcut:" << obj->first << std::endl;
			break;
		}
#endif
		if (collision_objs.size() == 0) {

			result[obj->first] = shortest_result;
			if (bg::equals(shortest_grasp, real_shortest_pt)) {

				std::cout << "shortcut:" << obj->first << std::endl;
				break;
			}
			//std::cout << "collision_objs num is 0" << std::endl;
			continue;
			//break;
		}
		else
		{
			//std::cout << "in collision" <<obj->first<< std::endl;
			bool is_valid = true;
			for (int j = 0; j < collision_objs.size(); j++) {
				if (checkNotInSameCluster(collision_objs[j], obs_outer_list, target_obj)) {
					//std::cout << "not in same cluster" << std::endl;
					is_valid = false;
					break;
				}
			}
			if (is_valid)
			{
				//	if (use_cluster_heu) {
				//		if (bg::distance(target_obj_inner, parent_poly) > 700) {
				//			continue;
				//		}
				//	}
				result[obj->first] = shortest_result;
			}
		}


	}
	return result;
	/*
	if (obj_index > last_index) {
	grasp_poses = graspCubeToBaseMap[obj_index-1];
	}
	else {
	grasp_poses = graspCubeToBaseMap[obj_index];
	}
	*/
	/*
	std::cout << "current one:" << seq[i] << std::endl;
	grasp_poses = graspCubeToBaseMap[obj_index];
	if (grasp_poses.size() == 0) {
	std::cout << "no valid sequence, no grasp poses available" << std::endl;
	return 100000;
	}
	for (int j = 0; j < grasp_poses.size(); j++) {
	std::vector<std::pair<double, double> > path;
	double temp_dist = new_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, env, v_graph);
	if (temp_dist < nearest_dist) {
	nearest_dist = temp_dist;
	closest_grasp = grasp_poses[j];
	}
	}
	*/
}

// std::map<int, graspDist> Roadmap::getCandidateObjects(Polygon2_list obs_list, Polygon2_list obs_outer_list, TreeNode* parent_node, PRM& planner) {
// 	std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(obs_list, obs_outer_list);
// 	std::vector<Point_2> grasp_poses;
// 	std::map<int, graspDist> result;
// 	bool use_cluster_heu = false;
// 	if (parent_node->getParent() != NULL){
// 	if (parent_node->getParent()->childrenNumber() > 1) {
// 		use_cluster_heu = true;
// 	}
// 	}
// 	int parent_index = 0;
// 	Polygon_2 parent_poly;
// 	for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
// 		if (parent_index == parent_node->index_) {
// 			parent_poly = *ob;
// 			break;
// 		}
// 		parent_index++;
// 	}
// 	//std::cout << "getCandidates" << std::endl;
// 	//std::cout << "obs_list size:" << obs_list.size() << std::endl;
// 	for (auto obj = graspCubeToBaseMap.begin(); obj != graspCubeToBaseMap.end(); obj++) {
// 		// if current object has no availabe grasp poses, then continue to next object
// 		if (obj->second.size() == 0) {
// 			continue;
// 		}
// 		//std::cout << "obj " << obj->first << std::endl;
// 		Polygon_2 target_obj;
// 		Polygon_2 target_obj_inner;
// 		int target_index = 0;
// 		for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
// 			if (target_index == obj->first) {
// 				target_obj = *ob;
// 				break;
// 			}
// 			target_index++;
// 		}
// 		target_index = 0;
// 		for (auto ob = m_objectPolyList.begin(); ob != m_objectPolyList.end(); ob++) {
// 			if (target_index == obj->first) {
// 				target_obj_inner = *ob;
// 				break;
// 			}
// 			target_index++;
// 		}

// 		graspDist shortest_result;
// 		graspDist realshortest_result;
// 		Polygon2_list target_list, target_inner_list;
// 		target_list.push_back(target_obj);
// 		target_inner_list.push_back(target_obj_inner);
// 		std::map<int, std::vector<Point_2>> target_grasps = new_findGraspablePoses(target_inner_list, target_list);
// 		double shortestDist = 1000000;
// 		Point_2 real_shortest_pt;
// #ifndef HAVE_OBS
// 		for (int j = 0; j < target_grasps[target_index].size(); j++) {
// 			if (bg::distance(Point_2(START_X, START_Y), target_grasps[target_index][j]) < shortestDist) {
// 				shortestDist = bg::distance(Point_2(START_X, START_Y), target_grasps[target_index][j]);
// 				//std::list<Point_2>  path;
// 				//shortestDist = new_computeShortestPath(START_X, START_Y, target_grasps[target_index][j].get<0>(), target_grasps[target_index][j].get<1>(), path, planner);
// 				real_shortest_pt = target_grasps[target_index][j];
// 				realshortest_result.shortestDist = shortestDist;
				
// 			}
// 		}
// #endif
// #ifdef HAVE_OBS

// 		for (int j = 0; j < target_grasps[target_index].size(); j++) {
// 			std::list<Point_2>  path;
// 			if (bg::distance(Point_2(START_X, START_Y), target_grasps[target_index][j]) < shortestDist) {
// 				shortestDist = bg::distance(Point_2(START_X, START_Y), target_grasps[target_index][j]);
// 				//std::list<Point_2>  path;
// 				//shortestDist = new_computeShortestPath(START_X, START_Y, target_grasps[target_index][j].get<0>(), target_grasps[target_index][j].get<1>(), path, planner);
// 				real_shortest_pt = target_grasps[target_index][j];
// 				realshortest_result.shortestDist = shortestDist;

// 			}
// 		}
// #endif
// 		bool real_rrt_shortest_valid = false;
// 		for (int p = 0; p < obj->second.size(); p++) {
// 			if (bg::equals(real_shortest_pt, obj->second[p])) {
// 				real_rrt_shortest_valid = true;
// 				break;
// 			}
// 		}
// 		Point_2 shortest_grasp = getShortestDistGrasp(obj->second, obs_outer_list, shortest_result.shortestDist, planner);
		
// 		//std::cout << "rrt shortest_grasp:" << shortest_grasp.get<0>() << "," << shortest_grasp.get<1>() << std::endl;
// 		//std::cout << "real shortest_grasp:" << real_shortest_pt.get<0>() << "," << real_shortest_pt.get<1>() << std::endl;
// 		shortest_result.grasp = shortest_grasp;
// 		realshortest_result.grasp = real_shortest_pt;
// 		Linestring_2 shortest_linestring;
// 		bg::append(shortest_linestring, shortest_grasp);
// 		bg::append(shortest_linestring, Point_2(START_X, START_Y));
// 		Segment_2 shortest_line(shortest_grasp, Point_2(START_X, START_Y));
// 		std::vector<Polygon_2> collision_objs = checkLineCollision(shortest_linestring, target_obj, obs_outer_list);
// #ifndef HAVE_OBS
// 		Linestring_2 real_shortest_linestring;
// 		bg::append(real_shortest_linestring, real_shortest_pt);
// 		bg::append(real_shortest_linestring, Point_2(START_X, START_Y));
// 		std::vector<Polygon_2> real_collision_objs = checkLineCollision(real_shortest_linestring, target_obj, obs_outer_list);
// 		if ((real_collision_objs.size() == 0) && real_rrt_shortest_valid) {
// 			result.clear();
// 			result[obj->first] = realshortest_result;
// 			//std::cout << "fastshortcut:" << obj->first << std::endl;
// 			break;
// 		}
// #endif
// 		if (collision_objs.size() == 0) {

// 			result[obj->first] = shortest_result;
// 			if (bg::equals(shortest_grasp, real_shortest_pt)) {

// 				std::cout << "shortcut:" << obj->first << std::endl;
// 				break;
// 			}
// 			//std::cout << "collision_objs num is 0" << std::endl;
// 			continue;
// 			//break;
// 		}
// 		else 
// 		{
// 			//std::cout << "in collision" <<obj->first<< std::endl;
// 			bool is_valid = true;
// 			for (int j = 0; j < collision_objs.size(); j++) {
// 				if (checkNotInSameCluster(collision_objs[j], obs_outer_list, target_obj)) {
// 					//std::cout << "not in same cluster" << std::endl;
// 					is_valid = false;
// 					break;
// 				}
// 			}
// 			if (is_valid)
// 			{
// 			//	if (use_cluster_heu) {
// 			//		if (bg::distance(target_obj_inner, parent_poly) > 700) {
// 			//			continue;
// 			//		}
// 			//	}
// 				result[obj->first] = shortest_result;
// 			}
// 		}

		
// 	}
// 	return result;
// 	/*
// 	if (obj_index > last_index) {
// 	grasp_poses = graspCubeToBaseMap[obj_index-1];
// 	}
// 	else {
// 	grasp_poses = graspCubeToBaseMap[obj_index];
// 	}
// 	*/
// 	/* 
// 	std::cout << "current one:" << seq[i] << std::endl;
// 	grasp_poses = graspCubeToBaseMap[obj_index];
// 	if (grasp_poses.size() == 0) {
// 		std::cout << "no valid sequence, no grasp poses available" << std::endl;
// 		return 100000;
// 	}
// 	for (int j = 0; j < grasp_poses.size(); j++) {
// 		std::vector<std::pair<double, double> > path;
// 		double temp_dist = new_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, env, v_graph);
// 		if (temp_dist < nearest_dist) {
// 			nearest_dist = temp_dist;
// 			closest_grasp = grasp_poses[j];
// 		}
// 	}
// 	*/
// }

double Roadmap::declutterMultiExitGreedy() {
	planner_t rrts_exit_0_greedy;
	planner_t rrts_exit_1_greedy;
	planner_t rrts_exit_2_greedy;
	planner_t rrts_exit_3_greedy;
	System robot2d_multiexit_greedy;
	std::vector<std::pair<double, double>> exit_list;
	exit_list.push_back(std::make_pair(EXIT_0_X, EXIT_0_Y));
	exit_list.push_back(std::make_pair(EXIT_1_X, EXIT_1_Y));
	exit_list.push_back(std::make_pair(EXIT_2_X, EXIT_2_Y));
	exit_list.push_back(std::make_pair(EXIT_3_X, EXIT_3_Y));

	Polygon2_list obs_list = m_objectPolyList;
	Polygon2_list obs_outer_list = m_objectOuterPolyList;
	std::vector<planner_t> rrts_list;
	robot2d_multiexit_greedy.setNumDimensions(2);
	robot2d_multiexit_greedy.regionOperating.setNumDimensions(2);
#ifdef BIG_ENV
	robot2d_multiexit_greedy.regionOperating.center[0] = 2500.0;
	robot2d_multiexit_greedy.regionOperating.center[1] = 2500.0;
	robot2d_multiexit_greedy.regionOperating.size[0] = 5000.0;
	robot2d_multiexit_greedy.regionOperating.size[1] = 5000.0;
#endif
#ifdef SMALL_ENV
	robot2d_multiexit_greedy.regionOperating.center[0] = 500.0;
	robot2d_multiexit_greedy.regionOperating.center[1] = 500.0;
	robot2d_multiexit_greedy.regionOperating.size[0] = 1000.0;
	robot2d_multiexit_greedy.regionOperating.size[1] = 1000.0;
#endif
	robot2d_multiexit_greedy.regionGoal.setNumDimensions(2);
	robot2d_multiexit_greedy.regionGoal.center[0] = 300000.0;
	robot2d_multiexit_greedy.regionGoal.center[1] = 300000.0;
	robot2d_multiexit_greedy.regionGoal.size[0] = 2.0;
	robot2d_multiexit_greedy.regionGoal.size[1] = 2.0;


	// insert obstacles into system
	double min_x = 0;
	double max_x = 0;
	double min_y = 0;
	double max_y = 0;

	robot2d_multiexit_greedy.obstacles.clear();
	for (auto ob = obs_list.begin(); ob != obs_list.end(); ob++) {
		// region *obstacle;
		Polygon_2 * obstacle = new Polygon_2;
		*obstacle = *ob;
		// obstacle = new region;
		// obstacle->setNumDimensions(2);
		//convertPolygon2Region(*ob, obstacle->center[0], obstacle->center[1], obstacle->size[0], obstacle->size[1]);
		robot2d_multiexit_greedy.obstacles.push_back(obstacle);  // Add the obstacle to the list
	}


	std::list<Polygon_2*> backup_obstacles;
	backup_obstacles = robot2d_multiexit_greedy.obstacles;
	robot2d_multiexit_greedy.generateHashMap();
	// Add the system to the planner
	rrts_list.push_back(rrts_exit_0_greedy);
	rrts_list.push_back(rrts_exit_1_greedy);
	rrts_list.push_back(rrts_exit_2_greedy);
	rrts_list.push_back(rrts_exit_3_greedy);

	for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
		rrts_list[rrts_index].setSystem(robot2d_multiexit_greedy);

		// Set up the root vertex
		vertex_t &root = rrts_list[rrts_index].getRootVertex();
		State &rootState = root.getState();
		rootState[0] = exit_list[rrts_index].first;
		rootState[1] = exit_list[rrts_index].second;



		// Initialize the planner
		rrts_list[rrts_index].initialize();

		// This parameter should be larger than 1.5 for asymptotic 
		//   optimality. Larger values will weigh on optimization 
		//   rather than exploration in the RRT* algorithm. Lower 
		//   values, such as 0.1, should recover the RRT.
		rrts_list[rrts_index].setGamma(1.5);
#ifdef BIG_ENV
		for (int i = 0; i < 10000; i++)
			rrts_list[rrts_index].iteration();
#endif
#ifdef SMALL_ENV
		for (int i = 0; i < 2000; i++)
			rrts_list[rrts_index].iteration();
#endif
	}
	std::vector<int> picking_sequence;

	int obj_index = 0;
	int closest_obj_index = 0;
	int local_obj_index = 0;
	int closest_local_index = 0;
	Point_2 closest_grasp;
	double nearest_dist = 10000000;
	double total_dist = 0;
	std::vector<int> pick_obj_sequence;
	Polygon2_list::iterator nearest_obj_iterator;
	Polygon2_list::iterator obstaclePolyList_iterator;
	int current_exit_index = 0;
	int next_exit_index = 0;
	while (obs_outer_list.size() > 0) {

		closest_obj_index = 0;
		local_obj_index = 0;
		obj_index = 0;
		nearest_dist = 10000000;
		//std::cout << "obstacle size:" << obs_outer_list.size() << std::endl;
		std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(obs_list, obs_outer_list);
		//Environment  *env;
		//Visibility_Graph *v_graph;
		//new_buildVisibilityGraph(obs_outer_list, env, v_graph);
		for (auto i = obs_outer_list.begin(); i != obs_outer_list.end(); i++) {
			obj_index = 0;

			for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
				if (bg::equals(*i, *ob)) {
					break;
				}
				obj_index++;
			}
			std::vector<Point_2> grasp_poses = graspCubeToBaseMap[obj_index];
			for (int j = 0; j < grasp_poses.size(); j++) {
				std::list<Point_2 > path;
//				double temp_dist = visi_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, env, v_graph);
				double phase_1_dist = new_computeShortestPath(exit_list[current_exit_index].first, exit_list[current_exit_index].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[current_exit_index]);
				//std::cout << "temp_dist:" << temp_dist << " pose:"<< grasp_poses[j].get<0>()<<","<<grasp_poses[j].get<1>()<<std::endl;
				double phase_2_dist = 0;
				for (int k = 0; k < TOTAL_EXIT_NUM; k++) {
					phase_2_dist = new_computeShortestPath(exit_list[k].first, exit_list[k].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[k]);

					if (phase_1_dist + phase_2_dist < nearest_dist) {
						nearest_dist = phase_1_dist + phase_2_dist;
						closest_obj_index = obj_index;
						closest_grasp = grasp_poses[j];
						nearest_obj_iterator = i;
						closest_local_index = local_obj_index;
						next_exit_index = k;
					}
				}
			}
			//obj_index++;
			local_obj_index++;
		}
		std::cout << "pick " << closest_obj_index << " object, grasp pose:" << closest_grasp.get<0>() << "," << closest_grasp.get<1>() << "shortest dist:" << nearest_dist <<" next exit:"<<next_exit_index<< std::endl;
		total_dist += nearest_dist;
		picking_sequence.push_back(closest_obj_index);
		obs_outer_list.erase(nearest_obj_iterator);
		obstaclePolyList_iterator = obs_list.begin();
		auto delete_obs = robot2d_multiexit_greedy.obstacles.begin();
		int j = 0;
		while (j++ < closest_local_index) {
			obstaclePolyList_iterator++;
			delete_obs++;
		}
		Polygon_2* erase_region = *delete_obs;
		robot2d_multiexit_greedy.obstacles.erase(delete_obs);
		robot2d_multiexit_greedy.updateHashMap(erase_region);
#ifdef BIG_ENV
		for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
			for (int i = 0; i < 10; i++) {
				if (i == 9) {
					//std::cout << "full temp_iteration:" << std::endl;
					rrts_list[rrts_index].full_temp_iteration(erase_region);
				}
				else {
					rrts_list[rrts_index].temp_iteration(erase_region);
				}
			}

		}
#endif
#ifdef SMALL_ENV
		for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
			for (int i = 0; i < 5; i++)
				rrts_list[rrts_index].temp_iteration(*erase_region);
		}
#endif
		obs_list.erase(obstaclePolyList_iterator);
		current_exit_index = next_exit_index;
	}
	//rrts_local.temp_restore();
	for (auto p = backup_obstacles.end(); p != backup_obstacles.end(); p++) {
		delete *p;
	}
	std::cout << "total dist:" << total_dist << std::endl;
	return total_dist;

	
}

double Roadmap::declutterMultiExitSeparateGreedy() {
	planner_t rrts_exit_0_greedy;
	planner_t rrts_exit_1_greedy;
	planner_t rrts_exit_2_greedy;
	planner_t rrts_exit_3_greedy;
	System robot2d_multiexit_greedy;
	std::vector<std::pair<double, double>> exit_list;
	exit_list.push_back(std::make_pair(EXIT_0_X, EXIT_0_Y));
	exit_list.push_back(std::make_pair(EXIT_1_X, EXIT_1_Y));
	exit_list.push_back(std::make_pair(EXIT_2_X, EXIT_2_Y));
	exit_list.push_back(std::make_pair(EXIT_3_X, EXIT_3_Y));

	Polygon2_list obs_list = m_objectPolyList;
	Polygon2_list obs_outer_list = m_objectOuterPolyList;
	Polygon2_list exit_0_obs_outer_list, exit_1_obs_outer_list;
	Polygon2_list exit_0_obs_list, exit_1_obs_list;
	std::vector<Polygon2_list> exit_outer_vector, exit_vector;
	for (int i = 0; i < TOTAL_EXIT_NUM; i++) {
		Polygon2_list temp_list;
		exit_outer_vector.push_back(temp_list);
		exit_vector.push_back(temp_list);
	}
	std::vector<planner_t> rrts_list;
	robot2d_multiexit_greedy.setNumDimensions(2);
	robot2d_multiexit_greedy.regionOperating.setNumDimensions(2);
#ifdef BIG_ENV
	robot2d_multiexit_greedy.regionOperating.center[0] = 2500.0;
	robot2d_multiexit_greedy.regionOperating.center[1] = 2500.0;
	robot2d_multiexit_greedy.regionOperating.size[0] = 5000.0;
	robot2d_multiexit_greedy.regionOperating.size[1] = 5000.0;
#endif
#ifdef SMALL_ENV
	robot2d_multiexit_greedy.regionOperating.center[0] = 500.0;
	robot2d_multiexit_greedy.regionOperating.center[1] = 500.0;
	robot2d_multiexit_greedy.regionOperating.size[0] = 1000.0;
	robot2d_multiexit_greedy.regionOperating.size[1] = 1000.0;
#endif
	robot2d_multiexit_greedy.regionGoal.setNumDimensions(2);
	robot2d_multiexit_greedy.regionGoal.center[0] = 300000.0;
	robot2d_multiexit_greedy.regionGoal.center[1] = 300000.0;
	robot2d_multiexit_greedy.regionGoal.size[0] = 2.0;
	robot2d_multiexit_greedy.regionGoal.size[1] = 2.0;


	// insert obstacles into system
	double min_x = 0;
	double max_x = 0;
	double min_y = 0;
	double max_y = 0;

	robot2d_multiexit_greedy.obstacles.clear();
	for (auto ob = obs_list.begin(); ob != obs_list.end(); ob++) {
		// region *obstacle;
		Polygon_2 * obstacle = new Polygon_2;
		*obstacle = *ob;
		// obstacle = new region;
		// obstacle->setNumDimensions(2);
		//convertPolygon2Region(*ob, obstacle->center[0], obstacle->center[1], obstacle->size[0], obstacle->size[1]);
		robot2d_multiexit_greedy.obstacles.push_back(obstacle);  // Add the obstacle to the list
	}


	std::list<Polygon_2*> backup_obstacles;
	backup_obstacles = robot2d_multiexit_greedy.obstacles;
	robot2d_multiexit_greedy.generateHashMap();
	// Add the system to the planner
	rrts_list.push_back(rrts_exit_0_greedy);
	rrts_list.push_back(rrts_exit_1_greedy);
	rrts_list.push_back(rrts_exit_2_greedy);
	rrts_list.push_back(rrts_exit_3_greedy);

	for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
		rrts_list[rrts_index].setSystem(robot2d_multiexit_greedy);

		// Set up the root vertex
		vertex_t &root = rrts_list[rrts_index].getRootVertex();
		State &rootState = root.getState();
		rootState[0] = exit_list[rrts_index].first;
		rootState[1] = exit_list[rrts_index].second;



		// Initialize the planner
		rrts_list[rrts_index].initialize();

		// This parameter should be larger than 1.5 for asymptotic 
		//   optimality. Larger values will weigh on optimization 
		//   rather than exploration in the RRT* algorithm. Lower 
		//   values, such as 0.1, should recover the RRT.
		rrts_list[rrts_index].setGamma(1.5);
#ifdef BIG_ENV
		for (int i = 0; i < 10000; i++)
			rrts_list[rrts_index].iteration();
#endif
#ifdef SMALL_ENV
		for (int i = 0; i < 2000; i++)
			rrts_list[rrts_index].iteration();
#endif
	}
	std::vector<int> picking_sequence;

	int obj_index = 0;
	int closest_obj_index = 0;
	int local_obj_index = 0;
	int closest_local_index = 0;
	int delete_obs_list_index = 0;
	Point_2 closest_grasp;
	double nearest_dist = 10000000;
	double total_dist = 0;
	std::vector<int> pick_obj_sequence;
	Polygon2_list::iterator nearest_obj_iterator;
	Polygon2_list::iterator obstaclePolyList_iterator;
	int current_exit_index = 0;
	int next_exit_index = 0;
	int total_left_num = obs_list.size();
	std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(obs_list, obs_outer_list);
	//Environment  *env;
	//Visibility_Graph *v_graph;
	//new_buildVisibilityGraph(obs_outer_list, env, v_graph);
	
	for (auto i = obs_outer_list.begin(); i != obs_outer_list.end(); i++) {
		obj_index = 0;
		auto obs_inner = obs_list.begin();
		for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
			if (bg::equals(*i, *ob)) {
				break;
			}
			obj_index++;
			obs_inner++;
		}

		std::vector<Point_2> grasp_poses = graspCubeToBaseMap[obj_index];
		if (grasp_poses.size() > 0) {
			for (int j = 0; j < grasp_poses.size(); j++) {
				double exit_shortest_dist = 10000000;
				int exit_index = 0;
				for (int e = 0; e < exit_list.size(); e++) {
					double to_exit_dist = bg::distance(grasp_poses[j], Point_2(exit_list[e].first, exit_list[e].second));
					if (to_exit_dist < exit_shortest_dist) {
						exit_shortest_dist = to_exit_dist;
						exit_index = e;
					}
				}
				exit_outer_vector[exit_index].push_back(*i);
				exit_vector[exit_index].push_back(*obs_inner);
				// if (grasp_poses[j].get<1>() > 2500) {
				// 	exit_0_obs_outer_list.push_back(*i);
				// 	exit_0_obs_list.push_back(*obs_inner);
				// }
				// else{
				// 	exit_1_obs_outer_list.push_back(*i);
				// 	exit_1_obs_list.push_back(*obs_inner);
				// }
				break;
			}
		}
		else {
			Point_2 center;
			bg::centroid(*i ,center);
			double exit_shortest_dist = 10000000;
			int exit_index = 0;
			for (int e = 0; e < exit_list.size(); e++) {
				double to_exit_dist = bg::distance(center, Point_2(exit_list[e].first, exit_list[e].second));
				if (to_exit_dist < exit_shortest_dist) {
					exit_shortest_dist = to_exit_dist;
					exit_index = e;
				}
			}
			exit_outer_vector[exit_index].push_back(*i);
			exit_vector[exit_index].push_back(*obs_inner);
		}
	}
	   

	// start from exit 0
	std::vector<int> covered_exit;
	covered_exit.push_back(0);
	if (current_exit_index == 0) {
		while (total_left_num > 0) {
			if (exit_outer_vector[current_exit_index].size() > 0) {
				while (exit_outer_vector[current_exit_index].size() > 0) {
					closest_obj_index = 0;
					local_obj_index = 0;
					obj_index = 0;
					nearest_dist = 10000000;
					//std::cout << "obstacle size:" << obs_outer_list.size() << std::endl;
					//std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(exit_vector[current_exit_index], exit_outer_vector[current_exit_index]);
					graspCubeToBaseMap.clear();
					std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(obs_list, obs_outer_list);
					int obs_global_index = 0;
					//Environment  *env;
					//Visibility_Graph *v_graph;
					//new_buildVisibilityGraph(obs_outer_list, env, v_graph);
					for (auto i = exit_outer_vector[current_exit_index].begin(); i != exit_outer_vector[current_exit_index].end(); i++) {
						obj_index = 0;

						for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
							if (bg::equals(*i, *ob)) {
								break;
							}
							obj_index++;
						}
						obs_global_index = 0;

						for (auto ob = obs_outer_list.begin(); ob != obs_outer_list.end(); ob++) {
							if (bg::equals(*i, *ob)) {
								break;
							}
							obs_global_index++;
						}
						std::vector<Point_2> grasp_poses = graspCubeToBaseMap[obj_index];
						for (int j = 0; j < grasp_poses.size(); j++) {
							std::list<Point_2 > path;
							//				double temp_dist = visi_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, env, v_graph);
							double phase_1_dist = new_computeShortestPath(exit_list[current_exit_index].first, exit_list[current_exit_index].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[current_exit_index]);
							//std::cout << "temp_dist:" << temp_dist << " pose:"<< grasp_poses[j].get<0>()<<","<<grasp_poses[j].get<1>()<<std::endl;
							double phase_2_dist = 0;

							if (exit_outer_vector[current_exit_index].size() > 1) {
								phase_2_dist = phase_1_dist;

								if (phase_1_dist + phase_2_dist < nearest_dist) {
									nearest_dist = phase_1_dist + phase_2_dist;
									closest_obj_index = obj_index;
									closest_grasp = grasp_poses[j];
									nearest_obj_iterator = i;
									closest_local_index = local_obj_index;
									next_exit_index = current_exit_index;
									delete_obs_list_index = obs_global_index;
								}
							}
							else {
								double min_to_exit_dist = 10000000;
								for (int e = 0; e < exit_outer_vector.size(); e++) {
									bool is_new = true;
									for (int pp = 0; pp < covered_exit.size(); pp++) {
										if (e == covered_exit[pp]) {
											is_new = false;
											break;
										}
									}
									if (!is_new) {
										continue;
									}
									if (exit_outer_vector[e].size() == 0) {
										continue;
									}
									if (e != current_exit_index) {
										phase_2_dist = new_computeShortestPath(exit_list[e].first, exit_list[e].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[e]);
										if (phase_2_dist < min_to_exit_dist) {
											min_to_exit_dist = phase_2_dist;
											next_exit_index = e;
										}
									}
								}
								nearest_dist = phase_1_dist + phase_2_dist;
								closest_obj_index = obj_index;
								closest_grasp = grasp_poses[j];
								nearest_obj_iterator = i;
								closest_local_index = local_obj_index;
								delete_obs_list_index = obs_global_index;

							}
							//for (int k = 0; k < TOTAL_EXIT_NUM; k++) {
							//	phase_2_dist = new_computeShortestPath(exit_list[k].first, exit_list[k].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[k]);


							//}
						}
						//obj_index++;
						local_obj_index++;
					}
					std::cout << "pick " << closest_obj_index << " object, grasp pose:" << closest_grasp.get<0>() << "," << closest_grasp.get<1>() << "shortest dist:" << nearest_dist << " next exit:" << next_exit_index << std::endl;
					total_dist += nearest_dist;
					picking_sequence.push_back(closest_obj_index);
					exit_outer_vector[current_exit_index].erase(nearest_obj_iterator);
					//obs_outer_list.erase(nearest_obj_iterator);
					obstaclePolyList_iterator = exit_vector[current_exit_index].begin();
					//auto delete_obs = robot2d_multiexit_greedy.obstacles.begin();
					auto delete_obs = backup_obstacles.begin();
					auto global_obj_delete = obs_list.begin();
					auto global_outer_obj_delete = obs_outer_list.begin();
					int j = 0;
					while (j++ < closest_local_index) {
						obstaclePolyList_iterator++;
						
					}
					j = 0;
					while (j++ < closest_obj_index) {
						delete_obs++;
						
					}
					j = 0;
					while (j++ < delete_obs_list_index) {
						global_obj_delete++;
						global_outer_obj_delete++;
					}
					obs_list.erase(global_obj_delete);
					obs_outer_list.erase(global_outer_obj_delete);
					Polygon_2* erase_region = *delete_obs;
					auto final_delete_obs_0 = robot2d_multiexit_greedy.obstacles.begin();
					for (auto p = robot2d_multiexit_greedy.obstacles.begin(); p != robot2d_multiexit_greedy.obstacles.end(); p++) {
						if (bg::equals(**p, *erase_region)) {
							break;
						}
						final_delete_obs_0++;
					}
					robot2d_multiexit_greedy.obstacles.erase(final_delete_obs_0);

					//robot2d_multiexit_greedy.obstacles.erase(delete_obs);
					robot2d_multiexit_greedy.updateHashMap(erase_region);
#ifdef BIG_ENV
					for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
						for (int i = 0; i < 10; i++) {
							if (i == 9) {
								//std::cout << "full temp_iteration:" << std::endl;
								rrts_list[rrts_index].full_temp_iteration(erase_region);
							}
							else {
								rrts_list[rrts_index].temp_iteration(erase_region);
							}
						}

					}
#endif
#ifdef SMALL_ENV
					for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
						for (int i = 0; i < 5; i++)
							rrts_list[rrts_index].temp_iteration(*erase_region);
					}
#endif
					exit_vector[current_exit_index].erase(obstaclePolyList_iterator);
					total_left_num--;

				}

				covered_exit.push_back(current_exit_index);
				current_exit_index = next_exit_index;
			}
			else {

				// when current exit's region has no objects. Actually, this only happens at the first exit( exit 0 )
				
				for (int e = 0; e < exit_outer_vector.size(); e++) {
					bool is_new = true;
					for (int pp = 0; pp < covered_exit.size(); pp++) {
						if (e == covered_exit[pp]) {
							is_new = false;
							break;
						}
					}
					if (!is_new) {
						continue;
					}
					if (exit_outer_vector[e].size() == 0) {
						continue;
					}
					if (e != current_exit_index) {
						double phase_dist = bg::distance(Point_2(exit_list[current_exit_index].first, exit_list[current_exit_index].second), Point_2(exit_list[e].first, exit_list[e].second));
						current_exit_index = e;
						total_dist += phase_dist;
						break;
					}
				}
			}
		}

	}








// 	if (current_exit_index == 0) {
// 		while (exit_0_obs_outer_list.size() > 0) {
// 			closest_obj_index = 0;
// 			local_obj_index = 0;
// 			obj_index = 0;
// 			nearest_dist = 10000000;
// 			//std::cout << "obstacle size:" << obs_outer_list.size() << std::endl;
// 			std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(exit_0_obs_list, exit_0_obs_outer_list);
// 			//Environment  *env;
// 			//Visibility_Graph *v_graph;
// 			//new_buildVisibilityGraph(obs_outer_list, env, v_graph);
// 			for (auto i = exit_0_obs_outer_list.begin(); i != exit_0_obs_outer_list.end(); i++) {
// 				obj_index = 0;

// 				for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
// 					if (bg::equals(*i, *ob)) {
// 						break;
// 					}
// 					obj_index++;
// 				}
// 				std::vector<Point_2> grasp_poses = graspCubeToBaseMap[obj_index];
// 				for (int j = 0; j < grasp_poses.size(); j++) {
// 					std::list<Point_2 > path;
// 					//				double temp_dist = visi_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, env, v_graph);
// 					double phase_1_dist = new_computeShortestPath(exit_list[current_exit_index].first, exit_list[current_exit_index].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[current_exit_index]);
// 					//std::cout << "temp_dist:" << temp_dist << " pose:"<< grasp_poses[j].get<0>()<<","<<grasp_poses[j].get<1>()<<std::endl;
// 					double phase_2_dist = 0;

// 					if (exit_0_obs_list.size() > 1 || exit_1_obs_list.size() == 0) {
// 						phase_2_dist = phase_1_dist;

// 						if (phase_1_dist + phase_2_dist < nearest_dist) {
// 							nearest_dist = phase_1_dist + phase_2_dist;
// 							closest_obj_index = obj_index;
// 							closest_grasp = grasp_poses[j];
// 							nearest_obj_iterator = i;
// 							closest_local_index = local_obj_index;
// 							next_exit_index = 0;
// 						}
// 					}
// 					else {
// 						phase_2_dist = new_computeShortestPath(exit_list[1].first, exit_list[1].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[1]);
// 						next_exit_index = 1;
// 						nearest_dist = phase_1_dist + phase_2_dist;
// 						closest_obj_index = obj_index;
// 						closest_grasp = grasp_poses[j];
// 						nearest_obj_iterator = i;
// 						closest_local_index = local_obj_index;
// 					}
// 					//for (int k = 0; k < TOTAL_EXIT_NUM; k++) {
// 					//	phase_2_dist = new_computeShortestPath(exit_list[k].first, exit_list[k].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[k]);

						
// 					//}
// 				}
// 				//obj_index++;
// 				local_obj_index++;
// 			}
// 			std::cout << "pick " << closest_obj_index << " object, grasp pose:" << closest_grasp.get<0>() << "," << closest_grasp.get<1>() << "shortest dist:" << nearest_dist << " next exit:" << next_exit_index << std::endl;
// 			total_dist += nearest_dist;
// 			picking_sequence.push_back(closest_obj_index);
// 			exit_0_obs_outer_list.erase(nearest_obj_iterator);
// 			//obs_outer_list.erase(nearest_obj_iterator);
// 			obstaclePolyList_iterator = exit_0_obs_list.begin();
// 			//auto delete_obs = robot2d_multiexit_greedy.obstacles.begin();
// 			auto delete_obs = backup_obstacles.begin();
// 			int j = 0;
// 			while (j++ < closest_local_index) {
// 				obstaclePolyList_iterator++;
// 			}
// 			j = 0;
// 			while (j++ < closest_obj_index) {
// 				delete_obs++;

// 			}
// 			Polygon_2* erase_region = *delete_obs;
// 			auto final_delete_obs_0 = robot2d_multiexit_greedy.obstacles.begin();
// 			for (auto p = robot2d_multiexit_greedy.obstacles.begin(); p != robot2d_multiexit_greedy.obstacles.end(); p++) {
// 				if (bg::equals(**p, *erase_region)) {
// 					break;
// 				}
// 				final_delete_obs_0++;
// 			}
// 			robot2d_multiexit_greedy.obstacles.erase(final_delete_obs_0);

// 			//robot2d_multiexit_greedy.obstacles.erase(delete_obs);
// 			robot2d_multiexit_greedy.updateHashMap(erase_region);
// #ifdef BIG_ENV
// 			for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
// 				for (int i = 0; i < 10; i++) {
// 					if (i == 9) {
// 						//std::cout << "full temp_iteration:" << std::endl;
// 						rrts_list[rrts_index].full_temp_iteration(erase_region);
// 					}
// 					else {
// 						rrts_list[rrts_index].temp_iteration(erase_region);
// 					}
// 				}

// 			}
// #endif
// #ifdef SMALL_ENV
// 			for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
// 				for (int i = 0; i < 5; i++)
// 					rrts_list[rrts_index].temp_iteration(*erase_region);
// 			}
// #endif
// 			exit_0_obs_list.erase(obstaclePolyList_iterator);
// 			current_exit_index = next_exit_index;
// 		}

// 		std::cout << "switch to exit 1" << std::endl;

// 	}
// 	current_exit_index = 1;
// 	while (exit_1_obs_outer_list.size() > 0) {
// 		closest_obj_index = 0;
// 		local_obj_index = 0;
// 		obj_index = 0;
// 		nearest_dist = 10000000;
// 		//std::cout << "obstacle size:" << obs_outer_list.size() << std::endl;
// 		std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(exit_1_obs_list, exit_1_obs_outer_list);
// 		//Environment  *env;
// 		//Visibility_Graph *v_graph;
// 		//new_buildVisibilityGraph(obs_outer_list, env, v_graph);
// 		for (auto i = exit_1_obs_outer_list.begin(); i != exit_1_obs_outer_list.end(); i++) {
// 			obj_index = 0;

// 			for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
// 				if (bg::equals(*i, *ob)) {
// 					break;
// 				}
// 				obj_index++;
// 			}
// 			std::vector<Point_2> grasp_poses = graspCubeToBaseMap[obj_index];
// 			for (int j = 0; j < grasp_poses.size(); j++) {
// 				std::list<Point_2 > path;
// 				//				double temp_dist = visi_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, env, v_graph);
// 				double phase_1_dist = new_computeShortestPath(exit_list[current_exit_index].first, exit_list[current_exit_index].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[current_exit_index]);
// 				//std::cout << "temp_dist:" << temp_dist << " pose:"<< grasp_poses[j].get<0>()<<","<<grasp_poses[j].get<1>()<<std::endl;
// 				double phase_2_dist = 0;

// 				//if (exit_0_obs_list.size() > 1) {
// 					phase_2_dist = phase_1_dist;

// 					if (phase_1_dist + phase_2_dist < nearest_dist) {
// 						nearest_dist = phase_1_dist + phase_2_dist;
// 						closest_obj_index = obj_index;
// 						closest_grasp = grasp_poses[j];
// 						nearest_obj_iterator = i;
// 						closest_local_index = local_obj_index;
// 						next_exit_index = 1;
// 					}
// 				//}
// 				//else {
// 				//	phase_2_dist = new_computeShortestPath(exit_list[1].first, exit_list[1].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[1]);
// 				//	next_exit_index = 1;
// 				//}
// 				//for (int k = 0; k < TOTAL_EXIT_NUM; k++) {
// 				//	phase_2_dist = new_computeShortestPath(exit_list[k].first, exit_list[k].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[k]);


// 				//}
// 			}
// 			//obj_index++;
// 			local_obj_index++;
// 		}
// 		std::cout << "pick " << closest_obj_index << " object, grasp pose:" << closest_grasp.get<0>() << "," << closest_grasp.get<1>() << "shortest dist:" << nearest_dist << " next exit:" << next_exit_index << std::endl;
// 		total_dist += nearest_dist;
// 		picking_sequence.push_back(closest_obj_index);
// 		exit_1_obs_outer_list.erase(nearest_obj_iterator);
// 		//obs_outer_list.erase(nearest_obj_iterator);
// 		obstaclePolyList_iterator = exit_1_obs_list.begin();
// 		//auto delete_obs = robot2d_multiexit_greedy.obstacles.begin();
// 		auto delete_obs = backup_obstacles.begin();

// 		int j = 0;
// 		while (j++ < closest_local_index) {
// 			obstaclePolyList_iterator++;
// 		}
// 		j = 0;
// 		while (j++ < closest_obj_index) {
// 			delete_obs++;

// 		}
// 		Polygon_2* erase_region = *delete_obs;
// 		auto final_delete_obs = robot2d_multiexit_greedy.obstacles.begin();
// 		for (auto p = robot2d_multiexit_greedy.obstacles.begin(); p != robot2d_multiexit_greedy.obstacles.end(); p++) {
// 			if (bg::equals(**p, *erase_region)) {
// 				break;
// 			}
// 			final_delete_obs++;
// 		}
// 		robot2d_multiexit_greedy.obstacles.erase(final_delete_obs);
// 		robot2d_multiexit_greedy.updateHashMap(erase_region);
// #ifdef BIG_ENV
// 		for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
// 			for (int i = 0; i < 10; i++) {
// 				if (i == 9) {
// 					//std::cout << "full temp_iteration:" << std::endl;
// 					rrts_list[rrts_index].full_temp_iteration(erase_region);
// 				}
// 				else {
// 					rrts_list[rrts_index].temp_iteration(erase_region);
// 				}
// 			}

// 		}
// #endif
// #ifdef SMALL_ENV
// 		for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
// 			for (int i = 0; i < 5; i++)
// 				rrts_list[rrts_index].temp_iteration(*erase_region);
// 		}
// #endif
// 		exit_1_obs_list.erase(obstaclePolyList_iterator);
// 		current_exit_index = next_exit_index;
// 	}



// 	while (obs_outer_list.size() > 0) {

// 		closest_obj_index = 0;
// 		local_obj_index = 0;
// 		obj_index = 0;
// 		nearest_dist = 10000000;
// 		//std::cout << "obstacle size:" << obs_outer_list.size() << std::endl;
// 		std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(obs_list, obs_outer_list);
// 		//Environment  *env;
// 		//Visibility_Graph *v_graph;
// 		//new_buildVisibilityGraph(obs_outer_list, env, v_graph);
// 		for (auto i = obs_outer_list.begin(); i != obs_outer_list.end(); i++) {
// 			obj_index = 0;

// 			for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
// 				if (bg::equals(*i, *ob)) {
// 					break;
// 				}
// 				obj_index++;
// 			}
// 			std::vector<Point_2> grasp_poses = graspCubeToBaseMap[obj_index];
// 			for (int j = 0; j < grasp_poses.size(); j++) {
// 				std::list<Point_2 > path;
// 				//				double temp_dist = visi_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, env, v_graph);
// 				double phase_1_dist = new_computeShortestPath(exit_list[current_exit_index].first, exit_list[current_exit_index].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[current_exit_index]);
// 				//std::cout << "temp_dist:" << temp_dist << " pose:"<< grasp_poses[j].get<0>()<<","<<grasp_poses[j].get<1>()<<std::endl;
// 				double phase_2_dist = 0;
// 				for (int k = 0; k < TOTAL_EXIT_NUM; k++) {
// 					phase_2_dist = new_computeShortestPath(exit_list[k].first, exit_list[k].second, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_list[k]);

// 					if (phase_1_dist + phase_2_dist < nearest_dist) {
// 						nearest_dist = phase_1_dist + phase_2_dist;
// 						closest_obj_index = obj_index;
// 						closest_grasp = grasp_poses[j];
// 						nearest_obj_iterator = i;
// 						closest_local_index = local_obj_index;
// 						next_exit_index = k;
// 					}
// 				}
// 			}
// 			//obj_index++;
// 			local_obj_index++;
// 		}
// 		std::cout << "pick " << closest_obj_index << " object, grasp pose:" << closest_grasp.get<0>() << "," << closest_grasp.get<1>() << "shortest dist:" << nearest_dist << " next exit:" << next_exit_index << std::endl;
// 		total_dist += nearest_dist;
// 		picking_sequence.push_back(closest_obj_index);
// 		obs_outer_list.erase(nearest_obj_iterator);
// 		obstaclePolyList_iterator = obs_list.begin();
// 		auto delete_obs = robot2d_multiexit_greedy.obstacles.begin();
// 		int j = 0;
// 		while (j++ < closest_local_index) {
// 			obstaclePolyList_iterator++;
// 			delete_obs++;
// 		}
// 		Polygon_2* erase_region = *delete_obs;
// 		robot2d_multiexit_greedy.obstacles.erase(delete_obs);
// 		robot2d_multiexit_greedy.updateHashMap(erase_region);
// #ifdef BIG_ENV
// 		for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
// 			for (int i = 0; i < 10; i++) {
// 				if (i == 9) {
// 					//std::cout << "full temp_iteration:" << std::endl;
// 					rrts_list[rrts_index].full_temp_iteration(erase_region);
// 				}
// 				else {
// 					rrts_list[rrts_index].temp_iteration(erase_region);
// 				}
// 			}

// 		}
// #endif
// #ifdef SMALL_ENV
// 		for (int rrts_index = 0; rrts_index < rrts_list.size(); rrts_index++) {
// 			for (int i = 0; i < 5; i++)
// 				rrts_list[rrts_index].temp_iteration(*erase_region);
// 		}
// #endif
// 		obs_list.erase(obstaclePolyList_iterator);
// 		current_exit_index = next_exit_index;
// 	}
	//rrts_local.temp_restore();
	for (auto p = backup_obstacles.end(); p != backup_obstacles.end(); p++) {
		delete *p;
	}
	std::cout << "total dist:" << total_dist << std::endl;
	return total_dist;


}


double Roadmap::declutterUsingGreedy(){
	System robot2d_1;
	planner_t rrts_1;
#ifdef USE_PRM
	PRM prm_1;
#endif
	robot2d_1.setNumDimensions(2);
	robot2d_1.regionOperating.setNumDimensions(2);
#ifdef BIG_ENV
	robot2d_1.regionOperating.center[0] = 2500.0;
	robot2d_1.regionOperating.center[1] = 2500.0;
	robot2d_1.regionOperating.size[0] = 5000.0;
	robot2d_1.regionOperating.size[1] = 5000.0;
#endif
#ifdef SMALL_ENV
	robot2d_1.regionOperating.center[0] = 500.0;
	robot2d_1.regionOperating.center[1] = 500.0;
	robot2d_1.regionOperating.size[0] = 1000.0;
	robot2d_1.regionOperating.size[1] = 1000.0;
#endif
	robot2d_1.regionGoal.setNumDimensions(2);
	robot2d_1.regionGoal.center[0] = 300000.0;
	robot2d_1.regionGoal.center[1] = 300000.0;
	robot2d_1.regionGoal.size[0] = 2.0;
	robot2d_1.regionGoal.size[1] = 2.0;

	Polygon2_list obs_list = m_objectPolyList;
	Polygon2_list obs_outer_list = m_objectOuterPolyList;

	// insert obstacles into system
	double min_x = 0;
	double max_x = 0;
	double min_y = 0;
	double max_y = 0;

	robot2d_1.obstacles.clear();
	for(auto ob = obs_list.begin(); ob != obs_list.end(); ob++) {
		Polygon_2 * region  = new Polygon_2;
		*region = *ob;
		// obstacle = new region;
		// obstacle->setNumDimensions(2);
		//convertPolygon2Region(*ob, obstacle->center[0], obstacle->center[1], obstacle->size[0], obstacle->size[1]);
		robot2d_1.obstacles.push_back(region);  // Add the obstacle to the list
	}

	
	robot2d_1.generateHashMap();
	// Add the system to the planner
#ifdef USE_RRT
	rrts_1.setSystem(robot2d_1);
#endif
#ifdef USE_PRM
	//prm_1.setSystem(robot2d_1);
#endif
	// Set up the root vertex

#ifdef USE_RRT
	vertex_t &root = rrts_1.getRootVertex();
	State &rootState = root.getState();
	rootState[0] = START_X;
	rootState[1] = START_Y;



	// Initialize the planner
	rrts_1.initialize();

	// This parameter should be larger than 1.5 for asymptotic 
	//   optimality. Larger values will weigh on optimization 
	//   rather than exploration in the RRT* algorithm. Lower 
	//   values, such as 0.1, should recover the RRT.
	rrts_1.setGamma(1.5);
#endif
#ifdef BIG_ENV
#ifdef USE_PRM
	//prm_1.createGraph(8000, 5);
#endif
#ifdef USE_RRT
	for (int i = 0; i < 10000; i++)
	 	rrts_1.iteration();
#endif
#endif
#ifdef SMALL_ENV
	//prm_1.createGraph(2000, 10);
	// for (int i = 0; i < 2000; i++)
	// 	rrts_1.iteration();
#endif
	//generateComb(5);

	std::vector<int> picking_sequence;

	int obj_index = 0;
	int closest_obj_index = 0;
	int local_obj_index = 0;
	int closest_local_index = 0;
	Point_2 closest_grasp;
	double nearest_dist = 10000000;
	double total_dist = 0;
	std::vector<int> pick_obj_sequence;
	Polygon2_list::iterator nearest_obj_iterator = obs_outer_list.begin();
	Polygon2_list::iterator obstaclePolyList_iterator;
	while (obs_outer_list.size() > 0) {
		closest_obj_index = 0;
		local_obj_index = 0;
		obj_index = 0;
		nearest_dist = 10000000;
		std::cout << "obstacle size:" << obs_outer_list.size() << std::endl;
		std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(obs_list, obs_outer_list);
		Environment  *env;
		Visibility_Graph *v_graph;
		//new_buildVisibilityGraph(obs_outer_list, env, v_graph);
		for (auto i = obs_outer_list.begin(); i != obs_outer_list.end(); i++) {
			obj_index = 0;

			for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
				if (bg::equals(*i, *ob)) {
					break;
				}
				obj_index++;
			}
			std::vector<Point_2> grasp_poses = graspCubeToBaseMap[obj_index];
			if (grasp_poses.size() == 0) {
				std::cout << "target object has no graspable poses" << std::endl;
			}
			//std::cout << "object :"<<obj_index<<" grasp_poses:" << grasp_poses.size() << std::endl;
			for (int j = 0; j < grasp_poses.size(); j++) {
				std::list<Point_2 > path;
#ifdef USE_PRM
				//double temp_dist = new_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, prm_1);
#endif
#ifdef USE_RRT
				double temp_dist = new_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_1);
#endif
				//std::cout << "temp_dist:" << temp_dist << " pose:"<< grasp_poses[j].get<0>()<<","<<grasp_poses[j].get<1>()<<std::endl;
				if (temp_dist < nearest_dist) {
					nearest_dist = temp_dist;
					closest_obj_index = obj_index;
					closest_grasp = grasp_poses[j];
					nearest_obj_iterator = i;
					closest_local_index = local_obj_index;
				}
			}
			//obj_index++;
			local_obj_index++;
		}
		std::cout << "pick " << closest_obj_index << " object, grasp pose:" << closest_grasp.get<0>() << "," << closest_grasp.get<1>() << "shortest dist:" << nearest_dist << std::endl;
		total_dist += nearest_dist;
		picking_sequence.push_back(closest_obj_index);
 		obs_outer_list.erase(nearest_obj_iterator);
		obstaclePolyList_iterator = obs_list.begin();
		auto delete_obs = robot2d_1.obstacles.begin();
		int j = 0;
		while (j++ < closest_local_index) {
			obstaclePolyList_iterator++;
			delete_obs++;
		}
		Polygon_2 *erase_region = *delete_obs;
		robot2d_1.obstacles.erase(delete_obs);
		robot2d_1.updateHashMap(erase_region);

#ifdef BIG_ENV
#ifdef USE_RRT
		for (int i = 0; i < 10; i++) {
			if(i == 9){ 
				rrts_1.full_temp_iteration(erase_region);
			}
			else {
				rrts_1.temp_iteration(erase_region);
			}

		}
#endif
#ifdef USE_PRM
		//prm_1.sampleInRegion(*erase_region , 20, 10);
#endif
#endif
#ifdef SMALL_ENV
		for (int i = 0; i < 5; i++)
			rrts_1.temp_iteration(erase_region);
		//prm_1.sampleInRegion(*erase_region , 10, 10);

#endif
		obs_list.erase(obstaclePolyList_iterator);
		//delete erase_region;

	}
#ifdef USE_RRT
	rrts_1.temp_restore();
#endif
#ifdef USE_PRM
	prm_1.restore_temp();
#endif
	std::cout << "total dist:" << total_dist << std::endl;
	return total_dist;
}



double Roadmap::declutterUsingLocalGreedy(Polygon2_list obs_list, Polygon2_list  obs_outer_list){
	planner_t rrts_local;
	System robot2d_local;
	robot2d_local.setNumDimensions(2);
	robot2d_local.regionOperating.setNumDimensions(2);
#ifdef BIG_ENV
	robot2d_local.regionOperating.center[0] = 2500.0;
	robot2d_local.regionOperating.center[1] = 2500.0;
	robot2d_local.regionOperating.size[0] = 5000.0;
	robot2d_local.regionOperating.size[1] = 5000.0;
#endif
#ifdef SMALL_ENV
	robot2d_local.regionOperating.center[0] = 500.0;
	robot2d_local.regionOperating.center[1] = 500.0;
	robot2d_local.regionOperating.size[0] = 1000.0;
	robot2d_local.regionOperating.size[1] = 1000.0;
#endif
	robot2d_local.regionGoal.setNumDimensions(2);
	robot2d_local.regionGoal.center[0] = 300000.0;
	robot2d_local.regionGoal.center[1] = 300000.0;
	robot2d_local.regionGoal.size[0] = 2.0;
	robot2d_local.regionGoal.size[1] = 2.0;
	

	// insert obstacles into system
	double min_x = 0;
	double max_x = 0;
	double min_y = 0;
	double max_y = 0;


	robot2d_local.obstacles.clear();
	for (auto ob = obs_list.begin(); ob != obs_list.end(); ob++) {
		// region *obstacle;
		Polygon_2 * obstacle = new Polygon_2;
		*obstacle = *ob;
		// obstacle = new region;
		// obstacle->setNumDimensions(2);
		//convertPolygon2Region(*ob, obstacle->center[0], obstacle->center[1], obstacle->size[0], obstacle->size[1]);
		robot2d_local.obstacles.push_back(obstacle);  // Add the obstacle to the list
	}
	
	robot2d_local.generateHashMap();
	// Add the system to the planner
	rrts_local.setSystem(robot2d_local);

	// Set up the root vertex
	vertex_t &root = rrts_local.getRootVertex();
	State &rootState = root.getState();
	rootState[0] = START_X;
	rootState[1] = START_Y;



	// Initialize the planner
	rrts_local.initialize();

	// This parameter should be larger than 1.5 for asymptotic 
	//   optimality. Larger values will weigh on optimization 
	//   rather than exploration in the RRT* algorithm. Lower 
	//   values, such as 0.1, should recover the RRT.
	rrts_local.setGamma(1.5);
#ifdef BIG_ENV
	for (int i = 0; i < 1000; i++)
		rrts_local.iteration();
#endif
#ifdef SMALL_ENV
	for (int i = 0; i < 2000; i++)
		rrts_local.iteration();
#endif
	//generateComb(5);

	std::vector<int> picking_sequence;

	int obj_index = 0;
	int closest_obj_index = 0;
	int local_obj_index = 0;
	int closest_local_index = 0;
	Point_2 closest_grasp;
	double nearest_dist = 10000000;
	double total_dist = 0;
	std::vector<int> pick_obj_sequence;
	Polygon2_list::iterator nearest_obj_iterator;
	Polygon2_list::iterator obstaclePolyList_iterator;
	while (obs_outer_list.size() > 0) {
		closest_obj_index = 0;
		local_obj_index = 0;
		obj_index = 0;
		nearest_dist = 10000000;
		std::cout << "obstacle size:" << obs_outer_list.size() << std::endl;
		std::map<int, std::vector<Point_2>> graspCubeToBaseMap = new_findGraspablePoses(obs_list, obs_outer_list);
		Environment  *env;
		Visibility_Graph *v_graph;
		//new_buildVisibilityGraph(obs_outer_list, env, v_graph);
		for (auto i = obs_outer_list.begin(); i != obs_outer_list.end(); i++) {
			obj_index = 0;
			
			for (auto ob = m_objectOuterPolyList.begin(); ob != m_objectOuterPolyList.end(); ob++) {
				if (bg::equals(*i, *ob)) {
					break;
				}
				obj_index++;
			}
			std::vector<Point_2> grasp_poses = graspCubeToBaseMap[obj_index];
			for (int j = 0; j < grasp_poses.size(); j++) {
				std::list<Point_2 > path;
				double temp_dist = new_computeShortestPath(START_X, START_Y, grasp_poses[j].get<0>(), grasp_poses[j].get<1>(), path, rrts_local);
				//std::cout << "temp_dist:" << temp_dist << " pose:"<< grasp_poses[j].get<0>()<<","<<grasp_poses[j].get<1>()<<std::endl;
				if (temp_dist < nearest_dist) {
					nearest_dist = temp_dist;
					closest_obj_index = obj_index;
					closest_grasp = grasp_poses[j];
					nearest_obj_iterator = i;
					closest_local_index = local_obj_index;
				}
			}
			//obj_index++;
			local_obj_index++;
		}
		std::cout << "pick " << closest_obj_index << " object, grasp pose:" << closest_grasp.get<0>() << "," << closest_grasp.get<1>() << "shortest dist:" << nearest_dist << std::endl;
		total_dist += nearest_dist;
		picking_sequence.push_back(closest_obj_index);
		obs_outer_list.erase(nearest_obj_iterator);
		obstaclePolyList_iterator = obs_list.begin();
		auto delete_obs = robot2d_local.obstacles.begin();
		int j = 0;
		while (j++ < closest_local_index) {
			obstaclePolyList_iterator++;
			delete_obs++;
		}
		Polygon_2* erase_region = *delete_obs;
		robot2d_local.obstacles.erase(delete_obs);
		robot2d_local.updateHashMap(erase_region);
#ifdef BIG_ENV
		for (int i = 0; i < 10; i++)
			rrts_local.temp_iteration(erase_region);
#endif
#ifdef SMALL_ENV
		for (int i = 0; i < 5; i++)
			rrts_local.temp_iteration(*erase_region);
#endif
		obs_list.erase(obstaclePolyList_iterator);
		delete erase_region;
	}
	rrts_local.temp_restore();

	std::cout << "total dist:" << total_dist << std::endl;
	return total_dist;
}
		



























Segment_2 Roadmap::pointPairToSegment(std::pair<int, int> e){
	Point_2 first = m_vidPointMap[e.first];
	Point_2 second = m_vidPointMap[e.second];
	Segment_2 seg(first, second);
	return seg;
}



int Roadmap::intersectObsAndCycle(Polygon_2 & poly, Graph & g, std::vector<Point_2>& v_list, std::vector<std::pair<int, int>>& cycleEdge_list, 
	std::vector<Segment_2>& polyEdge_list){
	int interNum = 0;
	std::vector<std::pair<int , int>> cycleVector;
	std::vector<std::pair<int, int>> sorted_cycleVector;
    g.getCycleEdgeVector(cycleVector);
    int farestIndex = 0;
    double maxDist = 0;

    // make sure that the detected point list starts from the side rather than the middle of all the detected points.
    // easy to identify the starting and ending crossing points and connect them along the obstacle boundary.
    // The process is based on the idea that the farthest edge of the surrounding cycle is not in the middle of all the 
    // crossing points.

    for(int k = 0; k < cycleVector.size(); k++){
    	Segment_2 segDist = pointPairToSegment(cycleVector[k]);
    	double dist = bg::distance(segDist, poly);
    	if(dist > maxDist){
    		farestIndex = k;
    	}
    }
	for(int q = 0; q < cycleVector.size(); q++){
		if(farestIndex + q < cycleVector.size()){
			sorted_cycleVector.push_back(cycleVector[farestIndex+q]);
		}else{
			int index = (farestIndex + q) % cycleVector.size();
			sorted_cycleVector.push_back(cycleVector[index]);
		}
	}    

	for(int j = 0; j < sorted_cycleVector.size(); j ++){

		Segment_2 segCycle = pointPairToSegment(sorted_cycleVector[j]);
		for(int i = 0; i < poly.outer().size() - 1; i++){
    		Segment_2 seg(poly.outer()[i], poly.outer()[i+1]);	
    		std::vector<Point_2> output;
    		if(bg::intersection(seg, segCycle, output)){
    			if(output.size() > 0 && output.size() == 1){

    				v_list.push_back(output[0]);
    				cycleEdge_list.push_back(sorted_cycleVector[j]);
    				polyEdge_list.push_back(seg);
    				interNum++;

    			}
    		}

    	}
		
	}
	return interNum;

}


double Roadmap::getAddingX(Point_2 end, Point_2 start){
	return (end.get<0>() - start.get<0>()) / bg::distance(start, end);
}

double Roadmap::getAddingY(Point_2 end, Point_2 start){
	return (end.get<1>() - start.get<1>()) / bg::distance(start, end);
}

void Roadmap::reverseVector(std::vector<int> & vect){
	int size = vect.size();
	int pivot;
	int temp;
	if(size % 2 == 0){
		pivot = size / 2;
		for(int i = 0; i < pivot; i++){
			temp = vect[i];
			vect[i] = vect[size-i-1];
			vect[size-i-1] = temp;
		}
	}else{
		pivot = (size / 2);
		for(int i = 0; i < pivot; i++){
			temp = vect[i];
			vect[i] = vect[size-i-1];
			vect[size-i-1] = temp;
		}
	}
}

double Roadmap::shortestDistPair(Segment_2 seg, Point_2 p, Point_2 &closestPt){
  	Point_2 start = seg.first;
  	Point_2 end = seg.second;
  	Point_2 v1, v2;
  	// Return minimum distance between line segment vw and point p
  	const double L_2 = pow(start.get<0>() - end.get<0>(), 2) + pow(start.get<1>() - end.get<1>(), 2);// i.e. |w-v|^2 -  avoid a sqrt
  	if (L_2 == 0.0) return bg::distance(p, start);   // v == w case
  	// Consider the line extending the segment, parameterized as v + t (w - v).
  	// We find projection of point p onto the line. 
  	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
  	// We clamp t from [0,1] to handle points outside the segment vw.
  
  	v1.set<0>(p.get<0>()-start.get<0>());
  	v1.set<1>(p.get<1>()-start.get<1>());
  	v2.set<0>(end.get<0>()-start.get<0>());
  	v2.set<1>(end.get<1>()-start.get<1>());
  	const double t = rmax(0.0, rmin(1.0, dot(v1, v2) / L_2));
  	closestPt.set<0>(start.get<0>() + t*(end.get<0>()-start.get<0>()));
  	closestPt.set<1>(start.get<1>() + t*(end.get<1>()-start.get<1>()));
  	//const vec2 projection = v + t * (w - v);   Projection falls on the segment
  	return bg::distance(p, closestPt);
}

double Roadmap::rmax(double v1, double v2){
	if(v1 < v2)
		return v2;
	else
		return v1;
}

bool Roadmap::isNearBoundary(Segment_2 test){
	Point_2 p_s = test.first;
	Point_2 p_e = test.second;
	if(p_s.get<1>() < 60 || p_s.get<1>() > 740 || p_s.get<0>() < 60 || p_s.get<0>() > 1140){
		return true;
	}
	if(p_e.get<1>() < 60 || p_e.get<1>() > 740 || p_e.get<0>() < 60 || p_e.get<0>() > 1140){
		return true;
	}
	return false;
}

double Roadmap::rmin(double v1, double v2){
	if(v1 < v2)
		return v1;
	else
		return v2;
}




void Roadmap::recoverConnectivity_3(int& vIDCount){
	int vid = 2*n_w*n_h;
	double slack = m_edgeLength / 50;
	double step = m_edgeLength / 12;
	int i = 0;
	int poly_num = 0;
	
	QPen vertexPen = QPen(Qt::blue, 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	//if(m_objectPolyList.size()>3){		
	pointAlongPoly pAP;
	Point_2 startPivot, endPivot, startPivotNext, endPivotNext;
	Point_2 startPt, endPt, startCyclePt, endCyclePt;
	Graph finalJointCycle, tempJointCycle, newCycle, currentCycle;

	Polygon_2 currentPoly, shortestPoly;
	bool build_complete = false;
	bool finish_one = false;

	for(std::vector<Graph*>::iterator g = m_obsBoundingCycleVec.begin(); g != m_obsBoundingCycleVec.end(); g++){
		m_gidGraphMap[i] = *(*g);
		m_graphGidMap[*(*g)] = i;
		m_isGraphUsedMap[i] = false;
		i++;
	}
	int j = 0;
	int tempGid = 0;
	std::vector<int> currentJointCycleGid_vector;
	while(! build_complete){
		j = 0;	
		finish_one = false;	
		currentJointCycleGid_vector.clear();
		while(!finish_one){ 
			for(std::vector<Graph*>::iterator g = m_obsBoundingCycleVec.begin(); g != m_obsBoundingCycleVec.end(); g++){
				newCycle = *(*g);
				tempGid = m_graphGidMap[newCycle];
				if(!m_isGraphUsedMap[tempGid]){
					if(j == 0){
						tempJointCycle = *(*g);
						tempGid = m_graphGidMap[*(*g)];
						currentJointCycleGid_vector.push_back(tempGid);
						m_isGraphUsedMap[tempGid] = true;
						j++;
					}else{
						newCycle = *(*g);
						if(isGraphIntersection(tempJointCycle, newCycle)){
							cout<<"bounding graph has intersection"<<endl;
							tempJointCycle = getJointCycle(tempJointCycle, newCycle);	
							tempGid = m_graphGidMap[newCycle];
							currentJointCycleGid_vector.push_back(tempGid);							
							m_isGraphUsedMap[tempGid] = true;
							j++;
						}else{
							cout<<"bounding graph no intersection"<<endl;
						}
					}
				}	
			}		
			bool no_new_intersection = true;
			for(std::vector<Graph*>::iterator g = m_obsBoundingCycleVec.begin(); g != m_obsBoundingCycleVec.end(); g++){
				newCycle = *(*g);
				tempGid = m_graphGidMap[newCycle];
				if(!m_isGraphUsedMap[tempGid]){
					if(isGraphIntersection(tempJointCycle, *(*g))){
						std::cout<<"still has graph connected"<<std::endl;
						no_new_intersection = false;
						break;
					}		
				}	
					
			}		
			if(no_new_intersection){
				finish_one = true;
				m_jointCycleVector.push_back(tempJointCycle);
				for(int k = 0; k < currentJointCycleGid_vector.size();k++){
					m_gidJointCycleMap[currentJointCycleGid_vector[k]] = tempJointCycle;
				}
			}		
		}		
		build_complete = true;
		for(std::vector<Graph*>::iterator g = m_obsBoundingCycleVec.begin(); g != m_obsBoundingCycleVec.end(); g++){
			newCycle = *(*g);
			tempGid = m_graphGidMap[newCycle];
			if(!m_isGraphUsedMap[tempGid]){
				build_complete = false;
				break;
			}
		}	
	}				









	// for(std::vector<Graph*>::iterator g = m_obsBoundingCycleVec.begin(); g != m_obsBoundingCycleVec.end(); g++){
	// 	if(i == 0){
	// 		tempJointCycle = *(*g);
	// 	}else{
	// 		newCycle = *(*g);
	// 		tempJointCycle = getJointCycle(tempJointCycle, newCycle);
	// 	}		i++;

	// }
	// m_jointCycle = tempJointCycle;



	for(int i = 0; i < m_jointCycleVector.size(); i++){
		for(std::set<int>::iterator vi = m_jointCycleVector[i].getVertexSet().begin(); vi != m_jointCycleVector[i].getVertexSet().end(); vi ++){
			m_pointUsed[*vi] = false;
		}
	}

	std::vector<Segment_2> surEdgeList;
	std::vector<std::vector<Segment_2>> vectorEdgeList;
	int pid = 0;
	for(Polygon2_list::iterator obsit = m_objectPolyList.begin(); obsit != m_objectPolyList.end(); obsit++){
		currentPoly = *obsit;
		surEdgeList = getSurroundingEdges(currentPoly, poly_num);
		vectorEdgeList.push_back(surEdgeList);
		int cross_nums = 0;
		poly_num ++;

		Point_2 startPt_outer, startPt_inner, endPt_outer, endPt_inner;
		std::vector<Point_2> outerList, innerList;
		outerList.resize(4);
		innerList.resize(4);
		for(int i = 0;i < surEdgeList.size();i++){
			Point_2 p_s = surEdgeList[i].first;
			Point_2 p_e = surEdgeList[i].second;
			if(isNearBoundary(surEdgeList[i])){
				continue;
			}
			if(bg::distance(p_s, currentPoly) > m_edgeLength*1.0 && bg::distance(p_e, currentPoly) < m_edgeLength*1.0 ){
				mm_scene->addEllipse(p_e.get<0>() - 1, p_e.get<1>() - 1, 0.5, 0.5, vertexPen);
				outerList[cross_nums] = p_s;
				innerList[cross_nums] = p_e;
				cross_nums ++;


				if(cross_nums == 1){
					startPt_outer = p_s;
					startPt_inner = p_e;
				}else if(cross_nums == 2){
					endPt_inner = p_e;
					endPt_outer = p_s;
				}
			}else if(bg::distance(p_s, currentPoly) < m_edgeLength*1.0 && bg::distance(p_e, currentPoly) > m_edgeLength*1.0 ){
				mm_scene->addEllipse(p_s.get<0>() - 1, p_s.get<1>() - 1, 0.5, 0.5, vertexPen);
				outerList[cross_nums] = p_e;
				innerList[cross_nums] = p_s;
				cross_nums ++;
				if(cross_nums == 1){
					startPt_outer = p_e;
					startPt_inner = p_s;
				}else if(cross_nums == 2){
					endPt_inner = p_s;
					endPt_outer = p_e;
				}
			}

		}
		if(cross_nums == 4){
			std::vector<Point_2> pivot;
			std::vector<Point_2> pivotNext;
			pivot.resize(4);
			pivotNext.resize(4);
			pointAlongPoly pAP;
			std::vector<Polygon_2> oppoPolyList;
			Point_2 startPt, endPt, startCyclePt, endCyclePt;
			Polygon_2 oppo_poly, oppo_poly_1, oppo_poly_2, oppo_poly_3, oppo_poly_4;
			double tempDistance;
			double shortestDist = std::numeric_limits<double>::max();
			for(Polygon2_list::iterator po = m_objectPolyList.begin(); po != m_objectPolyList.end(); po++){
				if(!bg::equals(*po, currentPoly)){
					oppoPolyList.push_back(*po);
					tempDistance = bg::distance(*po, currentPoly);
					if(tempDistance < shortestDist){
						shortestDist = tempDistance;
						oppo_poly = *po;
					}
				}
			}
			for (Polygon2_list::iterator po = m_objectPolyList.begin(); po != m_objectPolyList.end(); po++) {
				if (!bg::equals(*po, currentPoly)) {
					oppoPolyList.push_back(*po);
					tempDistance = bg::distance(*po, currentPoly);
					if (tempDistance < shortestDist) {
						shortestDist = tempDistance;
						oppo_poly = *po;
					}
				}
			}
			getShortestBetweenTwoPtPoly(currentPoly, outerList[0], innerList[0], pivot[0], pivotNext[0], oppo_poly);
			getShortestBetweenTwoPtPoly(currentPoly, outerList[1], innerList[1], pivot[1], pivotNext[1], oppo_poly);
			getShortestBetweenTwoPtPoly(currentPoly, outerList[2], innerList[2], pivot[2], pivotNext[2], oppo_poly);
			getShortestBetweenTwoPtPoly(currentPoly, outerList[3], innerList[3], pivot[3], pivotNext[3], oppo_poly);
			std::cout << "outerList[3]: " <<outerList[3].get<0>() << "," << outerList[3].get<1>() << " " << "innerList[3]:" << innerList[3].get<0>() << "," << innerList[3].get<1>() << std::endl;
			std::cout << "Pivot[0]: " << pivot[0].get<0>() << "," << pivot[0].get<1>() << " " << "PivotNext[0]:" << pivotNext[0].get<0>() << "," <<pivotNext[0].get<1>() << std::endl;
			std::cout << "Pivot[1]: " << pivot[1].get<0>() << "," << pivot[1].get<1>() << " " << "PivotNext[1]:" << pivotNext[1].get<0>() << "," << pivotNext[1].get<1>() << std::endl;
			std::cout << "Pivot[2]: " << pivot[2].get<0>() << "," << pivot[2].get<1>() << " " << "PivotNext[2]:" << pivotNext[2].get<0>() << "," << pivotNext[2].get<1>() << std::endl;
			std::cout << "Pivot[3]: " << pivot[3].get<0>() << "," << pivot[3].get<1>() << " " << "PivotNext[3]:" << pivotNext[3].get<0>() << "," << pivotNext[3].get<1>() << std::endl;

			findStartEndPair(currentPoly, pivot, pivotNext, startPivot, startPivotNext, endPivot, endPivotNext);
			std::cout << "startPivot: " << startPivot.get<0>() << "," << startPivot.get<1>() << " " << "startPivotNext:" << startPivotNext.get<0>() << "," << startPivotNext.get<1>() << std::endl;
			std::cout << "endPivot: " << endPivot.get<0>() << "," << endPivot.get<1>() << " " << "endPivotNext:" << endPivotNext.get<0>() << "," << endPivotNext.get<1>() << std::endl;
			pAP = getStartEndAroundPoly_2(currentPoly, oppo_poly, m_gidJointCycleMap[pid], startPivot, startPivotNext, endPivot, endPivotNext, startPt, endPt, startCyclePt, endCyclePt, step,  oppoPolyList);
			connectAlongPoly(currentPoly, startPt, endPt, startCyclePt, endCyclePt, pAP, vid, vIDCount);

			findStartEndPair(currentPoly, pivot, pivotNext, startPivot, startPivotNext, endPivot, endPivotNext);
			std::cout << "startPivot: " << startPivot.get<0>() << "," << startPivot.get<1>() << " " << "startPivotNext:" << startPivotNext.get<0>() << "," << startPivotNext.get<1>() << std::endl;
			std::cout << "endPivot: " << endPivot.get<0>() << "," << endPivot.get<1>() << " " << "endPivotNext:" << endPivotNext.get<0>() << "," << endPivotNext.get<1>() << std::endl;
			pAP = getStartEndAroundPoly_2(currentPoly, oppo_poly, m_gidJointCycleMap[pid], startPivot, startPivotNext, endPivot, endPivotNext, startPt, endPt, startCyclePt, endCyclePt, step,  oppoPolyList);
			connectAlongPoly(currentPoly, startPt, endPt, startCyclePt, endCyclePt, pAP, vid, vIDCount);
		}
		if(cross_nums == 2){	
			pointAlongPoly pAP;				
					
			std::vector<Polygon_2> oppoPolyList;
			Point_2 startPt, endPt, startCyclePt, endCyclePt;
			Polygon_2 oppo_poly;
			double tempDistance;
			double shortestDist = std::numeric_limits<double>::max();
			for (Polygon2_list::iterator po = m_objectPolyList.begin(); po != m_objectPolyList.end(); po++) {
				if (!bg::equals(*po, currentPoly)) {
					oppoPolyList.push_back(*po);
					tempDistance = bg::distance(*po, currentPoly);
					std::cout << "tempDistance:" << tempDistance << std::endl;
					if (tempDistance < shortestDist) {
						shortestDist = tempDistance;
						oppo_poly = *po;
						std::cout << "oppo_poly size: " << oppo_poly.outer().size() << std::endl;		
					}
				}

			}
				std::cout << "startPt_outer: " << startPt_outer.get<0>() << "," << startPt_outer.get<1>() << " " << "startPt_inner:" << startPt_inner.get<0>() << "," << startPt_inner.get<1>() << std::endl;
		
				std::cout << "endPt_outer: " << endPt_outer.get<0>() << "," << endPt_outer.get<1>() << " " << "endPt_inner:" << endPt_inner.get<0>() << "," << endPt_inner.get<1>() << std::endl;
				getShortestBetweenTwoPtPoly(currentPoly, startPt_outer, startPt_inner, startPivot, startPivotNext, oppo_poly);
				getShortestBetweenTwoPtPoly(currentPoly, endPt_outer, endPt_inner, endPivot, endPivotNext, oppo_poly);
				std::cout << "startPivot: " << startPivot.get<0>() << "," << startPivot.get<1>() << " " << "startPivotNext:" << startPivotNext.get<0>() << "," << startPivotNext.get<1>() << std::endl;
				std::cout << "endPivot: " << endPivot.get<0>() << "," << endPivot.get<1>() << " " << "endPivotNext:" << endPivotNext.get<0>() << "," << endPivotNext.get<1>() << std::endl;

				pAP = getStartEndAroundPoly_2(currentPoly, oppo_poly, m_gidJointCycleMap[pid], startPivot, startPivotNext, endPivot, endPivotNext, startPt, endPt, startCyclePt, endCyclePt, step, oppoPolyList);
				connectAlongPoly(currentPoly, startPt, endPt, startCyclePt, endCyclePt, pAP, vid, vIDCount);
			
		}
		std::cout<<"cross_nums:"<<cross_nums<<std::endl;
		pid ++;
	}

	m_gidJointCycleMap.clear();
	m_jointCycleVector.clear();
m_obsBoundingCycleVec.clear();
m_isGraphUsedMap.clear();
m_newAddedSeg.clear();

	/*
	// this segment code is for adding extra connection by inserting pivot point in the joint spot
		bool pass = true;
		std::vector<int> newAddedFGList;
		m_newConnection.clear();
		std::vector<Point_2> jointPtList;
		for (voronoi_diagram<double>::const_vertex_iterator it = m_vd.vertices().begin(); it != m_vd.vertices().end(); ++it) {
    		const voronoi_diagram<double>::vertex_type &vert = *it;
    		int OnNum = 0;
    		for(int i = 0;i < vectorEdgeList.size();i++){
    			std::vector<Segment_2> edgeList = vectorEdgeList[i];
    			if(isVertexOnEdgeList(vert, edgeList)){
    				OnNum++;
    			}
    		}
    		if(OnNum == 3){
    			std::cout<<"found one"<<std::endl;
    			std::cout<<vert.x()<<","<<vert.y()<<std::endl;
    			Point_2 p(vert.x(),vert.y());
    			jointPtList.push_back(p);
    		}
    	}
    	for(int k = 0; k < jointPtList.size(); k++){
    		m_vidPointMap[vid] = jointPtList[k];
			m_pointVidMap[jointPtList[k]] = vid;
			m_vidFGMap[vIDCount] = vid; 
			m_vidGFMap[vid] = vIDCount;
			std::vector<Point_2> singleConnectList;
			std::vector<Point_2> possiblePtList;
    		for(int j = 0; j < m_newAddedPt.size();j++){
				int observeID = m_newAddedPt[j];
				Point_2 observePt = m_vidPointMap[m_vidFGMap[observeID]];
				Point_2 targetPt = jointPtList[k];

				if(bg::distance(observePt, jointPtList[k]) < 1.3*m_edgeLength && bg::distance(observePt, jointPtList[k]) > 0.9*m_edgeLength){
					possiblePtList.push_back(observePt);
					// for(int q = 0;q < singleConnectList.size();q++){
					// 	if(calculateAngle(jointPtList[k], singleConnectList[q], observePt)>0){
					// 		pass = false;
					// 		break;
					// 	}
					// }

					
					// for(int k = 0;k < m_newConnection.size();k++){
					// 	if(bg::distance(Segment_2(observePt, targetPt), m_newConnection[k])< m_edgeLength*1.25){
					// 		pass = false;
					// 		break;
					// 	}
					// }
					//m_newConnection.push_back(Segment_2(observePt, targetPt));



					// if(pass==true){
					// 	singleConnectList.push_back(observePt);
					// 	m_finalGraph.addEdge(observeID, vIDCount);
					// 	newAddedFGList.push_back(m_vidFGMap[observeID]);
					// 	newAddedFGList.push_back(vid);
					// 	addEdgeFGMap(observeID, vIDCount, newAddedFGList);
					// 	m_graph.addEdge(m_vidFGMap[observeID], vid);		
					// }else{
					// 	continue;
					// }
					
				}
			}
			for(int r = 0;r < 10;r++){
				singleConnectList.clear();
				std::random_shuffle(possiblePtList.begin(), possiblePtList.end());
				for(int j = 0;j < possiblePtList.size();j++){
					Point_2 observePt = possiblePtList[j];
					int observeID = m_vidGFMap[m_pointVidMap[observePt]];

					if(singleConnectList.size()==0){
						pass = true;
					}else{
						for(int q = 0;q < singleConnectList.size();q++){
							if(calculateAngle(jointPtList[k], singleConnectList[q], observePt)>-0.05){
								pass = false;
								break;
							}
						}
					}
					

					if(pass==true){
						singleConnectList.push_back(observePt);
						
					}else{
						continue;
					}
				}
				if(singleConnectList.size()>1){
					break;
				}
			}
			for(int i = 0;i < singleConnectList.size();i++){
				newAddedFGList.clear();
				int observeID = m_vidGFMap[m_pointVidMap[singleConnectList[i]]];
				std::set<int> nei_g = m_graph.getNeighborSet(m_vidFGMap[observeID]);
				std::set<int> nei_f = m_finalGraph.getNeighborSet(observeID);
				m_finalGraph.addEdge(observeID, vIDCount);
				newAddedFGList.push_back(m_vidFGMap[observeID]);
				newAddedFGList.push_back(vid);
				addEdgeFGMap(observeID, vIDCount, newAddedFGList);
				m_graph.addEdge(m_vidFGMap[observeID], vid);		
				for(std::set<int>::iterator it = nei_g.begin();it != nei_g.end();it++){
					std::cout<<"all possible:"<<vIDCount<<":"<<*it<<":"<<observeID<<std::endl;	
					if(calculateAngle(singleConnectList[i], jointPtList[k], m_vidPointMap[*it])>-0.2){
						for(std::set<int>::iterator fit = nei_f.begin();fit != nei_f.end();fit++){
							std::pair<int, int> nei_p(*fit, observeID);
							std::vector<int> g_set = m_edgeFGMap[nei_p];
							for(int q = 0; q < g_set.size();q++){
								if(g_set[q]==*it){
										std::pair<std::pair<int,int>, int> pai;
								pai.second = observeID;
								std::pair<int,int> p1(vIDCount, *fit);
								pai.first = p1;
								m_specialPair.push_back(pai);		
								std::cout<<vIDCount<<":"<<*fit<<":"<<observeID<<std::endl;		
							
								}
							}
							
							
						}
							
					}
					
				}

				
			}
			
			
			vid++;
			vIDCount++;
    	}
    	above code is for adding extra connection by inserting pivot point in the joint spot
		*/
	/*
// this code is for searching for available extra connection based on existing connections

    	pass = true;
		newAddedFGList.clear();
		for(int i = 0; i < m_newAddedPt.size();i++){
			int observeID = m_newAddedPt[i];
			Point_2 observePt = m_vidPointMap[m_vidFGMap[observeID]];

			Point_2 targetPt;
			for(int j = 0;j < m_newAddedPt.size();j ++){
				pass = true;
				newAddedFGList.clear();
				targetPt = m_vidPointMap[m_vidFGMap[m_newAddedPt[j]]];
				if(j != i && isTwoPtInTwoPoly(observePt, targetPt, m_objectPolyList)){
					
					if(bg::distance(observePt, targetPt) < 1.3*m_edgeLength && bg::distance(observePt, targetPt) > 0.7*m_edgeLength){
						for(Polygon2_list::iterator obsit = m_objectPolyList.begin(); obsit != m_objectPolyList.end(); obsit++){
							Linestring_2 tempL;
							bg::append(tempL, targetPt);
							bg::append(tempL, observePt);
							if(bg::crosses(tempL, *obsit)){
								pass = false;
								break;
							}
						}


						for(int k = 0;k < m_newConnection.size();k++){
							if(bg::distance(Segment_2(observePt, targetPt), m_newConnection[k])< m_edgeLength*1.25){
								pass = false;
								break;
							}
						}
						if(pass == false){
							continue;			
						}else{
							m_newConnection.push_back(Segment_2(observePt, targetPt));
							m_finalGraph.addEdge(observeID, m_newAddedPt[j]);
							newAddedFGList.push_back(m_vidFGMap[observeID]);
							newAddedFGList.push_back(m_vidFGMap[m_newAddedPt[j]]);
							addEdgeFGMap(observeID, m_newAddedPt[j], newAddedFGList);
							m_graph.addEdge(m_vidFGMap[observeID], m_vidFGMap[m_newAddedPt[j]]);
						}
					}

				}
				
			}
		}
// above code is for searching for available extra connection based on existing connections

	*/

}


bool Roadmap::isVertexOnEdgeList(voronoi_diagram<double>::vertex_type v, std::vector<Segment_2> &edgeList){
	Point_2 p;
	p.set<0>(v.x());
	p.set<1>(v.y());
	for(int i = 0;i < edgeList.size();i++){
		if(isPointInSeg(p, edgeList[i])){
			return true;
		}
	}
	return false;
}

bool Roadmap::isTwoPtInTwoPoly(Point_2 observePt, Point_2 targetPt,Polygon2_list& m_objectPolyList){
	int i = 0;
	int observe_int = -1;
	int target_int = -1;
	for(Polygon2_list::iterator obsit = m_objectPolyList.begin(); obsit != m_objectPolyList.end(); obsit++){
		if(bg::intersects(observePt, *obsit) && bg::intersects(targetPt, *obsit)){
			return false;
		}
		if(isPtOnPoly(observePt, *obsit)){
			observe_int = i;
		}
		if(isPtOnPoly(targetPt, *obsit)){
			target_int = i;
		}
		i++;
	}
	if(observe_int == -1){
		return false;
	}
	if(target_int == -1){
		return false;
	}
	if(target_int == observe_int){
		return false;
	}
	return true;
}

void Roadmap::findStartEndPair(Polygon_2 poly, std::vector<Point_2>& pivot, std::vector<Point_2>& pivotNext, Point_2 &startPivot, Point_2 &startPivotNext, Point_2 &endPivot, Point_2 &endPivotNext){
	int size = pivot.size();
	Point_2 start = pivot[0];
	Point_2 startPrevious = pivotNext[0];
	Point_2 tempPt, tempPreviousPt;
	int index;
	//tempPt = start;
	//tempPreviousPt = startPrevious;
	do{
		tempPt = nextInPolygon(start, startPrevious, poly, "367");
		startPrevious = start;
		start = tempPt;
	}while(!isOneOfList(start, pivot, index));
	//nextInPolygon(Point_2 currentPoint, Point_2 previousPoint, Polygon_2 poly, std::string str)
	//startPivot, Point_2 &startPivotNext, Point_2 &endPivot, Point_2 &endPivotNext
	startPivot = pivot[0];
	startPivotNext = pivotNext[0];
	endPivot = start;
	endPivotNext = pivotNext[index];
	pivot.erase(pivot.begin()+index);
	pivotNext.erase(pivotNext.begin()+index);
	pivot.erase(pivot.begin());
	pivotNext.erase(pivotNext.begin());

}

bool Roadmap::isOneOfList(Point_2 testPt, std::vector<Point_2> pivotList, int& index){
	for(int i = 0;i < pivotList.size(); i++){
		if(bg::equals(testPt, pivotList[i])){
			index = i;
			return true;
		}
	}
	return false;
}

void Roadmap::recoverConnectivity_2(int& vIDCount){
	int vid = 2*n_w*n_h;
	double slack = m_edgeLength / 50;
	double step = m_edgeLength / 12;
	Polygon_2 poly_1, poly_2, poly_3;
	Graph cycle_1, cycle_2, cycle_3;
	Point_2 point_1, point_2, point_3;
	Segment_2 intersectSeg, intersectSeg_12, intersectSeg_13, intersectSeg_23;
	int shortestPtIndex, shortestPtIndex_12, shortestPtIndex_13, shortestPtIndex_23; 
	int i = 0;
	Point_2 shortestPt, shortestPt_12, shortestPt_13, shortestPt_23, shortestPolyPt, shortestPolyPt_12, shortestPolyPt_13, shortestPolyPt_23;
	Point_2 leftNextPt_1, leftNextPt_2, rightNextPt_1, rightNextPt_2, leftNextPt_3, rightNextPt_3;
	Graph jointCycle, jointCycle_12, jointCycle_123;
	Segment_2 shortestSeg;
	std::vector<Polygon_2> oppoPolyList;
	bool hasPoly_1, hasPoly_2, hasPoly_3;
	// shortestPolyPt : the vertex point of one of the polygon
	// shortestPt : the point in the segment of another polygon
	double shortestDist = std::numeric_limits<double>::max();
	double shortestDist_12 = std::numeric_limits<double>::max();
	double shortestDist_13 = std::numeric_limits<double>::max();
	double shortestDist_23 = std::numeric_limits<double>::max();
	hasPoly_1 = false;
	hasPoly_2 = false;
	hasPoly_3 = false;
	m_newAddedSeg.clear();
	if(m_objectPolyList.size()>3){				
		pointAlongPoly pAP;
		Point_2 startPt, endPt, startCyclePt, endCyclePt;
		Graph finalJointCycle, tempJointCycle, newCycle, currentCycle;
		Polygon_2 currentPoly, shortestPoly;
		for(std::vector<Graph*>::iterator g = m_obsBoundingCycleVec.begin(); g != m_obsBoundingCycleVec.end(); g++){
			if(i == 0){
				tempJointCycle = *(*g);
			}else{
				newCycle = *(*g);
				tempJointCycle = getJointCycle(tempJointCycle, newCycle);
			}
			i++;
		}
		m_jointCycle = tempJointCycle;
		for(std::set<int>::iterator vi = m_jointCycle.getVertexSet().begin(); vi != m_jointCycle.getVertexSet().end(); vi ++){
				m_pointUsed[*vi] = false;
			}
		for(Polygon2_list::iterator obsit = m_objectPolyList.begin(); obsit != m_objectPolyList.end(); obsit++){
			currentPoly = *obsit;
			double tempDistance;
			oppoPolyList.clear();
			
			shortestDist = std::numeric_limits<double>::max();
			for(Polygon2_list::iterator po = m_objectPolyList.begin(); po != m_objectPolyList.end(); po++){
				if(!bg::equals(*po, currentPoly)){
					oppoPolyList.push_back(*po);
					tempDistance = bg::distance(*po, currentPoly);
					if(tempDistance < shortestDist){
						shortestDist = tempDistance;
						shortestPoly = *po;
					}
				}
			}
			shortestDist = std::numeric_limits<double>::max();
			getShortestBetweenTwoPoly(currentPoly, shortestPoly, shortestDist, shortestPt, shortestPolyPt, intersectSeg, shortestPtIndex);

			getPivotBetweenPoly(currentPoly, shortestPoly, shortestSeg, intersectSeg, shortestPt, shortestPolyPt, shortestPtIndex, point_1, point_2, leftNextPt_1, rightNextPt_1, leftNextPt_2, rightNextPt_2);

			pAP = getStartEndAroundPoly(currentPoly, shortestPoly, shortestSeg, m_jointCycle, point_1, leftNextPt_1, rightNextPt_1, startPt, endPt, startCyclePt, endCyclePt, step, oppoPolyList);
			connectAlongPoly(currentPoly, startPt, endPt, startCyclePt, endCyclePt, pAP, vid, vIDCount);
		}
		bool pass = true;
		std::vector<int> newAddedFGList;
		for(int i = 0; i < m_newAddedPt.size();i++){
			int observeID = m_newAddedPt[i];
			Point_2 observePt = m_vidPointMap[m_vidFGMap[observeID]];
			Point_2 targetPt;
			for(int j = 0;j < m_newAddedPt.size();j ++){
				pass = true;
				newAddedFGList.clear();
				if(j != i){
					targetPt = m_vidPointMap[m_vidFGMap[m_newAddedPt[j]]];
					if(bg::distance(observePt, targetPt) < 1.35*m_edgeLength && bg::distance(observePt, targetPt) > 0.8*m_edgeLength){
						for(Polygon2_list::iterator obsit = m_objectPolyList.begin(); obsit != m_objectPolyList.end(); obsit++){
							Linestring_2 tempL;
							bg::append(tempL, targetPt);
							bg::append(tempL, observePt);
							if(bg::crosses(tempL, *obsit)){
								pass = false;
								break;
							}
						}

						if(pass == false){
							continue;			
						}else{
							m_finalGraph.addEdge(observeID, m_newAddedPt[j]);
							newAddedFGList.push_back(m_vidFGMap[observeID]);
							newAddedFGList.push_back(m_vidFGMap[m_newAddedPt[j]]);
							addEdgeFGMap(observeID, m_newAddedPt[j], newAddedFGList);
							m_graph.addEdge(m_vidFGMap[observeID], m_vidFGMap[m_newAddedPt[j]]);
						}
					}

				}
				
			}
		}



	}else{  
	for(Polygon2_list::iterator obsit = m_objectPolyList.begin(); obsit != m_objectPolyList.end(); obsit++){
		if(i == 0){ 
			poly_1 = *obsit;
			hasPoly_1 = true;
		}else if(i == 1){
			poly_2 = *obsit;
			hasPoly_2 = true;
		}else if(i == 2){
			poly_3 = *obsit;
			hasPoly_3 = true;
		}
		i++;
	}

	i = 0;

	for(std::vector<Graph*>::iterator g = m_obsBoundingCycleVec.begin(); g != m_obsBoundingCycleVec.end(); g++){
		if(i == 0){ 
			cycle_1 = *(*g);
		}else if(i == 1){
			cycle_2 = *(*g);
		}else if(i == 2){
			cycle_3 = *(*g);
		}
		i++;
	}
		 	
	

	if(hasPoly_1 && hasPoly_2 && hasPoly_3){  	   	

		getShortestBetweenTwoPoly(poly_1, poly_2, shortestDist_12, shortestPt_12, shortestPolyPt_12, intersectSeg_12, shortestPtIndex_12);
		getShortestBetweenTwoPoly(poly_1, poly_3, shortestDist_13, shortestPt_13, shortestPolyPt_13, intersectSeg_13, shortestPtIndex_13);
		getShortestBetweenTwoPoly(poly_2, poly_3, shortestDist_23, shortestPt_23, shortestPolyPt_23, intersectSeg_23, shortestPtIndex_23);
	
		if(shortestDist_12 > m_edgeLength*3.6 || !isGraphIntersection(cycle_1, cycle_2)){
			cout<<"poly_1 and poly_2 far away from each other"<<endl;
			return;
		}
		if(shortestDist_13 > m_edgeLength*3.6 || !isGraphIntersection(cycle_1, cycle_3)){
			cout<<"poly_1 and poly_3 far away from each other"<<endl;
			return;
		}
		if(shortestDist_23 > m_edgeLength*3.6 || !isGraphIntersection(cycle_2, cycle_3)){
			cout<<"poly_2 and poly_3 far away from each other"<<endl;
			return;
		}
		




		if(shortestDist_12 < shortestDist_13 && shortestDist_12 < shortestDist_23){
			
			Point_2 startPt_1, startPt_2, endPt_1, endPt_2, startPt_3, endPt_3;
			Point_2 startCyclePt_1, startCyclePt_2, endCyclePt_1, endCyclePt_2, startCyclePt_3, endCyclePt_3;
			pointAlongPoly pAP_1, pAP_2, pAP_3;
			Mpoint_2 mp12;
			Polygon_2 poly_12;
			Segment_2 shortestSeg_12_3, intersectSeg_12_3;
			Point_2 shortestPt_12_3, shortestPolyPt_12_3, leftNextPt_12, rightNextPt_12, point_12;
			int shortestPtIndex_12_3;
			double shortestDist_12_3;
			cout<<"12-3"<<endl;
			jointCycle_12 = getJointCycle(cycle_1, cycle_2);
			jointCycle_123 = getJointCycle(jointCycle_12, cycle_3);
			m_jointCycle = jointCycle_123;
			for(std::set<int>::iterator vi = jointCycle_123.getVertexSet().begin(); vi != jointCycle_123.getVertexSet().end(); vi ++){
				m_pointUsed[*vi] = false;
			}
			getPivotBetweenPoly(poly_1, poly_2, shortestSeg, intersectSeg_12, shortestPt_12, shortestPolyPt_12, shortestPtIndex_12, point_1, point_2, leftNextPt_1, rightNextPt_1, leftNextPt_2, rightNextPt_2);
			oppoPolyList.clear();
			oppoPolyList.push_back(poly_2);
			oppoPolyList.push_back(poly_3);
			pAP_1 = getStartEndAroundPoly(poly_1, poly_2, shortestSeg, jointCycle_123, point_1, leftNextPt_1, rightNextPt_1, startPt_1, endPt_1, startCyclePt_1, endCyclePt_1, step, oppoPolyList);
			connectAlongPoly(poly_1, startPt_1, endPt_1, startCyclePt_1, endCyclePt_1, pAP_1, vid, vIDCount);
			oppoPolyList.clear();
			oppoPolyList.push_back(poly_1);
			oppoPolyList.push_back(poly_3);
			pAP_2 = getStartEndAroundPoly(poly_2, poly_1, shortestSeg, jointCycle_123, point_2, leftNextPt_2, rightNextPt_2, startPt_2, endPt_2, startCyclePt_2, endCyclePt_2, step, oppoPolyList);
			connectAlongPoly(poly_2, startPt_2, endPt_2, startCyclePt_2, endCyclePt_2, pAP_2, vid, vIDCount);
			
			for(int i = 0; i < poly_1.outer().size(); i ++){
			 	bg::append(mp12, poly_1.outer()[i]);
			 	//std::cout<<poly_1.outer()[i].get<0>()<<","<<poly_1.outer()[i].get<1>()<<std::endl;
			}
			for(int i = 0; i < poly_2.outer().size(); i ++){
			 	bg::append(mp12, poly_2.outer()[i]);
			 	//std::cout<<poly_1.outer()[i].get<0>()<<","<<poly_1.outer()[i].get<1>()<<std::endl;
			}
			bg::convex_hull(mp12, poly_12);
			if(bg::intersects(poly_12, poly_3)){
				cout<<"3rd polygon too close, program failed."<<endl;
				return ;
			}
			getShortestBetweenTwoPoly(poly_3, poly_12, shortestDist_12_3, shortestPt_12_3, shortestPolyPt_12_3, intersectSeg_12_3, shortestPtIndex_12_3);
			getPivotBetweenPoly(poly_3, poly_12, shortestSeg_12_3, intersectSeg_12_3, shortestPt_12_3, shortestPolyPt_12_3, shortestPtIndex_12_3, point_3, point_12, leftNextPt_3, rightNextPt_3, leftNextPt_12, rightNextPt_12);
			oppoPolyList.clear();
			oppoPolyList.push_back(poly_1);
			oppoPolyList.push_back(poly_2);
			pAP_3 = getStartEndAroundPoly(poly_3, poly_12, shortestSeg_12_3, jointCycle_123, point_3, leftNextPt_3, rightNextPt_3, startPt_3, endPt_3, startCyclePt_3, endCyclePt_3, step, oppoPolyList);
			connectAlongPoly(poly_3, startPt_3, endPt_3, startCyclePt_3, endCyclePt_3, pAP_3, vid, vIDCount);



		}else if(shortestDist_13 < shortestDist_12 && shortestDist_13 < shortestDist_23){
			
			Point_2 startPt_1, startPt_2, endPt_1, endPt_2, startPt_3, endPt_3;
			Point_2 startCyclePt_1, startCyclePt_2, endCyclePt_1, endCyclePt_2, startCyclePt_3, endCyclePt_3;
			pointAlongPoly pAP_1, pAP_2, pAP_3;
			Mpoint_2 mp13;
			Polygon_2 poly_13;
			Segment_2 shortestSeg_13_2, intersectSeg_13_2;
			Point_2 shortestPt_13_2, shortestPolyPt_13_2, leftNextPt_13, rightNextPt_13, point_13;
			int shortestPtIndex_13_2;
			double shortestDist_13_2;
			cout<<"13-2"<<endl;
			Graph jointCycle_13 = getJointCycle(cycle_1, cycle_3);
			jointCycle_123 = getJointCycle(jointCycle_13, cycle_2);
			m_jointCycle = jointCycle_123;
			
			for(std::set<int>::iterator vi = jointCycle_123.getVertexSet().begin(); vi != jointCycle_123.getVertexSet().end(); vi ++){
				m_pointUsed[*vi] = false;
			}
			getPivotBetweenPoly(poly_1, poly_3, shortestSeg, intersectSeg_13, shortestPt_13, shortestPolyPt_13, shortestPtIndex_13, point_1, point_3, leftNextPt_1, rightNextPt_1, leftNextPt_3, rightNextPt_3);
			oppoPolyList.clear();
			oppoPolyList.push_back(poly_2);
			oppoPolyList.push_back(poly_3);
			pAP_1 = getStartEndAroundPoly(poly_1, poly_3, shortestSeg, jointCycle_123, point_1, leftNextPt_1, rightNextPt_1, startPt_1, endPt_1, startCyclePt_1, endCyclePt_1, step, oppoPolyList);
			connectAlongPoly(poly_1, startPt_1, endPt_1, startCyclePt_1, endCyclePt_1, pAP_1, vid, vIDCount);
			oppoPolyList.clear();
			oppoPolyList.push_back(poly_1);
			oppoPolyList.push_back(poly_2);
			pAP_3 = getStartEndAroundPoly(poly_3, poly_1, shortestSeg, jointCycle_123, point_3, leftNextPt_3, rightNextPt_3, startPt_3, endPt_3, startCyclePt_3, endCyclePt_3, step, oppoPolyList);
			connectAlongPoly(poly_3, startPt_3, endPt_3, startCyclePt_3, endCyclePt_3, pAP_3, vid, vIDCount);
			
			for(int i = 0; i < poly_1.outer().size(); i ++){
			 	bg::append(mp13, poly_1.outer()[i]);
			 	//std::cout<<poly_1.outer()[i].get<0>()<<","<<poly_1.outer()[i].get<1>()<<std::endl;
			}
			for(int i = 0; i < poly_3.outer().size(); i ++){
			 	bg::append(mp13, poly_3.outer()[i]);
			 	//std::cout<<poly_1.outer()[i].get<0>()<<","<<poly_1.outer()[i].get<1>()<<std::endl;
			}
			bg::convex_hull(mp13, poly_13);
			if(bg::intersects(poly_13, poly_2)){
				cout<<"2rd polygon too close, program failed."<<endl;
				return ;
			}
			getShortestBetweenTwoPoly(poly_2, poly_13, shortestDist_13_2, shortestPt_13_2, shortestPolyPt_13_2, intersectSeg_13_2, shortestPtIndex_13_2);
			getPivotBetweenPoly(poly_2, poly_13, shortestSeg_13_2, intersectSeg_13_2, shortestPt_13_2, shortestPolyPt_13_2, shortestPtIndex_13_2, point_2, point_13, leftNextPt_2, rightNextPt_2, leftNextPt_13, rightNextPt_13);
			oppoPolyList.clear();
			oppoPolyList.push_back(poly_1);
			oppoPolyList.push_back(poly_3);
			pAP_2 = getStartEndAroundPoly(poly_2, poly_13, shortestSeg_13_2, jointCycle_123, point_2, leftNextPt_2, rightNextPt_2, startPt_2, endPt_2, startCyclePt_2, endCyclePt_2, step, oppoPolyList);
			connectAlongPoly(poly_2, startPt_2, endPt_2, startCyclePt_2, endCyclePt_2, pAP_2, vid, vIDCount);




		}else if(shortestDist_23 < shortestDist_12 && shortestDist_23 < shortestDist_13){
			Point_2 startPt_1, startPt_2, endPt_1, endPt_2, startPt_3, endPt_3;
			Point_2 startCyclePt_1, startCyclePt_2, endCyclePt_1, endCyclePt_2, startCyclePt_3, endCyclePt_3;
			pointAlongPoly pAP_1, pAP_2, pAP_3;
			Mpoint_2 mp23;
			Polygon_2 poly_23;
			Segment_2 shortestSeg_23_1, intersectSeg_23_1;
			Point_2 shortestPt_23_1, shortestPolyPt_23_1, leftNextPt_23, rightNextPt_23, point_23;
			int shortestPtIndex_23_1;
			double shortestDist_23_1;
			cout<<"23-1"<<endl;
			Graph jointCycle_23 = getJointCycle(cycle_2, cycle_3);
			 jointCycle_123 = getJointCycle(jointCycle_23, cycle_1);
			 m_jointCycle = jointCycle_123;
			for(std::set<int>::iterator vi = jointCycle_123.getVertexSet().begin(); vi != jointCycle_123.getVertexSet().end(); vi ++){
				m_pointUsed[*vi] = false;
			}
			getPivotBetweenPoly(poly_2, poly_3, shortestSeg, intersectSeg_23, shortestPt_23, shortestPolyPt_23, shortestPtIndex_23, point_2, point_3, leftNextPt_2, rightNextPt_2, leftNextPt_3, rightNextPt_3);
			oppoPolyList.clear();
			oppoPolyList.push_back(poly_1);
			oppoPolyList.push_back(poly_3);
			pAP_2 = getStartEndAroundPoly(poly_2, poly_3, shortestSeg, jointCycle_123, point_2, leftNextPt_2, rightNextPt_2, startPt_2, endPt_2, startCyclePt_2, endCyclePt_2, step, oppoPolyList);
			connectAlongPoly(poly_2, startPt_2, endPt_2, startCyclePt_2, endCyclePt_2, pAP_2, vid, vIDCount);
			oppoPolyList.clear();
			oppoPolyList.push_back(poly_1);
			oppoPolyList.push_back(poly_2);
			pAP_3 = getStartEndAroundPoly(poly_3, poly_2, shortestSeg, jointCycle_123, point_3, leftNextPt_3, rightNextPt_3, startPt_3, endPt_3, startCyclePt_3, endCyclePt_3, step, oppoPolyList);
			connectAlongPoly(poly_3, startPt_3, endPt_3, startCyclePt_3, endCyclePt_3, pAP_3, vid, vIDCount);
			
			for(int i = 0; i < poly_2.outer().size(); i ++){
			 	bg::append(mp23, poly_2.outer()[i]);
			 	//std::cout<<poly_1.outer()[i].get<0>()<<","<<poly_1.outer()[i].get<1>()<<std::endl;
			}
			for(int i = 0; i < poly_3.outer().size(); i ++){
			 	bg::append(mp23, poly_3.outer()[i]);
			 	//std::cout<<poly_1.outer()[i].get<0>()<<","<<poly_1.outer()[i].get<1>()<<std::endl;
			}
			bg::convex_hull(mp23, poly_23);
			if(bg::intersects(poly_23, poly_1)){
				cout<<"1st polygon too close, program failed."<<endl;
				return ;
			}
			getShortestBetweenTwoPoly(poly_1, poly_23, shortestDist_23_1, shortestPt_23_1, shortestPolyPt_23_1, intersectSeg_23_1, shortestPtIndex_23_1);
			getPivotBetweenPoly(poly_1, poly_23, shortestSeg_23_1, intersectSeg_23_1, shortestPt_23_1, shortestPolyPt_23_1, shortestPtIndex_23_1, point_1, point_23, leftNextPt_1, rightNextPt_1, leftNextPt_23, rightNextPt_23);
			oppoPolyList.clear();
			oppoPolyList.push_back(poly_2);
			oppoPolyList.push_back(poly_3);
			pAP_1 = getStartEndAroundPoly(poly_1, poly_23, shortestSeg_23_1, jointCycle_123, point_1, leftNextPt_1, rightNextPt_1, startPt_1, endPt_1, startCyclePt_1, endCyclePt_1, step, oppoPolyList);
			connectAlongPoly(poly_1, startPt_1, endPt_1, startCyclePt_1, endCyclePt_1, pAP_1, vid, vIDCount);



		}

	}else if(hasPoly_1 && hasPoly_2 && !hasPoly_3){
		pointAlongPoly pAP_1, pAP_2;
		Point_2 startCyclePt_1, startCyclePt_2, endCyclePt_1, endCyclePt_2;
		Point_2 startPt_1, startPt_2, endPt_1, endPt_2;
		jointCycle_12 = getJointCycle(cycle_1, cycle_2);
		m_jointCycle = jointCycle_12;
		getShortestBetweenTwoPoly(poly_1, poly_2, shortestDist_12, shortestPt_12, shortestPolyPt_12, intersectSeg_12, shortestPtIndex_12);
		cout<<"shortestPt:"<<shortestPt_12.get<0>()<<","<<shortestPt_12.get<1>()<<endl;
		cout<<"shortestPolyPt_12:"<<shortestPolyPt_12.get<0>()<<","<<shortestPolyPt_12.get<1>()<<endl;
		getPivotBetweenPoly(poly_1, poly_2, shortestSeg, intersectSeg_12, shortestPt_12, shortestPolyPt_12, shortestPtIndex_12, point_1, point_2, leftNextPt_1, rightNextPt_1, leftNextPt_2, rightNextPt_2);
		cout<<"point_1:"<<point_1.get<0>()<<","<<point_1.get<1>()<<endl;
		cout<<"point_2:"<<point_2.get<0>()<<","<<point_2.get<1>()<<endl;
		cout<<"leftNextPt_1:"<<leftNextPt_1.get<0>()<<","<<leftNextPt_1.get<1>()<<endl;
		cout<<"rightNextPt_1:"<<rightNextPt_1.get<0>()<<","<<rightNextPt_1.get<1>()<<endl;
		cout<<"leftNextPt_2:"<<leftNextPt_2.get<0>()<<","<<leftNextPt_2.get<1>()<<endl;
		cout<<"rightNextPt_2:"<<rightNextPt_2.get<0>()<<","<<rightNextPt_2.get<1>()<<endl;
		oppoPolyList.clear();
		oppoPolyList.push_back(poly_2);
		pAP_1 = getStartEndAroundPoly(poly_1, poly_2, shortestSeg, jointCycle_12, point_1, leftNextPt_1, rightNextPt_1, startPt_1, endPt_1, startCyclePt_1, endCyclePt_1, step, oppoPolyList);
		connectAlongPoly(poly_1, startPt_1, endPt_1, startCyclePt_1, endCyclePt_1, pAP_1, vid, vIDCount);
		oppoPolyList.clear();
		oppoPolyList.push_back(poly_1);
		pAP_2 = getStartEndAroundPoly(poly_2, poly_1, shortestSeg, jointCycle_12, point_2, leftNextPt_2, rightNextPt_2,startPt_2, endPt_2, startCyclePt_2, endCyclePt_2, step, oppoPolyList);
		connectAlongPoly(poly_2, startPt_2, endPt_2, startCyclePt_2, endCyclePt_2, pAP_2, vid, vIDCount);
	}
}

}

void Roadmap::getPivotBetweenPoly(Polygon_2 poly_1, Polygon_2 poly_2, Segment_2 &shortestSeg, Segment_2 intersectSeg, Point_2 shortestPt, Point_2 shortestPolyPt, int shortestPtIndex, Point_2 &point_1, Point_2 &point_2, Point_2 &leftNextPt_1, Point_2 &rightNextPt_1, Point_2 &leftNextPt_2, Point_2 &rightNextPt_2){
	shortestSeg.first = shortestPt;
	shortestSeg.second = shortestPolyPt;

	//if(bg::intersects(shortestPt, poly_1)){
	if(isPtFromPoly(shortestPolyPt, poly_2)){
	leftNextPt_1 = intersectSeg.first;
		rightNextPt_1 = intersectSeg.second;
		point_1 = shortestPt;
		point_2 = shortestPolyPt;
		if(shortestPtIndex > 0 && shortestPtIndex < poly_2.outer().size()-1){
			leftNextPt_2 = poly_2.outer()[shortestPtIndex-1];
			rightNextPt_2 = poly_2.outer()[shortestPtIndex+1];
		}else if(shortestPtIndex == 0){
			leftNextPt_2 = poly_2.outer()[poly_2.outer().size()-2];
			rightNextPt_2 = poly_2.outer()[shortestPtIndex+1];
		}else if(shortestPtIndex == poly_2.outer().size()-2){
			leftNextPt_2 = poly_2.outer()[shortestPtIndex-1];
			rightNextPt_2 = poly_2.outer()[0];
		}
		
	}else{
		leftNextPt_2 = intersectSeg.first;
		rightNextPt_2 = intersectSeg.second;
		point_1 = shortestPolyPt;
		point_2 = shortestPt;
		if(shortestPtIndex > 0 && shortestPtIndex < poly_1.outer().size()-1){
			leftNextPt_1 = poly_1.outer()[shortestPtIndex-1];
			rightNextPt_1 = poly_1.outer()[shortestPtIndex+1];
		}else if(shortestPtIndex == 0){
			leftNextPt_1 = poly_1.outer()[poly_1.outer().size()-2];
			rightNextPt_1 = poly_1.outer()[shortestPtIndex+1];
		}else if(shortestPtIndex == poly_1.outer().size()-2){
			leftNextPt_1 = poly_1.outer()[shortestPtIndex-1];
			rightNextPt_1 = poly_1.outer()[0];
		}
	}
}

void Roadmap::connectAlongPoly(Polygon_2 poly, Point_2 startPt, Point_2 endPt, Point_2 startCyclePt, Point_2 endCyclePt, pointAlongPoly pAP, int &vid, int &vIDCount){
	Point_2 startPoint = startPt;
	Point_2 endPoint = endPt;
	Point_2 crossStartPt = startCyclePt;
	Point_2 crossEndPt = endCyclePt;
	Point_2 polyNextPt;
	double overlapLength = 0;
	double crossEdgeLength = 0;

	double startHalfEdgeLength = bg::distance(crossStartPt, startPoint);
	double endHalfEdgeLength = bg::distance(crossEndPt, endPoint);
	overlapLength += endHalfEdgeLength;
	overlapLength += startHalfEdgeLength;
	overlapLength += polygonCrossLength(startPoint, endPoint, pAP.polyCurrentPt, pAP.polyPreviousPt, pAP.polyEndPt, pAP.polyEndAfterPt, poly);
	std::cout<<"overlapLength is"<<overlapLength<<std::endl;

	int edgeNums = 0;
	if(overlapLength > m_edgeLength && overlapLength < 1.6* m_edgeLength){
		edgeNums = 1;
	}else if(overlapLength > 1.6* m_edgeLength && overlapLength < 2.7* m_edgeLength){
		edgeNums = 2;
	}else if(overlapLength > 2.7* m_edgeLength && overlapLength < 3.9* m_edgeLength){
		edgeNums = 3;
	}else if(overlapLength > 3.9* m_edgeLength && overlapLength < 4.9* m_edgeLength){
		edgeNums = 4;
	}else{
		if(fmod(overlapLength , m_edgeLength) > 0.9 * m_edgeLength){
			edgeNums = overlapLength / m_edgeLength + 1;
		}else{
			edgeNums = overlapLength / m_edgeLength;
		}
					

	}
	//Point_2 singleStart, singleEnd;
	crossEdgeLength = overlapLength / double(edgeNums);
	std::cout<<"edgeNums:"<<edgeNums<<std::endl;
	std::cout<<"single edge's length"<<crossEdgeLength<<std::endl;

	

	//registerRegularPoint(vid, startPoint, crossStartPt);
	//vid++;


	std::vector<int> vidFGlist;

// vidFGlist, polyPreviousPt, polyEndPt, startHalfEdgeLength, endPoint, startPoint, crossEdgeLength,
// vid, vIDCount, crossStartPt, crossEndPt, overlapLength 
// polyCurrentPt, polyEndAfterPt, polyNextPt, poly_1
	if(bg::equals(pAP.polyPreviousPt, pAP.polyEndPt)){
		std::cout<<"poly_1 cross one edge"<<std::endl;
		double additiveLength = 0;
		additiveLength += startHalfEdgeLength;
		m_pointDistMap.clear();
		m_pointDistMap[startPoint] = startHalfEdgeLength;
		if(startHalfEdgeLength > crossEdgeLength){
			cout<<"startHalfEdgeLength > crossEdgeLength   762"<<endl;
			double addingX = getAddingX(startPoint, crossStartPt);
			double addingY = getAddingY(startPoint, crossStartPt);
			Point_2 firstPt(crossStartPt.get<0>()+crossEdgeLength*addingX, crossStartPt.get<1>()+crossEdgeLength*addingY);
			registerRegularPoint(vid, firstPt, crossStartPt);
			m_newAddedSeg.push_back(Segment_2(firstPt, crossStartPt));
			vidFGlist.clear();
			vidFGlist.push_back(m_pointVidMap[crossStartPt]);
			vidFGlist.push_back(vid);
			reverseVector(vidFGlist);
			registerFinalPoint(vIDCount, vid, firstPt, crossStartPt, vidFGlist);
			m_newAddedPt.push_back(vIDCount);
			cout<<vIDCount<<"is the id for firstPt"<<endl;
			vIDCount ++;
			vid++;
			additiveLength = crossEdgeLength;
			registerRegularPoint(vid, startPoint, firstPt);
			m_newAddedSeg.push_back(Segment_2(startPoint, firstPt));
			vid++;	
			m_pointDistMap[endPoint] = overlapLength - endHalfEdgeLength;
			if(m_pointDistMap[endPoint] < additiveLength + crossEdgeLength){
				if(overlapLength <= 2*crossEdgeLength){
					registerRegularPoint(vid, endPoint, startPoint);
					m_newAddedSeg.push_back(Segment_2(startPoint, endPoint));
					registerRegularPoint(vid, endPoint, crossEndPt);
					m_newAddedSeg.push_back(Segment_2(crossEndPt, endPoint));
					vid++;	
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(m_pointVidMap[startPoint]);
					vidFGlist.push_back(m_pointVidMap[firstPt]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[firstPt]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[firstPt]], vidFGlist);
				}else{
					registerRegularPoint(vid, endPoint, startPoint);
					vid++;
					double addingX = getAddingX(crossEndPt, endPoint);
					double addingY = getAddingY(crossEndPt, endPoint);
					Point_2 currentPt(endPoint.get<0>()+(2*crossEdgeLength-m_pointDistMap[endPoint])*addingX, endPoint.get<1>()+(2*crossEdgeLength-m_pointDistMap[endPoint])*addingY);
					registerRegularPoint(vid, currentPt, endPoint);
					m_newAddedSeg.push_back(Segment_2(currentPt, endPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[firstPt]);
					vidFGlist.push_back(m_pointVidMap[startPoint]);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(vid);
					reverseVector(vidFGlist);
					registerFinalPoint(vIDCount, vid, currentPt, firstPt, vidFGlist);
					m_newAddedPt.push_back(vIDCount);
					registerRegularPoint(vid, currentPt, crossEndPt);
					m_newAddedSeg.push_back(Segment_2(crossEndPt, currentPt));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[currentPt]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]], vidFGlist);
					vid++;
					vIDCount++;
				}	
			}else{
				double addingX = getAddingX(endPoint, startPoint);
				double addingY = getAddingY(endPoint, startPoint);
				Point_2 newFirstPt(startPoint.get<0>()+(2*crossEdgeLength - startHalfEdgeLength)*addingX, startPoint.get<1>()+(2*crossEdgeLength - startHalfEdgeLength)*addingY);
		
		//  record the newly found discrete graph point firstPt into m_graph, m_finalGraph
				registerRegularPoint(vid, newFirstPt, startPoint);
				vidFGlist.clear();
				vidFGlist.push_back(vid);
				vidFGlist.push_back(m_pointVidMap[startPoint]);
				vidFGlist.push_back(m_pointVidMap[firstPt]);
				registerFinalPoint(vIDCount, vid, newFirstPt, firstPt, vidFGlist);
				vid++;
				vIDCount ++;


				additiveLength = 2*crossEdgeLength;
				Point_2 tempPt, currentPt;
				currentPt = newFirstPt;

				while(additiveLength < overlapLength - endHalfEdgeLength - crossEdgeLength){
					tempPt.set<0>(currentPt.get<0>()+addingX * crossEdgeLength);
					tempPt.set<1>(currentPt.get<1>()+addingY * crossEdgeLength);
					registerRegularPoint(vid, tempPt, currentPt);
					vidFGlist.clear();
					vidFGlist.push_back(vid);
					vidFGlist.push_back(m_pointVidMap[currentPt]);
					// the order of parameters is from the newly one  from last one, so is the m_graph point list
					registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
					vid++;
					vIDCount++;
					additiveLength += crossEdgeLength;
					currentPt = tempPt;
					
				}
				
				if(endHalfEdgeLength > crossEdgeLength){
					registerRegularPoint(vid, endPoint, currentPt);
					vid++;
					double addingX = getAddingX(crossEndPt, endPoint);
					double addingY = getAddingY(crossEndPt, endPoint);
					Point_2 lastPoint;
					lastPoint.set<0>(endPoint.get<0>()+addingX*(crossEdgeLength - overlapLength + endHalfEdgeLength + additiveLength));
					lastPoint.set<1>(endPoint.get<1>()+addingY*(crossEdgeLength - overlapLength + endHalfEdgeLength + additiveLength));
					registerRegularPoint(vid, lastPoint, endPoint);
					vidFGlist.clear();
					vidFGlist.push_back(vid);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(m_pointVidMap[currentPt]);
					// the order of parameters is from the newly one  from last one, so is the m_graph point list
					registerFinalPoint(vIDCount, vid, lastPoint, currentPt, vidFGlist);
					registerRegularPoint(vid, lastPoint, crossEndPt);
					m_newAddedSeg.push_back(Segment_2(crossEndPt, lastPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[lastPoint]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastPoint]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastPoint]], vidFGlist);
					vid++;
					vIDCount++;

				}else{
					registerRegularPoint(vid, endPoint, currentPt);
					registerRegularPoint(vid, endPoint, crossEndPt);
					m_newAddedSeg.push_back(Segment_2(crossEndPt, endPoint));
					m_newAddedSeg.push_back(Segment_2(currentPt, endPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(m_pointVidMap[currentPt]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]], vidFGlist);
					vid++;
					vIDCount++;
				}
			}
		}else{
			cout<<"startHalfEdgeLength < crossEdgeLength   762"<<endl;
			double addingX = getAddingX(endPoint, startPoint);
			double addingY = getAddingY(endPoint, startPoint);
			Point_2 firstPt(startPoint.get<0>()+(crossEdgeLength - additiveLength)*addingX, startPoint.get<1>()+(crossEdgeLength - additiveLength)*addingY);
			registerRegularPoint(vid, startPoint, crossStartPt);
			vid++;
		//  record the newly found discrete graph point firstPt into m_graph, m_finalGraph
			registerRegularPoint(vid, firstPt, startPoint);
			vidFGlist.clear();
			vidFGlist.push_back(vid);
			vidFGlist.push_back(m_pointVidMap[startPoint]);
			vidFGlist.push_back(m_pointVidMap[crossStartPt]);
			registerFinalPoint(vIDCount, vid, firstPt, crossStartPt, vidFGlist);
			vid++;
			vIDCount ++;

			additiveLength = crossEdgeLength;
			Point_2 tempPt, currentPt;
			currentPt = firstPt;
		
			while(additiveLength < overlapLength - endHalfEdgeLength - crossEdgeLength){
				tempPt.set<0>(currentPt.get<0>()+addingX * crossEdgeLength);
				tempPt.set<1>(currentPt.get<1>()+addingY * crossEdgeLength);
				registerRegularPoint(vid, tempPt, currentPt);
				vidFGlist.clear();
				vidFGlist.push_back(vid);
				vidFGlist.push_back(m_pointVidMap[currentPt]);
				// the order of parameters is from the newly one  from last one, so is the m_graph point list
				registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
				vid++;
				vIDCount++;
				additiveLength += crossEdgeLength;
				currentPt = tempPt;
					
			}
				
				if(endHalfEdgeLength > crossEdgeLength){
					registerRegularPoint(vid, endPoint, currentPt);
					vid++;
					double addingX = getAddingX(crossEndPt, endPoint);
					double addingY = getAddingY(crossEndPt, endPoint);
					Point_2 lastPoint;
					lastPoint.set<0>(endPoint.get<0>()+addingX*(crossEdgeLength - overlapLength + endHalfEdgeLength + additiveLength));
					lastPoint.set<1>(endPoint.get<1>()+addingY*(crossEdgeLength - overlapLength + endHalfEdgeLength + additiveLength));
					registerRegularPoint(vid, lastPoint, endPoint);
					vidFGlist.clear();
					vidFGlist.push_back(vid);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(m_pointVidMap[currentPt]);
					// the order of parameters is from the newly one  from last one, so is the m_graph point list
					registerFinalPoint(vIDCount, vid, lastPoint, currentPt, vidFGlist);
					registerRegularPoint(vid, lastPoint, crossEndPt);
					m_newAddedSeg.push_back(Segment_2(crossEndPt, lastPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[lastPoint]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastPoint]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastPoint]], vidFGlist);
					vid++;
					vIDCount++;

				}else{
					registerRegularPoint(vid, endPoint, currentPt);
					registerRegularPoint(vid, endPoint, crossEndPt);
					m_newAddedSeg.push_back(Segment_2(crossEndPt, endPoint));
					m_newAddedSeg.push_back(Segment_2(currentPt, endPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(m_pointVidMap[currentPt]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]], vidFGlist);
					vid++;
					vIDCount++;
				}
		}






	
	}else if(bg::equals(pAP.polyCurrentPt, pAP.polyEndPt)){
		std::cout<<"poly_1 cross two edge"<<std::endl;
		m_pointDistMap.clear();
		m_pointDistMap[pAP.polyCurrentPt] = startHalfEdgeLength + bg::distance(pAP.polyCurrentPt, startPoint);
		double additiveLength = 0;
		if(m_pointDistMap[pAP.polyCurrentPt] > crossEdgeLength){
			Point_2 lastFinalPt;
			if(startHalfEdgeLength > crossEdgeLength){
				cout<<"startHalfEdgeLength > crossEdgeLength   762"<<endl;
				double addingX = getAddingX(startPoint, crossStartPt);
				double addingY = getAddingY(startPoint, crossStartPt);
				Point_2 firstPt(crossStartPt.get<0>()+crossEdgeLength*addingX, crossStartPt.get<1>()+crossEdgeLength*addingY);
				registerRegularPoint(vid, firstPt, crossStartPt);
				m_newAddedSeg.push_back(Segment_2(firstPt, crossStartPt));
				vidFGlist.clear();
				vidFGlist.push_back(m_pointVidMap[crossStartPt]);
				vidFGlist.push_back(vid);
				reverseVector(vidFGlist);
				registerFinalPoint(vIDCount, vid, firstPt, crossStartPt, vidFGlist);
				m_newAddedPt.push_back(vIDCount);
				cout<<vIDCount<<"is the id for firstPt"<<endl;
				vIDCount ++;
				vid++;
				additiveLength = crossEdgeLength;
				registerRegularPoint(vid, startPoint, firstPt);
				m_newAddedSeg.push_back(Segment_2(startPoint, firstPt));
				vid++;
				if(m_pointDistMap[pAP.polyCurrentPt] > crossEdgeLength && m_pointDistMap[pAP.polyCurrentPt] < 2*crossEdgeLength){
					registerRegularPoint(vid, pAP.polyCurrentPt, startPoint);
					m_newAddedSeg.push_back(Segment_2(pAP.polyCurrentPt, startPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[firstPt]);
					vidFGlist.push_back(m_pointVidMap[startPoint]);
					vidFGlist.push_back(vid);
					vid++;
					lastFinalPt = firstPt;

						
								
				}else{
					double addingX = getAddingX(pAP.polyCurrentPt, startPoint);
					double addingY = getAddingY(pAP.polyCurrentPt, startPoint);
					Point_2 secondPt(startPoint.get<0>()+(2*crossEdgeLength-startHalfEdgeLength)*addingX, startPoint.get<1>()+(2*crossEdgeLength-startHalfEdgeLength)*addingY);
					registerRegularPoint(vid, secondPt, startPoint);
					m_newAddedSeg.push_back(Segment_2(secondPt, startPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[firstPt]);
					vidFGlist.push_back(m_pointVidMap[startPoint]);
					vidFGlist.push_back(vid);
					reverseVector(vidFGlist);
					registerFinalPoint(vIDCount, vid, secondPt, firstPt, vidFGlist);
					m_newAddedPt.push_back(vIDCount);
					vIDCount ++;
					vid++;
					additiveLength = 2*crossEdgeLength;
					Point_2 currentPt, tempPt;
					currentPt = secondPt;
					while(m_pointDistMap[pAP.polyCurrentPt] > additiveLength + crossEdgeLength){
						tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
						tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
						registerRegularPoint(vid, tempPt, currentPt);
						m_newAddedSeg.push_back(Segment_2(tempPt, currentPt));
						vidFGlist.clear();
						vidFGlist.push_back(m_pointVidMap[currentPt]);
						vidFGlist.push_back(vid);
						reverseVector(vidFGlist);
						registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
						m_newAddedPt.push_back(vIDCount);
						additiveLength += crossEdgeLength;
						currentPt = tempPt;
						vid++;
						vIDCount++;
					}
					registerRegularPoint(vid, pAP.polyCurrentPt, currentPt);
					m_newAddedSeg.push_back(Segment_2(pAP.polyCurrentPt, currentPt));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[currentPt]);
					vidFGlist.push_back(vid);
					vid++;
					lastFinalPt = currentPt;
				}


			}else{
				cout<<"startHalfEdgeLength <= crossEdgeLength  830"<<endl;
				registerRegularPoint(vid, startPoint, crossStartPt);
				m_newAddedSeg.push_back(Segment_2(startPoint, crossStartPt));
				vid++;
				double addingX = getAddingX(pAP.polyCurrentPt, startPoint);
				double addingY = getAddingY(pAP.polyCurrentPt, startPoint);
				Point_2 firstPt(startPoint.get<0>()+(crossEdgeLength - startHalfEdgeLength)*addingX, startPoint.get<1>()+(crossEdgeLength - startHalfEdgeLength)*addingY);
				registerRegularPoint(vid, firstPt, startPoint);
				m_newAddedSeg.push_back(Segment_2(firstPt, startPoint));
				vidFGlist.clear();
				vidFGlist.push_back(m_pointVidMap[crossStartPt]);
				vidFGlist.push_back(m_pointVidMap[startPoint]);
				vidFGlist.push_back(vid);
				reverseVector(vidFGlist);
				registerFinalPoint(vIDCount, vid, firstPt, crossStartPt, vidFGlist);
				m_newAddedPt.push_back(vIDCount);
				vIDCount ++;
				vid++;
				additiveLength = crossEdgeLength;
				Point_2 currentPt, tempPt;
				currentPt = firstPt;
				while(m_pointDistMap[pAP.polyCurrentPt] > additiveLength + crossEdgeLength){
					tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
					tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
					registerRegularPoint(vid, tempPt, currentPt);
					m_newAddedSeg.push_back(Segment_2(tempPt, currentPt));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[currentPt]);
					vidFGlist.push_back(vid);
					reverseVector(vidFGlist);
					registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
					m_newAddedPt.push_back(vIDCount);
					additiveLength += crossEdgeLength;
					currentPt = tempPt;
					vid++;
					vIDCount++;
				}
				registerRegularPoint(vid, pAP.polyCurrentPt, currentPt);
				m_newAddedSeg.push_back(Segment_2(pAP.polyCurrentPt, currentPt));
				vidFGlist.clear();
				vidFGlist.push_back(m_pointVidMap[currentPt]);
				vidFGlist.push_back(vid);
				vid++;
				lastFinalPt = currentPt;
			}

			double lastFinalPtDist = additiveLength;
						
			additiveLength = m_pointDistMap[pAP.polyCurrentPt];

			m_pointDistMap[endPoint] = overlapLength - endHalfEdgeLength;
			if(m_pointDistMap[endPoint] < lastFinalPtDist + crossEdgeLength){
				
				if(endHalfEdgeLength > crossEdgeLength){
					
					registerRegularPoint(vid, endPoint, pAP.polyCurrentPt);
					vid++;	
					double addingX = getAddingX(crossEndPt, endPoint);
					double addingY = getAddingY(crossEndPt, endPoint);
					Point_2 lastPoint;
					lastPoint.set<0>(endPoint.get<0>()+addingX*(crossEdgeLength - overlapLength + endHalfEdgeLength + lastFinalPtDist));
					lastPoint.set<1>(endPoint.get<1>()+addingY*(crossEdgeLength - overlapLength + endHalfEdgeLength + lastFinalPtDist));
					registerRegularPoint(vid, lastPoint, endPoint);
					vidFGlist.clear();
					vidFGlist.push_back(vid);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(m_pointVidMap[pAP.polyCurrentPt]);
					vidFGlist.push_back(m_pointVidMap[lastFinalPt]);
					// the order of parameters is from the newly one  from last one, so is the m_graph point list
					registerFinalPoint(vIDCount, vid, lastPoint, lastFinalPt, vidFGlist);
					registerRegularPoint(vid, lastPoint, crossEndPt);
					m_newAddedSeg.push_back(Segment_2(crossEndPt, lastPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[lastPoint]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastPoint]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastPoint]], vidFGlist);
					vid++;
					vIDCount++;

				}else{
					registerRegularPoint(vid, endPoint, pAP.polyCurrentPt);	
					registerRegularPoint(vid, endPoint, crossEndPt);
					vid++;
					m_newAddedSeg.push_back(Segment_2(crossEndPt, endPoint));
					m_newAddedSeg.push_back(Segment_2(pAP.polyCurrentPt, endPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(m_pointVidMap[pAP.polyCurrentPt]);
					vidFGlist.push_back(m_pointVidMap[lastFinalPt]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastFinalPt]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastFinalPt]], vidFGlist);
					vid++;
					vIDCount++;
				}


			}else{  
				Point_2 currentPt, tempPt;
				double addingX = getAddingX(endPoint, pAP.polyCurrentPt);
				double addingY = getAddingY(endPoint, pAP.polyCurrentPt);
				double addOnLength = lastFinalPtDist + crossEdgeLength - m_pointDistMap[pAP.polyCurrentPt];
				tempPt.set<0>(pAP.polyCurrentPt.get<0>() + addOnLength*addingX);
				tempPt.set<1>(pAP.polyCurrentPt.get<1>() + addOnLength*addingY);
				registerRegularPoint(vid, tempPt, pAP.polyCurrentPt);
				vidFGlist.clear();
				vidFGlist.push_back(vid);
				vidFGlist.push_back(m_pointVidMap[pAP.polyCurrentPt]);
				vidFGlist.push_back(m_pointVidMap[lastFinalPt]);
				registerFinalPoint(vIDCount, vid, tempPt, lastFinalPt, vidFGlist);
				additiveLength = lastFinalPtDist + crossEdgeLength;
				currentPt = tempPt;
				vid++;
				vIDCount++;
				while(m_pointDistMap[endPoint] > additiveLength + crossEdgeLength){
					tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
					tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
					registerRegularPoint(vid, tempPt, currentPt);
					vidFGlist.clear();
					vidFGlist.push_back(vid);
					vidFGlist.push_back(m_pointVidMap[currentPt]);			
					registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
					additiveLength += crossEdgeLength;
					currentPt = tempPt;
					vid++;
					vIDCount++;

					
				}
				lastFinalPt = currentPt;
				if(endHalfEdgeLength > crossEdgeLength){
					
					registerRegularPoint(vid, endPoint, lastFinalPt);
					vid++;	
					double addingX = getAddingX(crossEndPt, endPoint);
					double addingY = getAddingY(crossEndPt, endPoint);
					Point_2 lastPoint;
					lastPoint.set<0>(endPoint.get<0>()+addingX*(crossEdgeLength - overlapLength + endHalfEdgeLength + additiveLength));
					lastPoint.set<1>(endPoint.get<1>()+addingY*(crossEdgeLength - overlapLength + endHalfEdgeLength + additiveLength));
					registerRegularPoint(vid, lastPoint, endPoint);
					vidFGlist.clear();
					vidFGlist.push_back(vid);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(m_pointVidMap[lastFinalPt]);
					// the order of parameters is from the newly one  from last one, so is the m_graph point list
					registerFinalPoint(vIDCount, vid, lastPoint, lastFinalPt, vidFGlist);
					registerRegularPoint(vid, lastPoint, crossEndPt);
					m_newAddedSeg.push_back(Segment_2(crossEndPt, lastPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[lastPoint]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastPoint]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastPoint]], vidFGlist);
					vid++;
					vIDCount++;

				}else{
					registerRegularPoint(vid, endPoint, lastFinalPt);	
					registerRegularPoint(vid, endPoint, crossEndPt);
					vid++;
					m_newAddedSeg.push_back(Segment_2(crossEndPt, endPoint));
					m_newAddedSeg.push_back(Segment_2(lastFinalPt, endPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(m_pointVidMap[lastFinalPt]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastFinalPt]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastFinalPt]], vidFGlist);
					vid++;
					vIDCount++;
				}




			}
		}else{
			Point_2 lastFinalPt;
			cout<<"m_pointDistMap[pAP.polyCurrentPt] < crossEdgeLength"<<std::endl;
			registerRegularPoint(vid, startPoint, crossStartPt);
			vid++;			
			additiveLength += bg::distance(startPoint, crossStartPt);
			registerRegularPoint(vid, pAP.polyCurrentPt, startPoint);
			vid++;
			additiveLength += bg::distance(startPoint, pAP.polyCurrentPt);
			double addingX = getAddingX(endPoint, pAP.polyCurrentPt);
			double addingY = getAddingY(endPoint, pAP.polyCurrentPt);
			Point_2 tempPt, currentPt;
			tempPt.set<0>(pAP.polyCurrentPt.get<0>()+(crossEdgeLength - additiveLength)*addingX);
			tempPt.set<1>(pAP.polyCurrentPt.get<1>()+(crossEdgeLength - additiveLength)*addingY);
			registerRegularPoint(vid, tempPt, pAP.polyCurrentPt);
			vidFGlist.clear();
			vidFGlist.push_back(m_pointVidMap[tempPt]);
			vidFGlist.push_back(m_pointVidMap[pAP.polyCurrentPt]);
			vidFGlist.push_back(m_pointVidMap[startPoint]);
			vidFGlist.push_back(m_pointVidMap[crossStartPt]);
			registerFinalPoint(vIDCount, vid, tempPt, crossStartPt, vidFGlist);
			vid++;
			vIDCount++;
			additiveLength = crossEdgeLength;
			currentPt = tempPt;
			m_pointDistMap[endPoint] = overlapLength - endHalfEdgeLength;
			while(m_pointDistMap[endPoint] > additiveLength + crossEdgeLength){
				tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
				tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
				registerRegularPoint(vid, tempPt, currentPt);
				vidFGlist.clear();
				vidFGlist.push_back(vid);
				vidFGlist.push_back(m_pointVidMap[currentPt]);
				registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
				additiveLength += crossEdgeLength;
				currentPt = tempPt;
				vid++;
				vIDCount++;

					
			}
			lastFinalPt = currentPt;
			
			if(endHalfEdgeLength > crossEdgeLength){
					
					registerRegularPoint(vid, endPoint, lastFinalPt);
					vid++;	
					double addingX = getAddingX(crossEndPt, endPoint);
					double addingY = getAddingY(crossEndPt, endPoint);
					Point_2 lastPoint;
					lastPoint.set<0>(endPoint.get<0>()+addingX*(crossEdgeLength - overlapLength + endHalfEdgeLength + additiveLength));
					lastPoint.set<1>(endPoint.get<1>()+addingY*(crossEdgeLength - overlapLength + endHalfEdgeLength + additiveLength));
					registerRegularPoint(vid, lastPoint, endPoint);
					vidFGlist.clear();
					vidFGlist.push_back(vid);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(m_pointVidMap[lastFinalPt]);
					// the order of parameters is from the newly one  from last one, so is the m_graph point list
					registerFinalPoint(vIDCount, vid, lastPoint, lastFinalPt, vidFGlist);
					registerRegularPoint(vid, lastPoint, crossEndPt);
					m_newAddedSeg.push_back(Segment_2(crossEndPt, lastPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[lastPoint]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastPoint]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastPoint]], vidFGlist);
					vid++;
					vIDCount++;

			}else{
					registerRegularPoint(vid, endPoint, lastFinalPt);	
					registerRegularPoint(vid, endPoint, crossEndPt);
					vid++;
					m_newAddedSeg.push_back(Segment_2(crossEndPt, endPoint));
					m_newAddedSeg.push_back(Segment_2(lastFinalPt, endPoint));
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(m_pointVidMap[endPoint]);
					vidFGlist.push_back(m_pointVidMap[lastFinalPt]);
					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastFinalPt]]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[lastFinalPt]], vidFGlist);
					vid++;
					vIDCount++;
			}


		}



			
		
					
	}else{ // cross multiple edges
		std::cout<<"poly_1 cross multiple edge"<<std::endl;
					m_pointDistMap.clear();
					m_pointDistMap[pAP.polyCurrentPt] = startHalfEdgeLength + bg::distance(pAP.polyCurrentPt, startPoint);
					double additiveLength = 0;
					cout<<"pAP.polyCurrentPt: "<<pAP.polyCurrentPt.get<0>()<<","<<pAP.polyCurrentPt.get<1>()<<endl;
					cout<<"startHalfEdgeLength is "<<startHalfEdgeLength<<endl;
					if(m_pointDistMap[pAP.polyCurrentPt] > crossEdgeLength){
						Point_2 lastFinalPt;
						cout<<"m_pointDistMap[pAP.polyCurrentPt] > crossEdgeLength  760"<<endl;
						if(startHalfEdgeLength > crossEdgeLength){
							cout<<"startHalfEdgeLength > crossEdgeLength   762"<<endl;
							double addingX = getAddingX(startPoint, crossStartPt);
							double addingY = getAddingY(startPoint, crossStartPt);
							Point_2 firstPt(crossStartPt.get<0>()+crossEdgeLength*addingX, crossStartPt.get<1>()+crossEdgeLength*addingY);
							registerRegularPoint(vid, firstPt, crossStartPt);
							m_newAddedSeg.push_back(Segment_2(firstPt, crossStartPt));
							vidFGlist.clear();
							vidFGlist.push_back(m_pointVidMap[crossStartPt]);
							vidFGlist.push_back(vid);
							reverseVector(vidFGlist);
							registerFinalPoint(vIDCount, vid, firstPt, crossStartPt, vidFGlist);
							m_newAddedPt.push_back(vIDCount);
							cout<<vIDCount<<"is the id for firstPt"<<endl;
							vIDCount ++;
							vid++;
							additiveLength = crossEdgeLength;
							registerRegularPoint(vid, startPoint, firstPt);
							m_newAddedSeg.push_back(Segment_2(startPoint, firstPt));
							vid++;
							if(m_pointDistMap[pAP.polyCurrentPt] > crossEdgeLength && m_pointDistMap[pAP.polyCurrentPt] < 2*crossEdgeLength){
								registerRegularPoint(vid, pAP.polyCurrentPt, startPoint);
								m_newAddedSeg.push_back(Segment_2(pAP.polyCurrentPt, startPoint));
								vidFGlist.clear();
								vidFGlist.push_back(m_pointVidMap[firstPt]);
								vidFGlist.push_back(m_pointVidMap[startPoint]);
								vidFGlist.push_back(vid);
								vid++;
								lastFinalPt = firstPt;

						
								
							}else{
								double addingX = getAddingX(pAP.polyCurrentPt, startPoint);
								double addingY = getAddingY(pAP.polyCurrentPt, startPoint);
								Point_2 secondPt(startPoint.get<0>()+(2*crossEdgeLength-startHalfEdgeLength)*addingX, startPoint.get<1>()+(2*crossEdgeLength-startHalfEdgeLength)*addingY);
								registerRegularPoint(vid, secondPt, startPoint);
								m_newAddedSeg.push_back(Segment_2(secondPt, startPoint));
								vidFGlist.clear();
								vidFGlist.push_back(m_pointVidMap[firstPt]);
								vidFGlist.push_back(m_pointVidMap[startPoint]);
								vidFGlist.push_back(vid);
								reverseVector(vidFGlist);
								registerFinalPoint(vIDCount, vid, secondPt, firstPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								vIDCount ++;
								vid++;
								additiveLength = 2*crossEdgeLength;
								Point_2 currentPt, tempPt;
								currentPt = secondPt;
								while(m_pointDistMap[pAP.polyCurrentPt] > additiveLength + crossEdgeLength){
									tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
									tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
									registerRegularPoint(vid, tempPt, currentPt);
									m_newAddedSeg.push_back(Segment_2(tempPt, currentPt));
									vidFGlist.clear();
									vidFGlist.push_back(m_pointVidMap[currentPt]);
									vidFGlist.push_back(vid);
									reverseVector(vidFGlist);
									registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
									m_newAddedPt.push_back(vIDCount);
									additiveLength += crossEdgeLength;
									currentPt = tempPt;
									vid++;
									vIDCount++;
								}
								registerRegularPoint(vid, pAP.polyCurrentPt, currentPt);
								m_newAddedSeg.push_back(Segment_2(pAP.polyCurrentPt, currentPt));
								vidFGlist.clear();
								vidFGlist.push_back(m_pointVidMap[currentPt]);
								vidFGlist.push_back(vid);
								vid++;
								lastFinalPt = currentPt;
							}

							
						}else{
							cout<<"startHalfEdgeLength <= crossEdgeLength  830"<<endl;
							registerRegularPoint(vid, startPoint, crossStartPt);
							m_newAddedSeg.push_back(Segment_2(startPoint, crossStartPt));
							vid++;
							double addingX = getAddingX(pAP.polyCurrentPt, startPoint);
							double addingY = getAddingY(pAP.polyCurrentPt, startPoint);
							Point_2 firstPt(startPoint.get<0>()+(crossEdgeLength - startHalfEdgeLength)*addingX, startPoint.get<1>()+(crossEdgeLength - startHalfEdgeLength)*addingY);
							registerRegularPoint(vid, firstPt, startPoint);
							m_newAddedSeg.push_back(Segment_2(firstPt, startPoint));
							vidFGlist.clear();
							vidFGlist.push_back(m_pointVidMap[crossStartPt]);
							vidFGlist.push_back(m_pointVidMap[startPoint]);
							vidFGlist.push_back(vid);
							reverseVector(vidFGlist);
							registerFinalPoint(vIDCount, vid, firstPt, crossStartPt, vidFGlist);
							m_newAddedPt.push_back(vIDCount);
							vIDCount ++;
							vid++;
							additiveLength = crossEdgeLength;
							Point_2 currentPt, tempPt;
							currentPt = firstPt;
							while(m_pointDistMap[pAP.polyCurrentPt] > additiveLength + crossEdgeLength){
								tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
								tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
								registerRegularPoint(vid, tempPt, currentPt);
								m_newAddedSeg.push_back(Segment_2(tempPt, currentPt));
								vidFGlist.clear();
								vidFGlist.push_back(m_pointVidMap[currentPt]);
								vidFGlist.push_back(vid);
								reverseVector(vidFGlist);
								registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								additiveLength += crossEdgeLength;
								currentPt = tempPt;
								vid++;
								vIDCount++;
							}
							registerRegularPoint(vid, pAP.polyCurrentPt, currentPt);
							m_newAddedSeg.push_back(Segment_2(pAP.polyCurrentPt, currentPt));
							vidFGlist.clear();
							vidFGlist.push_back(m_pointVidMap[currentPt]);
							vidFGlist.push_back(vid);
							vid++;
							lastFinalPt = currentPt;
						}
						

						
						double lastFinalPtDist = additiveLength;
						
						additiveLength = m_pointDistMap[pAP.polyCurrentPt];


						while(!bg::equals((polyNextPt = nextInPolygon(pAP.polyCurrentPt, pAP.polyPreviousPt, poly, "759")), pAP.polyEndAfterPt)){
							m_pointDistMap[polyNextPt] = m_pointDistMap[pAP.polyCurrentPt] + bg::distance(pAP.polyCurrentPt, polyNextPt);
							double addingX = getAddingX(polyNextPt, pAP.polyCurrentPt);
							double addingY = getAddingY(polyNextPt, pAP.polyCurrentPt);
							Point_2 tempPt, currentPt;
							if(m_pointDistMap[polyNextPt] > lastFinalPtDist + crossEdgeLength){
								Point_2 firstPt(pAP.polyCurrentPt.get<0>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingX, pAP.polyCurrentPt.get<1>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingY);
								registerRegularPoint(vid, firstPt, pAP.polyCurrentPt);
								m_newAddedSeg.push_back(Segment_2(firstPt, pAP.polyCurrentPt));
								vidFGlist.push_back(vid);

								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, firstPt, lastFinalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								lastFinalPt = firstPt;
								vidFGlist.clear();
								vIDCount ++;
								vid++;
								additiveLength = lastFinalPtDist + crossEdgeLength;
								currentPt = firstPt;
								vidFGlist.push_back(m_pointVidMap[currentPt]);
								while(m_pointDistMap[polyNextPt] > additiveLength + crossEdgeLength){
									tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
									tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
									registerRegularPoint(vid, tempPt, currentPt);
									m_newAddedSeg.push_back(Segment_2(tempPt, currentPt));
									vidFGlist.push_back(vid);
									reverseVector(vidFGlist); // not implemented yet
									registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
									m_newAddedPt.push_back(vIDCount);
									vidFGlist.clear();
									vidFGlist.push_back(vid);
									additiveLength += crossEdgeLength;
									currentPt = tempPt;
									vid++;
									vIDCount++;
								}
								lastFinalPtDist = additiveLength;
								lastFinalPt = currentPt;
								registerRegularPoint(vid, polyNextPt, currentPt);
								m_newAddedSeg.push_back(Segment_2(polyNextPt, currentPt));
								vid++;
								additiveLength = m_pointDistMap[polyNextPt];
								pAP.polyPreviousPt = pAP.polyCurrentPt;
								pAP.polyCurrentPt = polyNextPt;
								vidFGlist.push_back(m_pointVidMap[pAP.polyCurrentPt]);
							}else{
								registerRegularPoint(vid, polyNextPt, pAP.polyCurrentPt);
								m_newAddedSeg.push_back(Segment_2(polyNextPt, pAP.polyCurrentPt));
								vid++;
								additiveLength += bg::distance(pAP.polyCurrentPt, polyNextPt);
								pAP.polyPreviousPt = pAP.polyCurrentPt;
								pAP.polyCurrentPt = polyNextPt;
								vidFGlist.push_back(m_pointVidMap[pAP.polyCurrentPt]);
							}

						}	


						Point_2 tempPt, currentPt;
						double addingX = getAddingX(polyNextPt, pAP.polyCurrentPt);
						double addingY = getAddingY(polyNextPt, pAP.polyCurrentPt);
						m_pointDistMap[endPoint] = overlapLength - endHalfEdgeLength;
						if(m_pointDistMap[endPoint] > lastFinalPtDist + crossEdgeLength){
							Point_2 firstPt(pAP.polyCurrentPt.get<0>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingX, pAP.polyCurrentPt.get<1>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingY);
							registerRegularPoint(vid, firstPt, pAP.polyCurrentPt);
							m_newAddedSeg.push_back(Segment_2(firstPt, pAP.polyCurrentPt));
							vidFGlist.push_back(vid);

							reverseVector(vidFGlist); // not implemented yet
							registerFinalPoint(vIDCount, vid, firstPt, lastFinalPt, vidFGlist);
							m_newAddedPt.push_back(vIDCount);
							vidFGlist.clear();
							vIDCount ++;
							vid++;
							additiveLength = lastFinalPtDist + crossEdgeLength;
							currentPt = firstPt;
							lastFinalPt = currentPt;
							vidFGlist.push_back(m_pointVidMap[currentPt]);




							while(m_pointDistMap[endPoint] > additiveLength + crossEdgeLength){
								tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
								tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
								registerRegularPoint(vid, tempPt, currentPt);
								m_newAddedSeg.push_back(Segment_2(tempPt, currentPt));
								vidFGlist.push_back(vid);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								vidFGlist.clear();
								vidFGlist.push_back(vid);
								additiveLength += crossEdgeLength;
								currentPt = tempPt;
								lastFinalPt = currentPt;
								vid++;
								vIDCount++;

					
							}
							registerRegularPoint(vid, endPoint, currentPt);
							m_newAddedSeg.push_back(Segment_2(endPoint, currentPt));

							if(endHalfEdgeLength < crossEdgeLength){
								registerRegularPoint(vid, endPoint, crossEndPt);
								m_newAddedSeg.push_back(Segment_2(endPoint, crossEndPt));
								vidFGlist.push_back(vid);
								vid++;
							
								vidFGlist.push_back(m_pointVidMap[crossEndPt]);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, currentPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								vidFGlist.clear();
							}else{
								vid++;
								double addingX = getAddingX(endPoint, crossEndPt);
								double addingY = getAddingY(endPoint, crossEndPt);
								Point_2 finalPt(crossEndPt.get<0>()+crossEdgeLength*addingX, crossEndPt.get<1>()+crossEdgeLength*addingY);
								registerRegularPoint(vid, finalPt, endPoint);
								m_newAddedSeg.push_back(Segment_2(finalPt, endPoint));
								vidFGlist.push_back(m_pointVidMap[endPoint]);
								vidFGlist.push_back(vid);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, finalPt, lastFinalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								registerRegularPoint(vid, finalPt, crossEndPt);
								m_newAddedSeg.push_back(Segment_2(finalPt, crossEndPt));
								vid ++;
								vIDCount ++;
								vidFGlist.clear();
								vidFGlist.push_back(m_pointVidMap[finalPt]);
								vidFGlist.push_back(m_pointVidMap[crossEndPt]);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, finalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								vidFGlist.clear();
							}
							

						}else{


							registerRegularPoint(vid, endPoint, pAP.polyCurrentPt);
							m_newAddedSeg.push_back(Segment_2(endPoint, pAP.polyCurrentPt));
							if(endHalfEdgeLength < crossEdgeLength){
								registerRegularPoint(vid, endPoint, crossEndPt);
								m_newAddedSeg.push_back(Segment_2(endPoint, crossEndPt));
								vidFGlist.push_back(vid);
								vid++;
							
								vidFGlist.push_back(m_pointVidMap[crossEndPt]);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, lastFinalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								vidFGlist.clear();
							}else{
								vid++;
								double addingX = getAddingX(endPoint, crossEndPt);
								double addingY = getAddingY(endPoint, crossEndPt);
								Point_2 finalPt(crossEndPt.get<0>()+crossEdgeLength*addingX, crossEndPt.get<1>()+crossEdgeLength*addingY);
								registerRegularPoint(vid, finalPt, endPoint);
								m_newAddedSeg.push_back(Segment_2(finalPt, endPoint));
								vidFGlist.push_back(m_pointVidMap[endPoint]);
								vidFGlist.push_back(vid);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, finalPt, lastFinalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								registerRegularPoint(vid, finalPt, crossEndPt);
								m_newAddedSeg.push_back(Segment_2(finalPt, crossEndPt));
								vid ++;
								vIDCount ++;
								vidFGlist.clear();
								vidFGlist.push_back(m_pointVidMap[finalPt]);
								vidFGlist.push_back(m_pointVidMap[crossEndPt]);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, finalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								vidFGlist.clear();
							}
							
						}


					}else{
						registerRegularPoint(vid, startPoint, crossStartPt);
						m_newAddedSeg.push_back(Segment_2(startPoint, crossStartPt));
						vid++;
						registerRegularPoint(vid, pAP.polyCurrentPt, startPoint);
						m_newAddedSeg.push_back(Segment_2(pAP.polyCurrentPt, startPoint));
						vidFGlist.clear();
						vidFGlist.push_back(m_pointVidMap[crossStartPt]);
						vidFGlist.push_back(m_pointVidMap[startPoint]);
						vidFGlist.push_back(vid);
						vid++;
						additiveLength = startHalfEdgeLength;
						additiveLength += bg::distance(startPoint, pAP.polyCurrentPt);
						double lastFinalPtDist = 0;
						Point_2 lastFinalPt = crossStartPt;
						while(!bg::equals((polyNextPt = nextInPolygon(pAP.polyCurrentPt, pAP.polyPreviousPt, poly, "884")), pAP.polyEndAfterPt)){
							m_pointDistMap[polyNextPt] = m_pointDistMap[pAP.polyCurrentPt] + bg::distance(pAP.polyCurrentPt, polyNextPt);
							double addingX = getAddingX(polyNextPt, pAP.polyCurrentPt);
							double addingY = getAddingY(polyNextPt, pAP.polyCurrentPt);
							Point_2 tempPt, currentPt;
							if(m_pointDistMap[polyNextPt] > lastFinalPtDist + crossEdgeLength){
								Point_2 firstPt(pAP.polyCurrentPt.get<0>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingX, pAP.polyCurrentPt.get<1>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingY);
								registerRegularPoint(vid, firstPt, pAP.polyCurrentPt);
								m_newAddedSeg.push_back(Segment_2(firstPt, pAP.polyCurrentPt));
								
								vidFGlist.push_back(vid);

								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, firstPt, lastFinalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								lastFinalPt = firstPt;
								vidFGlist.clear();
								vIDCount ++;
								vid++;
								additiveLength = lastFinalPtDist + crossEdgeLength;
								currentPt = firstPt;
								vidFGlist.push_back(m_pointVidMap[currentPt]);
								while(m_pointDistMap[polyNextPt] > additiveLength + crossEdgeLength){
									tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
									tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
									registerRegularPoint(vid, tempPt, currentPt);
									m_newAddedSeg.push_back(Segment_2(tempPt, currentPt));
									
									vidFGlist.push_back(vid);
									reverseVector(vidFGlist); // not implemented yet
									registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
									m_newAddedPt.push_back(vIDCount);
									vidFGlist.clear();
									vidFGlist.push_back(vid);
									additiveLength += crossEdgeLength;
									currentPt = tempPt;
									vid++;
									vIDCount++;
								}
								lastFinalPtDist = additiveLength;
								lastFinalPt = currentPt;
								registerRegularPoint(vid, polyNextPt, currentPt);
								m_newAddedSeg.push_back(Segment_2(polyNextPt, currentPt));
								vid++;
								additiveLength = m_pointDistMap[polyNextPt];
								pAP.polyPreviousPt = pAP.polyCurrentPt;
								pAP.polyCurrentPt = polyNextPt;
								vidFGlist.push_back(m_pointVidMap[pAP.polyCurrentPt]);
							}else{
								registerRegularPoint(vid, polyNextPt, pAP.polyCurrentPt);
								m_newAddedSeg.push_back(Segment_2(polyNextPt, pAP.polyCurrentPt));
								vid++;
								additiveLength += bg::distance(pAP.polyCurrentPt, polyNextPt);
								pAP.polyPreviousPt = pAP.polyCurrentPt;
								pAP.polyCurrentPt = polyNextPt;
								vidFGlist.push_back(m_pointVidMap[pAP.polyCurrentPt]);
							}

						}	

						double addingX = getAddingX(polyNextPt, pAP.polyCurrentPt);
						double addingY = getAddingY(polyNextPt, pAP.polyCurrentPt);
						m_pointDistMap[endPoint] = overlapLength - endHalfEdgeLength;
						Point_2 tempPt, currentPt;
						if(m_pointDistMap[endPoint] > lastFinalPtDist + crossEdgeLength){
							Point_2 firstPt(pAP.polyCurrentPt.get<0>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingX, pAP.polyCurrentPt.get<1>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingY);
							registerRegularPoint(vid, firstPt, pAP.polyCurrentPt);
							m_newAddedSeg.push_back(Segment_2(firstPt, pAP.polyCurrentPt));
							vidFGlist.push_back(vid);

							reverseVector(vidFGlist); // not implemented yet
							registerFinalPoint(vIDCount, vid, firstPt, lastFinalPt, vidFGlist);
							m_newAddedPt.push_back(vIDCount);
							vidFGlist.clear();
							vIDCount ++;
							vid++;
							additiveLength = lastFinalPtDist + crossEdgeLength;
							currentPt = firstPt;
							vidFGlist.push_back(m_pointVidMap[currentPt]);




							while(m_pointDistMap[endPoint] > additiveLength + crossEdgeLength){
								tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
								tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
								registerRegularPoint(vid, tempPt, currentPt);
								m_newAddedSeg.push_back(Segment_2(tempPt, currentPt));
								vidFGlist.push_back(vid);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								vidFGlist.clear();
								vidFGlist.push_back(vid);
								additiveLength += crossEdgeLength;
								currentPt = tempPt;
								vid++;
								vIDCount++;

					
							}
							registerRegularPoint(vid, endPoint, currentPt);
							m_newAddedSeg.push_back(Segment_2(endPoint, currentPt));
							if(endHalfEdgeLength < crossEdgeLength){
								registerRegularPoint(vid, endPoint, crossEndPt);
								m_newAddedSeg.push_back(Segment_2(endPoint, crossEndPt));
								vidFGlist.push_back(vid);
								vid++;
							
								vidFGlist.push_back(m_pointVidMap[crossEndPt]);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, currentPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								vidFGlist.clear();
							}else{
								vid++;
								double addingX = getAddingX(endPoint, crossEndPt);
								double addingY = getAddingY(endPoint, crossEndPt);
								Point_2 finalPt(crossEndPt.get<0>()+crossEdgeLength*addingX, crossEndPt.get<1>()+crossEdgeLength*addingY);
								registerRegularPoint(vid, finalPt, endPoint);
								m_newAddedSeg.push_back(Segment_2(finalPt, endPoint));
								vidFGlist.push_back(m_pointVidMap[endPoint]);
								vidFGlist.push_back(vid);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, finalPt, lastFinalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								registerRegularPoint(vid, finalPt, crossEndPt);
								m_newAddedSeg.push_back(Segment_2(finalPt, crossEndPt));
								vid ++;
								vIDCount ++;
								vidFGlist.clear();
								vidFGlist.push_back(m_pointVidMap[finalPt]);
								vidFGlist.push_back(m_pointVidMap[crossEndPt]);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, finalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								vidFGlist.clear();
							}



							// registerRegularPoint(vid, endPoint, crossEndPt);
							// vidFGlist.push_back(vid);
							// vid++;
							
							// vidFGlist.push_back(m_pointVidMap[crossEndPt]);
							// reverseVector(vidFGlist); // not implemented yet
							// registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, currentPt, vidFGlist);
							


							

						}else{
							// registerRegularPoint(vid, endPoint, pAP.polyCurrentPt);
							// registerRegularPoint(vid, endPoint, crossEndPt);
							// vidFGlist.push_back(vid);
							// vid++;
							// vidFGlist.push_back(m_pointVidMap[crossEndPt]);
							// reverseVector(vidFGlist);
							// registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, lastFinalPt, vidFGlist);
						



							registerRegularPoint(vid, endPoint, pAP.polyCurrentPt);
							m_newAddedSeg.push_back(Segment_2(endPoint, pAP.polyCurrentPt));
							if(endHalfEdgeLength < crossEdgeLength){
								registerRegularPoint(vid, endPoint, crossEndPt);
								m_newAddedSeg.push_back(Segment_2(endPoint, crossEndPt));
								vidFGlist.push_back(vid);
								vid++;
							
								vidFGlist.push_back(m_pointVidMap[crossEndPt]);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, lastFinalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								vidFGlist.clear();
							}else{
								vid++;
								double addingX = getAddingX(endPoint, crossEndPt);
								double addingY = getAddingY(endPoint, crossEndPt);
								Point_2 finalPt(crossEndPt.get<0>()+crossEdgeLength*addingX, crossEndPt.get<1>()+crossEdgeLength*addingY);
								registerRegularPoint(vid, finalPt, endPoint);
								m_newAddedSeg.push_back(Segment_2(finalPt, endPoint));
								vidFGlist.push_back(m_pointVidMap[endPoint]);
								vidFGlist.push_back(vid);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, finalPt, lastFinalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								registerRegularPoint(vid, finalPt, crossEndPt);
								m_newAddedSeg.push_back(Segment_2(finalPt, crossEndPt));
								vid ++;
								vIDCount ++;
								vidFGlist.clear();
								vidFGlist.push_back(m_pointVidMap[finalPt]);
								vidFGlist.push_back(m_pointVidMap[crossEndPt]);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, finalPt, vidFGlist);
								m_newAddedPt.push_back(vIDCount);
								vidFGlist.clear();
							}


						}
					}

	}// cross multiple edges






}

pointAlongPoly Roadmap::getStartEndAroundPoly_2(Polygon_2 poly, Polygon_2 oppo_poly, Graph jointCycle, Point_2 startPivot, Point_2 startPivotNext, Point_2 endPivot, Point_2 endPivotNext, Point_2 &startPt, Point_2 &endPt, Point_2 &startCyclePt, Point_2 &endCyclePt, double step, std::vector<Polygon_2> oppoPolyList){
	//startPt = getNewPoint(startPivot, step*leftAddingX_1, step*leftAddingY_1);
	//endPt = getNewPoint(pivotPt, step*rightAddingX_1, step*rightAddingY_1);
	startPt = startPivot;
	endPt = endPivot; 



	bool found = false;
	int time = 0;

	Point_2 nextPolyPt, nextnextPolyPt, previousPolyPt, currentTestPt;
	Point_2 polyEndPt, polyCurrentPt, polyPreviousPt, polyNextPt, polyEndAfterPt;
// for startPt_1
	nextPolyPt = startPivotNext;
	nextnextPolyPt = nextInPolygon(nextPolyPt, startPt, poly, "1516");
	previousPolyPt = nextInPolygon(nextPolyPt, nextnextPolyPt, poly, "1517");

	while(!found){
		if(time == 0){
			currentTestPt = startPt;
		}else{ 

			currentTestPt = getNextPoint(currentTestPt, nextPolyPt, poly, step);	
		}
		std::cout<<"currentTestPt-----startPt----"<<currentTestPt.get<0>()<<","<<currentTestPt.get<1>()<<std::endl;
		std::cout<<"nextPolyPt when startPt---"<<nextPolyPt.get<0>()<<","<<nextPolyPt.get<1>()<<std::endl;
		if(bg::equals(nextPolyPt, previousPolyPt)){
				found = false;
				std::cout<<"can not find suitable startPt"<<std::endl;
				break;
		}
		Point_2 lastPt, cyclePt;
		for(std::set<int>::iterator vi = jointCycle.getVertexSet().begin(); vi != jointCycle.getVertexSet().end(); vi ++){
			cyclePt = m_vidPointMap[*vi];
			if(m_pointUsed[*vi]){
				continue;
			}
			if (bg::equals(lastPt, cyclePt)) {
				break;
			}
			if(checkConnectSeg_2(jointCycle, currentTestPt, cyclePt, poly, oppo_poly, 1.35*m_edgeLength, 0.7*m_edgeLength) && checkConnectAngle(jointCycle, currentTestPt, cyclePt, oppoPolyList)){
				m_pointUsed[*vi] = true;
				found = true;
				startCyclePt = cyclePt;
				startPt = currentTestPt;
				break;
			}
			lastPt = cyclePt;
			if (vi == jointCycle.getVertexSet().end()) {
				break;
			}
		}
		
		time ++;
		
	}

	std::cout<<"startPt_1"<<startPt.get<0>()<<","<<startPt.get<1>()<<std::endl;
	polyPreviousPt = nextPolyPt;
	if (time == 1) {
		polyCurrentPt = startPt;
	}
	else {
		polyCurrentPt = nextInPolygon(startPt, nextPolyPt, poly, "426");
	}
	
	// std::cout<<"poly_1 vertexes"<<std::endl;
	// for(int i = 0; i < poly_1.outer().size(); i ++){
	// 	std::cout<<poly_1.outer()[i].get<0>()<<","<<poly_1.outer()[i].get<1>()<<std::endl;
	// }
	//  for endPt_1
	found = false;
	time = 0;
	nextPolyPt = endPivotNext;
	nextnextPolyPt = nextInPolygon(nextPolyPt, endPt, poly, "436");
	previousPolyPt = nextInPolygon(nextPolyPt, nextnextPolyPt, poly, "437");

	while(!found){
		if(time == 0){
			currentTestPt = endPt;
		}else{
			currentTestPt = getNextPoint(currentTestPt, nextPolyPt, poly, step);	
		}
		if(bg::equals(nextPolyPt, previousPolyPt)){
				found = false;
				std::cout<<"can not find suitable endPt_1"<<std::endl;
				break;
		}
		Point_2 lastPt, cyclePt;
		for(std::set<int>::iterator vi = jointCycle.getVertexSet().begin(); vi != jointCycle.getVertexSet().end(); vi ++){
			cyclePt = m_vidPointMap[*vi];
			
			if(m_pointUsed[*vi]){
				continue;
			}
			if (bg::equals(lastPt, cyclePt)) {
				break;
			}
			if(checkConnectSeg_2(jointCycle, currentTestPt, cyclePt, poly, oppo_poly, 1.35*m_edgeLength, 0.7*m_edgeLength)  && checkConnectAngle(jointCycle, currentTestPt, cyclePt, oppoPolyList)){
				m_pointUsed[*vi] = true;
				found = true;
				endCyclePt = cyclePt;
				endPt = currentTestPt;
				break;
			}
			lastPt = cyclePt;
			
			if (vi == jointCycle.getVertexSet().end()) {
				break;
			}
		}
		
		time ++;
	}
	std::cout << "endPt_1" << endPt.get<0>() << "," << endPt.get<1>() << std::endl;
	if (time == 1) {
		polyEndPt = endPt;
	}
	else {
		polyEndPt = nextInPolygon(endPt, nextPolyPt, poly, "1784");
	}
	
	polyEndAfterPt = nextPolyPt;
//	std::cout<<"poly_1 polyEndPt"<<polyEndPt.get<0>()<<","<<polyEndPt.get<1>()<<std::endl;
//	std::cout<<"poly_1 polyEndAfterPt"<<polyEndAfterPt.get<0>()<<","<<polyEndAfterPt.get<1>()<<std::endl;
	pointAlongPoly pAP;
	pAP.polyCurrentPt = polyCurrentPt;
	pAP.polyEndPt = polyEndPt;
	pAP.polyPreviousPt = polyPreviousPt;
	pAP.polyEndAfterPt = polyEndAfterPt;
	return pAP;


}
pointAlongPoly Roadmap::getStartEndAroundPoly(Polygon_2 poly, Polygon_2 oppo_poly, Segment_2 shortestSeg, Graph jointCycle, Point_2 pivotPt, Point_2 leftNextPt, Point_2 rightNextPt, Point_2 &startPt, Point_2 &endPt, Point_2 &startCyclePt, Point_2 &endCyclePt, double step, std::vector<Polygon_2> oppoPolyList){
	double leftAddingX_1 = getAddingX(leftNextPt, pivotPt);
	double leftAddingY_1 = getAddingY(leftNextPt, pivotPt);
	double rightAddingX_1 = getAddingX(rightNextPt, pivotPt);
	double rightAddingY_1 = getAddingY(rightNextPt, pivotPt);

	startPt = getNewPoint(pivotPt, 0.001*leftAddingX_1, 0.001*leftAddingY_1);
	endPt = getNewPoint(pivotPt, 0.001*rightAddingX_1, 0.001*rightAddingY_1);


	bool found = false;
	int time = 0;
	Point_2 nextPolyPt, nextnextPolyPt, previousPolyPt, currentTestPt;
	Point_2 polyEndPt, polyCurrentPt, polyPreviousPt, polyNextPt, polyEndAfterPt;
// for startPt_1
	nextPolyPt = leftNextPt;
	nextnextPolyPt = nextInPolygon(nextPolyPt, startPt, poly, "390");
	previousPolyPt = nextInPolygon(nextPolyPt, nextnextPolyPt, poly, "391");

	while(!found){
		if(time == 0){
			currentTestPt = startPt;
		}else{
			currentTestPt = getNextPoint(currentTestPt, nextPolyPt, poly, step);	
		}
		std::cout<<"currentTestPt-----startPt----"<<currentTestPt.get<0>()<<","<<currentTestPt.get<1>()<<std::endl;
		std::cout<<"nextPolyPt when startPt---"<<nextPolyPt.get<0>()<<","<<nextPolyPt.get<1>()<<std::endl;
		if(bg::equals(nextPolyPt, previousPolyPt)){
				found = false;
				std::cout<<"can not find suitable startPt"<<std::endl;
				break;
		}

		for(std::set<int>::iterator vi = jointCycle.getVertexSet().begin(); vi != jointCycle.getVertexSet().end(); vi ++){
			Point_2 cyclePt = m_vidPointMap[*vi];
			if(m_pointUsed[*vi]){
				continue;
			}

			if(checkConnectSeg(currentTestPt, cyclePt, shortestSeg, poly, oppo_poly, 1.35*m_edgeLength, 0.8*m_edgeLength) /*&& checkConnectAngle(m_jointCycle, currentTestPt, cyclePt, oppoPolyList)*/){
				m_pointUsed[*vi] = true;
				found = true;
				startCyclePt = cyclePt;
				startPt = currentTestPt;
				break;
			}
		}
		
		time ++;
		
	}
	std::cout<<"startPt_1"<<startPt.get<0>()<<","<<startPt.get<1>()<<std::endl;
	polyPreviousPt = nextPolyPt;
	polyCurrentPt = nextInPolygon(startPt, nextPolyPt, poly, "426");
	// std::cout<<"poly_1 vertexes"<<std::endl;
	// for(int i = 0; i < poly_1.outer().size(); i ++){
	// 	std::cout<<poly_1.outer()[i].get<0>()<<","<<poly_1.outer()[i].get<1>()<<std::endl;
	// }

//  for endPt_1
	found = false;
	time = 0;
	nextPolyPt = rightNextPt;
	nextnextPolyPt = nextInPolygon(nextPolyPt, endPt, poly, "436");
	previousPolyPt = nextInPolygon(nextPolyPt, nextnextPolyPt, poly, "437");


	while(!found){
		if(time == 0){
			currentTestPt = endPt;
		}else{
			currentTestPt = getNextPoint(currentTestPt, nextPolyPt, poly, step);	
		}
		if(bg::equals(nextPolyPt, previousPolyPt)){
				found = false;
				std::cout<<"can not find suitable endPt_1"<<std::endl;
				break;
		}

		for(std::set<int>::iterator vi = jointCycle.getVertexSet().begin(); vi != jointCycle.getVertexSet().end(); vi ++){
			Point_2 cyclePt = m_vidPointMap[*vi];
			if(m_pointUsed[*vi]){
				continue;
			}

			if(checkConnectSeg(currentTestPt, cyclePt, shortestSeg, poly, oppo_poly, 1.35*m_edgeLength, 0.8*m_edgeLength) /* && checkConnectAngle(m_jointCycle, currentTestPt, cyclePt, oppoPolyList)*/){
				m_pointUsed[*vi] = true;
				found = true;
				endCyclePt = cyclePt;
				endPt = currentTestPt;
				break;
			}
		}
		
		time ++;
	}

	polyEndPt = nextInPolygon(endPt, nextPolyPt, poly,"1784");
	polyEndAfterPt = nextPolyPt;
	std::cout<<"poly_1 polyEndPt"<<polyEndPt.get<0>()<<","<<polyEndPt.get<1>()<<std::endl;
	std::cout<<"poly_1 polyEndAfterPt"<<polyEndAfterPt.get<0>()<<","<<polyEndAfterPt.get<1>()<<std::endl;
	pointAlongPoly pAP;
	pAP.polyCurrentPt = polyCurrentPt;
	pAP.polyEndPt = polyEndPt;
	pAP.polyPreviousPt = polyPreviousPt;
	pAP.polyEndAfterPt = polyEndAfterPt;
	return pAP;
}


bool Roadmap::isGraphIntersection(Graph g1, Graph g2){
	Polygon_2 p1 = graphToPolygon(g1);
	Polygon_2 p2 = graphToPolygon(g2);
	return bg::intersects(p1, p2);
}

void Roadmap::getShortestBetweenTwoPtPoly(Polygon_2 poly, Point_2 forward, Point_2 backward, Point_2& previousPolyPt, Point_2& currentPolyPt, Polygon_2 oppo_poly){
	double shortestDist = std::numeric_limits<double>::max();
	double shortestPolyDist = std::numeric_limits<double>::max();
	int shortestSegIndex;
	Segment_2 shortestSeg;
	Point_2 tempForward, tempBackward;
	for(int j = 0; j < poly.outer().size(); j++){
		Segment_2 seg;
		if(j == poly.outer().size()-1){
			seg.first = poly.outer()[j];
			seg.second = poly.outer()[0];
	
		}else{
			seg.first = poly.outer()[j];
			seg.second = poly.outer()[j+1];
		}
		
		//Point_2 tempShortestPt;
		double dist;
		dist = bg::distance(backward, seg);
		//dist = shortestDistPair(seg, p, tempShortestPt);
		if(dist < shortestDist){
			shortestDist = dist;
			//shortestPt = tempShortestPt;
		//	shortestPolyPt = p;
			shortestSeg = seg;
			//shortestSegIndex = j;
		}
	}

	for (Polygon2_list::iterator po = m_objectPolyList.begin(); po != m_objectPolyList.end(); po++) {
		if (!bg::equals(*po, poly)) {
			double tempDistance = bg::distance(*po, shortestSeg);
			if (tempDistance < shortestPolyDist) {
				shortestPolyDist = tempDistance;
				oppo_poly = *po;
			}
		}
	}

	shortestDistPair(shortestSeg, forward, tempForward);
	shortestDistPair(shortestSeg, backward, tempBackward);
	std::cout<<"bg::distance(tempForward, shortestSeg.first)="<<bg::distance(tempForward, shortestSeg.first);
	std::cout<<"bg::distance(tempBackward, shortestSeg.first)="<<bg::distance(tempBackward, shortestSeg.first);	 
	if(bg::distance(tempForward, shortestSeg.first) < bg::distance(tempBackward, shortestSeg.first)){
		//previousPolyPt = nextInPolygon(shortestSeg.second, tempBackward, poly, "1638");
		//currentPolyPt = shortestSeg.second;
		//previousPolyPt = shortestSeg.second;
		//currentPolyPt = shortestSeg.first;
		previousPolyPt = shortestSeg.first;
		currentPolyPt = nextInPolygon(shortestSeg.first, shortestSeg.second, poly, "2607");
	}else if(bg::distance(tempForward, shortestSeg.first) > bg::distance(tempBackward, shortestSeg.first)){
		//previousPolyPt = nextInPolygon(shortestSeg.first, tempBackward, poly, "1640");
		//currentPolyPt = shortestSeg.first;
		//previousPolyPt = shortestSeg.first;
		//currentPolyPt = shortestSeg.second;
		previousPolyPt = shortestSeg.second;
		currentPolyPt = nextInPolygon(shortestSeg.second, shortestSeg.first, poly, "2607");
	}else if(bg::equals(tempForward, tempBackward)){
		if(bg::equals(tempForward, shortestSeg.first)){


			if(bg::distance(shortestSeg.first, oppo_poly) < bg::distance(shortestSeg.second, oppo_poly)){
				//previousPolyPt = nextInPolygon(shortestSeg.first, shortestSeg.second, poly, "2498");
				//currentPolyPt = shortestSeg.first;
				//previousPolyPt = shortestSeg.first;
				//currentPolyPt = shortestSeg.second;
				previousPolyPt = shortestSeg.second;
				currentPolyPt = nextInPolygon(shortestSeg.second, shortestSeg.first, poly, "2607");
			}else{
				//previousPolyPt = nextInPolygon(shortestSeg.second, shortestSeg.first, poly, "2501");
				//currentPolyPt = shortestSeg.second;
				//previousPolyPt = shortestSeg.second;
				//currentPolyPt = shortestSeg.first;
				previousPolyPt = shortestSeg.first;
				currentPolyPt = nextInPolygon(shortestSeg.first, shortestSeg.second, poly, "2607");
			}
			// if(bg::distance(forward, tempForward) < bg::distance(backward, tempForward)){
			// 	currentPolyPt = tempForward;
			// 	previousPolyPt = nextInPolygon(shortestSeg.first, shortestSeg.second, poly, "1638");
			// }else{
			// 	currentPolyPt = tempForward;
			// 	previousPolyPt = shortestSeg.second;
			// }
		}else if(bg::equals(tempForward, shortestSeg.second)){
			if(bg::distance(shortestSeg.first, oppo_poly) < bg::distance(shortestSeg.second, oppo_poly)){
				//previousPolyPt = nextInPolygon(shortestSeg.first, shortestSeg.second, poly, "2498");
				//currentPolyPt = shortestSeg.first;
				//previousPolyPt = shortestSeg.first;
				//currentPolyPt = shortestSeg.second;
				previousPolyPt = shortestSeg.second;
				currentPolyPt = nextInPolygon(shortestSeg.second, shortestSeg.first, poly, "2607");
			}else{
				//previousPolyPt = nextInPolygon(shortestSeg.second, shortestSeg.first, poly, "2501");
				//currentPolyPt = shortestSeg.second;
				//previousPolyPt = shortestSeg.second;
				//currentPolyPt = shortestSeg.first;
				previousPolyPt = shortestSeg.first;
				currentPolyPt = nextInPolygon(shortestSeg.first, shortestSeg.second, poly, "2607");
			}
		}
		
	}


}

void Roadmap::getShortestBetweenTwoPoly(Polygon_2 poly_1, Polygon_2 poly_2, double &shortestDist, Point_2& shortestPt, Point_2& shortestPolyPt, Segment_2 &intersectSeg, int &shortestPtIndex){
	for(int i = 0; i < poly_1.outer().size();i ++){
		Point_2 p = poly_1.outer()[i];
		for(int j = 0; j < poly_2.outer().size() - 1; j++){
			Segment_2 seg(poly_2.outer()[j], poly_2.outer()[j+1]);
			Point_2 tempShortestPt;
			double dist;
			dist = shortestDistPair(seg, p, tempShortestPt);
			if(dist < shortestDist){
				shortestDist = dist;
				shortestPt = tempShortestPt;
				shortestPolyPt = p;
				intersectSeg = seg;
				shortestPtIndex = i;
			}
		}
	}

	for(int i = 0; i < poly_2.outer().size();i ++){
		Point_2 p = poly_2.outer()[i];
		for(int j = 0; j < poly_1.outer().size() - 1; j++){
			Segment_2 seg(poly_1.outer()[j], poly_1.outer()[j+1]);
			Point_2 tempShortestPt;
			double dist;
			dist = shortestDistPair(seg, p, tempShortestPt);
			if(dist < shortestDist){
				shortestDist = dist;
				shortestPt = tempShortestPt;
				shortestPolyPt = p;
				intersectSeg = seg;
				shortestPtIndex = i;
			}
		}
	}

}

Point_2 Roadmap::getNextPoint(Point_2 currentPt, Point_2 &nextPolyPt, Polygon_2 poly, double length){
	Point_2 nextPt;
	Point_2 nextnextPolyPt;
	Point_2 previousPt;
	double currentLength = 0;
	currentLength = bg::distance(currentPt, nextPolyPt);
	double addingX = getAddingX(nextPolyPt, currentPt);
	double addingY = getAddingY(nextPolyPt, currentPt);
	if(length <= currentLength){
		nextPt = getNewPoint(currentPt, length*addingX, length*addingY);
	}else{
		while(currentLength < length){
			
			nextnextPolyPt = nextInPolygon(nextPolyPt, currentPt, poly, "1638");
			currentLength += bg::distance(nextPolyPt, nextnextPolyPt);
			previousPt = currentPt;
			currentPt = nextPolyPt;
			nextPolyPt = nextnextPolyPt;

		}
		
		
		addingX = getAddingX(nextPolyPt, currentPt);
		addingY = getAddingY(nextPolyPt, currentPt);
		double newLength = length - (currentLength - bg::distance(currentPt, nextPolyPt));
		nextPt = getNewPoint(currentPt, newLength*addingX, newLength*addingY);
		
	}

	return nextPt;
}
bool Roadmap::isPtOnPoly(Point_2 pt, Polygon_2 poly){
	Segment_2 seg;
	bool is = false;
	for(int i = 0;i < poly.outer().size();i++){
		if(i >= 0 && i < poly.outer().size()-1){
			seg.first = poly.outer()[i];
			seg.second = poly.outer()[i+1];
			if(isPointInSeg(pt, seg)){
				return true;
			}
		}else if(i == poly.outer().size()-1){
			seg.first = poly.outer()[poly.outer().size()-1];
			seg.second = poly.outer()[0];
			if(isPointInSeg(pt, seg)){
				return true;
			}
		}
	}
	return is;
}
bool Roadmap::isPointInSeg(Point_2 p, Segment_2 seg){
	Point_2 first = seg.first;
	Point_2 second = seg.second;
	bool notout_1 = false;
	bool notout_2 = false;
	double x1, x2, y1, y2, x, y, vx, vy, mag, dvx, dvy, vcx, vcy, magc, dvcx, dvcy;
	x1 = 0.0;
	x2 = 0.0;
	y1 = 0.0;
	y2 = 0.0;
	x = 0.0;
	y = 0.0;
	vx = 0.0;
	vy = 0.0;
	mag = 0.0;
	dvx = 0.0;
	dvy = 0.0;
	vcx = 0.0;
	vcy = 0.0;
	magc = 0.0;
	dvcx = 0.0;
	dvcy = 0.0;  
	if(p.get<0>() == first.get<0>() && p.get<1>() == first.get<1>()){
		return true;
	}
	if(p.get<0>() == second.get<0>() && p.get<1>() == second.get<1>()){
		return true;
	}
	x1 = first.get<0>();
	x2 = second.get<0>();
	y1 = first.get<1>();
	y2 = second.get<1>();
	x = p.get<0>();
	y = p.get<1>();
	vx = x2 - x1;
    vy = y2 - y1;
    mag = sqrt(vx*vx + vy*vy);
    // need to get the unit vector (direction)
    dvx = vx/mag; // this would be the unit vector (direction) x for the line
    dvy = vy/mag; // this would be the unit vector (direction) y for the line

    vcx = x - x1;
    vcy = y - y1;
    magc = sqrt(vcx*vcx + vcy*vcy);
    // need to get the unit vector (direction)
    dvcx = vcx/magc; // this would be the unit vector (direction) x for the point
    dvcy = vcy/magc; // this would be the unit vector (direction) y for the point

	//std::cout<<"dvcx:"<<dvcx<<" dvcy:"<<dvcy<<std::endl;   
	//std::cout<<"dvx:"<<dvx<<" dvy:"<<dvy<<std::endl;
	if (std::abs(dvcx-dvx) < 0.0001 && std::abs(dvcy-dvy) < 0.0001)
	{
		notout_1 = true;
    // the point is (more or less) on the line!
	}else{
		notout_1 =  false;
	}
	vx = x1 - x2;
	vy = y1 - y2;
	mag = sqrt(vx*vx + vy*vy);
	// need to get the unit vector (direction)
	dvx = vx / mag; // this would be the unit vector (direction) x for the line
	dvy = vy / mag; // this would be the unit vector (direction) y for the line

	vcx = x - x2;
	vcy = y - y2;
	magc = sqrt(vcx*vcx + vcy*vcy);
	// need to get the unit vector (direction)
	dvcx = vcx / magc; // this would be the unit vector (direction) x for the point
	dvcy = vcy / magc; // this would be the unit vector (direction) y for the point

					   //std::cout<<"dvcx:"<<dvcx<<" dvcy:"<<dvcy<<std::endl;   
					   //std::cout<<"dvx:"<<dvx<<" dvy:"<<dvy<<std::endl;
	if (std::abs(dvcx - dvx) < 0.0001 && std::abs(dvcy - dvy) < 0.0001)
	{
		notout_2 = true;
		// the point is (more or less) on the line!
	}
	else {
		notout_2 = false;
	}
	if (notout_1 && notout_2) {
		return true;
	}
	else {
		return false;
	}
}

bool Roadmap::isSegInKey(Point_2 p1, Point_2 p2) {
	Segment_2 seg(p1, p2);
	for (int i = 0; i < m_keyseg_vector.size(); i++) {
		if(bg::equals(seg, m_keyseg_vector[i])){
			return true;
		}
	}
	return false;
}

bool Roadmap::checkConnectAngle(Graph &jointCycle, Point_2 testPt, Point_2 cyclePt, std::vector<Polygon_2> oppoPolyList){
	int cyclePtVid = m_pointVidMap[cyclePt];
	std::set<int> neighbor = jointCycle.getNeighborSet(cyclePtVid);
	for(auto it = neighbor.begin();it != neighbor.end(); ++it){
		if(calculateAngle(cyclePt, testPt, m_vidPointMap[*it]) > -0.1){

			if (m_finalGraph.hasEdge(m_vidGFMap[*it], m_vidGFMap[m_pointVidMap[cyclePt]]) && !isSegInKey(cyclePt, m_vidPointMap[*it])) {
				
					m_finalGraph.removeEdge(m_vidGFMap[*it], m_vidGFMap[m_pointVidMap[cyclePt]]);
					cout<<"delete edge("<<m_vidGFMap[*it]<<","<<m_vidGFMap[m_pointVidMap[cyclePt]]<<") at 1534"<<endl;
			
					if (m_graph.hasEdge(*it, m_pointVidMap[cyclePt])) {
						m_graph.removeEdge(*it, m_pointVidMap[cyclePt]);
					}
					if (jointCycle.hasEdge(*it, m_pointVidMap[cyclePt])) {
						jointCycle.removeEdge(*it, m_pointVidMap[cyclePt]);
					}
			
			
			
			}
				
			return false;
		}
	}
	std::set<int> neighbor_1 = m_graph.getNeighborSet(cyclePtVid);
	for(auto bt = neighbor_1.begin();bt != neighbor_1.end();++bt){
		cout<<"neighbor vertex for "<<m_vidGFMap[m_pointVidMap[cyclePt]]<<"has "<<m_vidGFMap[*bt]<<endl;
	}
	if(neighbor_1.size() == 0){
		jointCycle.removeVertex(m_pointVidMap[cyclePt]);
		std::cout << "removeVertex at 2948" << endl;
		m_graph.removeVertex(m_pointVidMap[cyclePt]);
		m_finalGraph.removeVertex(m_vidGFMap[m_pointVidMap[cyclePt]]);
		return false;
	}
	if (neighbor_1.size() == 1) {
		m_keyseg_vector.push_back(Segment_2(cyclePt, m_vidPointMap[*(neighbor_1.begin())]));
	
	}
	
	if(neighbor_1.size() == 1){
		Segment_2 seg(cyclePt, m_vidPointMap[*(neighbor_1.begin())]);
		for(int i = 0;i < oppoPolyList.size();i++){
			if(bg::distance(seg, oppoPolyList[i]) < 0.8*m_edgeLength){
				jointCycle.removeVertex(m_pointVidMap[cyclePt]);
				m_graph.removeVertex(m_pointVidMap[cyclePt]);
				std::cout << "removeVertex at 2964" << endl;
				m_finalGraph.removeVertex(m_vidGFMap[m_pointVidMap[cyclePt]]);
				return false;
			}
		}
	}
	

	return true;
}

double Roadmap::calculateAngle(Point_2 cyclePt, Point_2 testPt, Point_2 neiPt){
	double a = bg::distance(testPt, neiPt);
	double c = bg::distance(testPt, cyclePt);
	double b = bg::distance(cyclePt, neiPt);

	double angleCos = (pow(b,2) + pow(c,2) - pow(a,2)) / (2*b*c);
	return angleCos; 
}

bool Roadmap::checkConnectSeg_2(Graph &jointCycle, Point_2 testPt, Point_2 cyclePt, Polygon_2 poly_1, Polygon_2 poly_2, double max, double min){
	double x = 0.0;
	double y = 0.0;
	double addingX = 0.0;
	double addingY = 0.0;
	double distance = bg::distance(testPt, cyclePt);

	if(distance > max){
		return false;
	}
	if(distance < min){
		std::cout<<"too small---distance:"<<distance<<"id:"<<m_vidGFMap[m_pointVidMap[cyclePt]]<<std::endl;
		jointCycle.removeVertex(m_pointVidMap[cyclePt]);
		std::cout << "removeVertex at 2995" << endl;
		//boost::this_thread::sleep(boost::posix_time::milliseconds(400));
		m_graph.removeVertex(m_pointVidMap[cyclePt]);
		m_finalGraph.removeVertex(m_vidGFMap[m_pointVidMap[cyclePt]]);
		return false;
	}
	for(int i = 0;i < m_newAddedSeg.size();i++){
		if(bg::distance(m_newAddedSeg[i], Segment_2(testPt, cyclePt)) < 0.7*m_edgeLength){
			std::cout << "removeVertex at 3001" << endl;
			jointCycle.removeVertex(m_pointVidMap[cyclePt]);
		m_graph.removeVertex(m_pointVidMap[cyclePt]);
		m_finalGraph.removeVertex(m_vidGFMap[m_pointVidMap[cyclePt]]);
			return false;
		}
	}
	addingX = getAddingX(cyclePt, testPt);
	addingY = getAddingY(cyclePt, testPt);
	x = testPt.get<0>();
	y = testPt.get<1>();
	testPt.set<0>(x+0.001*addingX);
	testPt.set<1>(y+0.001*addingY);
	Segment_2 testSeg(testPt, cyclePt);
	// if(bg::intersects(testSeg, shortestSeg)){
	// 	return false;
	// }
	if(bg::intersects(testSeg, poly_1)){
		return false;
	}
	if(bg::intersects(testSeg, poly_2)){
		return false;
	}
	std::cout<<"it is working!:"<<distance<<"id:"<<m_vidGFMap[m_pointVidMap[cyclePt]]<<std::endl;
	return true;
}


bool Roadmap::checkConnectSeg(Point_2 testPt, Point_2 cyclePt, Segment_2 shortestSeg, Polygon_2 poly_1, Polygon_2 poly_2, double max, double min){
	double x = 0.0;
	double y = 0.0;
	double addingX = 0.0;
	double addingY = 0.0;
	double distance = bg::distance(testPt, cyclePt);

	if(distance > max){
		return false;
	}
	if(distance < min){
		std::cout<<"too small---distance:"<<distance<<"id:"<<m_vidGFMap[m_pointVidMap[cyclePt]]<<std::endl;
		//m_jointCycle.removeVertex(m_pointVidMap[cyclePt]);
		//m_graph.removeVertex(m_pointVidMap[cyclePt]);
		//m_finalGraph.removeVertex(m_vidGFMap[m_pointVidMap[cyclePt]]);
		return false;
	}
	for(int i = 0;i < m_newAddedSeg.size();i++){
		if(bg::distance(m_newAddedSeg[i], Segment_2(testPt, cyclePt)) < 0.7*m_edgeLength){
			//m_jointCycle.removeVertex(m_pointVidMap[cyclePt]);
		//m_graph.removeVertex(m_pointVidMap[cyclePt]);
		//m_finalGraph.removeVertex(m_vidGFMap[m_pointVidMap[cyclePt]]);
			return false;
		}
	}
	addingX = getAddingX(cyclePt, testPt);
	addingY = getAddingY(cyclePt, testPt);
	x = testPt.get<0>();
	y = testPt.get<1>();
	testPt.set<0>(x+0.001*addingX);
	testPt.set<1>(y+0.001*addingY);
	Segment_2 testSeg(testPt, cyclePt);
	if(bg::intersects(testSeg, shortestSeg)){
		return false;
	}
	if(bg::intersects(testSeg, poly_1)){
		return false;
	}
	if(bg::intersects(testSeg, poly_2)){
		return false;
	}
	std::cout<<"it is working!:"<<distance<<"id:"<<m_vidGFMap[m_pointVidMap[cyclePt]]<<std::endl;
	return true;
}

Point_2 Roadmap::getNewPoint(Point_2 current, double addingX, double addingY){
	Point_2 nextPt;
	nextPt.set<0>(current.get<0>() + addingX);
	nextPt.set<1>(current.get<1>() + addingY);
	return nextPt;
}

Graph Roadmap::getJointCycle(Graph c1, Graph c2){
	std::vector<int> vVec_1, vVec_2;
	Polygon_2 p1, p2;
	std::set<int> deleteSet;
	Graph jointCycle;
	int size_1, size_2;
	if(c1.checkCycle()){  
		std::cout<<"c1 is cycle"<<endl;
		c1.getCycleVertexVector(vVec_1);
	}
	if(c2.checkCycle()){ 
		std::cout<<"c2 is cycle"<<endl;
		c2.getCycleVertexVector(vVec_2);

	}
	if(!isGraphIntersection(c1, c2)){
		std::cout<<"no intersection"<<std::endl;
		return c1;
	}		
	p1 = graphToPolygon(c1);
	p2 = graphToPolygon(c2);
	for(int i = 0; i < vVec_1.size(); i ++){
		if(pointInsideGraph(m_vidPointMap[vVec_1[i]], c2)){
			deleteSet.insert(vVec_1[i]);
			if(m_graph.hasVertex(vVec_1[i])){
				m_graph.removeVertex(vVec_1[i]);
			 	m_finalGraph.removeVertex(m_vidGFMap[vVec_1[i]]);
			}
		}
	}
	for(int i = 0; i < vVec_2.size(); i ++){
		if(pointInsideGraph(m_vidPointMap[vVec_2[i]], c1)){
			deleteSet.insert(vVec_2[i]);
			if(m_graph.hasVertex(vVec_2[i])){
				m_graph.removeVertex(vVec_2[i]);
			 	m_finalGraph.removeVertex(m_vidGFMap[vVec_2[i]]);
			}
		}
	}
	size_1 = vVec_1.size();
	size_2 = vVec_2.size();
	for(int i = 0; i < size_1; i ++){

		for(int j = 0; j < size_2; j++){
			if(vVec_1[i] == vVec_2[j]){	
				int front_1, back_1, front_2, back_2;
				if(i > 0 && i < size_1-1){
					front_1 = vVec_1[i-1];
					back_1 = vVec_1[i+1];
				}else if(i == 0){
					front_1 = vVec_1[size_1-1];
					back_1 = vVec_1[i+1];
				}else if(i == size_1-1){
					front_1 = vVec_1[i-1];
					back_1 = vVec_1[0];
				}
				if(j > 0 && j < size_2-1){
					front_2 = vVec_2[j-1];
					back_2 = vVec_2[j+1];
				}else if(j == 0){
					front_2 = vVec_2[size_2-1];
					back_2 = vVec_2[j+1];
				}else if(j == size_2-1){
					front_2 = vVec_2[j-1];
					back_2 = vVec_2[0];
				}


				if((front_1 == front_2 && back_1 == back_2) || (front_1 == back_2 && back_1 == front_2)){
					Point_2 side_1, side_2, midPt_1, midPt_2;
					double addingX = getAddingX(m_vidPointMap[vVec_1[i]], m_vidPointMap[front_1]);
					double addingY = getAddingY(m_vidPointMap[vVec_1[i]], m_vidPointMap[front_1]);
					side_1.set<0>(m_vidPointMap[vVec_1[i]].get<0>()+0.1*m_edgeLength*addingX);
					side_1.set<1>(m_vidPointMap[vVec_1[i]].get<1>()+0.1*m_edgeLength*addingY);
					midPt_1.set<0>((m_vidPointMap[vVec_1[i]].get<0>()+m_vidPointMap[front_1].get<0>())/2);
					midPt_1.set<1>((m_vidPointMap[vVec_1[i]].get<1>()+m_vidPointMap[front_1].get<1>())/2);
					midPt_2.set<0>((m_vidPointMap[vVec_1[i]].get<0>()+m_vidPointMap[back_1].get<0>())/2);
					midPt_2.set<1>((m_vidPointMap[vVec_1[i]].get<1>()+m_vidPointMap[back_1].get<1>())/2);
					side_2.set<0>((midPt_1.get<0>()+midPt_2.get<0>())/2);
					side_2.set<1>((midPt_1.get<1>()+midPt_2.get<1>())/2);
					if((bg::within(side_2, p1) && bg::within(side_1, p2)) || (bg::within(side_2, p2) && bg::within(side_1, p1))){
						deleteSet.insert(vVec_1[i]);
					}
					
				}else{
					if(front_1 == front_2){
						auto search_1 = deleteSet.find(back_1);
						auto search_2 = deleteSet.find(back_2);
						if(search_1 != deleteSet.end() && search_2 != deleteSet.end())
							deleteSet.insert(vVec_1[i]);
					}else if(front_1 == back_2){
						auto search_1 = deleteSet.find(back_1);
						auto search_2 = deleteSet.find(front_2);
						if(search_1 != deleteSet.end() && search_2 != deleteSet.end())
							deleteSet.insert(vVec_1[i]);
					}else if(back_1 == back_2){
						auto search_1 = deleteSet.find(front_1);
						auto search_2 = deleteSet.find(front_2);
						if(search_1 != deleteSet.end() && search_2 != deleteSet.end())
							deleteSet.insert(vVec_1[i]);
					}else if(back_1 == front_2){
						auto search_1 = deleteSet.find(front_1);
						auto search_2 = deleteSet.find(back_2);
						if(search_1 != deleteSet.end() && search_2 != deleteSet.end())
							deleteSet.insert(vVec_1[i]);
					}

				}


			
			}

		}
	}
	for(int i = 0; i < size_1; i ++){
		if(i != size_1 - 1){
			if(!jointCycle.hasEdge(vVec_1[i], vVec_1[i+1]))
				jointCycle.addEdge(vVec_1[i], vVec_1[i+1]);	
		}else{
			if(!jointCycle.hasEdge(vVec_1[i], vVec_1[0]))
				jointCycle.addEdge(vVec_1[i], vVec_1[0]);
		}
		
	}
	for(int i = 0; i < size_2; i ++){
		if(i != size_2 - 1){
			if(!jointCycle.hasEdge(vVec_2[i], vVec_2[i+1]))
				jointCycle.addEdge(vVec_2[i], vVec_2[i+1]);	
		}else{
			if(!jointCycle.hasEdge(vVec_2[i], vVec_2[0]))
				jointCycle.addEdge(vVec_2[i], vVec_2[0]);
		}
		
	}
	std::cout<<"--------------------------------"<<std::endl;
	std::cout<<"compute jointCycle, deleting finalGraph"<<std::endl;

	for(int i = 0; i < size_1; i ++){
		auto search = deleteSet.find(vVec_1[i]);
		if(search != deleteSet.end()){
			if(i > 0 && i < size_1-1){
				jointCycle.removeEdge(vVec_1[i], vVec_1[i-1]);
				jointCycle.removeEdge(vVec_1[i], vVec_1[i+1]);	
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_1[i]]<<":"<< m_vidGFMap[vVec_1[i-1]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[i-1]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[i-1]]);
				}
				if(m_graph.hasEdge(vVec_1[i], vVec_1[i-1])){
					m_graph.removeEdge(vVec_1[i], vVec_1[i-1]);
					
				}
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_1[i]]<<":"<< m_vidGFMap[vVec_1[i+1]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[i+1]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[i+1]]);
				}
				if(m_graph.hasEdge(vVec_1[i], vVec_1[i+1])){
					m_graph.removeEdge(vVec_1[i], vVec_1[i+1]);
				
				}
				
			}else if(i == 0){

				jointCycle.removeEdge(vVec_1[i], vVec_1[size_1-1]);
				jointCycle.removeEdge(vVec_1[i], vVec_1[i+1]);
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_1[i]]<<":"<< m_vidGFMap[vVec_1[size_1-1]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[size_1-1]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[size_1-1]]);
				}
				if(m_graph.hasEdge(vVec_1[i], vVec_1[size_1-1])){
					m_graph.removeEdge(vVec_1[i], vVec_1[size_1-1]);
				//	m_finalGraph.removeEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[size_1-1]]);
				}
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_1[i]]<<":"<< m_vidGFMap[vVec_1[i+1]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[i+1]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[i+1]]);
				}
				if(m_graph.hasEdge(vVec_1[i], vVec_1[i+1])){
					m_graph.removeEdge(vVec_1[i], vVec_1[i+1]);
				//	m_finalGraph.removeEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[i+1]]);
				}
				
			}else if(i == size_1-1){
				jointCycle.removeEdge(vVec_1[i], vVec_1[i-1]);
				jointCycle.removeEdge(vVec_1[i], vVec_1[0]);
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_1[i]]<<":"<< m_vidGFMap[vVec_1[i-1]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[i-1]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[i-1]]);
				}
				if(m_graph.hasEdge(vVec_1[i], vVec_1[i-1])){
					m_graph.removeEdge(vVec_1[i], vVec_1[i-1]);
				//	m_finalGraph.removeEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[i-1]]);
				}
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_1[i]]<<":"<< m_vidGFMap[vVec_1[0]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[0]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[0]]);
				}
				if(m_graph.hasEdge(vVec_1[i], vVec_1[0])){
					m_graph.removeEdge(vVec_1[i], vVec_1[0]);
				//	m_finalGraph.removeEdge(m_vidGFMap[vVec_1[i]], m_vidGFMap[vVec_1[0]]);
				}
				
			}
		}
	}

	for(int i = 0; i < size_2; i ++){
		auto search = deleteSet.find(vVec_2[i]);
		if(search != deleteSet.end()){
			if(i > 0 && i < size_2-1){
				jointCycle.removeEdge(vVec_2[i], vVec_2[i-1]);
				jointCycle.removeEdge(vVec_2[i], vVec_2[i+1]);
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_2[i]]<<":"<< m_vidGFMap[vVec_2[i-1]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i-1]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i-1]]);
				}
				if(m_graph.hasEdge(vVec_2[i], vVec_2[i-1])){
					m_graph.removeEdge(vVec_2[i], vVec_2[i-1]);
				//	m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i-1]]);
				}
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_2[i]]<<":"<< m_vidGFMap[vVec_2[i+1]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i+1]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i+1]]);
				}
				if(m_graph.hasEdge(vVec_2[i], vVec_2[i+1])){
					m_graph.removeEdge(vVec_2[i], vVec_2[i+1]);
				//	m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i+1]]);
				}	
				
			}else if(i == 0){
				jointCycle.removeEdge(vVec_2[i], vVec_2[size_2-1]);
				jointCycle.removeEdge(vVec_2[i], vVec_2[i+1]);
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_2[i]]<<":"<< m_vidGFMap[vVec_2[size_2-1]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[size_2-1]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[size_2-1]]);
				}
				if(m_graph.hasEdge(vVec_2[i], vVec_2[size_2-1])){
					m_graph.removeEdge(vVec_2[i], vVec_2[size_2-1]);
				//	m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[size_2-1]]);
				}
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_2[i]]<<":"<< m_vidGFMap[vVec_2[i+1]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i+1]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i+1]]);
				}
				if(m_graph.hasEdge(vVec_2[i], vVec_2[i+1])){
					m_graph.removeEdge(vVec_2[i], vVec_2[i+1]);
				//	m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i+1]]);
				}
				
			}else if(i == size_2-1){
				jointCycle.removeEdge(vVec_2[i], vVec_2[i-1]);
				jointCycle.removeEdge(vVec_2[i], vVec_2[0]);
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_2[i]]<<":"<< m_vidGFMap[vVec_2[i-1]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i-1]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i-1]]);
				}
				if(m_graph.hasEdge(vVec_2[i], vVec_2[i-1])){
					m_graph.removeEdge(vVec_2[i], vVec_2[i-1]);
				//	m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[i-1]]);
				}
				std::cout<<"deleting :  "<<m_vidGFMap[vVec_2[i]]<<":"<< m_vidGFMap[vVec_2[0]]<<std::endl;
				if(m_finalGraph.hasEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[0]])){
					m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[0]]);
				}
				if(m_graph.hasEdge(vVec_2[i], vVec_2[0])){
					m_graph.removeEdge(vVec_2[i], vVec_2[0]);
				//	m_finalGraph.removeEdge(m_vidGFMap[vVec_2[i]], m_vidGFMap[vVec_2[0]]);
				}
				
			}
		}
	}

	return jointCycle;
}

void Roadmap::recoverConnectivity(int& vIDCount){
	int vid = 2*n_w*n_h;
	double slack = m_edgeLength / 50;

	for(Polygon2_list::iterator obsit = m_objectPolyList.begin(); obsit != m_objectPolyList.end(); obsit++){
		for(std::vector<Graph*>::iterator g = m_obsBoundingCycleVec.begin(); g != m_obsBoundingCycleVec.end(); g++){
			     
			//	preprocessObs(*obsit);

			std::vector<Point_2> v_list;
			std::vector<std::pair<int, int>> cycleEdge_list;
			std::vector<Segment_2> polyEdge_list;
			double overlapLength = 0;
			int edgeNums = 0;
			double crossEdgeLength = 0;
			if(intersectObsAndCycle(*obsit, *(*g), v_list, cycleEdge_list, polyEdge_list) >= 2){
				int cross_size = v_list.size();
				Point_2 startPoint = v_list[0];
				Point_2 endPoint = v_list[cross_size-1];
				Point_2 crossStartPt, crossEndPt;
				bool crossInsideGraph = true;
				Segment_2 startCycle = pointPairToSegment(cycleEdge_list[0]);
				Segment_2 endCycle = pointPairToSegment(cycleEdge_list[cross_size-1]);
				if(isPointInCSpace(startCycle.first)){
					overlapLength += bg::distance(startPoint, startCycle.first);
					crossStartPt = startCycle.first;
					std::cout<<"crossStartPt:"<<crossStartPt.get<0>()<<","<<crossStartPt.get<1>()<<std::endl;
				}else{
					overlapLength += bg::distance(startPoint, startCycle.second);
					crossStartPt = startCycle.second;
					std::cout<<"crossStartPt:"<<crossStartPt.get<0>()<<","<<crossStartPt.get<1>()<<std::endl;
				}
				if(isPointInCSpace(endCycle.first)){
					overlapLength += bg::distance(endPoint, endCycle.first);
					crossEndPt = endCycle.first;
					std::cout<<"crossEndPt:"<<crossEndPt.get<0>()<<","<<crossEndPt.get<1>()<<std::endl;
				}else{
					overlapLength += bg::distance(endPoint, endCycle.second);
					crossEndPt = endCycle.second;
					std::cout<<"crossEndPt:"<<crossEndPt.get<0>()<<","<<crossEndPt.get<1>()<<std::endl;
				}
				std::cout<<"double left sides's length:"<<overlapLength<<std::endl;
				std::cout<<"size of v_list:"<<v_list.size()<<std::endl;

				for(int i = 0; i < v_list.size()-1; i++){
					if(bg::equals(polyEdge_list[i], polyEdge_list[i+1])){
						overlapLength += bg::length(Segment_2(v_list[i], v_list[i+1]));
					}else{
						overlapLength += neighborCrossLength(polyEdge_list[i], polyEdge_list[i+1], v_list[i], v_list[i+1], *(*g), *obsit, crossInsideGraph);
					}
					crossInsideGraph = ~crossInsideGraph;

				}
				std::cout<<"overlapLength is"<<overlapLength<<std::endl;
//				edgeNums = overlapLength / m_edgeLength;
				int edgeNums = 0;
				if(overlapLength > m_edgeLength && overlapLength < 1.6* m_edgeLength){
					edgeNums = 1;
				}else if(overlapLength > 1.6* m_edgeLength && overlapLength < 2.7* m_edgeLength){
					edgeNums = 2;
				}else if(overlapLength > 2.7* m_edgeLength && overlapLength < 3.9* m_edgeLength){
					edgeNums = 3;
				}else if(overlapLength > 3.9* m_edgeLength && overlapLength < 4.9* m_edgeLength){
					edgeNums = 4;
				}else{
					if(fmod(overlapLength , m_edgeLength) > 0.9 * m_edgeLength){
						edgeNums = overlapLength / m_edgeLength + 1;
					}else{
						edgeNums = overlapLength / m_edgeLength;
					}
					

				}
				//Point_2 singleStart, singleEnd;
				crossEdgeLength = overlapLength / double(edgeNums);
				std::cout<<"edgeNums:"<<edgeNums<<std::endl;
				std::cout<<"single edge's length"<<crossEdgeLength<<std::endl;
				// double leftLength;
				// for(int i = 0; i < edgeNums; i++){
				// 	if(i == 0){
				// 		singleStart = crossStartPt;
				// 		if(crossEdgeLength > bg::distance(startPoint, crossStartPt)){
				// 			leftLength = crossEdgeLength - bg::distance(startPoint, crossStartPt);
				// 		}
				// 		if(polyEdge_list[0].first)
				// 	}


				// }
				double startHalfEdgeLength = bg::distance(crossStartPt, startPoint);
				double endHalfEdgeLength = bg::distance(crossEndPt, endPoint);
				double lengthSoFar = 0;
				Point_2 polyStartingPt, polyEndPt, polyCurrentPt, polyPreviousPt, polyNextPt, polyEndAfterPt;
				std::vector<Point_2> stopingPoint_list;
				Segment_2 temp;
				temp.first = polyEdge_list[0].first;
				temp.second = v_list[0];
				temp.second.set<0>(v_list[0].get<0>() + slack*(temp.first.get<0>()-v_list[0].get<0>())/bg::length(temp));
				temp.second.set<1>(v_list[0].get<1>() + slack*(temp.first.get<1>()-v_list[0].get<1>())/bg::length(temp));
				Linestring_2 tempL;
				bg::append(tempL, polyEdge_list[0].first);
				Point_2 numTwo;
				numTwo.set<0>(v_list[0].get<0>() + slack*(temp.first.get<0>()-v_list[0].get<0>())/bg::length(temp));
				numTwo.set<1>(v_list[0].get<1>() + slack*(temp.first.get<1>()-v_list[0].get<1>())/bg::length(temp));
				bg::append(tempL, numTwo);
				Polygon_2 graphPoly = graphToPolygon(*(*g));
				if(bg::within(tempL, graphPoly) || bg::intersects(temp, graphPoly)){
					polyStartingPt = polyEdge_list[0].first;
					polyPreviousPt = polyEdge_list[0].second;
				}else{
					polyStartingPt = polyEdge_list[0].second;
					polyPreviousPt = polyEdge_list[0].first;
				}
				temp.first = polyEdge_list[cross_size-1].first;
				temp.second = v_list[cross_size-1];
				temp.second.set<0>(v_list[cross_size-1].get<0>() + slack*(temp.first.get<0>()-v_list[cross_size-1].get<0>())/bg::length(temp));
				temp.second.set<1>(v_list[cross_size-1].get<1>() + slack*(temp.first.get<1>()-v_list[cross_size-1].get<1>())/bg::length(temp));
				Linestring_2 tempL_2;
				bg::append(tempL_2, polyEdge_list[cross_size-1].first);
				Point_2 numTwo_2;
				numTwo_2.set<0>(v_list[cross_size-1].get<0>() + slack*(temp.first.get<0>()-v_list[cross_size-1].get<0>())/bg::length(temp));
				numTwo_2.set<1>(v_list[cross_size-1].get<1>() + slack*(temp.first.get<1>()-v_list[cross_size-1].get<1>())/bg::length(temp));
				bg::append(tempL_2, numTwo_2);
				if(bg::within(tempL_2, graphPoly) || bg::intersects(temp, graphPoly)){
					polyEndPt = polyEdge_list[cross_size-1].first;
					polyEndAfterPt = polyEdge_list[cross_size-1].second;
				}else{
					polyEndPt = polyEdge_list[cross_size-1].second;
					polyEndAfterPt = polyEdge_list[cross_size-1].first;
				}   
				polyCurrentPt = polyStartingPt;


//the starting half edge from the cycle to the boundary of the obstacle
				registerRegularPoint(vid, v_list[0], crossStartPt);
				vid++;
//
				std::vector<int> vidFGlist;

				// the cycle only cross one edge of the polygon
				if(bg::equals(polyPreviousPt, polyEndPt)){
					double additiveLength = 0;
					additiveLength += startHalfEdgeLength;
					double addingX = (v_list[cross_size-1].get<0>() - v_list[0].get<0>()) / bg::distance(v_list[0], v_list[cross_size-1]);
					double addingY = (v_list[cross_size-1].get<1>() - v_list[0].get<1>()) / bg::distance(v_list[0], v_list[cross_size-1]);
					Point_2 firstPt(v_list[0].get<0>()+(crossEdgeLength - additiveLength)*addingX, v_list[0].get<1>()+(crossEdgeLength - additiveLength)*addingY);
					stopingPoint_list.push_back(firstPt);
					
//  record the newly found discrete graph point firstPt into m_graph, m_finalGraph
					m_vidPointMap[vid] = firstPt;
					m_pointVidMap[firstPt] = vid;
					m_graph.addEdge(vid, m_pointVidMap[v_list[0]]);
					m_vidFGMap[vIDCount] = vid;
					m_vidGFMap[vid] = vIDCount;
					vidFGlist.clear();
					vidFGlist.push_back(vid);
					vidFGlist.push_back(m_pointVidMap[v_list[0]]);
					vidFGlist.push_back(m_pointVidMap[crossStartPt]);
					m_finalGraph.addEdge(vIDCount, m_vidGFMap[m_pointVidMap[crossStartPt]]);

					addEdgeFGMap(vIDCount, m_vidGFMap[m_pointVidMap[crossStartPt]], vidFGlist);
					vid++;
					vIDCount ++;
//


					additiveLength = crossEdgeLength;
					Point_2 tempPt, currentPt;
					currentPt = firstPt;
					while(additiveLength < overlapLength - 1.1*crossEdgeLength){
						tempPt.set<0>(currentPt.get<0>()+addingX * crossEdgeLength);
						tempPt.set<1>(currentPt.get<1>()+addingY * crossEdgeLength);
						m_vidPointMap[vid] = tempPt;
						m_pointVidMap[tempPt] = vid;
						m_graph.addEdge(vid, m_pointVidMap[currentPt]);
						m_finalGraph.addEdge(vIDCount, m_vidGFMap[m_pointVidMap[currentPt]]);
						m_vidFGMap[vIDCount] = vid;
						m_vidGFMap[vid] = vIDCount;
						vidFGlist.clear();
						vidFGlist.push_back(vid);
						vidFGlist.push_back(m_pointVidMap[currentPt]);
						addEdgeFGMap(vIDCount, m_vidGFMap[m_pointVidMap[currentPt]], vidFGlist);
						// the order of parameters is from the newly one  from last one, so is the m_graph point list
						additiveLength += crossEdgeLength;
						currentPt = tempPt;
						vid++;
						vIDCount++;
					}
					m_vidPointMap[vid] = v_list[cross_size-1];
					m_pointVidMap[v_list[cross_size-1]] = vid;
					m_graph.addEdge(vid, m_pointVidMap[currentPt]);
					m_graph.addEdge(vid, m_pointVidMap[crossEndPt]);

					m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]]);
					vidFGlist.clear();
					vidFGlist.push_back(m_pointVidMap[crossEndPt]);
					vidFGlist.push_back(vid);
					vidFGlist.push_back(m_pointVidMap[currentPt]);
					addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]], vidFGlist);
					vid++;

				}else if(bg::equals(polyCurrentPt, polyEndPt)){
					m_pointDistMap.clear();
					m_pointDistMap[polyCurrentPt] = startHalfEdgeLength + bg::distance(polyCurrentPt, startPoint);
					double additiveLength = 0;
					additiveLength += startHalfEdgeLength;
					if(m_pointDistMap[polyCurrentPt] > crossEdgeLength){
						double addingX = getAddingX(polyCurrentPt, startPoint);
						double addingY = getAddingY(polyCurrentPt, startPoint);
						Point_2 firstPt(v_list[0].get<0>()+(crossEdgeLength - additiveLength)*addingX, v_list[0].get<1>()+(crossEdgeLength - additiveLength)*addingY);
						registerRegularPoint(vid, firstPt, startPoint);
						vidFGlist.clear();
						vidFGlist.push_back(vid);
						vidFGlist.push_back(m_pointVidMap[startPoint]);
						vidFGlist.push_back(m_pointVidMap[crossStartPt]);
						registerFinalPoint(vIDCount, vid, firstPt, crossStartPt, vidFGlist);
						vIDCount ++;
						vid++;
						additiveLength = crossEdgeLength;
						Point_2 currentPt, tempPt;
						currentPt = firstPt;
						while(m_pointDistMap[polyCurrentPt] > additiveLength + crossEdgeLength){
							tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
							tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
							registerRegularPoint(vid, tempPt, currentPt);
							vidFGlist.clear();
							vidFGlist.push_back(vid);
							vidFGlist.push_back(m_pointVidMap[currentPt]);
							registerFinalPoint(vIDCount, vid, firstPt, crossStartPt, vidFGlist);
							additiveLength += crossEdgeLength;
							currentPt = tempPt;
							vid++;
							vIDCount++;
						}
						registerRegularPoint(vid, polyCurrentPt, currentPt);
						vid++;
						m_pointDistMap[endPoint] = overlapLength - endHalfEdgeLength;
						if(m_pointDistMap[endPoint] < additiveLength + crossEdgeLength){
							registerRegularPoint(vid, endPoint, polyCurrentPt);

							vid++;
							vidFGlist.clear();
							vidFGlist.push_back(m_pointVidMap[crossEndPt]);
							vidFGlist.push_back(m_pointVidMap[endPoint]);
							vidFGlist.push_back(m_pointVidMap[polyCurrentPt]);
							vidFGlist.push_back(m_pointVidMap[currentPt]);
							m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]]);
							addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]], vidFGlist);
						}else{  

							addingX = getAddingX(endPoint, polyCurrentPt);
							addingY = getAddingY(endPoint, polyCurrentPt);
							double addOnLength = additiveLength + crossEdgeLength - m_pointDistMap[polyCurrentPt];
							tempPt.set<0>(polyCurrentPt.get<0>() + addOnLength*addingX);
							tempPt.set<1>(polyCurrentPt.get<1>() + addOnLength*addingY);
							registerRegularPoint(vid, tempPt, polyCurrentPt);
							vidFGlist.clear();
							vidFGlist.push_back(vid);
							vidFGlist.push_back(m_pointVidMap[polyCurrentPt]);
							vidFGlist.push_back(m_pointVidMap[currentPt]);
							registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
							additiveLength += crossEdgeLength;
							currentPt = tempPt;
							vid++;
							vIDCount++;
							while(m_pointDistMap[endPoint] > additiveLength + crossEdgeLength){
								tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
								tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
								registerRegularPoint(vid, tempPt, currentPt);
								vidFGlist.clear();
								vidFGlist.push_back(vid);
								vidFGlist.push_back(m_pointVidMap[currentPt]);
								registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
								additiveLength += crossEdgeLength;
								currentPt = tempPt;
								vid++;
								vIDCount++;

					
							}
							registerRegularPoint(vid, endPoint, currentPt);
							vid++;
							vidFGlist.clear();
							vidFGlist.push_back(m_pointVidMap[crossEndPt]);
							vidFGlist.push_back(m_pointVidMap[endPoint]);
							vidFGlist.push_back(m_pointVidMap[currentPt]);
							m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]]);
							addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]], vidFGlist);
						}
					}else{
						registerRegularPoint(vid, polyCurrentPt, startPoint);
						vid++;
						additiveLength += bg::distance(startPoint, polyCurrentPt);
						double addingX = getAddingX(endPoint, polyCurrentPt);
						double addingY = getAddingY(endPoint, polyCurrentPt);
						Point_2 tempPt, currentPt;
						tempPt.set<0>(polyCurrentPt.get<0>()+(crossEdgeLength - additiveLength)*addingX);
						tempPt.set<1>(polyCurrentPt.get<1>()+(crossEdgeLength - additiveLength)*addingY);
						registerRegularPoint(vid, tempPt, polyCurrentPt);
						vidFGlist.clear();
						vidFGlist.push_back(m_pointVidMap[tempPt]);
						vidFGlist.push_back(m_pointVidMap[polyCurrentPt]);
						vidFGlist.push_back(m_pointVidMap[startPoint]);
						vidFGlist.push_back(m_pointVidMap[crossStartPt]);
						registerFinalPoint(vIDCount, vid, tempPt, crossStartPt, vidFGlist);
						vid++;
						vIDCount++;
						additiveLength = crossEdgeLength;
						currentPt = tempPt;
						m_pointDistMap[endPoint] = overlapLength - endHalfEdgeLength;
						while(m_pointDistMap[endPoint] > additiveLength + crossEdgeLength){
								tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
								tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
								registerRegularPoint(vid, tempPt, currentPt);
								vidFGlist.clear();
								vidFGlist.push_back(vid);
								vidFGlist.push_back(m_pointVidMap[currentPt]);
								registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
								additiveLength += crossEdgeLength;
								currentPt = tempPt;
								vid++;
								vIDCount++;

					
						}
						registerRegularPoint(vid, endPoint, currentPt);
							vid++;
							vidFGlist.clear();
							vidFGlist.push_back(m_pointVidMap[crossEndPt]);
							vidFGlist.push_back(m_pointVidMap[endPoint]);
							vidFGlist.push_back(m_pointVidMap[currentPt]);
							m_finalGraph.addEdge(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]]);
							addEdgeFGMap(m_vidGFMap[m_pointVidMap[crossEndPt]], m_vidGFMap[m_pointVidMap[currentPt]], vidFGlist);

					}
					
					
				


				}
				else{ // cross multiple edges
					m_pointDistMap.clear();
					m_pointDistMap[polyCurrentPt] = startHalfEdgeLength + bg::distance(polyCurrentPt, startPoint);
					double additiveLength = 0;
					additiveLength += startHalfEdgeLength;
					if(m_pointDistMap[polyCurrentPt] > crossEdgeLength){
						double addingX = getAddingX(polyCurrentPt, startPoint);
						double addingY = getAddingY(polyCurrentPt, startPoint);
						Point_2 firstPt(v_list[0].get<0>()+(crossEdgeLength - additiveLength)*addingX, v_list[0].get<1>()+(crossEdgeLength - additiveLength)*addingY);
						registerRegularPoint(vid, firstPt, startPoint);
						vidFGlist.clear();
						vidFGlist.push_back(m_pointVidMap[crossStartPt]);
						vidFGlist.push_back(m_pointVidMap[startPoint]);
						vidFGlist.push_back(vid);
						reverseVector(vidFGlist);
						registerFinalPoint(vIDCount, vid, firstPt, crossStartPt, vidFGlist);
						vIDCount ++;
						vid++;
						additiveLength = crossEdgeLength;
						Point_2 currentPt, tempPt;
						currentPt = firstPt;
						while(m_pointDistMap[polyCurrentPt] > additiveLength + crossEdgeLength){
							tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
							tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
							registerRegularPoint(vid, tempPt, currentPt);
							vidFGlist.clear();
							vidFGlist.push_back(m_pointVidMap[currentPt]);
							vidFGlist.push_back(vid);
							reverseVector(vidFGlist);
							registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
							additiveLength += crossEdgeLength;
							currentPt = tempPt;
							vid++;
							vIDCount++;
						}
						registerRegularPoint(vid, polyCurrentPt, currentPt);
						vidFGlist.clear();
						vidFGlist.push_back(m_pointVidMap[currentPt]);
						vidFGlist.push_back(vid);
						vid++;

						
						double lastFinalPtDist = additiveLength;
						Point_2 lastFinalPt = currentPt;
						additiveLength = m_pointDistMap[polyCurrentPt];


						while(!bg::equals((polyNextPt = nextInPolygon(polyCurrentPt, polyPreviousPt, *obsit, "2195")), polyEndAfterPt)){
							m_pointDistMap[polyNextPt] = m_pointDistMap[polyCurrentPt] + bg::distance(polyCurrentPt, polyNextPt);
							double addingX = getAddingX(polyNextPt, polyCurrentPt);
							double addingY = getAddingY(polyNextPt, polyCurrentPt);
							Point_2 tempPt, currentPt;
							if(m_pointDistMap[polyNextPt] > lastFinalPtDist + crossEdgeLength){
								Point_2 firstPt(polyCurrentPt.get<0>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingX, polyCurrentPt.get<1>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingY);
								registerRegularPoint(vid, firstPt, polyCurrentPt);
								
								vidFGlist.push_back(vid);

								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, firstPt, lastFinalPt, vidFGlist);
								lastFinalPt = firstPt;
								vidFGlist.clear();
								vIDCount ++;
								vid++;
								additiveLength = lastFinalPtDist + crossEdgeLength;
								currentPt = firstPt;
								vidFGlist.push_back(m_pointVidMap[currentPt]);
								while(m_pointDistMap[polyNextPt] > additiveLength + crossEdgeLength){
									tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
									tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
									registerRegularPoint(vid, tempPt, currentPt);
									
									vidFGlist.push_back(vid);
									reverseVector(vidFGlist); // not implemented yet
									registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
									vidFGlist.clear();
									vidFGlist.push_back(vid);
									additiveLength += crossEdgeLength;
									currentPt = tempPt;
									vid++;
									vIDCount++;
								}
								lastFinalPtDist = additiveLength;
								lastFinalPt = currentPt;
								registerRegularPoint(vid, polyNextPt, currentPt);
								vid++;
								additiveLength = m_pointDistMap[polyNextPt];
								polyPreviousPt = polyCurrentPt;
								polyCurrentPt = polyNextPt;
								vidFGlist.push_back(m_pointVidMap[polyCurrentPt]);
							}else{
								registerRegularPoint(vid, polyNextPt, polyCurrentPt);
								vid++;
								additiveLength += bg::distance(polyCurrentPt, polyNextPt);
								polyPreviousPt = polyCurrentPt;
								polyCurrentPt = polyNextPt;
								vidFGlist.push_back(m_pointVidMap[polyCurrentPt]);
							}

						}	


						addingX = getAddingX(polyNextPt, polyCurrentPt);
						addingY = getAddingY(polyNextPt, polyCurrentPt);
						m_pointDistMap[endPoint] = overlapLength - endHalfEdgeLength;
						if(m_pointDistMap[endPoint] > lastFinalPtDist + crossEdgeLength){
							Point_2 firstPt(polyCurrentPt.get<0>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingX, polyCurrentPt.get<1>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingY);
							registerRegularPoint(vid, firstPt, polyCurrentPt);
							vidFGlist.push_back(vid);

							reverseVector(vidFGlist); // not implemented yet
							registerFinalPoint(vIDCount, vid, firstPt, lastFinalPt, vidFGlist);
							vidFGlist.clear();
							vIDCount ++;
							vid++;
							additiveLength = lastFinalPtDist + crossEdgeLength;
							currentPt = firstPt;
							vidFGlist.push_back(m_pointVidMap[currentPt]);




							while(m_pointDistMap[endPoint] > additiveLength + crossEdgeLength){
								tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
								tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
								registerRegularPoint(vid, tempPt, currentPt);
								vidFGlist.push_back(vid);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
								vidFGlist.clear();
								vidFGlist.push_back(vid);
								additiveLength += crossEdgeLength;
								currentPt = tempPt;
								vid++;
								vIDCount++;

					
							}
							registerRegularPoint(vid, endPoint, currentPt);
							vidFGlist.push_back(vid);
							vid++;
							
							vidFGlist.push_back(m_pointVidMap[crossEndPt]);
							reverseVector(vidFGlist); // not implemented yet
							registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, currentPt, vidFGlist);
							vidFGlist.clear();

						}else{
							registerRegularPoint(vid, endPoint, polyCurrentPt);
							vidFGlist.push_back(vid);
							vid++;
							vidFGlist.push_back(m_pointVidMap[crossEndPt]);
							reverseVector(vidFGlist);
							registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, lastFinalPt, vidFGlist);
							vidFGlist.clear();
						}


					}else{



						registerRegularPoint(vid, polyCurrentPt, startPoint);
						vidFGlist.clear();
						vidFGlist.push_back(m_pointVidMap[crossStartPt]);
						vidFGlist.push_back(m_pointVidMap[startPoint]);
						vidFGlist.push_back(vid);
						vid++;

						additiveLength += bg::distance(startPoint, polyCurrentPt);
						double lastFinalPtDist = 0;
						Point_2 lastFinalPt = crossStartPt;
						while(!bg::equals((polyNextPt = nextInPolygon(polyCurrentPt, polyPreviousPt, *obsit, "2320")), polyEndAfterPt)){
							m_pointDistMap[polyNextPt] = m_pointDistMap[polyCurrentPt] + bg::distance(polyCurrentPt, polyNextPt);
							double addingX = getAddingX(polyNextPt, polyCurrentPt);
							double addingY = getAddingY(polyNextPt, polyCurrentPt);
							Point_2 tempPt, currentPt;
							if(m_pointDistMap[polyNextPt] > lastFinalPtDist + crossEdgeLength){
								Point_2 firstPt(polyCurrentPt.get<0>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingX, polyCurrentPt.get<1>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingY);
								registerRegularPoint(vid, firstPt, polyCurrentPt);
								
								vidFGlist.push_back(vid);

								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, firstPt, lastFinalPt, vidFGlist);
								lastFinalPt = firstPt;
								vidFGlist.clear();
								vIDCount ++;
								vid++;
								additiveLength = lastFinalPtDist + crossEdgeLength;
								currentPt = firstPt;
								vidFGlist.push_back(m_pointVidMap[currentPt]);
								while(m_pointDistMap[polyNextPt] > additiveLength + crossEdgeLength){
									tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
									tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
									registerRegularPoint(vid, tempPt, currentPt);
									
									vidFGlist.push_back(vid);
									reverseVector(vidFGlist); // not implemented yet
									registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
									vidFGlist.clear();
									vidFGlist.push_back(vid);
									additiveLength += crossEdgeLength;
									currentPt = tempPt;
									vid++;
									vIDCount++;
								}
								lastFinalPtDist = additiveLength;
								lastFinalPt = currentPt;
								registerRegularPoint(vid, polyNextPt, currentPt);
								vid++;
								additiveLength = m_pointDistMap[polyNextPt];
								polyPreviousPt = polyCurrentPt;
								polyCurrentPt = polyNextPt;
								vidFGlist.push_back(m_pointVidMap[polyCurrentPt]);
							}else{
								registerRegularPoint(vid, polyNextPt, polyCurrentPt);
								vid++;
								additiveLength += bg::distance(polyCurrentPt, polyNextPt);
								polyPreviousPt = polyCurrentPt;
								polyCurrentPt = polyNextPt;
								vidFGlist.push_back(m_pointVidMap[polyCurrentPt]);
							}

						}	
						double addingX = getAddingX(polyNextPt, polyCurrentPt);
						double addingY = getAddingY(polyNextPt, polyCurrentPt);
						m_pointDistMap[endPoint] = overlapLength - endHalfEdgeLength;
						Point_2 tempPt, currentPt;
						if(m_pointDistMap[endPoint] > lastFinalPtDist + crossEdgeLength){
							Point_2 firstPt(polyCurrentPt.get<0>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingX, polyCurrentPt.get<1>()+(lastFinalPtDist + crossEdgeLength - additiveLength)*addingY);
							registerRegularPoint(vid, firstPt, polyCurrentPt);
							vidFGlist.push_back(vid);

							reverseVector(vidFGlist); // not implemented yet
							registerFinalPoint(vIDCount, vid, firstPt, lastFinalPt, vidFGlist);
							vidFGlist.clear();
							vIDCount ++;
							vid++;
							additiveLength = lastFinalPtDist + crossEdgeLength;
							currentPt = firstPt;
							vidFGlist.push_back(m_pointVidMap[currentPt]);




							while(m_pointDistMap[endPoint] > additiveLength + crossEdgeLength){
								tempPt.set<0>(currentPt.get<0>() + crossEdgeLength*addingX);
								tempPt.set<1>(currentPt.get<1>() + crossEdgeLength*addingY);
								registerRegularPoint(vid, tempPt, currentPt);
								vidFGlist.push_back(vid);
								reverseVector(vidFGlist); // not implemented yet
								registerFinalPoint(vIDCount, vid, tempPt, currentPt, vidFGlist);
								vidFGlist.clear();
								vidFGlist.push_back(vid);
								additiveLength += crossEdgeLength;
								currentPt = tempPt;
								vid++;
								vIDCount++;

					
							}
							registerRegularPoint(vid, endPoint, currentPt);
							vidFGlist.push_back(vid);
							vid++;
							
							vidFGlist.push_back(m_pointVidMap[crossEndPt]);
							reverseVector(vidFGlist); // not implemented yet
							registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, currentPt, vidFGlist);
							


							

						}else{
							registerRegularPoint(vid, endPoint, polyCurrentPt);
							vidFGlist.push_back(vid);
							vid++;
							vidFGlist.push_back(m_pointVidMap[crossEndPt]);
							reverseVector(vidFGlist);
							registerFinalPoint(m_vidGFMap[m_pointVidMap[crossEndPt]], m_pointVidMap[crossEndPt], crossEndPt, lastFinalPt, vidFGlist);
						}
					}

				}// cross multiple edges

				




	  
			}// intersectObsAndCycle(*obsit, *(*g), v_list, cycleEdge_list, polyEdge_list)
		}// for each obsit
		
		
	}	// for each  surroundingCycle

} // recover connectivity function

void Roadmap::addEdgeFGMap(int fv, int sv, std::vector<int> vidFGlist){
	std::pair<int, int> finalGraphEdge;
	finalGraphEdge.first = fv;
	finalGraphEdge.second = sv;
	// std::vector<int> graphEdgeVector;
	// for(int i = 0;i < vidFGlist.size()-1; i++){
	// 	std::pair<int, int> graphEdge;
	// 	graphEdge.first = vidFGlist[i];
	// 	graphEdge.second = vidFGlist[i+1];
	// 	graphEdgeVector.push_back(graphEdge);
	// }
	m_edgeFGMap[finalGraphEdge] = vidFGlist;




	std::pair<int, int> finalGraphEdge_r;
	finalGraphEdge_r.first = sv;
	finalGraphEdge_r.second = fv;
	std::vector<int> graphEdgeVector_r;
	for(int i = vidFGlist.size()-1;i >= 0; i--){
		graphEdgeVector_r.push_back(vidFGlist[i]);
	}
	m_edgeFGMap[finalGraphEdge_r] = graphEdgeVector_r;
}

void Roadmap::registerRegularPoint(int& vid, Point_2& currentPt, Point_2& previousPt){
	m_vidPointMap[vid] = currentPt;
	m_pointVidMap[currentPt] = vid;
	m_graph.addEdge(vid, m_pointVidMap[previousPt]);
	
}

void Roadmap::registerFinalPoint(int& vIDCount, int& vid, Point_2& currentPt, Point_2& previousPt, std::vector<int>& vidFGlist ){
	m_finalGraph.addEdge(vIDCount, m_vidGFMap[m_pointVidMap[previousPt]]);
	m_vidFGMap[vIDCount] = vid; 
	m_vidGFMap[vid] = vIDCount;
	addEdgeFGMap(vIDCount, m_vidGFMap[m_pointVidMap[previousPt]], vidFGlist);
}

double Roadmap::polygonCrossLength(Point_2 start, Point_2 end, Point_2 polyCurrentPt, Point_2 polyPreviousPt, Point_2 polyEndPt, Point_2 polyEndAfterPt, Polygon_2 poly){
	double length = 0;
	length += bg::distance(start, polyCurrentPt);
	length += bg::distance(end, polyEndPt);
	Point_2 startPt = polyCurrentPt;
	Point_2 endPt = polyEndPt;
	Point_2 currentPt, nextPt, previousPt;

	if(!bg::equals(startPt, endPt)){
				
				currentPt = startPt;
				previousPt = polyPreviousPt;
				while(!bg::equals((nextPt = nextInPolygon(currentPt, previousPt, poly, "3936")), endPt)){
					length += bg::distance(currentPt, nextPt);
					previousPt = currentPt;
					currentPt = nextPt;
			
				}
				length += bg::distance(currentPt, endPt);
	}
	return length;
}

double Roadmap::neighborCrossLength(Segment_2 startSeg, Segment_2 endSeg, Point_2 startPoint, Point_2 endPoint ,Graph g, Polygon_2 poly, bool crossInsideGraph){
	double length = 0;
	if(bg::equals(startSeg, endSeg)){
		return bg::length(Segment_2(startPoint, endPoint));  
	}else{
		if(crossInsideGraph == true){
			Point_2 startPt, endPt;
			Point_2 currentPt, previousPt, nextPt;
			if(pointInsideGraph(startSeg.first, g)){
				startPt = startSeg.first;
				previousPt = startSeg.second;
				length += bg::distance(startPoint, startPt);
			}else{
				startPt = startSeg.second;
				previousPt = startSeg.first;
				length += bg::distance(startPoint, startPt);
			}

			if(pointInsideGraph(endSeg.first, g)){
				endPt = endSeg.first;
				length += bg::distance(endPoint, endPt);
			}else{
				endPt = endSeg.second;
				length += bg::distance(endPoint, endPt);
			}
			if(!bg::equals(startPt, endPt)){
				
				currentPt = startPt;
				while(!bg::equals((nextPt = nextInPolygon(currentPt, previousPt, poly, "2539")), endPt)){
					length += bg::distance(currentPt, nextPt);
					previousPt = currentPt;
					currentPt = nextPt;
			
				}
				length += bg::distance(currentPt, endPt);
			}




		}else{ // crossInsideGraph == false
			Point_2 startPt, endPt;
			Point_2 currentPt, previousPt, nextPt;
			if(pointInsideGraph(startSeg.first, g)){
				startPt = startSeg.second;
				previousPt = startSeg.first;
				length += bg::distance(startPoint, startPt);
			}else{
				startPt = startSeg.first;
				previousPt = startSeg.second;
				length += bg::distance(startPoint, startPt);
			}

			if(pointInsideGraph(endSeg.first, g)){
				endPt = endSeg.second;
				length += bg::distance(endPoint, endPt);
			}else{
				endPt = endSeg.first;
				length += bg::distance(endPoint, endPt);
			}
			if(!bg::equals(startPt, endPt)){
				
				currentPt = startPt;
				while(!bg::equals((nextPt = nextInPolygon(currentPt, previousPt, poly, "2574")), endPt)){
					length += bg::distance(currentPt, nextPt);
					previousPt = currentPt;
					currentPt = nextPt;
			
				}
				length += bg::distance(currentPt, endPt);
			}



		}// end of crossInsideGraph == false
		return length;
	}

	
}
						

void Roadmap::addPathOnBoundary(Point_2& v1, Point_2& v2, Graph& g, Polygon_2& poly, Segment_2& polyEdge_1,
	Segment_2& polyEdge_2,int& vid){
	Point_2 startPoint, endPoint, currentPoint, previousPoint, nextPoint;


	if(pointInSegment(v1, polyEdge_1) && pointInSegment(v2, polyEdge_2)){
		if(bg::equals(polyEdge_1, polyEdge_2)){
			m_graph.addEdge(m_pointVidMap[v1], m_pointVidMap[v2]);     
		}else{  
		if(pointInsideGraph(polyEdge_1.first, g)){
			m_vidPointMap[vid] = polyEdge_1.first;
			m_pointVidMap[polyEdge_1.first] = vid;

			m_graph.addEdge(m_pointVidMap[v1], vid);
			startPoint = polyEdge_1.first;

			previousPoint = polyEdge_1.second;
		}else{
			m_vidPointMap[vid] = polyEdge_1.second;
			m_pointVidMap[polyEdge_1.second] = vid;
			
			m_graph.addEdge(m_pointVidMap[v1], vid);
			startPoint = polyEdge_1.second;
			previousPoint = polyEdge_1.first;
		}
		vid++;
		if(pointInsideGraph(polyEdge_2.first, g)){
			m_vidPointMap[vid] = polyEdge_2.first;
			m_pointVidMap[polyEdge_2.first] = vid;

			m_graph.addEdge(m_pointVidMap[v2], vid);
			endPoint = polyEdge_2.first;
		}else{
			m_vidPointMap[vid] = polyEdge_2.second;
			m_pointVidMap[polyEdge_2.second] = vid;
			
			m_graph.addEdge(m_pointVidMap[v2], vid);
			endPoint = polyEdge_2.second;
		}
		currentPoint = startPoint;
		vid++;                                         

		while(!bg::equals((nextPoint = nextInPolygon(currentPoint, previousPoint, poly, "2635")), endPoint)){
			m_vidPointMap[vid] = nextPoint;           
			m_pointVidMap[nextPoint] = vid;
			m_graph.addEdge(m_pointVidMap[currentPoint], vid);
			previousPoint = currentPoint;
			currentPoint = nextPoint;
			vid++;
		}
  
    		}
	}
}

Polygon_2 Roadmap::graphToPolygon(Graph g){
	Polygon_2 poly;
	std::vector<int> vVec;
	g.getCycleVertexVector(vVec);
	for(int i = 0; i < vVec.size(); i++){
		Point_2 p = m_vidPointMap[vVec[i]];
		bg::append(poly.outer(), p);
	}
	bg::correct(poly);
	return poly; 
}

/*
*	return whether one point is inside a surrounding cycle represented as a Graph
*/
bool Roadmap::pointInsideGraph(Point_2& point, Graph& g){
	Polygon_2 poly;
	std::vector<int> vVec;
	g.getCycleVertexVector(vVec);
	for(int i = 0; i < vVec.size(); i++){
		Point_2 p = m_vidPointMap[vVec[i]];
		bg::append(poly.outer(), p);
		if(bg::equals(point, p)){
			return false;
		}
	}
	bg::correct(poly);
	if(bg::within(point, poly)){
		return true;
	}else{
		return false;
	}
}


bool Roadmap::pointInSegment(Point_2 v, Segment_2 seg){
	if(bg::covered_by(v, seg)){
		return true;
	}else{
		return false;
	}
}



Point_2 Roadmap::nextInPolygon(Point_2 currentPoint, Point_2 previousPoint, Polygon_2 poly, std::string str){
//	bg::correct(poly);
	int size = poly.outer().size();
	int i = 0;
	Segment_2 seg_1, seg_2;
	seg_1.first = Point_2(0.0, 0.0);
	seg_1.second = Point_2(0.0, 0.0);
	seg_2.first = Point_2(0.0, 0.0);
	seg_2.second = Point_2(0.0, 0.0);
	for(i = 0; i < size; i ++){
		if(bg::equals(poly.outer()[i], currentPoint)){
			if(i == 0){
				if(bg::equals(poly.outer()[size-2], previousPoint)){
					return poly.outer()[1];
				}else if(bg::equals(poly.outer()[1], previousPoint)){
					return poly.outer()[size-2];
				}else{
					//std::cout<<str<<"previousPoint not polyPt"<<std::endl;
					seg_1.first = poly.outer()[size-2]; 
					seg_1.second = poly.outer()[0];
					seg_2.first = poly.outer()[0];
					seg_2.second = poly.outer()[1];
					if(bg::intersects(previousPoint, seg_1) || isPointInSeg(previousPoint, seg_1)){
						return poly.outer()[1];
					}else if(bg::intersects(previousPoint, seg_2) || isPointInSeg(previousPoint, seg_2)){
						return poly.outer()[size-2];
					}
				}
			}
			if(bg::equals(poly.outer()[i-1], previousPoint)){
				return poly.outer()[i+1];
			}else if(bg::equals(poly.outer()[i+1], previousPoint)){
				return poly.outer()[i-1];
			}else{
				//std::cout<<str<<"previousPoint not polyPt"<<std::endl;
				seg_1.first = poly.outer()[i-1];
				seg_1.second = poly.outer()[i];
				seg_2.first = poly.outer()[i+1];
				seg_2.second = poly.outer()[i];
				if(isPointInSeg(previousPoint, seg_1)){
					return poly.outer()[i+1];
				}else if(isPointInSeg(previousPoint, seg_2)){
					return poly.outer()[i-1];
				}
			}
		}

		if(bg::equals(poly.outer()[i], previousPoint)){
			if(str == "426"){
				//std::cout<<"426 enters"<<std::endl;
			}
			if(i == 0){
				if(bg::equals(poly.outer()[size-2], currentPoint)){
					return poly.outer()[size-3];
				}else if(bg::equals(poly.outer()[1], currentPoint)){
					return poly.outer()[2];
				}else{
					//std::cout<<str<<"currentPoint not polyPt"<<std::endl;
					seg_1.first = poly.outer()[size-2];
					seg_1.second = poly.outer()[0];
					seg_2.first = poly.outer()[0];
					seg_2.second = poly.outer()[1];
					if(bg::intersects(currentPoint, seg_1) || isPointInSeg(currentPoint, seg_1)){
						return poly.outer()[size-2];
					}else if(bg::intersects(currentPoint, seg_2) || isPointInSeg(currentPoint, seg_2)){
						return poly.outer()[1];
					}
				}
			}
			if(i == 1){
				if(bg::equals(poly.outer()[0], currentPoint)){
					return poly.outer()[size-2];
				}else if(bg::equals(poly.outer()[2], currentPoint)){
					return poly.outer()[3];
				}else{
					//std::cout<<str<<"currentPoint not polyPt"<<std::endl;
					seg_1.first = poly.outer()[1];
					seg_1.second = poly.outer()[0];
					seg_2.first = poly.outer()[1];
					seg_2.second = poly.outer()[2];
					if(bg::intersects(currentPoint, seg_1) || isPointInSeg(currentPoint, seg_1)){
						return poly.outer()[0];
					}else if(bg::intersects(currentPoint, seg_2) || isPointInSeg(currentPoint, seg_2)){
						return poly.outer()[2];
					}
				}
			}
			if(bg::equals(poly.outer()[i-1], currentPoint)){
				return poly.outer()[i-2];
			}else if(bg::equals(poly.outer()[i+1], currentPoint)){
				return poly.outer()[i+2];
			}else{
				//std::cout<<str<<"currentPoint not polyPt"<<std::endl;
				if(str == "426"){
					//std::cout<<"426 enters here"<<std::endl;
				}
				seg_1.first = poly.outer()[i-1];
				seg_1.second = poly.outer()[i];
				seg_2.first = poly.outer()[i+1];
				seg_2.second = poly.outer()[i];
				if(bg::intersects(currentPoint, seg_1) || bg::within(currentPoint, seg_1) || bg::touches(currentPoint, seg_1) || isPointInSeg(currentPoint, seg_1)){
					return poly.outer()[i-1];
				}else if(bg::intersects(currentPoint, seg_2) || isPointInSeg(currentPoint, seg_2)){
					return poly.outer()[i+1];
				}
			}	
		}
	}
	std::cout<<"could not find the next point in"<<str<<std::endl;

	return Point_2(0, 0);
}
//preprocess the vertexes of the polygon, so each single segment's length is not that short, 
//easy for the following adding edges to the graph. 


void Roadmap::
removeExcessEdges(bool recoverConnection){
	// We do this in several steps. First, we go through each polygon obstacle boundary 
	// and delete all edges of the lattice that intersect with these boundaries. Then, 
	// we find the connected components of the remaining lattice graph. For each component
	// we only need to test one vertex to know whether it belongs to the configuration space
	// or not. We keep all components that belong to the configuration space

	// =====================================================================================
	// Compute the set of edges that falls on obstacle boundaries, at the same time also
	// compute the smallest cycle in the full lattice that encloses the obstacle. First do
	// it for the bounding polygon
	getIntersectingEdges(*m_pBoundingRect, m_boundaryBoundingCycle, true);

	// Then for all obstacles 
	for(Polygon2_list::iterator obsit = m_objectPolyList.begin(); obsit != m_objectPolyList.end(); obsit++){
		Graph* pg = new Graph();
		m_obsBoundingCycleVec.push_back(pg);
		getIntersectingEdges(*obsit, *pg);
	}

	// =====================================================================================
	// Remove edges that do not belong to the graph
	Graph g;
	std::set<std::pair<int, int> > edgeSet = m_graph.getEdgeSet();
	for(std::set<std::pair<int, int> >::iterator eit = edgeSet.begin(); eit != edgeSet.end(); eit++){
		int fv = (*eit).first;
		int sv = (*eit).second;
		if(!edgeInSet(fv, sv, m_edgeToBeRemovedSet)){
			g.addEdge(fv, sv);
		}
	}
	m_graph = g;

	// =====================================================================================
	// Go through all vertices and find all vertices and edges inside the configuration
	// space. To do so, iterate over all lattices points (the full lattice minus the edges that
	// crosses obstacle boundaries) and for each point, if it has not been checked, see whether
	// the point is inside the configuration space. Then we do a BFS from the point and visit
	// all points/edges connected to the point. We add the edges to our final graph if and only
	// if the starting vertex is in the c-space. This way, we need to do c-space membership
	// check geometrically only very limited number of times, usually around the number of
	// obstacles in the c-space.

	g.clear();
	std::set<int> visitedVertices;
	for(std::map<int, Point_2>::iterator vit = m_vidPointMap.begin(); vit != m_vidPointMap.end(); vit++){
		int vid = vit->first;
		if(visitedVertices.find(vid) == visitedVertices.end()){
			visitedVertices.insert(vid);
			// Test whether the vertex is inside the configuration space
			bool inCSpace = isPointInCSpace(vit->second);

			// Do BFS
			std::list<int> tempQueue;
			tempQueue.push_back(vid);
			while(tempQueue.size() > 0){
				int current = tempQueue.front();
				tempQueue.pop_front();

				std::set<int> neighborSet = m_graph.getNeighborSet(current);

				for(std::set<int>::iterator vit = neighborSet.begin(); vit != neighborSet.end(); vit++){
					// Retrieve first and second vertices
					int et = *vit;
					if(visitedVertices.find(et) == visitedVertices.end()){
						visitedVertices.insert(et);
						tempQueue.push_back(et);
					}
					// Add edge as needed
					if(inCSpace) g.addEdge(current, et);
				}
			}	
		}

	}
	m_graph.clear();
	m_graph = g;

	// Remove isolated vertices
	for(std::set<int>::iterator vit = m_vertexToBeRemovedSet.begin(); vit != m_vertexToBeRemovedSet.end(); vit++){
		 std::cout << "Removing vertex with id: " << *vit << std::endl;
		m_graph.removeVertex(*vit);
	}

	// Remove single degree edges
	std::set<int> vSetCopy = m_graph.getVertexSet();
	for(std::set<int>::iterator vit = vSetCopy.begin(); vit != vSetCopy.end(); vit++){
		if(m_graph.hasVertex(*vit)){
			if(m_graph.getNeighborSet(*vit).size() == 0){
				m_graph.removeVertex(*vit);
				 std::cout << "Removing vertex with id: " << *vit << std::endl;
			}
		}
	}
	int vIDCount;		
	m_finalGraph.clear();
	vIDCount = buildFinalGraph();
	if(recoverConnection){ 
		recoverConnectivity_2(vIDCount);
	}
}

int Roadmap::buildFinalGraph(){
	// Construct m_finalGraph
	std::set<std::pair<int, int>>& eSet = m_graph.getEdgeSet();
	int vIDCount = 0;
//	m_finalGraph.clear();
	for(std::set<std::pair<int, int> >::iterator eit = eSet.begin(); eit != eSet.end(); eit++){
		int fv = eit->first;
		int sv = eit->second;

		// Check whether fv was added
		if(m_vidGFMap.find(fv) == m_vidGFMap.end()){
			m_vidGFMap[fv] = vIDCount;
			m_vidFGMap[vIDCount] = fv;
			fv = vIDCount++;
		}else{
			fv = m_vidGFMap[fv];

		}

		// Check whether sv was added
		if(m_vidGFMap.find(sv) == m_vidGFMap.end()){
			m_vidGFMap[sv] = vIDCount;
			m_vidFGMap[vIDCount] =sv;
			sv = vIDCount++;
		}
		else{
			sv = m_vidGFMap[sv];
		}
		m_finalGraph.addEdge(fv, sv);
		std::pair<int, int> finalGraphEdge;
		finalGraphEdge.first = fv;
		finalGraphEdge.second = sv;
		std::vector<int> graphEdgeVector;
		graphEdgeVector.push_back(eit->first);
		graphEdgeVector.push_back(eit->second);
		m_edgeFGMap[finalGraphEdge] = graphEdgeVector;


		std::pair<int, int> finalGraphEdge_r;
		finalGraphEdge_r.first = sv;
		finalGraphEdge_r.second = fv;
		std::vector<int> graphEdgeVector_r;
	
		graphEdgeVector_r.push_back(eit->second);
		graphEdgeVector_r.push_back(eit->first);
		m_edgeFGMap[finalGraphEdge_r] = graphEdgeVector_r;
	}
	return vIDCount;
}

void Roadmap::buildComputeMap(){
	std::set<std::pair<int, int>>& eSet = m_finalGraph.getEdgeSet();
	int vIDCount = 0;
	m_computeGraph.clear();
	m_vidCFMap.clear();
	m_vidFCMap.clear();
	for(std::set<std::pair<int, int> >::iterator eit = eSet.begin(); eit != eSet.end(); eit++){
		int fv = eit->first;
		int sv = eit->second;

		// Check whether fv was added

		if(m_vidFCMap.find(fv) == m_vidFCMap.end()){
			m_vidFCMap[fv] = vIDCount;
			m_vidCFMap[vIDCount] = fv;
			fv = vIDCount++;
		}else{
			fv = m_vidFCMap[fv];

		}

		// Check whether sv was added
		if(m_vidFCMap.find(sv) == m_vidFCMap.end()){
			m_vidFCMap[sv] = vIDCount;
			m_vidCFMap[vIDCount] =sv;
			sv = vIDCount++;
		}
		else{
			sv = m_vidFCMap[sv];
		}
		if(!m_computeGraph.hasEdge(fv, sv)){ 
			m_computeGraph.addEdge(fv, sv);
		}
	}
}

void Roadmap::drawBoundingCycle(QGraphicsScene& scene){
	// Draw the cycles
	QPen regularPen = QPen(Qt::yellow, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	QPen illegalPen = QPen(QColor(255, 0, 0, 127), 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	for(std::vector<Graph*>::iterator git = m_obsBoundingCycleVec.begin(); git != m_obsBoundingCycleVec.end(); git++){
		Graph& g = *(*git);
		std::set<std::pair<int, int> > edgeSet = g.getEdgeSet();
		for(std::set<std::pair<int, int> >::iterator eit = edgeSet.begin(); eit != edgeSet.end(); eit++){
			int v1 = (*eit).first;
			int v2 = (*eit).second;
			Point_2 p1 = (m_vidPointMap[v1]);
			Point_2 p2 = (m_vidPointMap[v2]);
			if(m_graph.hasEdge(v1, v2)){
				scene.addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), regularPen);
			}
			else{
				scene.addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), illegalPen);
			}
		}
	}
}
void Roadmap::drawJointBoundingCycle(QGraphicsScene& scene){
	QPen regularPen = QPen(Qt::black, 2.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	std::set<std::pair<int, int> > edgeSet = m_jointCycle.getEdgeSet();
		for(std::set<std::pair<int, int> >::iterator eit = edgeSet.begin(); eit != edgeSet.end(); eit++){
			int v1 = (*eit).first;
			int v2 = (*eit).second;
			Point_2 p1 = (m_vidPointMap[v1]);
			Point_2 p2 = (m_vidPointMap[v2]);
			
				scene.addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), regularPen);
			
		}
}

bool Roadmap::isPointInCSpace(Point_2 &p){
	// Check whether the point is inside the bounding rect
	if(!bg::within(p, *m_pBoundingRect)){
		return false;
	}

    for(Polygon2_list::iterator pli = m_objectPolyList.begin(); pli != m_objectPolyList.end(); pli++){
		Polygon_2 &tp = *(pli);
		if(bg::within(p, tp)){
			return false;
		}
	}
	return true;
}

void Roadmap::drawPoint(Point_2 & p, QGraphicsScene& scene){
	Point_2 icp = (p);
	scene.addEllipse(icp.get<0>() - 0.025, icp.get<1>() - 0.025, 0.05, 0.05, QPen(Qt::red, 0.05, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
}

void Roadmap::drawVertexIds(QGraphicsScene& scene){
	// Draw the id text of the vretex
	QFont font;
	QPainterPath path;
	font.setPointSizeF(m_radius/1.5);
	font.setBold(false);
	for(std::set<int>::iterator vit = m_graph.getVertexSet().begin(); vit != m_graph.getVertexSet().end(); vit ++){
		Point_2 p = (m_vidPointMap[*vit]);
		//if(m_vidGFMap[*vit] != 0){  
		QGraphicsSimpleTextItem *ti = scene.addSimpleText(QString::number(*vit), font);
		ti->setPos(p.get<0>() + m_radius/2, p.get<1>() - m_radius/2);
		ti->setPen(QPen(QColor(Qt::green), 0.03*m_radius, Qt::SolidLine, Qt::RoundCap,Qt::RoundJoin));
		ti->setZValue(2);
	//}
	}
}


void Roadmap::new_buildVisibilityGraph(Polygon2_list temp_obstacle_list, Environment *& env, Visibility_Graph *&v_graph) {
	vector<Polygon> polyVec;

	// We assume that the infated obstacles do not intersect each other but may intersect the boundary.
	// Therefore, we test such intersection and obtain an updated boundary (the inside)
	Polygon boundaryPoly;
	convertFromBoosttoVisilibity(*m_pBoundingRect, boundaryPoly, true);
	polyVec.insert(polyVec.begin(), boundaryPoly);


	bool no_intersection = true;
	std::vector<Polygon_2> merge_obj_list;
	//Polygon2_list temp_obstacle_list = m_objectOuterPolyList;
	Polygon2_list final_list;
	while (temp_obstacle_list.size() > 0) {
		no_intersection = true;
		//std::cout << "temp_obstacle_list.size = " << temp_obstacle_list.size() << std::endl;
		Polygon_2 obj = temp_obstacle_list.front();
		temp_obstacle_list.pop_front();
		for (auto i = temp_obstacle_list.begin(); i != temp_obstacle_list.end(); i++) {
			if (bg::intersects(obj, *i)) {
				//	std::cout << "intersection happen" << std::endl;
				Polygon_2 temp = *i;
				temp_obstacle_list.erase(i);
				bg::union_(obj, temp, merge_obj_list);

				temp_obstacle_list.push_front(merge_obj_list[0]);
				for (int k = 0; k < merge_obj_list[0].outer().size(); k++) {
					//	std::cout << merge_obj_list[0].outer()[k].get<0>() << "," << merge_obj_list[0].outer()[k].get<1>() << std::endl;
				}
				merge_obj_list.clear();
				no_intersection = false;
				break;
			}
		}
		if (no_intersection) {
			final_list.push_back(obj);
		}
	}
	for (auto pli = final_list.begin(); pli != final_list.end(); pli++) {
		Polygon tempPoly;
		convertFromBoosttoVisilibity(*pli, tempPoly, false);
		polyVec.push_back(tempPoly);
	}

	/*
	for(Polygon2_list::iterator pli = m_objectOuterPolyList.begin(); pli != m_objectOuterPolyList.end(); pli++){
	// ECPolygon_2 ecPoly = convertToExactPolygon(*pli);
	// if(bg::intersection(*m_pBoundingRect, )){
	// 	vector<ECPolygon_with_holes_2> outVec;

	// 	// Remove the obstacle "from" the boundary polygon
	// 	CGAL::difference(boundary, ecPoly, back_inserter(outVec));

	// 	// Update boundary
	// 	boundary = outVec[0].outer_boundarget<1>();
	// }
	// else{
	// Add all non intersecting obstacles

	Polygon tempPoly;
	convertFromBoosttoVisilibity(*pli, tempPoly, false);
	polyVec.push_back(tempPoly);

	}
	*/
	// Insert the boundary 


	// Build environment and visibility graph	
	env = new Environment(polyVec);
	v_graph = new Visibility_Graph(*env, m_epsilon);
}

void Roadmap::buildVisibilityGraph(){
	// To build the visibility graph, we use the package VisiLibity, which is not precise arithematic, 
	// but good for our purpose since we do not use visibility graph as part of our main logic. 

	// Only build once per roadmap
	//if(m_pVisibilityGraph != 0) return;

	// Vector to hold all polygon for contructing VisiLibity object
	vector<Polygon> polyVec;

	// We assume that the infated obstacles do not intersect each other but may intersect the boundary.
	// Therefore, we test such intersection and obtain an updated boundary (the inside)
	Polygon boundaryPoly;
	convertFromBoosttoVisilibity(*m_pBoundingRect, boundaryPoly, true);
	polyVec.insert(polyVec.begin(), boundaryPoly);


	bool no_intersection = true;
	std::vector<Polygon_2> merge_obj_list;
	Polygon2_list temp_obstacle_list = m_objectOuterPolyList;
	Polygon2_list final_list;
	while (temp_obstacle_list.size() > 0) {
		no_intersection = true;
		//std::cout << "temp_obstacle_list.size = " << temp_obstacle_list.size() << std::endl;
		Polygon_2 obj = temp_obstacle_list.front();
		temp_obstacle_list.pop_front();
		for (auto i = temp_obstacle_list.begin(); i != temp_obstacle_list.end(); i++) {
			if (bg::intersects(obj, *i)) {
			//	std::cout << "intersection happen" << std::endl;
				Polygon_2 temp = *i;
				temp_obstacle_list.erase(i);
				bg::union_(obj, temp, merge_obj_list);

				temp_obstacle_list.push_front(merge_obj_list[0]);
				for (int k = 0; k < merge_obj_list[0].outer().size(); k++) {
				//	std::cout << merge_obj_list[0].outer()[k].get<0>() << "," << merge_obj_list[0].outer()[k].get<1>() << std::endl;
				}
				merge_obj_list.clear();
				no_intersection = false;
				break;
			}
		}
		if (no_intersection) {
			final_list.push_back(obj);
		}
	}
	for (auto pli = final_list.begin(); pli != final_list.end(); pli++) {
		Polygon tempPoly;
		convertFromBoosttoVisilibity(*pli, tempPoly, false);
		polyVec.push_back(tempPoly);
	}

	/* 
	for(Polygon2_list::iterator pli = m_objectOuterPolyList.begin(); pli != m_objectOuterPolyList.end(); pli++){
		// ECPolygon_2 ecPoly = convertToExactPolygon(*pli);
		// if(bg::intersection(*m_pBoundingRect, )){
		// 	vector<ECPolygon_with_holes_2> outVec;

		// 	// Remove the obstacle "from" the boundary polygon
		// 	CGAL::difference(boundary, ecPoly, back_inserter(outVec)); 

		// 	// Update boundary
		// 	boundary = outVec[0].outer_boundarget<1>();
		// }
		// else{
			// Add all non intersecting obstacles 

			Polygon tempPoly;
			convertFromBoosttoVisilibity(*pli, tempPoly, false);
			polyVec.push_back(tempPoly);
		
	}
	*/
	// Insert the boundary 
	

	// Build environment and visibility graph	
	m_pEnvironment = new Environment(polyVec);
	m_pVisibilityGraph = new Visibility_Graph(*m_pEnvironment, m_epsilon);
}

double Roadmap::new_computeShortestPath(Point_2& p1, Point_2& p2, std::list<Point_2> & path, Environment * env, Visibility_Graph *v_graph) {
	Point_2 p1x = (p1);
	Point_2 p2x = (p2);
	return new_computeShortestPath(p1x.get<0>(), p1x.get<1>(), p2x.get<0>(), p2x.get<1>(), path);
}

double Roadmap::computeShortestPath(Point_2& p1, Point_2& p2, std::list<Point_2> & path){
	Point_2 p1x = (p1);
	Point_2 p2x = (p2);
	return computeShortestPath(p1x.get<0>(), p1x.get<1>(), p2x.get<0>(), p2x.get<1>(), path);
}

double Roadmap::visi_computeShortestPath(double x1, double y1, double x2, double y2, std::list<Point_2> & path, Environment * env, Visibility_Graph *v_graph) {
	//State target(x2, y2);
	//double result_dist = 0;
	double visi_dist = 0;
	//rrts.connect(target, result_dist);

	Polyline pl = env->shortest_path(Point(x1, y1), Point(x2, y2), *v_graph, m_epsilon);
	for (int i = 0; i < pl.size(); i++) {
		path.push_back(Point_2(pl[i].x(), pl[i].y()));
	}
	visi_dist = getPathLength(path);
	//std::cout << "rrt:" << result_dist << "    visi:" << visi_dist << std::endl;
	return getPathLength(path);
	//return result_dist;
}

double Roadmap::new_computeShortestPath(double x1, double y1, double x2, double y2, std::list<Point_2> & path, planner_t & planner) {
	State target(x2, y2);
	double result_dist = 0;
	double visi_dist = 0;
	//prm.setGoalNode(x2, y2);
	//prm.setInitNode(x1, y1);
	//std::vector<Edge*> returned_path;
	//returned_path = prm.query(5, result_dist);
	//if (returned_path.size() > 0) {
	//	return result_dist;
	//}
	//else {
	//	return 1000000000;
	//}
	if (planner.connect(target, result_dist) > 0) {
	 	return result_dist;
	}
	else {
	 	return 100000000;
	}

	//Polyline pl = env->shortest_path(Point(x1, y1), Point(x2, y2), *v_graph, m_epsilon);
	//for (int i = 0; i < pl.size(); i++) {
	//	path.push_back(Point_2(pl[i].x(), pl[i].y()));
	//}
	//visi_dist = getPathLength(path);
	//std::cout << "rrt:" << result_dist << "    visi:" << visi_dist << std::endl;
	//return getPathLength(path);
	//return result_dist;
}

// double Roadmap::new_computeShortestPath(double x1, double y1, double x2, double y2, std::list<Point_2> & path, PRM & prm) {
// 	State target(x2, y2);
// 	double result_dist = 0;
// 	double visi_dist = 0;
// 	prm.setGoalNode(x2, y2);
// 	prm.setInitNode(x1, y1);
// 	std::vector<Edge*> returned_path;
// 	returned_path = prm.query(5, result_dist);
// 	if(returned_path.size() > 0){
// 		return result_dist;
// 	}else{
// 		return 1000000000;
// 	}
// 	// if (planner.connect(target, result_dist) > 0) {
// 	// 	return result_dist;
// 	// }
// 	// else {
// 	// 	return 100000000;
// 	// }

// 	//Polyline pl = env->shortest_path(Point(x1, y1), Point(x2, y2), *v_graph, m_epsilon);
// 	//for (int i = 0; i < pl.size(); i++) {
// 	//	path.push_back(Point_2(pl[i].x(), pl[i].y()));
// 	//}
// 	//visi_dist = getPathLength(path);
// 	//std::cout << "rrt:" << result_dist << "    visi:" << visi_dist << std::endl;
// 	//return getPathLength(path);
// 	//return result_dist;
// }
double Roadmap::new_computeShortestPath(double x1, double y1, double x2, double y2, std::list<Point_2> & path) {
	State target(x2, y2);
	double result_dist = 0;
	double visi_dist = 0;
	//if (rrts.connect(target, result_dist) > 0) {
	//	return result_dist;
	//}
	//else {
		return 100000000;
	//}
	
	//Polyline pl = env->shortest_path(Point(x1, y1), Point(x2, y2), *v_graph, m_epsilon);
	//for (int i = 0; i < pl.size(); i++) {
	//	path.push_back(Point_2(pl[i].x(), pl[i].y()));
	//}
	//visi_dist = getPathLength(path);
	//std::cout << "rrt:" << result_dist << "    visi:" << visi_dist << std::endl;
	//return getPathLength(path);
	//return result_dist;
}

double Roadmap::new_computeShortestPath_1(double x1, double y1, double x2, double y2, std::list<Point_2> & path) {
	State target(x2, y2);
	double result_dist = 0;
	double visi_dist = 0;
	//if (rrts_2.connect(target, result_dist) > 0) {
		return result_dist;
	//}
	//else {
	//	return 10000000;
	//}

	//Polyline pl = env->shortest_path(Point(x1, y1), Point(x2, y2), *v_graph, m_epsilon);
	//for (int i = 0; i < pl.size(); i++) {
	//	path.push_back(Point_2(pl[i].x(), pl[i].y()));
	//}
	//visi_dist = getPathLength(path);
	//std::cout << "rrt:" << result_dist << "    visi:" << visi_dist << std::endl;
	//return getPathLength(path);
	//return result_dist;
}

double Roadmap::computeShortestPath(double x1, double y1, double x2, double y2, std::list<Point_2> & path){
	Polyline pl = m_pEnvironment->shortest_path(Point(x1, y1), Point(x2, y2), *m_pVisibilityGraph, m_epsilon);
	for(int i = 0; i < pl.size(); i++){
		path.push_back(Point_2(pl[i].x(), pl[i].y()));
	}
	return getPathLength(path);
}

double Roadmap::new_computeShortestPath(double x1, double y1, double x2, double y2, vector<pair<double, double> >& path, Environment * env, Visibility_Graph *v_graph) {
	// Compute shortest path
	std::list<Point_2> shortestPath;
	double length = new_computeShortestPath(x1, y1, x2, y2, shortestPath);

	// Convert the path
	for (std::list<Point_2>::iterator vi = shortestPath.begin(); vi != shortestPath.end(); vi++) {
		Point_2 p = (*vi);
		path.push_back(pair<double, double>(p.get<0>(), p.get<1>()));
	}
	return length;
}


double Roadmap::computeShortestPath(double x1, double y1, double x2, double y2, vector<pair<double, double> >& path){
	// Compute shortest path
	std::list<Point_2> shortestPath;
	double length = computeShortestPath(x1, y1, x2, y2, shortestPath);

	// Convert the path
	for(std::list<Point_2>::iterator vi = shortestPath.begin(); vi != shortestPath.end(); vi++){
		Point_2 p = (*vi);
		path.push_back(pair<double, double>(p.get<0>(), p.get<1>()));
	}
	return length;
}



void Roadmap::addToScene(QGraphicsScene& scene, bool drawEdge, QPen edgePen, bool drawVertex, QPen vertexPen){
	// Paint the edges
	if(drawEdge){
		std::set<std::pair<int, int> > edgeSet = m_graph.getEdgeSet();
		for(std::set<std::pair<int, int> >::iterator eit = edgeSet.begin(); eit != edgeSet.end(); eit++){
			int v1 = (*eit).first;
			int v2 = (*eit).second;
			Point_2 p1 = (m_vidPointMap[v1]);
			Point_2 p2 = (m_vidPointMap[v2]);
			scene.addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), edgePen);
			/*}*/

			if(m_boundaryBoundingCycle.hasEdge(v1, v2)){
				scene.addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), QPen(Qt::blue, 0.05, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
			}
			else{
				for(std::vector<Graph*>::iterator git = m_obsBoundingCycleVec.begin(); git != m_obsBoundingCycleVec.end(); git++){
					if((*git)->hasEdge(v1, v2)){
						scene.addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), QPen(Qt::yellow, 0.05, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
					}
				}
			}
		}
	}

	// Paint vertices if needed and obtain the fill area
	if(!drawVertex){
		for(std::list<Point_2>::iterator it = m_pointList.begin(); it != m_pointList.end(); it ++){
			Point_2 p = (*it);
			scene.addEllipse(p.get<0>() - 0.025, p.get<1>() - 0.025, 0.05, 0.05, vertexPen);
		}
	}

	for(Path_Map::iterator pathIt = m_connectingPathMap.begin(); pathIt != m_connectingPathMap.end(); pathIt ++){
		std::list<Point_2>& shortestPath = pathIt->second;
		std::list<Point_2>::iterator vit = shortestPath.begin();
		if(vit != shortestPath.end()){
			Point_2 p = (*vit);
			vit++;
			for(; vit != shortestPath.end(); vit++){
				Point_2 p2 = (*vit);
				scene.addLine(p.get<0>(), p.get<1>(), p2.get<0>(), p2.get<1>(), QPen(Qt::magenta, 0.025, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
				p = p2;
			}
		}
	}

	// std::cout << shortestPath.size() << std::endl;

}

void Roadmap::getIntersectingEdges(Polygon_2 & poly, Graph& boundingCycle, bool outerBoundary){
	// Detecting all edges of the lattice graph that intersects the boundary of poly

	// =====================================================================================
	// Get a vertex of the poly and find the hexgaon of the lattice that contains the vetex.
	// For each col and row combo, there are three possible hexgaons the point may fall into

	// Locate the hexgon (relative to some arbitrary base choice)
	Point_2 p0 = poly.outer()[0];
	std::pair<int, int> crp = locateHexagonForPoint(p0);
	int col = crp.first;
	int row = crp.second;

	// Compute the current hexagon
	Point_2 p[6];
	Segment_2 e[6];
	Polygon_2 hex; 
	int pIndex[6];
	populateHexagon(col, row, p, e, hex, pIndex);

	// =====================================================================================
	// With a point of the obstacle and a hexgon containing the point, we iteratively check 
	// intersections between the segments of the obstacle and the hexagon. We assume that 
	// the obstacle would have at least three vertices. Note that a obstacle may be smaller than
	// a single hexagon. 

	std::set<std::pair<int, int> > tempEdgeToRemoveSet;
	int numVertices = poly.outer().size() - 1;// polygon in Boost.geometry has n+1 vertics, last one is same as first one
	int currentVertexIndex = 0;
	while(currentVertexIndex < numVertices){
		// Get the current segment of polygon to be checked
		Point_2 nextVertex = poly.outer()[currentVertexIndex==numVertices-1?0:currentVertexIndex+1];


		// Check whether nextVertex is outside of the current hexagon. If we are still in the 
		// same hexagon, then move to the next obstacle vertex
		if(bg::within(nextVertex, hex)) {  // true when nextVertex is within hex
			currentVertexIndex ++;
			continue;
		}

		// If we are here, then we jumped outside of a lattice hexagon. Figure out which edge is 
		// being intersected. We keep doing this until we cover the entire obstacle edge
		Point_2 currentVertex = poly.outer()[currentVertexIndex];
		Segment_2 obsEdge(currentVertex, nextVertex);


		std::pair<int, int> lastPair(-10, -10);
		while(true){
			int edgeIndex = 0;
			while(edgeIndex < 6){
				if(bg::intersects(e[edgeIndex], obsEdge)){
					std::pair<int, int> edgePair(pIndex[edgeIndex], pIndex[edgeIndex==5?0:edgeIndex+1]);
					std::pair<int, int> edgePairReverse(pIndex[edgeIndex==5?0:edgeIndex+1], pIndex[edgeIndex]);
					if(lastPair != edgePair && lastPair != edgePairReverse){
						lastPair = edgePair;
						break;
					}
				}
				edgeIndex ++;
			};

			if(edgeIndex < 6){
				// We hit an intersection, mark the edge as to be removed
				tempEdgeToRemoveSet.insert(lastPair);
				m_edgeToBeRemovedSet.insert(lastPair);

				// Figure out the next hexgon to be checked 
				if(col%2 == 0){
					switch(edgeIndex){
						case 0: col--; break;
						case 1: row++; break;
						case 2: col++; break;
						case 3: col++; row--; break;
						case 4: row--; break;
						case 5: col--; row--; break;
						default: break;
					}
				}
				else{
					switch(edgeIndex){
						case 0: col--; row++; break;
						case 1: row++; break;
						case 2: col++; row++; break;
						case 3: col++; break;
						case 4: row--; break;
						case 5: col--; break;
						default: break;
					}
				}
				populateHexagon(col, row, p, e, hex, pIndex);
			}
			else{
				// No more intersections, we are done with the current obstacle edge
				break;
			}
		}
		currentVertexIndex++;
	}

	// =====================================================================================
	// We now have all the edges of the lattice that lie on the polygonal obstacle boundary. 
	// Next, we locate the bounding cycle in the full lattice graph surrounding the obstacle
	if(tempEdgeToRemoveSet.size() > 0){
		// Iterate through edges to be removed and find one with an end vertex that 
		// has an associated edge in the configuraiton space
		int sIndex, t1Index, t2Index; 
		for(std::set<std::pair<int,int> >::iterator eit = tempEdgeToRemoveSet.begin();
		eit != tempEdgeToRemoveSet.end(); eit++){
			std::pair<int, int> edge = *eit;
			// Try one vertex
			sIndex = edge.first;
			if((t2Index = pointBelongToEdgeOutideObstacle(poly, sIndex, tempEdgeToRemoveSet, outerBoundary))!= -1)

			{
				t1Index = edge.second;
				break;
			}

			// Try the other vertex
			sIndex = edge.second;
			if((t2Index = pointBelongToEdgeOutideObstacle(poly, sIndex, tempEdgeToRemoveSet, outerBoundary))!= -1)
			{
				t1Index = edge.first;
				break;
			}

		}

		// Now edge (sIndex, t1Index) crosses the boundary, and (sIndex, t2Index) it outside the boundary 
		// We simply locate a hexagon containing these two edges to extend the cycle 
		Point_2 p1 = m_vidPointMap[sIndex]; 
		Point_2 p2 = m_vidPointMap[t1Index]; 
		Point_2 p3 = m_vidPointMap[t2Index];
		Point_2 midPoint = (Point_2((p2.get<0>() + p3.get<0>())/2, (p2.get<1>() + p3.get<1>())/2));
		crp = locateHexagonForPoint(midPoint);
		populateHexagon(crp.first, crp.second, p, e, hex, pIndex);

		int endIndex = sIndex;		// When we see this index again, we are done


		// Add valid edge to cycle
		addEdgeIfNotThere(sIndex, t2Index, boundingCycle);

		// With the first hexagon, we can keep going along the boundary 
		while(true){
			// Get next candidate index
			int nextIndex = getNextNodeInSequence(sIndex, t2Index, pIndex);
			
			p1 = m_vidPointMap[sIndex]; 
			p2 = m_vidPointMap[nextIndex]; 
			p3 = m_vidPointMap[t2Index];

			// Check whether the edge (t2Index, nextIndex) crosses boundary
			if(edgeInSet(t2Index, nextIndex, tempEdgeToRemoveSet)){
				// In this case, we move to the next hexagon
				crp = getBorderHexagon(crp.first, crp.second, nextIndex, t2Index, pIndex);
				populateHexagon(crp.first, crp.second, p, e, hex, pIndex);
				sIndex = nextIndex;


				// t2Index = sIndex;
			}
			else{
				// Edge is valid, update indices
				sIndex = t2Index;
				t2Index = nextIndex;

				// Add valid edge to cycle
				addEdgeIfNotThere(sIndex, t2Index, boundingCycle);
			}

			if(endIndex == t2Index) break;
		}
	}

	// =====================================================================================
	// Process bounding cycle to remove single degree vertices
	std::set<int> vSet = boundingCycle.getVertexSet();
	std::set<int> vSingleSet;
	// Collect single degree vertices
	for(std::set<int>::iterator vit = vSet.begin(); vit != vSet.end(); vit++){
		if(boundingCycle.getNeighborSet(*vit).size() == 1){
			vSingleSet.insert(*vit);
			// Need to check the neighbor as well
			int prevNbr = *vit;
			int nbr = *(boundingCycle.getNeighborSet(*vit).begin());
			while(boundingCycle.getNeighborSet(nbr).size()==2){
				vSingleSet.insert(nbr);
				std::set<int> nbrSet = boundingCycle.getNeighborSet(nbr);
				nbrSet.erase(prevNbr);

				prevNbr = nbr;
				nbr = *(nbrSet.begin());
			}
		}

	}
	// Delete them
	for(std::set<int>::iterator vit = vSingleSet.begin(); vit != vSingleSet.end(); vit++){
		boundingCycle.removeVertex(*vit);

		// It seems good to remove these single degree vertices 
		if(!outerBoundary)m_vertexToBeRemovedSet.insert(*vit);
	}

}

std::pair<int, int> Roadmap::getBorderHexagon(int col, int row, int i1, int i2, int pIndex[]){
	// First locate the index of i1
	int i1Index = 0;
	for(int i = 0; i < 6; i ++){
		if(pIndex[i] == i1){
			i1Index = i;
			break;
		}
	}

	// Figure out the edge index
	int edgeIndex = 0;
	int nextIndex = (i1Index == 5? 0 : i1Index + 1);
	if(pIndex[nextIndex] == i2){
		// Going clockwise
		edgeIndex = i1Index;
	}
	else{
		// Going counterclockwise
		edgeIndex = (i1Index + 6 - 1)%6;
	}

	// Locate the next hexagon
	if(col%2 == 0){
		switch(edgeIndex){
			case 0: col--; break;
			case 1: row++; break;
			case 2: col++; break;
			case 3: col++; row--; break;
			case 4: row--; break;
			case 5: col--; row--; break;
			default: break;
		}
	}
	else{
		switch(edgeIndex){
			case 0: col--; row++; break;
			case 1: row++; break;
			case 2: col++; row++; break;
			case 3: col++; break;
			case 4: row--; break;
			case 5: col--; break;
			default: break;
		}
	}
	return std::pair<int, int>(col, row);
}

bool Roadmap::edgeInSet(int v1, int v2, std::set<std::pair<int, int> >& edgeSet){
	std::pair<int, int> edge(v1, v2);
	std::pair<int, int> rEdge(v2, v1);
	return (edgeSet.find(edge) != edgeSet.end() || edgeSet.find(rEdge) != edgeSet.end());
}

void Roadmap::addEdgeIfNotThere(int v1, int v2, Graph & graph){
	if(!graph.hasEdge(v1, v2)){
		graph.addEdge(v1, v2);
	}
}

int Roadmap::getNextNodeInSequence(int i1, int i2, int pIndex[]){
	// First locate the index of i1
	int i1Index = 0;
	for(int i = 0; i < 6; i ++){
		if(pIndex[i] == i1){
			i1Index = i;
			break;
		}
	}
	int nextIndex = (i1Index == 5? 0 : i1Index + 1);
	if(pIndex[nextIndex] == i2){
		// Going clockwise
		return pIndex[(i1Index + 2)%6];
	}
	else{
		// Going counterclockwise
		return pIndex[(6 + i1Index - 2)%6];
	}
}

std::pair<int, int> Roadmap::locateHexagonForPoint(Point_2 &p0){
	// Compute the rectangular (col, row) 
	int col = (int)(floor((p0.get<0>() - xs)/(m_edgeLength*1.5)));
	int row = (int)(floor((p0.get<1>() - ys)/(m_edgeLength*sqrt3)));

	// Shift everything to the "origin"
	double dx = p0.get<0>() - (xs + m_edgeLength*1.5*col);
	double dy = p0.get<1>() - (ys + m_edgeLength*sqrt3*row);

	// For each col/row combo, which correspond to a rectangular region, there can be
	// three hexagons corrsponding to a point in that rectangle
	if(col%2 == 0){
		if(dx <= 0.5*m_edgeLength && dy <= sqrt3*m_edgeLength  - sqrt3*dx && dy >= sqrt3*dx){
			// We need to move left
			col --;
			// row ++;
		}
		else if(dy >= m_edgeLength*0.5*sqrt3){
			// Off by one row
			row ++;
		}
		else{
			// Already in the right place, do nothing
		}
	}
	else{
		if(dy >= sqrt3*0.5*m_edgeLength + sqrt3*dx){
			col--;
			row++;
		}
		else if(dy <= sqrt3*0.5*m_edgeLength - sqrt3*dx){
			col--;
		}
		else{
			// Already in the right place, do nothing
		}
	}
	return std::pair<int, int>(col, row);
}

int Roadmap::pointBelongToEdgeOutideObstacle(Polygon_2 & poly, int pIndex, 
	std::set<std::pair<int,int> > &tempEdgeToRemoveSet, bool outerBoundary){
	// Grab the point and check whether it is in the c-space 
	if(pointOutsideObstacle(poly, pIndex, outerBoundary)){
		// The point is not "in" the obstacle, get edges and check that they are not 
		// intersecting with the boundary. Because pIndex is outside obstacle, as long
		// as one edge from pIndex is not intersecting boundary, the edge must be outside 
		// the boundary
		std::set<int>& nbrSet = m_graph.getNeighborSet(pIndex);
		for(std::set<int>::iterator nit = nbrSet.begin(); nit != nbrSet.end(); nit++){
			int et = *nit;
			if(!edgeInSet(pIndex, et, tempEdgeToRemoveSet)){
				return et;
			}
		}
	}
	return -1;
}

bool Roadmap::pointOutsideObstacle(Polygon_2 & poly, int pIndex, bool outerBoundary){
	Point_2 p = m_vidPointMap[pIndex];
	bool boundedStatus = bg::within(p, poly);
	if((boundedStatus && outerBoundary) ||
		( !boundedStatus && !outerBoundary)){
		return true;
	}
	return false;
}

void Roadmap::populateHexagon(int col, int row, Point_2* p, Segment_2* e, Polygon_2 &hex, int* pIndex){
	hex.clear();
	if(col%2 == 0){
		pIndex[0] = (col + row*n_w)*2;
		pIndex[1] = (col + row*n_w)*2 + 1;
		pIndex[2] = (col + 1 + row*n_w)*2 + 1;
		pIndex[3] = (col + 1 + row*n_w)*2;
		pIndex[4] = (col + 1 + (row-1)*n_w)*2 + 1;
		pIndex[5] = (col + (row-1)*n_w)*2 + 1;
	}
	else{
		pIndex[0] = (col + row*n_w)*2 + 1;
		pIndex[1] = (col + (row+1)*n_w)*2;
		pIndex[2] = (col + 1 + (row+1)*n_w)*2;
		pIndex[3] = (col + 1 + row*n_w)*2 + 1;
		pIndex[4] = (col + 1 + row*n_w)*2;
		pIndex[5] = (col + row*n_w)*2;
	}
	// Then we get all vertices
	for(int i = 0; i < 6; i ++){
		p[i] = m_vidPointMap[pIndex[i]];
	}

	// Then the edges and the hexgaon
	for(int i = 0; i < 6; i ++){
		e[i] = Segment_2(p[i], p[i == 5? 0 : i+1]);
		bg::append(hex.outer(), p[i]);
	}
	bg::append(hex.outer(), p[0]);
}

void Roadmap::buildHexgaonLattice(){

	// Start the lattice at bottomLeftX, bottomLeftY. 
	// Assume that there are n_w and n_h hexgons inside the rectangle, then we should have
	// (1/2)*sideLength + n_w*sideLength*3/2 <= width and width < (1/2)*sideLength + (n_w + 1)*sideLength*3/2 
	// (sqrt(3)/2)*sideLength + n_h*(sqrt(3)/2)*sideLength <= height and height < (sqrt(3)/2)*sideLength + (n_h + 1)*(sqrt(3)/2)*sideLength
	// From these we can compute n_w and n_h. We add a few to make sure that we cover everything. 


	// Compute lattice nodes
	for(int i = 0; i < n_w; i ++){
		for(int j = 0; j < n_h; j ++){
			// Build one vertical "wave" of vertices
			Point_2 v0(xs + i*m_edgeLength*1.5 + (i%2 == 0? 0 : m_edgeLength*0.5), 
				ys + j*m_edgeLength*sqrt3);
			Point_2 v1(xs + m_edgeLength/2 + i*m_edgeLength*1.5 + (i%2 == 0? 0 : - m_edgeLength*0.5), 
				ys + sqrt3*m_edgeLength/2 + j*m_edgeLength*sqrt3);

			m_vidPointMap[(i + j*n_w)*2] = v0;
			m_pointVidMap[v0] = (i + j*n_w)*2;
			m_vidPointMap[(i + j*n_w)*2 + 1] = v1;
			m_pointVidMap[v1] = (i + j*n_w)*2 + 1;

			m_pointList.push_back(v0);
			m_pointList.push_back(v1);
		}
	}


	// Build adjacency, for each vertex, check whether its three neighbors are present
	for(int id = 0; id < n_w*n_h*2; id ++){
		// Compute w, h
		int w = (id/2)%n_w;
		int h = (id/2)/n_w;
		bool odd = (id%2==1);

		// If odd is true, check the vertex above, which has index (w, h + 1, 0)
		if(odd){
			if(h + 1 < n_h){
				m_graph.addEdge(id, (w + (h + 1)*n_w)*2);
			}
		}
		else{
			m_graph.addEdge(id, (w + (h)*n_w)*2 + 1);
		}
		// If odd is true and w is even, check (w + 1, h, 1)
		if(odd && w%2 == 0){
			if(w + 1 < n_w){
				m_graph.addEdge(id, (w + 1 + h*n_w)*2 + 1);
			}
		}

		// If odd is false and w is odd, check (w + 1, h, 0)
		if(odd == false && w%2 == 1){
			if(w + 1 < n_w){
				m_graph.addEdge(id, (w + 1 + h*n_w)*2);
			}
		}
	}
	std::set<int> neighbor_set;
	cout<<"m_graph:"<<endl;
	for(auto i = m_graph.vSet.begin();i != m_graph.vSet.end();i++){
		neighbor_set = m_graph.getNeighborSet(*i);
		cout << *i<<": ";
		for(auto j = neighbor_set.begin();j!= neighbor_set.end();j++){
			cout << *j<<" ";
		}
		cout << endl;
	}
}

void Roadmap::buildRectGridLattice() {
	// Point_2 bottomLeft = (*m_pBoundingRect).outer()[0];
	// Point_2 topRight = (*m_pBoundingRect).outer()[2];

	// bottomLeftX = bottomLeft.get<0>();
	// bottomLeftY = bottomLeft.get<1>();
	// width = std::abs(topRight.get<0>() - bottomLeft.get<0>());
	// height = std::abs(topRight.get<1>() - bottomLeft.get<1>());
	//sqrt3 = std::sqrt(3.0);

	// Compute number of columns and rows
	int rect_n_w = (int)(ceil(width/m_edgeLength)) + 1;
	int rect_n_h = (int)(ceil(height/m_edgeLength)) + 1;

	// The lattice start x, y
	double rect_xs = bottomLeftX;
	double rect_ys = bottomLeftY;


	for(int i = 0; i < rect_n_w; i ++){
		for(int j = 0; j < rect_n_h; j ++){
			// Build one vertical "wave" of vertices
			Point_2 v0(rect_xs + i*m_edgeLength, 
				rect_ys + j*m_edgeLength);
			

			m_vidPointMap[i + j*rect_n_w] = v0;
			m_pointVidMap[v0] = i + j*rect_n_w;

			m_pointList.push_back(v0);
		}
	}

	for(int id = 0; id < rect_n_w*rect_n_h; id ++){
		// Compute w, h
		int w = (id)%rect_n_w;
		int h = (id)/rect_n_w;
		

		// If odd is true, check the vertex above, which has index (w, h + 1, 0)
		
		if(h + 1 < rect_n_h){
			m_graph.addEdge(id, id + rect_n_w);
		}
		if(w+1 < rect_n_w){
			m_graph.addEdge(id, id+1);
		}
		
		
		
		// If odd is true and w is even, check (w + 1, h, 1)
		// if(odd && w%2 == 0){
		// 	if(w + 1 < n_w){
		// 		m_graph.addEdge(id, (w + 1 + h*n_w)*2 + 1);
		// 	}
		// }

		// // If odd is false and w is odd, check (w + 1, h, 0)
		// if(odd == false && w%2 == 1){
		// 	if(w + 1 < n_w){
		// 		m_graph.addEdge(id, (w + 1 + h*n_w)*2);
		// 	}
		// }
	}
	std::set<int> neighbor_set;
	cout<<"m_graph:"<<endl;
	for(auto i = m_graph.vSet.begin();i != m_graph.vSet.end();i++){
		neighbor_set = m_graph.getNeighborSet(*i);
		cout << *i<<": ";
		for(auto j = neighbor_set.begin();j!= neighbor_set.end();j++){
			cout << *j<<" ";
		}
		cout << endl;
	}
}

bool Roadmap::optimalLossTest(){
	int test_nums = 1000;
	int total_id_num = m_graph.vSet.size();
	int start = 0;
	int goal = 0;
	double directDist = 0;
	double hexgonDist = 0;
	double hexgonRatio = 0;
	double directDist_total = 0;
	double hexgonDist_total = 0;
	Point_2 startPt(0, 0);
	Point_2 goalPt(0, 0);
	int bfsStep = 0;
	int failed_time = 0;
	for(int i = 0; i < test_nums; i++){
		start = rand() % total_id_num;
		goal = rand() % total_id_num;
		cout << "testing " << i << "case:" << start << " " << goal << endl;
		startPt = m_vidPointMap[start];
		goalPt = m_vidPointMap[goal];
		directDist = bg::distance(startPt, goalPt);
		bfsStep = bfsShortestPath(start, goal);
		if(bfsStep < 0){
			failed_time ++;
			continue;

		}
		hexgonDist = (bfsStep * m_edgeLength - m_edgeLength)*0.91+m_edgeLength;
		hexgonDist_total += hexgonDist;
		directDist_total += directDist;
		
	}
	hexgonRatio = hexgonDist_total / directDist_total;
	cout << "optimalLossTest: "<<hexgonRatio<<endl;
	cout<<"failed time:"<<failed_time<<endl;
	return true;
}

 

// int Roadmap::bfsShortestPath(int start, int goal){
// 	list<bfs_node*> queue;
// 	bfs_node* start_node = new bfs_node(start, NULL);
// 	queue.push_back(start_node);
// 	int explore = 0;
// 	bfs_node *explore_node=new bfs_node(0, NULL);
// 	std::set<int> neighbor_set;
// 	int level = 0;
// 	std::map<int, bool> isvisited;
// 	while(queue.size() > 0){
// 		explore_node = queue.front();
// 		queue.pop_front();
// 		isvisited[explore_node->id] = true;
// 		if(m_graph.hasVertex(explore_node->id)){
// 			neighbor_set = m_graph.getNeighborSet(explore_node->id);
// 			if (neighbor_set.size() == 3) {
// 				return -1;
// 			}
// 		}	
// 		for(auto i = neighbor_set.begin(); i != neighbor_set.end(); i++){
// 			if (isvisited.find(*i) == isvisited.end()) {
// 				bfs_node* temp = new bfs_node(*i, explore_node);
// 				if (*i == goal) {
// 					while (temp->previous != NULL) {
// 						level++;
// 						temp = temp->previous;
// 					}

// 					return level;
// 				}
// 				queue.push_back(temp);
// 			}
// 		}
// 	}
// 	return -1;

// }

int Roadmap::bfsShortestPath(int start, int goal){
	list<int> queue;
	queue.push_back(start);

	int explore = 0;
	std::set<int> neighbor_set;
	int level = 0;
	std::map<int, int> previous_map;
	std::set<int> isvisited;
	previous_map[start] = -1;
	isvisited.insert(start);
	while(queue.size() > 0){
		explore = queue.front();
		queue.pop_front();
		if(m_graph.hasVertex(explore)){
			neighbor_set = m_graph.getNeighborSet(explore);
			//if (neighbor_set.size() == 3) {
			//	return -1;
			//}
		}	
		for(auto i = neighbor_set.begin(); i != neighbor_set.end(); i++){
			if (isvisited.find(*i) == isvisited.end()) {
				previous_map[*i] = explore;
				isvisited.insert(*i);
				if (*i == goal) {
					cout<<"path:"<<goal<<",";
					int temp = previous_map[*i];
					cout << temp << ",";
					while (temp != -1) {
						level++;

						temp = previous_map[temp];
						cout<<temp<<",";
					}
					cout<<endl;
					cout<<"level:"<<level;
					cout<<"queue size:"<<queue.size()<<endl;
					queue.clear();
					return level;
				}
				//if(queue.size()>100000){

				//	return -1;
				//}
				queue.push_back(*i);
			}
		}
	}
	return -1;

}

bool Roadmap::isSegFromPoly(Segment_2 seg, Polygon_2 poly){
	bool flag = false;
	for(int i = 0; i < poly.outer().size(); i ++){
			if(i >= 0 && i < poly.outer().size()-1){
				Point_2 first = poly.outer()[i];
				Point_2 second = poly.outer()[i+1];
				first.set<0>(std::floor(first.get<0>()));
				first.set<1>(std::floor(first.get<1>()));
				second.set<0>(std::floor(second.get<0>()));
				second.set<1>(std::floor(second.get<1>()));
				
				// if (first.get<0>() == seg.first.get<0>() && first.get<1>() == seg.first.get<1>()) {
				// 	std::cout << "isSegFromPoly" << std::endl;
				// 	std::cout <<"poly:"<< first.get<0>() << "," << first.get<1>() << "|" << second.get<0>() << "," << second.get<1>() << std::endl;
				// 	std::cout<<"seg:"<< seg.first.get<0>() << "," << seg.first.get<1>() << "|" << seg.second.get<0>() << "," << seg.second.get<1>() << std::endl;
				// 	flag = true;
				// 	break;
				// }
				
				if(bg::equals(seg, Segment_2(first, second))){
					flag = true;
					break;
				}
			}else if(i == poly.outer().size()-1){
				Point_2 first = poly.outer()[i];
				Point_2 second = poly.outer()[0];
				first.set<0>(std::floor(first.get<0>()));
				first.set<1>(std::floor(first.get<1>()));
				second.set<0>(std::floor(second.get<0>()));
				second.set<1>(std::floor(second.get<1>()));

				// if (first.get<0>() == seg.first.get<0>() && first.get<1>() == seg.first.get<1>()) {
				// 	std::cout << "isSegFromPoly" << std::endl;
				// 	std::cout << "poly:" << first.get<0>() << "," << first.get<1>() << "|" << second.get<0>() << "," << second.get<1>() << std::endl;
				// 	std::cout << "seg:" << seg.first.get<0>() << "," << seg.first.get<1>() << "|" << seg.second.get<0>() << "," << seg.second.get<1>() << std::endl;
				// 	flag = true;
				// 	break;
				// }
				if(bg::equals(seg, Segment_2(first, second))){
					flag = true;
					break;
				}
			}
	}
	return flag;
}

std::vector<Segment_2> Roadmap::getSurroundingEdges(Polygon_2 obsit, int poly_num){
	std::vector<Segment_2> surEdgeList;
	QPen edgePen = QPen(Qt::red, 2.5, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);

	for (voronoi_diagram<double>::const_cell_iterator it = m_vd.cells().begin(); it != m_vd.cells().end(); ++it) {
    	const voronoi_diagram<double>::cell_type &cell = *it;

    	if(cell.contains_segment()){
    		std::size_t index = it->source_index() - mv_points.size();
    		point_data<int> p0 = low(mv_segments[index]);
    		point_data<int> p1 = high(mv_segments[index]);
    		if (it->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
      			//printf("Cell #%ud contains segment start point: (%d, %d).\n",
             		//index, p0.x(), p0.y());
    		} else if (it->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) {
      			//printf("Cell #%ud contains segment end point: (%d, %d).\n",
             	//	index, p0.x(), p0.y());
    		} else {

      			//printf("Cell #%ud contains a segment: ((%d, %d), (%d, %d)). \n",
             	//	index, p0.x(), p0.y(), p1.x(), p1.y());
      			Segment_2 seg = Segment_2(Point_2(p0.x(), p0.y()), Point_2(p1.x(), p1.y()));
      			if(isSegFromPoly(seg, obsit)){
      				std::cout<<"--------"<<std::endl;
      				const voronoi_diagram<double>::edge_type *edge = cell.incident_edge();
    				// This is convenient way to iterate edges around Voronoi cell.
    				do {

      					if(edge->is_primary()){
      						if(edge->is_linear()){
      							Point_2 p1 = Point_2(edge->vertex0()->x(), edge->vertex0()->y());
								Point_2 p2 = Point_2(edge->vertex1()->x(), edge->vertex1()->y());
      							Linestring_2 surLine;
      							Segment_2 surEdge(p1, p2);
      							bg::append(surLine, p1);
      							bg::append(surLine, p2);
      							if(!bg::intersects(surLine, obsit) && !bg::within(surLine, obsit)){
      								surEdgeList.push_back(surEdge);	
      							//	std::cout<<"add edge:"<<"("<<p1.get<0>()<<","<<p1.get<1>()<<"),("<<p2.get<0>()<<","<<p2.get<1>()<<")"<<std::endl;
									
										mm_scene->addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), edgePen);
									
      								
      							}
      						}else if(edge->is_curved()){
      							std::vector<point_data<double>> samples;
								point_data<double> vertex0(edge->vertex0()->x(), edge->vertex0()->y());
        						samples.push_back(vertex0);
        						point_data<double> vertex1(edge->vertex1()->x(), edge->vertex1()->y());
        						samples.push_back(vertex1);
								sample_curved_edge(*edge, &samples);
								//std::cout<<"number of discretized segments: "<<samples.size()<<std::endl;
								Point_2 p1, p2;
								for(int j = 0;j < samples.size()-1;j ++){
									p1 = Point_2(samples[j].x(), samples[j].y());
									p2 = Point_2(samples[j+1].x(), samples[j+1].y());
									Linestring_2 surLine;
      								Segment_2 surEdge(p1, p2);
      								bg::append(surLine, p1);
      								bg::append(surLine, p2);
									if(!bg::intersects(surLine, obsit) && !bg::within(surLine, obsit)){
      									surEdgeList.push_back(surEdge);	
      								//	std::cout<<"add edge:"<<"("<<p1.get<0>()<<","<<p1.get<1>()<<"),("<<p2.get<0>()<<","<<p2.get<1>()<<")"<<std::endl;
									    
										    mm_scene->addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), edgePen);

									    
      									
      								}
								}
      						}
      					}
        					
      					edge = edge->next();
    				} while (edge != cell.incident_edge());
      			}
    		}

    	}else{
    		bool turnonFlag = false;
    		const voronoi_diagram<double>::edge_type *edge = cell.incident_edge();
    				// This is convenient way to iterate edges around Voronoi cell.
    		do{
				if (edge->is_finite()) {
					Point_2 p1 = Point_2(edge->vertex0()->x(), edge->vertex0()->y());
					Point_2 p2 = Point_2(edge->vertex1()->x(), edge->vertex1()->y());
					// if (isPtFromPoly(p1, obsit)) {
					// 	turnonFlag = true;
					// 	break;
					// }
					// if (isPtFromPoly(p2, obsit)) {
					// 	turnonFlag = true;
					// 	break;
					// }

				if (bg::distance(p1, obsit)< 1.5) {
						turnonFlag = true;
						break;
				}
				if (bg::distance(p2, obsit)< 1.5) {
						turnonFlag = true;
						break;
				}
				}
				edge = edge->next();
    		} while (edge != cell.incident_edge());
    		if(turnonFlag == true){
    			edge = cell.incident_edge();
    			do {

      					if(edge->is_primary()){
      						if(edge->is_linear()){
      							Point_2 p1 = Point_2(edge->vertex0()->x(), edge->vertex0()->y());
								Point_2 p2 = Point_2(edge->vertex1()->x(), edge->vertex1()->y());
      							Linestring_2 surLine;
      							Segment_2 surEdge(p1, p2);
      							bg::append(surLine, p1);
      							bg::append(surLine, p2);
      							if(!bg::intersects(surLine, obsit) && !bg::within(surLine, obsit)){
      								surEdgeList.push_back(surEdge);	
      							//	std::cout<<"add edge:"<<"("<<p1.get<0>()<<","<<p1.get<1>()<<"),("<<p2.get<0>()<<","<<p2.get<1>()<<")"<<std::endl;
									//if(poly_num == 4){
										mm_scene->addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), edgePen);
									//}
      								
      							}
      						}else if(edge->is_curved()){
      							std::vector<point_data<double>> samples;
								point_data<double> vertex0(edge->vertex0()->x(), edge->vertex0()->y());
        						samples.push_back(vertex0);
        						point_data<double> vertex1(edge->vertex1()->x(), edge->vertex1()->y());
        						samples.push_back(vertex1);
								sample_curved_edge(*edge, &samples);
								//std::cout<<"number of discretized segments: "<<samples.size()<<std::endl;
								Point_2 p1, p2;
								for(int j = 0;j < samples.size()-1;j ++){
									p1 = Point_2(samples[j].x(), samples[j].y());
									p2 = Point_2(samples[j+1].x(), samples[j+1].y());
									Linestring_2 surLine;
      								Segment_2 surEdge(p1, p2);
      								bg::append(surLine, p1);
      								bg::append(surLine, p2);
									if(!bg::intersects(surLine, obsit) && !bg::within(surLine, obsit)){
      									surEdgeList.push_back(surEdge);	
      							//		std::cout<<"add edge:"<<"("<<p1.get<0>()<<","<<p1.get<1>()<<"),("<<p2.get<0>()<<","<<p2.get<1>()<<")"<<std::endl;
									  //  if(poly_num == 4){
										    mm_scene->addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), edgePen);

									    //}
      									
      								}
								}
      						}
      					}
        					
      					edge = edge->next();
    				} while (edge != cell.incident_edge());
    		}
    		

    	}


	}
	return surEdgeList;
}

bool Roadmap::isPtFromPoly(Point_2 pt, Polygon_2 poly){
	bool flag = false;
	for(int i = 0; i < poly.outer().size(); i ++){
				Point_2 testPt = poly.outer()[i];
				
				if (std::floor(testPt.get<0>()) == std::floor(pt.get<0>()) && std::floor(testPt.get<1>()) == std::floor(pt.get<1>())) {
					// std::cout << "isPtFromPoly" << std::endl;
					// std::cout <<"poly:"<< testPt.get<0>() << "," << testPt.get<1>()<< std::endl;
					// std::cout<<"pt:"<< pt.get<0>() << "," << pt.get<1>()<< std::endl;
					flag = true;
					break;
				}
				
				
			
	}
	return flag;
}

void Roadmap::drawVoronoiDiagram(QGraphicsScene& scene, bool drawVetex){
	m_vd.clear();
	QPen edgePen = QPen(Qt::green, 2.5, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
	QPen vertexPen = QPen(Qt::blue, 0.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	mm_scene = &scene;
	int x_1, y_1, x_2, y_2;
	for(Polygon2_list::iterator obsit = m_polyVoronoiList.begin(); obsit != m_polyVoronoiList.end(); obsit++){
		for(int i = 0; i < (*obsit).outer().size(); i ++){
			if(i >= 0 && i < (*obsit).outer().size()-1){
				Point_2 first = (*obsit).outer()[i];
				Point_2 second = (*obsit).outer()[i+1];
				x_1 = std::floor(first.get<0>());
				y_1 = std::floor(first.get<1>());
				x_2 = std::floor(second.get<0>());
				y_2 = std::floor(second.get<1>());
				//mv_points.push_back(Point_v(x_1, y_1));
				//mv_points.push_back(Point_v(x_2, y_2));
				point_data<int> lp(x_1, y_1);
				point_data<int> hp(x_2, y_2);
				mv_segments.push_back(segment_data<int>(lp, hp));
			}else if(i == (*obsit).outer().size()-1){
				Point_2 first = (*obsit).outer()[i];
				Point_2 second = (*obsit).outer()[0];
				x_1 = std::floor(first.get<0>());
				y_1 = std::floor(first.get<1>());
				x_2 = std::floor(second.get<0>());
				y_2 = std::floor(second.get<1>());
				//mv_points.push_back(Point_v(x_1, y_1));
				//mv_points.push_back(Point_v(x_2, y_2));
				point_data<int> lp(x_1, y_1);
				point_data<int> hp(x_2, y_2);
				mv_segments.push_back(segment_data<int>(lp, hp));
			}
			

		}
	}
	
 	construct_voronoi(mv_segments.begin(), mv_segments.end(),
                    &m_vd);

 	for(voronoi_diagram<double>::const_edge_iterator it = m_vd.edges().begin();it != m_vd.edges().end(); ++it) {
		if (it->is_primary() && it->is_finite()) {
			if (it->is_curved()) {
				std::vector<point_data<double>> samples;
				point_data<double> vertex0(it->vertex0()->x(), it->vertex0()->y());
        		samples.push_back(vertex0);
        		point_data<double> vertex1(it->vertex1()->x(), it->vertex1()->y());
        		samples.push_back(vertex1);
				sample_curved_edge(*it, &samples);
			//	std::cout<<"number of discretized segments: "<<samples.size()<<std::endl;
				Point_2 p1, p2;
				for(int j = 0;j < samples.size()-1;j ++){
					p1 = Point_2(samples[j].x(), samples[j].y());
					p2 = Point_2(samples[j+1].x(), samples[j+1].y());
					scene.addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), edgePen);
				}
				
			}else{ 


				Point_2 p1 = Point_2(it->vertex0()->x(), it->vertex0()->y());
				Point_2 p2 = Point_2(it->vertex1()->x(), it->vertex1()->y());
				scene.addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), edgePen);
			}
		}
    }
}

 void Roadmap::sample_curved_edge(const voronoi_diagram<double>::edge_type & edge, std::vector<point_data<double>>* sampled_edge) {
    double max_dist = 0.001 * width;
    point_data<int> point_curve = edge.cell()->contains_point() ?
        retrieve_point(*edge.cell()) :
        retrieve_point(*edge.twin()->cell());
    //std::cout<<"point:("<<point_curve.x()<<","<<point_curve.y()<<")"<<std::endl;
    segment_data<int> segment_curve = edge.cell()->contains_point() ?
        retrieve_segment(*edge.twin()->cell()) :
        retrieve_segment(*edge.cell());
    if(low(segment_curve).x() != high(segment_curve).x() || low(segment_curve).y() != high(segment_curve).y()){ 
    	
    //	std::cout<<"segment:"<<"("<<low(segment_curve).x()<<","<<low(segment_curve).y()<<") and ("<<high(segment_curve).x()<<","<<high(segment_curve).y()<<")"<<std::endl;
    	voronoi_visual_utils<double>::discretize<int, int, point_data, segment_data>(
        point_curve, segment_curve, max_dist, sampled_edge);
	}	
  }

 point_data<int> Roadmap::retrieve_point(const voronoi_diagram<double>::cell_type& cell) {
    voronoi_diagram<double>::cell_type::source_index_type index = cell.source_index();
    voronoi_diagram<double>::cell_type::source_category_type category = cell.source_category();
    if (category == SOURCE_CATEGORY_SINGLE_POINT) {
      return mv_points[index];
    }
    index -= mv_points.size();
//	std::cout << "index is: " << index << std::endl;
    if (category == SOURCE_CATEGORY_SEGMENT_START_POINT) {
		//std::cout << "line 4022 x:" << low(mv_segments[index]).x() << " y:" << low(mv_segments[index]).y() << std::endl;
      return low(mv_segments[index]);
    } else {
		//std::cout << "line 4025" << std::endl;
      return high(mv_segments[index]);
    }
  }

  segment_data<int> Roadmap::retrieve_segment(const voronoi_diagram<double>::cell_type& cell) {
    voronoi_diagram<double>::cell_type::source_index_type index = cell.source_index() - mv_points.size();
    return mv_segments[index];
  }

void Roadmap::drawHexagonLattice(QGraphicsScene& scene, bool drawVetex){
	QPen edgePen = QPen(Qt::gray, 1.25, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
	QPen vertexPen = QPen(Qt::blue, 0.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	std::set<std::pair<int, int> > edgeSet = m_graph.getEdgeSet();
	std::set<int> vSet;
	for(std::set<std::pair<int, int> >::iterator eit = edgeSet.begin(); eit != edgeSet.end(); eit++){
		int v1 = (*eit).first;
		int v2 = (*eit).second;
		vSet.insert(v1); vSet.insert(v2);
		Point_2 p1 = (m_vidPointMap[v1]);
		Point_2 p2 = (m_vidPointMap[v2]);
		scene.addLine(p1.get<0>(), p1.get<1>(), p2.get<0>(), p2.get<1>(), edgePen);
	}

	// Paint vertices if needed and obtain the fill area
	if(drawVetex){
		for(std::set<int>::iterator vit = vSet.begin(); vit != vSet.end(); vit ++){
			Point_2 p = (m_vidPointMap[*vit]);
			scene.addEllipse(p.get<0>() - 0.25, p.get<1>() - 0.25, 0.5, 0.5, vertexPen);
		}
	}
}

void Roadmap::snapToGraph(vector<pair<double, double> >&coords, vector<int>&snapped, vector<Point_2>& snappedPt){
	std::set<int> usedVertices;
	std::set<int> vSet;
	double minDist = -1;
	int bestV=-1;
	// For each member in coords, locate the hexagon it belongs and then locate the closest 
	// graph vertex to the said coordinate. 
	vSet = m_finalGraph.getVertexSet();


	for(int r = 0; r < coords.size(); r ++){
		double x = coords[r].first;
		double y = coords[r].second;
		Point_2 p0 = Point_2(x, y);
		Point_2 p0x = (p0);

		bestV = -1;
		minDist = -1;
		for(auto vi = vSet.begin(); vi != vSet.end(); ++vi){
		// 	// Only work with vertices inside the configuration space and not used
			if(usedVertices.find(*vi) == usedVertices.end()){
		 		if(minDist < 0){
		 			minDist = bg::distance(p0, m_vidPointMap[m_vidFGMap[*vi]]);
		 			bestV = *vi;
		 		}
		 		else{

					double dist = bg::distance(p0, m_vidPointMap[m_vidFGMap[*vi]]);
		 			if(minDist > dist){
		 				minDist = dist;
		 				bestV = *vi;

		 			}
		 		}
		 	}
		}
		




	


		// Snap!
		if(bestV != -1){
#ifdef _DEBUG
			cout << "Snapping point (" << x << ", " << y << ") to vertex " << bestV << endl;
#endif
			snapped.push_back(bestV);
			snappedPt.push_back(m_vidPointMap[m_vidFGMap[bestV]]);
			usedVertices.insert(bestV);
		}
		else{
			throw "ERROR: Cannot find suitable vertex for snapping";
		}
	}

}
void Roadmap::createRandomGoals(int numRobots, double spacing, vector<pair<double, double>> & goals){
	createRandomCoords(numRobots, spacing, goals);
}
void Roadmap::createRandomStartGoalPairs(int numRobots, double spacing,  vector<pair<double, double> >& starts, 
	vector<pair<double, double> >& goals){

	createRandomCoords(numRobots, spacing, starts);
	createRandomCoords(numRobots, spacing, goals);
}

void Roadmap::createRandomStartGoalPairsAtRegion(int numRobots, double spacing, vector<pair<double, double>> & starts, 
	vector<pair<double,double>> & goals, double left, double right, double up, double bottom, double g_left, double g_right, double g_up, double g_bottom){
	int a, b;
	a = numRobots / 2;
	b = numRobots - a;
	createRandomCoordsAtRegion(a, spacing, starts, left, right, up, bottom);
	createRandomCoordsAtRegion(a, spacing, goals, g_left, g_right, g_up, g_bottom);
	createRandomCoordsAtRegion(b, spacing, goals, left, right, up, bottom);
	createRandomCoordsAtRegion(b, spacing, starts, g_left, g_right, g_up, g_bottom);

}

void Roadmap::createRandomCoordsAtRegion(int numRobots, double spacing, vector<pair<double, double>> & coords, double left, double right, double up, double bottom){
// Uniform sample from the free space and pick 5 start/goal pairs
	std::vector<Point_2> bbox = m_pBoundingRect->outer();
	Point_2 top_left, bottom_right;
	double xmax, xmin, ymax, ymin;
	int added_num = 0;
	xmin = std::numeric_limits<double>::max();
	ymin = std::numeric_limits<double>::max();
	xmax = std::numeric_limits<double>::min();
	ymax = std::numeric_limits<double>::min();
	for(int i = 0; i < bbox.size(); i ++){
		if(xmax < bbox[i].get<0>()){
			xmax = bbox[i].get<0>();
		}
		if(xmin > bbox[i].get<0>()){
			xmin = bbox[i].get<0>();
		}
		if(ymax < bbox[i].get<1>()){
			ymax = bbox[i].get<1>();
		}
		if(ymin > bbox[i].get<1>()){
			ymin = bbox[i].get<1>();
		}
	}
	xmin = xmin + left*(xmax - xmin);
	xmax = xmin + right*(xmax - xmin);
	ymin = ymin + bottom*(ymax - ymin);
	ymax = ymin + up*(ymax - ymin);
	while(true){

		long trialCount = 0;
		for(int i = 0; i < numRobots; i ++){
			while(trialCount < numRobots*30000){
				trialCount ++;
#ifdef _DEBUG
				if(trialCount %1000 == 0) {cout << trialCount << endl;}
#endif



				double x = (rand()%10000)/10000.*(xmax - xmin)  + xmin;
				double y = (rand()%10000)/10000.*(ymax - ymin)  + ymin;
#ifdef _DEBUG
				cout << "Randomly created point with x=" << x <<", y=" << y;
#endif
				Point_2 p(x, y);
				// Check that the point is in free configuration space
				if(isPointInCSpace(p)){
					bool good = true;
					// Check that the point has good distance from existing points
					for(int vi = 0; vi < coords.size(); vi ++){
						if(getDistance(p, coords[vi].first, coords[vi].second) < (spacing > 2 ? spacing : 2)*m_radius){
							good = false;
						}
					}

					if(good){
						// No problem! We have a good set
						coords.push_back(pair<double, double>(x, y));
						added_num++;
#ifdef _DEBUG
						cout << " - added to set for robot " << i << endl;
#endif
						break;
					}
					else{
#ifdef _DEBUG
						cout << endl; 
#endif
					}

				}
				else{
#ifdef _DEBUG
					cout << endl; 
#endif
				}
			}
			if(trialCount >= numRobots*30000){
				// cout << coords.size() << endl;
				coords.clear();
				break;
			}
		}
		if(added_num== numRobots)return;
	}
}

void Roadmap::createRandomCoords(int numRobots, double spacing, vector<pair<double, double> >& coords){
	// Uniform sample from the free space and pick 5 start/goal pairs
	std::vector<Point_2> bbox = m_pBoundingRect->outer();
	Point_2 top_left, bottom_right;
	double xmax, xmin, ymax, ymin;
	xmin = std::numeric_limits<double>::max();
	ymin = std::numeric_limits<double>::max();
	xmax = std::numeric_limits<double>::min();
	ymax = std::numeric_limits<double>::min();
	for(int i = 0; i < bbox.size(); i ++){
		if(xmax < bbox[i].get<0>()){
			xmax = bbox[i].get<0>();
		}
		if(xmin > bbox[i].get<0>()){
			xmin = bbox[i].get<0>();
		}
		if(ymax < bbox[i].get<1>()){
			ymax = bbox[i].get<1>();
		}
		if(ymin > bbox[i].get<1>()){
			ymin = bbox[i].get<1>();
		}
	}

	while(true){

		long trialCount = 0;
		for(int i = 0; i < numRobots; i ++){
			while(trialCount < numRobots*30000){
				trialCount ++;
#ifdef _DEBUG
				if(trialCount %1000 == 0) {cout << trialCount << endl;}
#endif



				double x = (rand()%10000)/10000.*(xmax - xmin)  + xmin;
				double y = (rand()%10000)/10000.*(ymax - ymin)  + ymin;
#ifdef _DEBUG
				cout << "Randomly created point with x=" << x <<", y=" << y;
#endif
				Point_2 p(x, y);
				// Check that the point is in free configuration space
				if(isPointInCSpace(p)){
					bool good = true;
					// Check that the point has good distance from existing points
					for(int vi = 0; vi < coords.size(); vi ++){
						if(getDistance(p, coords[vi].first, coords[vi].second) < (spacing > 2 ? spacing : 2)*m_radius){
							good = false;
						}
					}

					if(good){
						// No problem! We have a good set
						coords.push_back(pair<double, double>(x, y));
#ifdef _DEBUG
						cout << " - added to set for robot " << i << endl;
#endif
						break;
					}
					else{
#ifdef _DEBUG
						cout << endl; 
#endif
					}

				}
				else{
#ifdef _DEBUG
					cout << endl; 
#endif
				}
			}
			if(trialCount >= numRobots*30000){
				// cout << coords.size() << endl;
				coords.clear();
				break;
			}
		}
		if(coords.size() == numRobots)return;
	}
}

bool Roadmap::generateTestfile(vector<pair<int, int> >& sgVec, map<int, vector<pair<double, double>> >& out_paths,
	string& fileFolder, string& fileNameExtra, int i, int number_robot, double snapLength, double visibilityLength){
	map<int, vector<int>> paths;
	// Solve the problem by calling external java solver
	buildComputeMap();
	for(int r = 0; r < sgVec.size(); r ++){
		
		if(m_vidFCMap[sgVec[r].first]==0 || m_vidFCMap[sgVec[r].second] == 0)
			return false;
	}
	// =====================================================================================
	// First write the problem 
	string pfString = fileFolder + "\\p" + fileNameExtra + "_"+ std::to_string(static_cast<long long>(number_robot)) + "_" + std::to_string(static_cast<long long>(i)) + ".txt";
	string sfString = fileFolder + "\\s" + fileNameExtra + "_"+ std::to_string(static_cast<long long>(number_robot)) + "_" + std::to_string(static_cast<long long>(i)) + ".txt";
	cout<<"generating "<<fileNameExtra<<":"<<i<<endl;
	ofstream ps(pfString.c_str(), ios::out);

	// Write the the number of robots
	ps << m_computeGraph.getVertexSet().size() << endl;
	std::cout<<m_computeGraph.getVertexSet().size()<<std::endl;
	for(int k = 0;k < m_newAddedPt.size();k++){
		ps << m_vidFCMap[m_newAddedPt[k]] << " ";
	}
	ps << endl;
	m_newAddedPt.clear();
	// Write out all the edges
	std::set<pair<int, int> >& edgeSet = m_computeGraph.getEdgeSet();
	for(std::set<pair<int, int> >::iterator ei = edgeSet.begin(); ei != edgeSet.end(); ei ++){
		ps << ei->first << ":" << ei->second << " ";
		std::cout<<ei->first << ":" << ei->second << " ";
	}
	ps << endl;


	// Write out the starts and then the goals
	for(int r = 0; r < sgVec.size(); r ++){
		
		ps << m_vidFCMap[sgVec[r].first] << " ";

		cout<<sgVec[r].first << " ";
	}
	
	ps << endl;
	cout<<endl;
	for(int r = 0; r < sgVec.size(); r ++){
		ps << m_vidFCMap[sgVec[r].second] << " ";
		cout<<sgVec[r].second << " ";
	}

	ps << endl;
	ps << snapLength;
	ps << endl;
	ps << visibilityLength;
	ps << endl;
	

#ifdef _ADD_SPECIAL_CONS
	for(int k = 0; k < m_specialPair.size(); k++){
		std::pair<std::pair<int, int>, int> pai = m_specialPair[k];
		ps << m_vidFCMap[pai.first.first] << ":" << m_vidFCMap[pai.second] << ":" << m_vidFCMap[pai.first.second] << " ";
	}
	ps << endl;
#endif
	ps.close();
	return true;
}

bool Roadmap::solveGeneratedProblem(map<int, vector<pair<double, double>> >& out_paths,
	string& fileFolder, string& fileNameExtra, int i, int number_robot, double & snapLength, double & visibilityLength, int & shortcut_times, int & smooth_nums){
	int shortcut = 0;
	map<int, vector<int>> paths;
	vector<int> added_list;
	//buildComputeMap();
	string pfString = fileFolder + "\\p" + fileNameExtra + "_"+ std::to_string(static_cast<long long>(number_robot)) + "_" + std::to_string(static_cast<long long>(i)) + ".txt";
	string sfString = fileFolder + "\\s" + fileNameExtra + "_"+ std::to_string(static_cast<long long>(number_robot)) + "_" + std::to_string(static_cast<long long>(i)) + ".txt";
	// Solve the problem by calling external java solver
	// Make system call 

	std::cout<<pfString;
	//string callStr = "java -cp gurobi.jar;test_mp.jar Main 12";		
	string callStr = "java -cp gurobi.jar;split_1.jar projects.multipath.general.algorithms.Main";


	if(fileNameExtra.length() > 0) {
		callStr.append(" ").append(pfString).append(" ").append(sfString);
	}
	int j = system(callStr.c_str());
	ifstream ps(pfString.c_str(), ios::in);
	string new_added_line, num_line; 
	getline(ps, num_line);
	getline(ps, new_added_line);      
	QString q_new_added_line(new_added_line.c_str());
	QStringList q_new_list = q_new_added_line.split(" ");
	for(int i = 0; i < q_new_list.size(); i ++){
		if(q_new_list[i].toInt()!= 0){ 
			added_list.push_back(q_new_list[i].toInt());
		}
	}
	ifstream pre_ss(sfString.c_str(), ios::in);
	string pre_line; 
	getline(pre_ss, pre_line);      
	QString pre_s(pre_line.c_str());
	QStringList pre_qsl = pre_s.split(" ");
	int path_length = pre_qsl.size();
	std::vector<bool> smooth_list(path_length);
	for(int i = 0;i < smooth_list.size(); i++){
		smooth_list[i] = true;
	}
	smooth_nums = 0;
	ifstream ss(sfString.c_str(), ios::in);
	if(ss.good()){
		for(int r = 0; r < number_robot; r++){
			string line; 
			getline(ss, line);      
			QString s(line.c_str());
			QStringList qsl = s.split(" ");
			paths[r] = vector<int>();
			out_paths[r] = vector<pair<double, double>>();
			for(int t = 0; t < qsl.size(); t ++){
				paths[r].push_back(m_vidCFMap[qsl[t].toInt()]);
				for(int k = 0; k < added_list.size(); k++){
					if(qsl[t].toInt() == added_list[k]){
						smooth_list[t] = false;
						break;
					}
				}
				//out_paths[r].push_back(getVertexLocationFromID(m_vidCFMap[qsl[t].toInt()]));
			}
			paths[r].pop_back();
			//out_paths[r].pop_back();
		}
		for(int i = 0;i < smooth_list.size();i++){
			if(smooth_list[i] == true){
				smooth_nums ++;
			}
		}
		string snapLine; 
		getline(ss, snapLine);
		string visibilityLine;
		getline(ss, visibilityLine);
		snapLength = std::stod(snapLine);
		visibilityLength = std::stod(visibilityLine);
		
		ss.close();
	}
	else{
		ss.close();
		return false;
	}

	

	// Solve the problem locally
	// ILPSolver solver(&m_finalGraph);
	// solver.solve(sgVec, paths, -1);


	improvePaths(paths, out_paths, shortcut);
	shortcut_times = shortcut;
	return true;
}

bool Roadmap::solveProblem(vector<pair<int, int> >& sgVec, map<int, vector<pair<double, double>> >& out_paths,
	string& fileFolder, string& fileNameExtra){
	map<int, vector<int>> paths;
	int shortcut_times = 0;
	// Solve the problem by calling external java solver
	buildComputeMap();
	map<int, vector<pair<double, double>> > generate_paths;
	// =====================================================================================
	// First write the problem 
	string pfString = fileFolder + "\\p" + fileNameExtra + ".txt";
	string sfString = fileFolder + "\\s" + fileNameExtra + ".txt";
	string pathString = fileFolder + "\\path_" + fileNameExtra + ".txt";
	string dpfString = fileFolder + "\\dp" + fileNameExtra + ".txt";
	string dsfString = fileFolder + "\\ds" + fileNameExtra + ".txt";
	ofstream ps(pfString.c_str(), ios::out);

	// Write the the number of robots
	ps << m_computeGraph.getVertexSet().size() << endl;
	std::cout<<m_computeGraph.getVertexSet().size()<<std::endl;
	// Write out all the edges
	ps << endl;
	std::set<pair<int, int> >& edgeSet = m_computeGraph.getEdgeSet();
	for(std::set<pair<int, int> >::iterator ei = edgeSet.begin(); ei != edgeSet.end(); ei ++){
		ps << ei->first << ":" << ei->second << " ";
		std::cout<<ei->first << ":" << ei->second << " ";
	}
	ps << endl;


	// Write out the starts and then the goals
	for(int r = 0; r < sgVec.size(); r ++){
		ps << m_vidFCMap[sgVec[r].first] << " ";
	}
	ps << endl;
	for(int r = 0; r < sgVec.size(); r ++){
		ps << m_vidFCMap[sgVec[r].second] << " ";
	}
	ps << endl;
#ifdef _ADD_SPECIAL_CONS
	for(int k = 0; k < m_specialPair.size(); k++){
		std::pair<std::pair<int, int>, int> pai = m_specialPair[k];
		ps << m_vidFCMap[pai.first.first] << ":" << m_vidFCMap[pai.second] << ":" << m_vidFCMap[pai.first.second] << " ";
	}
	ps << endl;
#endif
	ps << 0 << endl;
	ps<< 0 << endl;
	ps.close();

	ofstream dps(dpfString.c_str(), ios::out);

	// Write the the number of robots
	dps << m_finalGraph.getVertexSet().size() << endl;
	//std::cout<<m_coGraph.getVertexSet().size()<<std::endl;
	// Write out all the edges
	edgeSet = m_finalGraph.getEdgeSet();
	for(std::set<pair<int, int> >::iterator ei = edgeSet.begin(); ei != edgeSet.end(); ei ++){
		dps << ei->first << ":" << ei->second << " ";
		//std::cout<<ei->first << ":" << ei->second << " ";
	}
	dps << endl;

	// Write out the starts and then the goals
	for(int r = 0; r < sgVec.size(); r ++){
		dps << sgVec[r].first << " ";
	}
	dps << endl;
	for(int r = 0; r < sgVec.size(); r ++){
		dps << sgVec[r].second << " ";
	}
	dps << endl;
	dps.close();	

	// =====================================================================================
	// Make system call 
	//string callStr = "java -cp gurobi.jar;test_mp.jar Main 12";
	string callStr = "java -cp gurobi.jar;old_mp_fixed.jar projects.multipath.general.algorithms.Main";

	if(fileNameExtra.length() > 0) {
		callStr.append(" ").append(pfString).append(" ").append(sfString);
	}
	int i = system(callStr.c_str());

	cout<< "output path:---------------- "<< pathString <<endl;
	ofstream pts(pathString.c_str(), ios::out);
	// =====================================================================================
	// Read in the solution, there should be sgVec.size() robots
	ifstream ss(sfString.c_str(), ios::in);
	if(ss.good()){
		for(int r = 0; r < sgVec.size(); r++){
			string line; 
			getline(ss, line);      
			QString s(line.c_str());
			QStringList qsl = s.split(" ");
			paths[r] = vector<int>();
			out_paths[r] = vector<pair<double, double>>();
			for(int t = 0; t < qsl.size(); t ++){
				paths[r].push_back(m_vidCFMap[qsl[t].toInt()]);
				generate_paths[r].push_back(getVertexLocationFromID(m_vidCFMap[qsl[t].toInt()]));
			}
			paths[r].pop_back();
			generate_paths[r].pop_back();
		}
		ss.close();
	}
	else{
		ss.close();
		return false;
	}

	ofstream dss(dsfString.c_str(), ios::out);
	for(int r = 0;r < sgVec.size();r++){
		for(int i = 0; i < paths[r].size();i++){
			dss << paths[r][i] << " ";
		}
		dss << endl;
		

	}
	dss.close();
	for(int r = 0; r < sgVec.size(); r++){
		for(int i = 0;i < generate_paths[r].size();i++){
			pts << generate_paths[r][i].first << " "<< generate_paths[r][i].second<<" ";
		}
		pts<< endl;
	}
	// Solve the problem locally
	// ILPSolver solver(&m_finalGraph);
	// solver.solve(sgVec, paths, -1);


	improvePaths(paths, out_paths, shortcut_times);
	// std::cout<<"paths after smoothing"<<endl;
	// for(int r = 0;r < out_paths.size();r++){
	// 	std::cout<<r<<" robot path"<<endl;
	// 	for(int i = 0; i < out_paths[r].size();i++){
	// 			std::cout<<out_paths[r][i].first<<","<<out_paths[r][i].second<<" ";

	// 	}
	// 	std::cout<<std::endl;
	// }
//transform the paths[] here, 

	// for(int r = 0; r < sgVec.size(); r++){
		
	// 	std::vector<int> addedPointList;
	// 	for(int i = 0; i < paths[r].size()-1;i++){

	// 		if(paths[r][i] == paths[r][i+1]){
	// 			if(i == 0){
	// 				addedPointList.push_back(m_vidFGMap[paths[r][i]]);
	// 				addedPointList.push_back(m_vidFGMap[paths[r][i+1]]);
	// 			}else{
	// 				addedPointList.push_back(m_vidFGMap[paths[r][i+1]]);
	// 			}
				
	// 		}else{
	// 			std::pair<int, int> seg(paths[r][i], paths[r][i+1]);

	// 			std::vector<int> seg_list = m_edgeFGMap[seg];
	// 			if(i == 0){
	// 				for(int j = 0; j < seg_list.size(); j++){
	// 					addedPointList.push_back(seg_list[j]);
	// 				}
	// 			}else{
	// 				for(int j = 1; j < seg_list.size(); j++){
	// 					addedPointList.push_back(seg_list[j]);
	// 				}
	// 			}
	// 		}
			
			
	// 	// replace seg_list with paths[r][i] and paths[r][i+1]
	// 	}
	// 	paths[r] = addedPointList;
	// }

	
	

	// then paths[r] is ready to send to the animation process. need to build the m_vidFGMap 
	// 
	return true;
}

// double Roadmap::getEdgeLength(){
// 	return m_edgeLength;
// }


std::vector<std::pair<double, double>> Roadmap::getVertexListLocationFromID(int pathStep, pair<double, double> start, pair<double, double> end, pair<double, double> before, pair<double, double> after, int mode){
			std::vector<int> addedPointList;
			std::vector<std::pair<double, double>> pointList;
			Point_2 startPt(0, 0); 
			Point_2 endPt(0, 0);
			Point_2 beforePt(0, 0);
			Point_2 afterPt(0, 0);
			endPt.set<1>(end.second);
			startPt.set<0>(start.first);
			startPt.set<1>(start.second);
			endPt.set<0>(end.first);
			beforePt.set<0>(before.first);
			beforePt.set<1>(before.second);
			afterPt.set<0>(after.first);
			afterPt.set<1>(after.second);
		//	cout << "startPt:" << startPt.get<0>() << "," << startPt.get<1>() << endl;
		//	cout << "endPt:" << endPt.get<0>() << "," << endPt.get<1>() << endl;
			if((m_pointVidMap.find(startPt) != m_pointVidMap.end()) && (m_pointVidMap.find(endPt) != m_pointVidMap.end())){
		//		cout << "normal grid" <<startPt.get<0>()<<","<<startPt.get<1>()<< ","<<endPt.get<0>()<<","<<endPt.get<1>()<< endl;
				int startVid = m_vidGFMap[m_pointVidMap[startPt]];
				int endVid = m_vidGFMap[m_pointVidMap[endPt]];
				int beforeVid, afterVid;
				if(m_pointVidMap.find(beforePt) != m_pointVidMap.end()){
					beforeVid = m_vidGFMap[m_pointVidMap[beforePt]];
				}else{
					beforeVid = -1;
				}
				if(m_pointVidMap.find(afterPt) != m_pointVidMap.end()){
					afterVid = m_vidGFMap[m_pointVidMap[afterPt]];
				}else{
					afterVid = -1;
				}
				if(isTestPtInNormalGrid(startVid) && isTestPtInNormalGrid(endVid)){
					if(mode == 0){
						if(startVid == endVid){
							std::pair<double, double> currentPoint;
							currentPoint.first = m_vidPointMap[m_vidFGMap[startVid]].get<0>();
							currentPoint.second = m_vidPointMap[m_vidFGMap[startVid]].get<1>();
							pointList.push_back(currentPoint);
							currentPoint.first = m_vidPointMap[m_vidFGMap[endVid]].get<0>();
							currentPoint.second = m_vidPointMap[m_vidFGMap[endVid]].get<1>();
							pointList.push_back(currentPoint);
						}else{
							std::pair<double, double> currentPoint;
							currentPoint.first = m_vidPointMap[m_vidFGMap[startVid]].get<0>();
							currentPoint.second = m_vidPointMap[m_vidFGMap[startVid]].get<1>();
							pointList.push_back(currentPoint);
							if(isTestPtInNormalGrid(afterVid)){
								currentPoint.first = (start.first + end.first)/2;
								currentPoint.second = (start.second + end.second)/2;
								pointList.push_back(currentPoint);
								currentPoint = getCurveMidPoint(m_vidPointMap[m_vidFGMap[startVid]], m_vidPointMap[m_vidFGMap[endVid]], m_vidPointMap[m_vidFGMap[afterVid]]);
								pointList.push_back(currentPoint);
							}else{
								currentPoint.first = m_vidPointMap[m_vidFGMap[endVid]].get<0>();
								currentPoint.second = m_vidPointMap[m_vidFGMap[endVid]].get<1>();
								pointList.push_back(currentPoint);
							}
						}
					}else if(mode == 1){
						if(startVid == endVid){
							std::pair<double, double> currentPoint;
							currentPoint.first = m_vidPointMap[m_vidFGMap[startVid]].get<0>();
							currentPoint.second = m_vidPointMap[m_vidFGMap[startVid]].get<1>();
							pointList.push_back(currentPoint);
							currentPoint.first = m_vidPointMap[m_vidFGMap[endVid]].get<0>();
							currentPoint.second = m_vidPointMap[m_vidFGMap[endVid]].get<1>();
							pointList.push_back(currentPoint);
						}else{
							std::pair<double, double> currentPoint;
							if(isTestPtInNormalGrid(beforeVid)){
								currentPoint = getCurveMidPoint(m_vidPointMap[m_vidFGMap[beforeVid]], m_vidPointMap[m_vidFGMap[startVid]], m_vidPointMap[m_vidFGMap[endVid]]);
								pointList.push_back(currentPoint);
								currentPoint.first = (start.first + end.first)/2;
								currentPoint.second = (start.second + end.second)/2;
								pointList.push_back(currentPoint);
								
							}else{
								currentPoint.first = m_vidPointMap[m_vidFGMap[startVid]].get<0>();
								currentPoint.second = m_vidPointMap[m_vidFGMap[startVid]].get<1>();
								pointList.push_back(currentPoint);
							}
							currentPoint.first = m_vidPointMap[m_vidFGMap[endVid]].get<0>();
							currentPoint.second = m_vidPointMap[m_vidFGMap[endVid]].get<1>();
							pointList.push_back(currentPoint);
							
						}
					}else if(mode == 2){
						if(startVid == endVid){
							std::pair<double, double> currentPoint;
							if(isTestPtInNormalGrid(afterVid) && isTestPtInNormalGrid(beforeVid)){
								currentPoint = getCurveMidPoint(m_vidPointMap[m_vidFGMap[beforeVid]], m_vidPointMap[m_vidFGMap[endVid]], m_vidPointMap[m_vidFGMap[afterVid]]);
								pointList.push_back(currentPoint);
								pointList.push_back(currentPoint);
							}else{
								currentPoint.first = m_vidPointMap[m_vidFGMap[endVid]].get<0>();
								currentPoint.second = m_vidPointMap[m_vidFGMap[endVid]].get<1>();
								pointList.push_back(currentPoint);
								pointList.push_back(currentPoint);
							}
						
						}else{
							std::pair<double, double> currentPoint;
							if(isTestPtInNormalGrid(beforeVid)){
								currentPoint = getCurveMidPoint(m_vidPointMap[m_vidFGMap[beforeVid]], m_vidPointMap[m_vidFGMap[startVid]], m_vidPointMap[m_vidFGMap[endVid]]);
								pointList.push_back(currentPoint);

							}else{
								currentPoint.first = m_vidPointMap[m_vidFGMap[startVid]].get<0>();
								currentPoint.second = m_vidPointMap[m_vidFGMap[startVid]].get<1>();
								pointList.push_back(currentPoint);
							}
							currentPoint.first = (start.first + end.first)/2;
							currentPoint.second = (start.second + end.second)/2;
							pointList.push_back(currentPoint);
							if(afterVid == endVid){
								currentPoint.first = m_vidPointMap[m_vidFGMap[endVid]].get<0>();
								currentPoint.second = m_vidPointMap[m_vidFGMap[endVid]].get<1>();
								pointList.push_back(currentPoint);	
							}else{
								if(isTestPtInNormalGrid(afterVid)){
									currentPoint = getCurveMidPoint(m_vidPointMap[m_vidFGMap[startVid]], m_vidPointMap[m_vidFGMap[endVid]], m_vidPointMap[m_vidFGMap[afterVid]]);
									pointList.push_back(currentPoint);

								}else{
									currentPoint.first = m_vidPointMap[m_vidFGMap[endVid]].get<0>();
									currentPoint.second = m_vidPointMap[m_vidFGMap[endVid]].get<1>();
									pointList.push_back(currentPoint);
								}
							}
						}
					}// end of mode 2
				}else{ 


		//		cout << "m_pointVidMap[endPt]=" << m_pointVidMap[endPt] << ",endVid=" << endVid << endl;
		//		cout << "m_pointVidMap[startPt]=" << m_pointVidMap[startPt] << ",startVid=" << startVid << endl;
					if(startVid == endVid){
						addedPointList.push_back(m_vidFGMap[startVid]);
						addedPointList.push_back(m_vidFGMap[endVid]);
					}else{ 
						std::pair<int, int> seg(startVid, endVid);

						std::vector<int> seg_list = m_edgeFGMap[seg];
						if(seg_list.size() == 0){
							addedPointList.push_back(m_vidFGMap[startVid]);
							addedPointList.push_back(m_vidFGMap[endVid]);
						}else{ 
							for(int j = 0; j < seg_list.size(); j++){
								addedPointList.push_back(seg_list[j]);
							}
						}
					}
				
					for(int i = 0; i < addedPointList.size(); i++){
						std::pair<double, double> currentPoint;
						currentPoint.first = m_vidPointMap[addedPointList[i]].get<0>();
						currentPoint.second = m_vidPointMap[addedPointList[i]].get<1>();
						pointList.push_back(currentPoint);
					}
				}
			}else{
				cout << "shortout path" << endl;
				pointList.push_back(start);
				pointList.push_back(end);
			}
			if (pointList.size() == 0) {
		//		cout << "no output" << endl;
			}

			// std::vector<int> addedPointList;
			// if(start == end){
			// 	addedPointList.push_back(m_vidFGMap[start]);
			// 	addedPointList.push_back(m_vidFGMap[end]);
			// }else{
			// 	std::pair<int, int> seg(start, end);

			// 	std::vector<int> seg_list = m_edgeFGMap[seg];
			// 	for(int j = 0; j < seg_list.size(); j++){
			// 		addedPointList.push_back(seg_list[j]);
			// 	}
			// }
			
			return pointList;
			
}

pair<double, double> Roadmap::getCurveMidPoint(Point_2 start, Point_2 end, Point_2 after){
	pair<double, double> mid_point;
	if(bg::equals(start, end)){
		mid_point.first = end.get<0>();
		mid_point.second = end.get<1>();
		return mid_point;
	}
	if(bg::equals(end, after)){
		mid_point.first = end.get<0>();
		mid_point.second = end.get<1>();
		return mid_point;
	}
	pair<double, double> mid_start_end, mid_end_after, mid_start_after;
	mid_start_end.first = (start.get<0>() + end.get<0>()) / 2;
	mid_start_end.second = (start.get<1>() + end.get<1>()) / 2;
	mid_end_after.first = (end.get<0>() + after.get<0>()) / 2;
	mid_end_after.second = (end.get<1>() + after.get<1>()) / 2;
	mid_start_after.first = (mid_start_end.first + mid_end_after.first) / 2;
	mid_start_after.second = (mid_start_end.second + mid_end_after.second) / 2;
	mid_point.first = mid_start_after.first*0.7 + end.get<0>()*0.3;
	mid_point.second = mid_start_after.second*0.7 + end.get<1>()*0.3;
	return mid_point;
}

bool Roadmap::isTestPtInNormalGrid(int test){
	for(int j = 0; j < m_newAddedPt.size();j ++){
			if(test == m_newAddedPt[j]){
				return false;
			}
	}
	if(test < 0){
		return false;
	}
	return true;
}

bool Roadmap::isTestPathInNormalGrid(std::vector<int> test_path){
	for(int i = 0;i < test_path.size(); i++){
		for(int j = 0; j < m_newAddedPt.size();j ++){
			if(test_path[i] == m_newAddedPt[j]){
				return false;
			}
		}
	}
	return true;
	
}

bool Roadmap::check2stepIntersection(std::vector<int> test_path, std::vector<int> other_path){
	Point_2 test_start, test_end, other_start, other_end;
	test_start.set<0>(getVertexLocationFromID(test_path[0]).first);
	test_start.set<1>(getVertexLocationFromID(test_path[0]).second);
	test_end.set<0>(getVertexLocationFromID(test_path[2]).first);
	test_end.set<1>(getVertexLocationFromID(test_path[2]).second);
	other_start.set<0>(getVertexLocationFromID(other_path[0]).first);
	other_start.set<1>(getVertexLocationFromID(other_path[0]).second);
	other_end.set<0>(getVertexLocationFromID(other_path[2]).first);
	other_end.set<1>(getVertexLocationFromID(other_path[2]).second);
	if(bg::equals(test_start, test_end)){
		return true;
	}
	Segment_2 test_seg(test_start, test_end);
	Segment_2 other_seg(other_start, other_end);
	if(bg::equals(test_seg, other_seg)){
		return true;
	}

	if(bg::intersects(test_seg, other_seg)){
		return true;
	}
	return false;

}

bool Roadmap::check3stepIntersection(std::vector<int> test_path, std::vector<int> other_path){
	
	Point_2 test_start, test_end, other_start, other_end;
	test_start.set<0>(getVertexLocationFromID(test_path[0]).first);
	test_start.set<1>(getVertexLocationFromID(test_path[0]).second);
	test_end.set<0>(getVertexLocationFromID(test_path[3]).first);
	test_end.set<1>(getVertexLocationFromID(test_path[3]).second);
	other_start.set<0>(getVertexLocationFromID(other_path[0]).first);
	other_start.set<1>(getVertexLocationFromID(other_path[0]).second);
	other_end.set<0>(getVertexLocationFromID(other_path[3]).first);
	other_end.set<1>(getVertexLocationFromID(other_path[3]).second);

	Segment_2 test_seg(test_start, test_end);
	Segment_2 other_seg(other_start, other_end);

	if(bg::equals(test_seg, other_seg)){
		return true;
	}
	if(!bg::intersects(test_seg, other_seg)){
		double distance = bg::distance(test_seg, other_seg);
		if(distance < m_edgeLength){
			double inner_product = (test_end.get<0>()-test_start.get<0>())*(other_end.get<0>()-other_start.get<0>()) + (test_end.get<1>()-test_start.get<1>())*(other_end.get<1>()-other_start.get<1>());
			if(inner_product > 0){
				return false;
			}else{
				return true;
			}
		}else{
			return false;
		}
	}else{
		return true;
	}

	if(bg::equals(test_start, other_start) || bg::equals(test_start, other_end) || bg::equals(test_end, other_start) || bg::equals(test_end, other_end)){
		double inner_product = (test_end.get<0>()-test_start.get<0>())*(other_end.get<0>()-other_start.get<0>()) + (test_end.get<1>()-test_start.get<1>())*(other_end.get<1>()-other_start.get<1>());
		if(inner_product > 0){
			return false;
		}else{
			return true;
		}
	}

}

bool Roadmap::checkAroundSeg(std::vector<int> test_path, std::vector<int> other_path){
	Point_2 test_start, test_end, other_start, other_end;
	test_start.set<0>(getVertexLocationFromID(test_path[0]).first);
	test_start.set<1>(getVertexLocationFromID(test_path[0]).second);
	test_end.set<0>(getVertexLocationFromID(test_path[2]).first);
	test_end.set<1>(getVertexLocationFromID(test_path[2]).second);
	other_start.set<0>(getVertexLocationFromID(other_path[0]).first);
	other_start.set<1>(getVertexLocationFromID(other_path[0]).second);
	other_end.set<0>(getVertexLocationFromID(other_path[2]).first);
	other_end.set<1>(getVertexLocationFromID(other_path[2]).second);

	Segment_2 test_seg(test_start, test_end);
	Segment_2 other_seg(other_start, other_end);
	if(!bg::intersects(test_seg, other_seg)){
		double distance = bg::distance(test_seg, other_seg);
		if(distance < 1.8*m_edgeLength){
			return true;
		}else{
			return false;
		}
	}else{
		return true;
	}
}

bool Roadmap::checkRobotCollision(pair<double, double> current_test, pair<double, double> ori_current_test){
	double distance = sqrt(pow(current_test.first - ori_current_test.first, 2) + pow(current_test.second - ori_current_test.second, 2));
	if(distance > 2*m_radius){
		return false;
	}else{
		return true;
	}
}

void Roadmap::improvePaths(map<int, vector<int> >& paths, map<int, vector<pair<double, double>>> & out_paths, int & shortcut_times){
	// Remove obvious ossilation from the paths
	for(int t = 1; t < paths[0].size() - 1; t ++){
		for(int r = 0; r < paths.size(); r ++){
			// Do we have single step ossilation?
			if(paths[r][t - 1] == paths[r][t + 1] && paths[r][t - 1] != paths[r][t]){
				// Check whether any other robots goes to paths[r][t - 1] at t
				bool conflict = false;
				for(int ori = 0; ori < paths.size(); ori ++){
					if(ori == r) continue;
					if(paths[ori][t] == paths[r][t - 1]){
						conflict = true;
					}
				}

				// If not, let the robot stay
				if(!conflict){
					paths[r][t] = paths[r][t - 1];
				}
			}
			// Do a two step look ahead as well
			else if((t < paths[0].size() - 2) && paths[r][t - 1] == paths[r][t + 2] && 
				paths[r][t] == paths[r][t + 1] && paths[r][t - 1] != paths[r][t]){
				// Check whether any other robots goes to paths[r][t - 1] at t
				bool conflict = false;
				for(int ori = 0; ori < paths.size(); ori ++){
					if(ori == r) continue;
					if(paths[ori][t] == paths[r][t - 1] || paths[ori][t + 1] == paths[r][t - 1]){
						conflict = true;
					}
				}


				// If not, let the robot stay
				if(!conflict){
					paths[r][t+1] = paths[r][t] = paths[r][t - 1];
				}
			}
		}
	}
	for(int r = 0; r < paths.size(); r ++){
		
		for(int t = 0; t < paths[r].size();t++){
			pair<double, double> test;
			test.first = m_vidPointMap[m_vidFGMap[paths[r][t]]].get<0>();
			test.second = m_vidPointMap[m_vidFGMap[paths[r][t]]].get<1>();

			out_paths[r].push_back(test);
		}
		

	}
	/*
	bool intersection = false;
	map<int, vector<int>> no_stop_paths;
	//build no stop paths
	for(int r = 0; r < paths.size(); r ++){
		no_stop_paths[r] = vector<int>();
		no_stop_paths[r].push_back(paths[r][0]);
		for(int t = 1; t < paths[r].size();t++){
			int current_fid = paths[r][t];
			int previous_fid = paths[r][t-1];
			no_stop_paths[r].push_back(current_fid);
		}
		

	}
	*/
	/*
	// pad each robot's path, so that they have the same length, ones reach destination first, will wait 
	// at its location until the end
	for(int r = 0;r < no_stop_paths.size();r++){
		int padded_size = max_size - no_stop_paths[r].size();
		int current_size = no_stop_paths[r].size();
		int padded_fid = no_stop_paths[r][current_size-1];
		for(int i = 0;i < padded_size;i++){
			no_stop_paths[r].push_back(padded_fid);
		}
	}


	
	std::cout<<"paths after no stop"<<endl;
	for(int r = 0;r < no_stop_paths.size();r++){
		std::cout<<r<<" robot path"<<endl;
		for(int i = 0; i < no_stop_paths[r].size();i++){
				std::cout<<no_stop_paths[r][i]<<" ";

		}
		std::cout<<std::endl;
	}
	*/
	//group each two steps together into one group, seperate each group into 10 small steps 
	
	//map<int, vector<pair<double, double>>> no_stop_sliced_paths;
	//for(int r = 0; r < no_stop_paths.size(); r ++){
	//	no_stop_sliced_paths[r] = vector<pair<double, double>>();
	//}
/*
	std::vector<int> test_path, other_path;
	for(int t = 0; t < no_stop_paths[0].size()-2; t += 2){

		for(int r = 0; r < no_stop_paths.size(); r ++){
			test_path.clear();
			test_path.push_back(no_stop_paths[r][t]);
			test_path.push_back(no_stop_paths[r][t+1]);
			test_path.push_back(no_stop_paths[r][t+2]);
			if((test_path[0] != test_path[1]) && (test_path[1] != test_path[2])){ 
				pair<double, double> current_test, test_start, test_end, previous_test;
				double single_x, single_y;
				test_start.first = getVertexLocationFromID(test_path[0]).first;
				test_start.second = getVertexLocationFromID(test_path[0]).second;
				test_end.first = getVertexLocationFromID(test_path[2]).first;
				test_end.second = getVertexLocationFromID(test_path[2]).second;
				single_x = (test_end.first - test_start.first) / 10.0;
				single_y = (test_end.second - test_start.second) / 10.0;
				for(int j = 0; j <10; j++){
					current_test.first = test_start.first + j*single_x;
					current_test.second = test_start.second + j*single_y;
					no_stop_sliced_paths[r].push_back(current_test);
				}
			}else if((test_path[0] != test_path[1]) && (test_path[1] == test_path[2])){
				pair<double, double> current_test, test_start, test_end, previous_test;
				double single_x, single_y;
				test_start.first = getVertexLocationFromID(test_path[0]).first;
				test_start.second = getVertexLocationFromID(test_path[0]).second;
				test_end.first = getVertexLocationFromID(test_path[2]).first;
				test_end.second = getVertexLocationFromID(test_path[2]).second;
				single_x = (test_end.first - test_start.first) / 6.0;
				single_y = (test_end.second - test_start.second) / 6.0;
				for(int j = 0; j <6; j++){
					current_test.first = test_start.first + j*single_x;
					current_test.second = test_start.second + j*single_y;
					no_stop_sliced_paths[r].push_back(current_test);
				}
			}else if((test_path[0] == test_path[1]) && (test_path[1] == test_path[2])){
				pair<double, double> current_test, test_start, test_end, previous_test;
				double single_x, single_y;
				test_start.first = getVertexLocationFromID(test_path[0]).first;
				test_start.second = getVertexLocationFromID(test_path[0]).second;
				for(int j = 0; j <10; j++){
					current_test.first = test_start.first;
					current_test.second = test_start.second;
					no_stop_sliced_paths[r].push_back(current_test);
				}
			}else if((test_path[0] == test_path[1]) && (test_path[1] != test_path[2])){
				pair<double, double> current_test, test_start, test_end, previous_test;
				double single_x, single_y;
				test_start.first = getVertexLocationFromID(test_path[0]).first;
				test_start.second = getVertexLocationFromID(test_path[0]).second;
				test_end.first = getVertexLocationFromID(test_path[2]).first;
				test_end.second = getVertexLocationFromID(test_path[2]).second;
				single_x = (test_end.first - test_start.first) / 6.0;
				single_y = (test_end.second - test_start.second) / 6.0;
				for(int j = 0; j <6; j++){
					current_test.first = test_start.first + j*single_x;
					current_test.second = test_start.second + j*single_y;
					no_stop_sliced_paths[r].push_back(current_test);
				}
			}

			if(t == no_stop_paths[0].size()-4){
				std::vector<int> last_test_path, last_other_path;
				last_test_path.clear();
				last_test_path.push_back(no_stop_paths[r][t+2]);
				last_test_path.push_back(no_stop_paths[r][t+3]);
				if(last_test_path[0] != last_test_path[1]){ 
					pair<double, double> current_test, test_start, test_end, previous_test;
					double single_x, single_y;
					test_start.first = getVertexLocationFromID(last_test_path[0]).first;
					test_start.second = getVertexLocationFromID(last_test_path[0]).second;
					test_end.first = getVertexLocationFromID(last_test_path[1]).first;
					test_end.second = getVertexLocationFromID(last_test_path[1]).second;
					single_x = (test_end.first - test_start.first) / 6.0;
					single_y = (test_end.second - test_start.second) / 6.0;
					for(int j = 0; j <6; j++){
						current_test.first = test_start.first + j*single_x;
						current_test.second = test_start.second + j*single_y;
						no_stop_sliced_paths[r].push_back(current_test);
					}	
				}else if(last_test_path[0] == last_test_path[1]){
					pair<double, double> current_test, test_start, test_end, previous_test;
					double single_x, single_y;
					test_start.first = getVertexLocationFromID(last_test_path[0]).first;
					test_start.second = getVertexLocationFromID(last_test_path[0]).second;
					test_end.first = getVertexLocationFromID(last_test_path[1]).first;
					test_end.second = getVertexLocationFromID(last_test_path[1]).second;
					single_x = (test_end.first - test_start.first) / 6.0;
					single_y = (test_end.second - test_start.second) / 6.0;
					for(int j = 0; j <6; j++){
						current_test.first = test_start.first + j*single_x;
						current_test.second = test_start.second + j*single_y;
						no_stop_sliced_paths[r].push_back(current_test);
					}	
				}
			}
		}

	}

	std::cout<<"paths after seperating"<<endl;
	for(int r = 0;r < no_stop_sliced_paths.size();r++){
		std::cout<<r<<" robot path"<<endl;
		for(int i = 0; i < no_stop_sliced_paths[r].size();i++){
				std::cout<<no_stop_sliced_paths[r][i].first<<","<<no_stop_sliced_paths[r][i].second<<" ";

		}
		std::cout<<std::endl;
	}
	// move all the robot along their own paths simultaneously, check collision to decide which robot gets to move forward,
	// which robot gets to stay unmoved
	int max_sliced_size = 0;
	for(int r = 0;r < no_stop_sliced_paths.size();r ++){
		if(no_stop_sliced_paths[r].size() > max_sliced_size){
			max_sliced_size = no_stop_sliced_paths[r].size();
		}
	}
	for(int r = 0;r < no_stop_sliced_paths.size();r++){
		int padded_size = max_sliced_size - no_stop_sliced_paths[r].size();
		int current_size = no_stop_sliced_paths[r].size();
		pair<double, double> padded_pt = no_stop_sliced_paths[r][current_size-1];
		for(int i = 0;i < padded_size;i++){
			no_stop_sliced_paths[r].push_back(padded_pt);
		}
	}
	for(int t = 1;t < max_sliced_size;t++){
		for(int r = 0;r < no_stop_sliced_paths.size();r++){
			if(!IfRobotMoveForward(no_stop_sliced_paths, r, t)){
				no_stop_sliced_paths[r].insert(no_stop_sliced_paths[r].begin()+t+1, no_stop_sliced_paths[r][t]);
			}
		}
	}
	std::cout<<"paths after insertion"<<endl;
	for(int r = 0;r < no_stop_sliced_paths.size();r++){
		std::cout<<r<<" robot path"<<endl;
		for(int i = 0; i < no_stop_sliced_paths[r].size();i++){
				std::cout<<no_stop_sliced_paths[r][i].first<<","<<no_stop_sliced_paths[r][i].second<<" ";

		}
		std::cout<<std::endl;
	}
	int no_stop_sliced_max_size = 0;
	for(int r = 0; r < no_stop_sliced_paths.size(); r ++){
		if(no_stop_sliced_paths[r].size() > no_stop_sliced_max_size){ 
			no_stop_sliced_max_size = no_stop_sliced_paths[r].size();
		}

	}
	// pad each robot's path, so that they have the same length, ones reach destination first, will wait 
	// at its location until the end
	for(int r = 0;r < no_stop_sliced_paths.size();r++){
		int padded_size = no_stop_sliced_max_size - no_stop_sliced_paths[r].size();
		int current_size = no_stop_sliced_paths[r].size();
		pair<double, double> padded_pt = no_stop_sliced_paths[r][current_size-1];
		for(int i = 0;i < padded_size;i++){
			no_stop_sliced_paths[r].push_back(padded_pt);
		}
	}
	std::cout<<"final make span:"<<no_stop_sliced_paths[0].size()*0.1732<<endl;
*/



	bool intersection = false;
	std::vector<int> test_path, other_path;
	int number_shortcut = 0;
	std::vector<bool> shortcut_list(paths.size());
	for(int t = 0; t < paths[0].size()-3; t += 2){
		for(int r = 0; r < paths.size(); r ++){
			test_path.clear();
			test_path.push_back(paths[r][t]);
			test_path.push_back(paths[r][t+1]);
			test_path.push_back(paths[r][t+2]);
			if(isTestPathInNormalGrid(test_path)){  
				for(int ori = 0;ori < paths.size(); ori++){
					if(r == ori) continue;
					other_path.clear();
					other_path.push_back(paths[ori][t]);
					other_path.push_back(paths[ori][t+1]);
					other_path.push_back(paths[ori][t+2]);
					if(check2stepIntersection(test_path, other_path)){
						intersection = true;
						break;
					}else{
						intersection = false;
					}
				}
				if(!intersection){
					shortcut_list[r] = true;
					pair<double, double> start = out_paths[r][t];
					pair<double, double> end = out_paths[r][t+2];
 					double length = sqrt(pow(start.first - end.first, 2) + pow(start.second - end.second, 2));
 					double length_x = end.first - start.first;
 					double length_y = end.second - start.second;
					std::cout << "shortcut happened with robot " <<r+1 << std::endl;
 					if((paths[r][t] == paths[r][t+1]) && (paths[r][t+1] == paths[r][t+2])){
 						out_paths[r][t+1].first = start.first; 
	 					out_paths[r][t+1].second = start.second;
	 					out_paths[r][t+2].first = start.first;
	 					out_paths[r][t+2].second = start.second;
 					}else if(paths[r][t] == paths[r][t+1]){
	 					out_paths[r][t+1].first = start.first; 
	 					out_paths[r][t+1].second = start.second;
	 					out_paths[r][t+2].first = end.first;
	 					out_paths[r][t+2].second = end.second;
 					}else if(paths[r][t+1] == paths[r][t+2]){
						
	 					out_paths[r][t+1].first = end.first; 
	 					out_paths[r][t+1].second = end.second;
	 					out_paths[r][t+2].first = end.first;
	 					out_paths[r][t+2].second = end.second;
 					
 					}else{
						
 						double single_length_x = length_x / 2;
	 					double single_length_y = length_y / 2;
	 					out_paths[r][t+1].first = start.first + single_length_x; 
	 					out_paths[r][t+1].second = start.second + single_length_y;
	 					out_paths[r][t+2].first = start.first + 2* single_length_x;
	 					out_paths[r][t+2].second = start.second + 2* single_length_y;	
 					}
 					
				}else{
					shortcut_list[r] = false;
				}

			}
		}
		bool is_shortcut = true;
		for(int i = 0;i < paths.size();i++){
			if(!shortcut_list[i]){
				is_shortcut = false;
			}
		}
		if(is_shortcut){
			number_shortcut ++;
		}
	}
	std::cout<<"final make span:"<<out_paths[0].size()<<endl;
	std::cout<<"shortcut happens:"<<number_shortcut<<" times"<<endl;
	shortcut_times = number_shortcut;
}

bool Roadmap::IfRobotMoveForward(map<int, vector<pair<double, double>>> &paths,int r,int t){
	pair<double, double> current_test, previous_test;
	current_test = paths[r][t];
	previous_test = paths[r][t-1];
	bool move_forward = true;
	for(int ori = 0; ori < paths.size();ori ++){
		if(ori == r) continue;
		if(checkRobotCollision(current_test, paths[ori][t])){
			if(!checkRobotCollision(current_test, paths[ori][t-1])){
				move_forward = true;
				//paths[ori][t] = paths[ori][t-1];
			}
			if(!checkRobotCollision(previous_test, paths[ori][t])){
				move_forward = false;
			}
		}else{
			continue;
		}
	}
	return move_forward;
}

pair<double, double> Roadmap::getVertexLocationFromID(int vid){
	// Get vid in the original graph
	vid = m_vidFGMap[vid];

	// Locate the vid
	Point_2& p = m_vidPointMap[vid];
	Point_2 pp = (p);

	return pair<double, double>(pp.get<0>(), pp.get<1>());
}