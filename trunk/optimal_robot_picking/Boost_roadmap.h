/*
 *  The core class (header) for roadmap building. Roadmap building is done in four steps:
 *	1. Overlay a regular lattice (currently only hexagonal lattice) 
 *	2. Remove lattice eges that intersect with configuration space obstacles, and compute the 
 *	   smallest cycle on the remaining lattice that encloses the C-space obstacles. 
 *  3. When a cycle computed about does not lie completely in the free C-space, compute 
 *	   shortest paths in C-space that complete the cycle
 *	4. Post-processing the computed shortest paths for inclusion into the lattice graph
 *
 *  Created on: Jan 30, 2015
 *  Author: Jingjin Yu
 */

#ifndef _O_ROADMAP_H_
#define _O_ROADMAP_H_

#include "Boost_types.h"
#include "graph.h"
#include "shortest_path/visilibity.hpp"
#include "Boost_helper_functions.h"
#include <algorithm>
#include <cmath>
#include <QColor>
#include <QPen>
#include <QBrush>
#include <QGraphicsScene>
#include <cstdio>
#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/polygon.hpp>
#include "voronoi_visual_utils.hpp"
#include <vector>
#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#include "rrts.hpp"
#include "system_single_integrator.h"
#include "kdtree.h"
//#include "prm.h"

using namespace std;
using namespace VisiLibity;
using namespace boost::polygon;
using namespace RRTstar;
using namespace SingleIntegrator;
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

typedef Planner<State, Trajectory, System> planner_t;
typedef Vertex<State, Trajectory, System> vertex_t;

//#define USE_PRM
#define USE_RRT
// namespace boost {
// namespace polygon {

// template <>
// struct geometry_concept<Point_v> {
//   typedef point_concept type;
// };

// template <>
// struct point_traits<Point_v> {
//   typedef int coordinate_type;

//   static inline coordinate_type get(
//       const Point_v& point, orientation_2d orient) {
//     return (orient == HORIZONTAL) ? point.a : point.b;
//   }
// };

// template <>
// struct geometry_concept<Segment_v> {
//   typedef segment_concept type;
// };

// template <>
// struct segment_traits<Segment_v> {
//   typedef int coordinate_type;
//   typedef Point_v point_type;

//   static inline point_type get(const Segment_v& segment, direction_1d dir) {
//     return dir.to_int() ? segment.p1 : segment.p0;
//   }
// };
// }  // polygon
// }  // boost

class TreeNode
{
private:
	std::string textContent;
	std::string tagName;

	TreeNode *parent;

	std::vector<TreeNode *> children;

	int countNodesRec(TreeNode *root, int& count);

public:
	std::pair<double, double> coordinate;
	std::string node_type;
	double distance_till_now;
	std::set<int> all_parents_index;
	double distance;
	int index_;
	TreeNode();
	TreeNode(int index, double dist);

	void appendChild(TreeNode *child);
	void setParent(TreeNode *parent);

	void popBackChild();
	void removeChild(int pos);

	bool hasChildren();
	bool hasParent();

	TreeNode* getParent();
	TreeNode* getChild(int pos);

	int childrenNumber();
	int grandChildrenNum();

	std::string getTextContent();
	std::string getTagName();
	std::vector<TreeNode*> getAllParents();
};



struct graspDist {
	Point_2 grasp;
	double shortestDist;
};

class Roadmap{
private:
	double							m_edgeLength;			// Edge length of the hexgaon
	Polygon_2*						m_pBoundingRect;		// Bounding polygon
	Polygon2_list					m_obstaclePolyList;		// Obstacle polygon list pointer
	Polygon2_list                   m_obstacleOuterPolyList; // obstacle polygon obtained through minkswich
	Polygon2_list                   m_polyVoronoiList;
	Polygon2_list                   m_objectPolyList;
	Polygon2_list                   m_objectOuterPolyList;
	std::list<Point_2>				m_pointList;			// List of all vertices
	std::map<int, Point_2>			m_vidPointMap;			// Vertex id to point map
	std::map<Point_2, int, PointCompare >			m_pointVidMap;			// Point to vertex id map
	QGraphicsScene* mm_scene;
	Graph							m_graph;				// The lattice graph
	Graph							m_finalGraph;			// The final graph for doing planning
	Graph 							m_computeGraph;
	std::map<int, int>				m_vidFGMap;				// Vertex id MAP from final graph to graph
	std::map<int, int>				m_vidGFMap;				// Vertex id MAP from graph to final graph
	std::map<std::pair<int, int>, std::vector<int>>  m_edgeFGMap;
	std::map<Point_2, double, PointCompare>  m_pointDistMap;
	std::map<int, int>              m_vidFCMap;
	std::map<int, int>              m_vidCFMap;
	std::map<int, std::vector<Point_2>> m_graspcenterMap;
	std::map<int, std::vector<Point_2>> m_graspCubeToBaseMap;
	std::map<int, Graph>            m_gidGraphMap;
	//std::map<Point_2, double>     m_graspToDistance;
	std::map<Graph, int>            m_graphGidMap;
	std::map<int, bool>             m_isGraphUsedMap;
	std::vector<Graph>              m_jointCycleVector;
	std::map<int, Graph>            m_gidJointCycleMap;

	std::set<std::pair<int, int> >  m_edgeToBeRemovedSet;	// Edges to be removed
	std::set<int>					m_vertexToBeRemovedSet;	// Vertices to be removed (single degree)

	Graph							m_boundaryBoundingCycle;// Cycle just inside the bounding rect
	std::vector<Graph*>				m_obsBoundingCycleVec;	// List of cycles around obstacle
	Graph 							m_jointCycle;
	std::map<int, bool> 			m_pointUsed;   
	std::vector<Segment_2>          m_newAddedSeg;
	std::vector<Segment_2>          m_newConnection;
	std::vector<int>            m_newAddedPt;
	std::vector<Segment_2> m_keyseg_vector;
	voronoi_diagram<double> m_vd;
	std::vector<segment_data<int>> mv_segments;
	std::vector<point_data<int>> mv_points;
	typedef std::map<std::pair<int, int>, std::list<Point_2> > Path_Map;
	Path_Map m_connectingPathMap;							// Look up map for "bridging" paths
	std::vector<std::pair<std::pair<int,int>,int>> m_specialPair;
	// Temporary local variables
	double bottomLeftX, bottomLeftY, width, height;			// Bounding rectangle paramaters
	double sqrt3;											// sqrt(3)
	double xs, ys;											// Starting x, y for the raw lattice 
	int	   n_w, n_h;										// Raw lattice columns and rows

	


	// Visibility graph
	double						m_epsilon;
	Environment *				m_pEnvironment;
	Visibility_Graph *			m_pVisibilityGraph;

	double							m_radius;
	//planner_t rrts_4;
	//System robot2d_4;
	//planner_t rrts_3;
	//System robot2d_3;
	//planner_t rrts_1;
	//planner_t rrts_2;
	//System robot2d_2;
	//System robot2d_1;
	//planner_t rrts;
	//System robot2d;

	

	std::list<region*> temp_deleted_obstacles;

public:
	


	Roadmap(){m_epsilon = 0.000001; m_pEnvironment = 0; m_pVisibilityGraph = 0;}

	//double getEdgeLength();

	// Build roadmap
	void buildRoadmap(Polygon2_list* pObsList, Polygon2_list* pObsInnerList, Polygon_2* pbondingRect,Polygon2_list* penvVoronoiList,double radius, QGraphicsScene & scene, Polygon2_list* pObsObstacleList = NULL, Polygon2_list* pObsObstacleInnerList = NULL);


	bool isSegInKey(Point_2 p1, Point_2 p2);
	// Retrive all vertices and edges
	std::list<Point_2> & getAllVertices(){return m_pointList;};
	bool isTwoPtInTwoPoly(Point_2 observePt, Point_2 targetPt,Polygon2_list& m_obstaclePolyList);
	bool isTestPathInNormalGrid(std::vector<int> test_path);
	bool optimalLossTest();
int bfsShortestPath(int start, int goal);
void buildRectGridLattice();

	// Paint method
	void addToScene(QGraphicsScene& scene, bool drawEdge = true, 
		QPen edgePen = QPen(Qt::red, 0.025, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin),
		bool drawVertex = true, 
		QPen vertexPen = QPen(Qt::red, 0.05, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	bool isVertexOnEdgeList(voronoi_diagram<double>::vertex_type v, std::vector<Segment_2> &edgeList);
	bool check3stepIntersection(std::vector<int> test_path, std::vector<int> other_path);
	bool solveGeneratedProblem( map<int, vector<pair<double, double>> >& out_paths,
	string& fileFolder, string& fileNameExtra, int i, int number_robot, double & snapLength, double& visibilityLength, int& shortcut_times, int& smooth_nums);
	void recoverConnectivity_3(int& vIDCount);
	// Build a hexagonal lattice that covers the entire bounding rectangle. The rectangle should 
	// contain the configuraiton space and not the free space
	bool checkConnectSeg_2(Graph& jointCycle, Point_2 testPt, Point_2 cyclePt, Polygon_2 poly_1, Polygon_2 poly_2, double max, double min);
	pointAlongPoly getStartEndAroundPoly(Polygon_2 poly, Polygon_2 oppo_poly, Segment_2 shortestSeg, Graph jointCycle, Point_2 pivotPt, Point_2 leftNextPt, Point_2 rightNextPt, Point_2 &startPt, Point_2 &endPt, Point_2 &startCyclePt, Point_2 &endCyclePt, double step, std::vector<Polygon_2> oppoPolyList);
	void drawVoronoiDiagram(QGraphicsScene& scene, bool drawVetex);
	void sample_curved_edge(const voronoi_diagram<double>::edge_type & edge, std::vector<point_data<double>>* sampled_edge);
	point_data<int> retrieve_point(const voronoi_diagram<double>::cell_type& cell);
    segment_data<int> retrieve_segment(const voronoi_diagram<double>::cell_type& cell);
	std::vector<Segment_2> getSurroundingEdges(Polygon_2 obsit, int);
	void buildHexgaonLattice();
	void findGraspablePoses();
	pointAlongPoly getStartEndAroundPoly_2(Polygon_2 poly, Polygon_2 oppo_poly, Graph jointCycle, Point_2 startPivot, Point_2 startPivotNext, Point_2 endPivot, Point_2 endPivotNext, Point_2 &startPt, Point_2 &endPt, Point_2 &startCyclePt, Point_2 &endCyclePt, double step, std::vector<Polygon_2> oppoPolyList);
	void findStartEndPair(Polygon_2 poly, std::vector<Point_2>& pivot, std::vector<Point_2>& pivotNext, Point_2 &startPivot, Point_2 &startPivotNext, Point_2 &endPivot, Point_2 &endPivotNext);
	bool isOneOfList(Point_2 testPt, std::vector<Point_2> pivotList, int& index);

	void drawHexagonLattice(QGraphicsScene& scene, bool drawVetex = false);
	double rmax(double v1, double v2);
	double rmin(double v1, double v2);
	bool isSegFromPoly(Segment_2 seg, Polygon_2 poly);
	bool isPtFromPoly(Point_2 pt, Polygon_2 poly);
	// Remove edges that do not fully belong to the configuration space
	void removeExcessEdges(bool a = true);
	bool isGraphIntersection(Graph g1, Graph g2);
	void getShortestBetweenTwoPtPoly(Polygon_2 poly, Point_2 forward, Point_2 backward, Point_2& previousPolyPt, Point_2& currentPolyPt, Polygon_2 oppo_poly);

	void getShortestBetweenTwoPoly(Polygon_2 poly_1, Polygon_2 poly_2, double &shortestDist, Point_2& shortestPt, Point_2& shortestPolyPt, Segment_2 &intersectSeg, int &shortestPtIndex);
	void drawBoundingCycle(QGraphicsScene& scene);
	void drawJointBoundingCycle(QGraphicsScene& scene);
	void buildComputeMap();
	bool checkConnectAngle(Graph & jointCycle, Point_2 testPt, Point_2 cyclePt, std::vector<Polygon_2> oppoPolyList);
	double calculateAngle(Point_2 cyclePt, Point_2 testPt, Point_2 neiPt);
	double getAddingX(Point_2 end, Point_2 start);
	
	double getAddingY(Point_2 end, Point_2 start);

	void getPivotBetweenPoly(Polygon_2 poly_1, Polygon_2 poly_2, Segment_2 &shortestSeg, Segment_2 intersectSeg, Point_2 shortestPt, Point_2 shortestPolyPt, int shortestPtIndex, Point_2 &point_1, Point_2 &point_2, Point_2 &leftNextPt_1, Point_2 &rightNextPt_1, Point_2 &leftNextPt_2, Point_2 &rightNextPt_2);
	void connectAlongPoly(Polygon_2 poly, Point_2 startPt, Point_2 endPt, Point_2 startCyclePt, Point_2 endCyclePt, pointAlongPoly pAP, int &vid, int &vIDCount);

	void addEdgeFGMap(int fv, int sv, std::vector<int> vidFGlist);
	int intersectObsAndCycle(Polygon_2 & poly, Graph & g, std::vector<Point_2>& v_list, std::vector<std::pair<int, int>>& cycleEdge_list, 
	std::vector<Segment_2>& polyEdge_list);
	bool generateTestfile(vector<pair<int, int> >& sgVec, map<int, vector<pair<double, double>> >& out_paths,
	string& fileFolder, string& fileNameExtra, int i, int number_of_robot, double snapLength, double visibilityLength);
	double shortestDistPair(Segment_2 seg, Point_2 p, Point_2 &closestPt);
	Point_2 getNextPoint(Point_2 currentPt, Point_2 &nextPolyPt, Polygon_2 poly, double length);
	bool checkConnectSeg(Point_2 testPt, Point_2 cyclePt, Segment_2 shortestSeg, Polygon_2 poly_1, Polygon_2 poly_2, double max, double min);
	Point_2 getNewPoint(Point_2 current, double addingX, double addingY);
	Graph getJointCycle(Graph c1, Graph c2);
	bool isPtOnPoly(Point_2 pt, Polygon_2 poly);

	bool isPointInSeg(Point_2 p, Segment_2 seg);
	bool isNearBoundary(Segment_2 test);

	void registerRegularPoint(int& vid, Point_2& currentPt, Point_2& previousPt);
	void registerFinalPoint(int& vIDCount, int& vid, Point_2& currentPt, Point_2& previousPt, std::vector<int>& vidFGlist );
	double neighborCrossLength(Segment_2 startSeg, Segment_2 endSeg, Point_2 startPoint, Point_2 endPoint ,Graph g, Polygon_2 poly, bool crossInsideGraph);
    void recoverConnectivity(int&);
    void recoverConnectivity_2(int& vIDCount);
    bool pointInsideGraph(Point_2& point, Graph& g);
    bool pointInSegment(Point_2 v, Segment_2 seg);
	void addPathOnBoundary(Point_2& v1, Point_2& v2, Graph& g, Polygon_2& poly, Segment_2& polyEdge_1, Segment_2& polyEdge_2,int& vid);
	Point_2 nextInPolygon(Point_2 currentPoint, Point_2 previousPoint, Polygon_2 poly, std::string str);
	Segment_2 pointPairToSegment(std::pair<int, int> e);
	void createRandomGoals(int numRobots, double spacing, vector<pair<double, double>> & goals);

	void drawPoint(Point_2 & p, QGraphicsScene& scene);
	void drawVertexIds(QGraphicsScene& scene);
	Polygon_2 graphToPolygon(Graph g);
	// Build the final graph for graph-based computation
	int buildFinalGraph();
	double polygonCrossLength(Point_2 start, Point_2 end, Point_2 polyCurrentPt, Point_2 polyPreviousPt, Point_2 polyEndPt, Point_2 polyEndAfterPt, Polygon_2 poly);

	// Generate random start and goal pairs
	void createRandomStartGoalPairs(int numRobots, double spacing, vector<pair<double, double> >& starts, 
		vector<pair<double, double> >& goals);
	void createRandomStartGoalPairsAtRegion(int numRobots, double spacing, vector<pair<double, double>> & starts, 
	vector<pair<double,double>> & goals, double left, double right, double up, double bottom, double, double, double, double);
	void createRandomCoordsAtRegion(int numRobots, double spacing, vector<pair<double, double>> & coords, double left, double right, double up, double bottom);

	void reverseVector(std::vector<int> & a);

	// Solve a problem
	bool solveProblem(vector<pair<int, int> >& sgVec, map<int, vector<pair<double, double>> >& paths, 
		string& fileFolder, string& fileNameExtra = string(""));

	// Post process path to reduce ossilation 
	void improvePaths(map<int, vector<int> >& paths, map<int, vector<pair<double, double>>>& out_paths, int & shortcut_times);

	// Retrive the location of a vertex given the int id in m_finalGraph
	pair<double, double> getVertexLocationFromID(int vid);
std::vector<std::pair<double, double>> getVertexListLocationFromID(int pathStep, pair<double, double> start, pair<double, double> end, pair<double, double> before, pair<double, double> after, int mode);

	// Build visibility graph
	void buildVisibilityGraph();
	bool isTestPtInNormalGrid(int test);
	bool checkAroundSeg(std::vector<int> test_path, std::vector<int> other_path);
	bool checkRobotCollision(pair<double, double> current_test, pair<double, double> ori_current_test);
pair<double, double> getCurveMidPoint(Point_2 start, Point_2 end, Point_2 after);
	bool check2stepIntersection(std::vector<int> test_path, std::vector<int> other_path);

	// Compute shortest path between points 
	double computeShortestPath(Point_2& p1, Point_2& p2, std::list<Point_2> & path);
	double computeShortestPath(double x1, double y1, double x2, double y2, std::list<Point_2> & path);
	double computeShortestPath(double x1, double y1, double x2, double y2, vector<pair<double, double> >& path);
	double new_computeShortestPath(Point_2& p1, Point_2& p2, std::list<Point_2> & path, Environment * env, Visibility_Graph *v_graph);
	double new_computeShortestPath(double x1, double y1, double x2, double y2, std::list<Point_2> & path);
	double new_computeShortestPath(double x1, double y1, double x2, double y2, vector<pair<double, double> >& path, Environment * env, Visibility_Graph *v_graph);
	bool IfRobotMoveForward(map<int, vector<pair<double, double>>> &paths,int r,int t);
	double declutterUsingGreedy();
	//void declutterUsingTree();
	void new_buildVisibilityGraph(Polygon2_list temp_obstacle_list, Environment *& env, Visibility_Graph *&v_graph);
	std::map<int, std::vector<Point_2>> new_findGraspablePoses(Polygon2_list obs_list, Polygon2_list obs_outer_list);
	//double declutterUsingSequence(std::vector<int> seq);
	//std::map<int, graspDist> getCandidateObjects(Polygon2_list obs_list, Polygon2_list obs_outer_list, TreeNode * parent_node, PRM& planner);
	std::map<int, graspDist> getCandidateObjects(Polygon2_list obs_list, Polygon2_list obs_outer_list, TreeNode * parent_node, planner_t& planner);
	bool checkNotInSameCluster(Polygon_2 collision_obj, Polygon2_list obs_outer_list, Polygon_2 target_obj);
	Point_2 getShortestDistGrasp(std::vector<Point_2> grasp_poses, Polygon2_list obs_outer_list, double& dist);
	std::vector<Polygon_2> checkLineCollision(Linestring_2 shortest_line, Polygon_2 target_obj, Polygon2_list obs_outer_list);
	double declutterUsingTruncatedTree(int num_objs);
	double visi_computeShortestPath(double x1, double y1, double x2, double y2, std::list<Point_2> & path, Environment * env, Visibility_Graph *v_graph);
	double new_computeShortestPath_1(double x1, double y1, double x2, double y2, std::list<Point_2> & path);
	double new_computeShortestPath(double x1, double y1, double x2, double y2, std::list<Point_2> & path, planner_t & planner);

	double declutterUsingMultipleGreedy(int num_objs);
	std::map<int, graspDist> getAllObjects(Polygon2_list obs_list, Polygon2_list obs_outer_list, planner_t& planner);
	//double new_computeShortestPath(double x1, double y1, double x2, double y2, std::list<Point_2> & path, PRM &rrts);
	//Point_2 getShortestDistGrasp(std::vector<Point_2> grasp_poses, Polygon2_list obs_outer_list, double & dist, PRM& planner);
	Point_2 getShortestDistGrasp(std::vector<Point_2> grasp_poses, Polygon2_list obs_outer_list, double & dist, planner_t& planner);
	double declutterUsingParticleGreedy(int num_objs);
	double declutterUsingMCTS(int num_objs);
	double declutterUsingLocalGreedy(Polygon2_list obs_outer_list, Polygon2_list obs_list);
	double declutterMultiExitGreedy();
	double declutterMultiExitSeparateGreedy();
	double declutterUsingMultiExitTruncatedTree(int num_objs);
	std::map<int, graspDist> getCandidateObjectsFromExit(Polygon2_list obs_list, Polygon2_list obs_outer_list, std::pair<double, double> exit_start, planner_t& planner);

private:
	// Refresh edge list
	void refreshEdgeList();

	// Check whether a point is inside the configuration space
	bool isPointInCSpace(Point_2 &p);

	// Get the (col, row) of the hexagon a point belongs to 
	std::pair<int, int> locateHexagonForPoint(Point_2 &p);

	// Check whether and edge is in an edge set
	bool edgeInSet(int v1, int v2, std::set<std::pair<int, int> >& edgeSet);

	// Add edge to graph if one is not already in
	void addEdgeIfNotThere(int v1, int v2, Graph & graph);

	// Get the next hexagon bordering the current at an edge
	std::pair<int, int> getBorderHexagon(int col, int row, int v1, int v2, int pIndex[]);

	// Get the next vertex in a sequence of vertices
	int getNextNodeInSequence(int i1, int i2, int pIndex[]);

	// Compute a hexgon on the lattice given a (col, row)
	void populateHexagon(int col, int row, Point_2* p, Segment_2* e, Polygon_2 &hex, int* pIndex);

	// Compute all intersecting edges of the full lattice with the polygon obstacle
	void getIntersectingEdges(Polygon_2 & poly, Graph& boundingCycle, bool outerBoundary = false);

	// Check whether a point belongs to some edge not in the obstacle
	int pointBelongToEdgeOutideObstacle(Polygon_2 & poly, int pIndex, std::set<std::pair<int,int> > &tempEdgeToRemoveSet, bool outerBoundary = false);

	// Check whether a point is outside an obstacle
	bool pointOutsideObstacle(Polygon_2 & poly, int pIndex, bool outerBoundary = false);

	// Generate random start and goal pairs
	void createRandomCoords(int numRobots, double spacing, vector<pair<double, double> >& coords);

public:

	// Snapping a set of robots to grid points
	void snapToGraph(vector<pair<double, double> >&coords, vector<int>&snapped, vector<Point_2>& snappedPt);

};





#endif //_OPTMLY_ROADMAP_H_
	
