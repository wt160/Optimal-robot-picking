/*
 *  Some helper functions for geometric computing
 *
 *  Created on: Jan 30, 2015
 *  Author: Jingjin Yu
 */

#ifndef _O_CGAL_HELPER_H_
#define _O_CGAL_HELPER_H_

#include "Boost_types.h"
#include "graph.h"
#include "shortest_path/visilibity.hpp"
#include <map>
#include <cmath>
#define BIG_ENV
using namespace std;
using namespace VisiLibity;

#define USE_TETRIS 
#define SPECIAL_STRUCTURE
//#define STACK
#define OBJ_WIDTH 32
#define OBJ_LENGTH 150
//#define ALL_FLAT 
#define NOT_AXIS_ALIGNED
//#define DIFFERENT_SIZE
#define MIXED_CLUSTER
//double getTriangleArea(const Point_2& p1, Point_2 p2, Point_2 p3);
// Populate the polygon as an approximate disc with segments nuber of sides
//void populateApproximateDisc(Polygon_2 &poly, Point_2 &center, double radius, int segments = 18);
//std::vector<Point_2> getGraspCenterFromCube(Polygon_2 current);

// Compute Minkowski sum of a polygon with a disc 
Polygon_2 growPolygonByRadius(Polygon_2 &poly, double radius, int split_num);

// Compute the distance between two points
double getDistance(Point_2& p1, Point_2& p2);
double getDistance(Point_2& p, double x, double y);

// Compute path length
double getPathLength(std::list<Point_2> path);
double dot(Point_2 v1, Point_2 v2);

// All pairs shortest path computation
void getAllPairsShortestPath (Graph* pGraph, map<int, map<int, int> > &dist);

// Convert a CGAL polygon to a VisiLibity polygon
void convertFromBoosttoVisilibity(Polygon_2 &poly, VisiLibity::Polygon &visiPoly, bool ccw = true);

std::vector<Point_2> getCirclePoints(Point_2 center, double radius, double orientation);
// Does two polygons have boundary edges that intersect?
//bool boundaryInterset(ECPolygon_2 &poly1, ECPolygon_2 &poly2);
std::vector<std::vector<int>> generateComb(int K);
// // Convert to exact contruction
//ECPoint_2 convertToExactPoint(Point_2 &p);
//ECPolygon_2 convertToExactPolygon(Polygon_2 &poly);

//double getShortestDistance(Polygon_2 p1, Polygon_2 p2, Point_2 & p, Segment_2& s);

void generatePoly(Polygon_2& result, double center_x, double center_y, double yaw);
void generatePoly(Polygon_2& result, double center_x, double center_y, double yaw, double width, double length);
void generateTetrisBlock(Polygon_2& result, double start_x, double start_y, double yaw, int type);

void getBoundingPoly(Polygon_2& target, Point_2& upper_left, Point_2& upper_right, Point_2& down_left, Point_2& down_right, double dist);
void convertPolygon2Region(Polygon_2 poly, double& center_x, double& center_y, double& size_x, double& size_y);

#endif //_O_CGAL_HELPER_H_
