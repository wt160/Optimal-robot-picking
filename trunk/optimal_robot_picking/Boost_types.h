/*
 *	Type definitions
 *
 *  Created on: Jan 30, 2015
 *  Author: Jingjin Yu
 */

#ifndef _O_CGAL_TYPES_H_
#define _O_CGAL_TYPES_H_

#define _CRT_SECURE_NO_WARNINGS 1
#include "number_type.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <boost/geometry.hpp>
#include <boost/foreach.hpp>
namespace bg = boost::geometry;


typedef bg::model::point<double, 2, bg::cs::cartesian> Point_2;
typedef bg::model::segment<Point_2> Segment_2;
typedef bg::model::polygon<Point_2, false> Polygon_2;
typedef std::list<Polygon_2> Polygon2_list;
typedef bg::model::linestring<Point_2> Linestring_2;
typedef bg::model::box<Point_2> Box_2;
typedef bg::model::multi_point<Point_2> Mpoint_2;
typedef struct PointAlongPoly pointAlongPoly;
struct PointCompare{

bool operator()(const Point_2& l, const Point_2& r ) const{
	if (l.get<0>() < r.get<0>())  return true;
    if (l.get<0>() > r.get<0>())  return false;
    // Otherwise a are equal
    if (l.get<1>() < r.get<1>())  return true;
    if (l.get<1>() > r.get<1>())  return false;
    // Otherwise both are equal
    return false;
}

};


typedef struct bfs_node{
    int id;
    bfs_node* previous;
	bfs_node(int c_id,bfs_node* c_previous) :id(c_id), previous(c_previous) {}
}bfs_node;
// struct Point_v{
//   int a;
//   int b;
//   Point_v(int x, int y) : a(x), b(y) {}
// };

// struct Segment_v{
//   Point_v p0;
//   Point_v p1;
//   Segment_v(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
// };
struct PointAlongPoly{ 
	Point_2 polyEndPt; 
	Point_2 polyCurrentPt; 
	Point_2 polyPreviousPt; 
	Point_2 polyEndAfterPt;
};
// typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
// typedef CGAL::Polygon_set_2<K> Polygon_set_2;

// typedef Polygon_2::Vertex_iterator VertexIterator;
// typedef Polygon_2::Edge_const_iterator EdgeIterator;

// typedef CGAL::Point_set_2<K>::Vertex_handle Vertex_handle;

// // Kernel used for extracting some intermediate data
// typedef CGAL::Exact_predicates_inexact_constructions_kernel ICK;
// typedef ICK::Point_2 ICPoint_2;
// typedef CGAL::Polygon_2<ICK> ICPolygon_2;


// typedef CGAL::Exact_predicates_exact_constructions_kernel EK;
// typedef EK::Point_2 ECPoint_2;
// typedef EK::Segment_2 ECSegment_2;
// typedef CGAL::Polygon_2<EK> ECPolygon_2;
// typedef CGAL::Polygon_with_holes_2<EK> ECPolygon_with_holes_2;


#endif //_OPTMLY_CGAL_TYPES_H_
