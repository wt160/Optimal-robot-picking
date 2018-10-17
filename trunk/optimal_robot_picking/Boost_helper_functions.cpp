/*
 *	Helper function implementations
 *
 *  Created on: Jan 30, 2015
 *  Author: Jingjin Yu
 */
#include "Boost_helper_functions.h"

// void populateApproximateDisc(Polygon_2 &poly, Point_2 &center, double radius, int segments){
// 	for(int i = 0; i < segments; i ++){
// 		poly.push_back (Point_2 ( center.get<0>() + radius*cos(i*PI*2/segments), center.y() + radius*sin(i*PI*2/segments)));
// 	}
// }

//std::vector<Point_2> getGraspCenterFromCube(Polygon_2 current) {

//}

Polygon_2 growPolygonByRadius(Polygon_2 &poly, double radius, int split_num){
	// Get the disk for computing Minkowski sum
	Polygon_2 disc;
    const int points_per_circle = split_num;
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(radius);
    boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;
	// Do computation
	boost::geometry::model::multi_polygon<Polygon_2> result;
	boost::geometry::buffer(poly, result,
                distance_strategy, side_strategy,
                join_strategy, end_strategy, circle_strategy);


	return result[0];
}

double getPathLength(std::list<Point_2> path){
	double length = 0;
	std::list<Point_2>::iterator vit = path.begin();
	if(vit != path.end()){
		Point_2 p = *vit;
		vit++;
		for(; vit != path.end(); vit++){
			Point_2 p2 = *vit;
			length += sqrt((p2.get<0>() - p.get<0>())*(p2.get<0>() - p.get<0>()) + (p2.get<1>() - p.get<1>())*(p2.get<1>() - p.get<1>()));
			p = p2;
		}
	}
	return length;
}

double dot(Point_2 v1, Point_2 v2){
  double product;
  product = v1.get<0>()*v2.get<0>() + v1.get<1>()*v2.get<1>();
  return product;
}

double getDistance(Point_2& p1x, Point_2& p2x){
	
	Point_2 p1 = p1x;
	Point_2 p2 = p2x;
	return sqrt((p2.get<0>() - p1.get<0>())*(p2.get<0>() - p1.get<0>()) + (p2.get<1>() - p1.get<1>())*(p2.get<1>() - p1.get<1>()));
}

double getDistance(Point_2& p, double x, double y){
	return sqrt((p.get<0>() - x)*(p.get<0>() - x) + (p.get<1>() - y)*(p.get<1>() - y));
}

void getAllPairsShortestPath (Graph* pGraph, map<int, map<int, int> > &dist){
	int V = pGraph->getVertexSet().size();

    /* dist[][] will be the output matrix that will finally have the shortest 
      distances between every pair of vertices */
    int i, j, k;
 
    /* Initialize the solution matrix same as input graph matrix. Or 
       we can say the initial values of shortest distances are based
       on shortest paths considering no intermediate vertex. */
    for (i = 0; i < V; i++)
	{
        for (j = 0; j < V; j++){
            dist[i][j] = (pGraph->hasEdge(i, j) ? 1 : (i == j?0:100000000));
		}
	}
 
    /* Add all vertices one by one to the set of intermediate vertices.
      ---> Before start of a iteration, we have shortest distances between all
      pairs of vertices such that the shortest distances consider only the
      vertices in set {0, 1, 2, .. k-1} as intermediate vertices.
      ----> After the end of a iteration, vertex no. k is added to the set of
      intermediate vertices and the set becomes {0, 1, 2, .. k} */
    for (k = 0; k < V; k++)
    {
        // Pick all vertices as source one by one
        for (i = 0; i < V; i++)
        {
            // Pick all vertices as destination for the
            // above picked source
            for (j = 0; j < V; j++)
            {
                // If vertex k is on the shortest path from
                // i to j, then update the value of dist[i][j]
                if (dist[i][k] + dist[k][j] < dist[i][j])
                    dist[i][j] = dist[i][k] + dist[k][j];
            }
        }
    }
}

void convertFromBoosttoVisilibity(Polygon_2 &poly, Polygon &visiPoly, bool ccw){
	// Reverse orientation as needed
	// if((poly.is_counterclockwise_oriented() && ccw == false) || 
	//    (!poly.is_counterclockwise_oriented() && ccw)){
	// 	poly.reverse_orientation();
	// }
  bg::correct(poly);

	// Get the vertices and convert
	 
	std::vector<Point> vp;
	for(int i = 0; i < poly.outer().size()-1; i++){
		Point_2 p = poly.outer()[i];
		vp.push_back(Point(p.get<0>(), p.get<1>()));
	}

	visiPoly.set_vertices(vp);
	if (!ccw) {
		visiPoly.reverse();
	}
	// visiPoly.eliminate_redundant_vertices();
  //visiPoly.enforce_standard_form();
}

std::vector<Point_2> getCirclePoints(Point_2 center, double radius, double orientation) {
	// each is 30 degree apart
	std::vector<Point_2> circle_pts;
	for (int i = 0; i < 12; i++) {
		Point_2 temp;
		temp.set<0>(center.get<0>() + cos(orientation + 30.0/180.0*3.14159*i)*radius);
		temp.set<1>(center.get<1>() + sin(orientation + 30.0/180.0*3.14159*i)*radius);
		circle_pts.push_back(temp);
	}
	return circle_pts;
}

void getBoundingPoly(Polygon_2& target, Point_2& upper_left, Point_2& upper_right, Point_2& down_left, Point_2& down_right, double dist){
	double upper = 1000000;
	double down = -1000000;
	double right = -1000000;
	double left = 1000000;
	for(int i = 0; i < target.outer().size();i ++){
		if(target.outer()[i].get<0>() < left){
			left = target.outer()[i].get<0>();
		}
		if(target.outer()[i].get<0>() > right){
			right = target.outer()[i].get<0>();
		}
		if(target.outer()[i].get<1>() > down){
			down = target.outer()[i].get<1>();
		}
		if(target.outer()[i].get<1>() < upper){
			upper = target.outer()[i].get<1>();
		}
	}

	upper_left.set<0>(left - dist);
	upper_left.set<1>(upper - dist);
	upper_right.set<0>(right + dist);
	upper_right.set<1>(upper - dist);
	down_left.set<0>(left - dist);
	down_left.set<1>(down + dist);
	down_right.set<0>(right + dist);
	down_right.set<1>(down + dist);
}

void generatePoly(Polygon_2& result, double center_x, double center_y, double yaw) {
	result.outer().clear();
	Point_2 upper_right, upper_left, down_right, down_left;
	double x, y;
	double length = OBJ_LENGTH / 2.0;  double width = OBJ_WIDTH / 2.0;
	x = length; y = width;
	if (std::abs(yaw - 1.57) < 0.001) {
		/*  
		upper_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		upper_right.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
		x = -length; y = width;
		upper_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		upper_left.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
		x = length; y = -width;
		down_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		down_right.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
		x = -length; y = -width;
		down_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		down_left.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
		*/
		upper_right.set<0>(center_x + y);
		upper_right.set<1>(center_y - x);

		upper_left.set<0>(center_x - y);
		upper_left.set<1>(center_y - x);

		down_right.set<0>(center_x + y);
		down_right.set<1>(center_y + x);

		down_left.set<0>(center_x - y);
		down_left.set<1>(center_y + x);
	}
	else if (std::abs(yaw - 0) < 0.0001) {
		upper_right.set<0>(center_x + x);
		upper_right.set<1>(center_y - y);
		
		upper_left.set<0>(center_x - x);
		upper_left.set<1>(center_y - y);
		
		down_right.set<0>(center_x + x);
		down_right.set<1>(center_y + y);

		down_left.set<0>(center_x - x);
		down_left.set<1>(center_y + y);
	}
	else {
		upper_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		upper_right.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
		x = -length; y = width;
		upper_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		upper_left.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
		x = length; y = -width;
		down_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		down_right.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
		x = -length; y = -width;
		down_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		down_left.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
	}
	bg::append(result.outer(), down_left);
	bg::append(result.outer(), upper_left);
	bg::append(result.outer(), upper_right);
	bg::append(result.outer(), down_right);
	
	bg::correct(result);
}

void generatePoly(Polygon_2& result, double center_x, double center_y, double yaw, double diff_length, double diff_width) {
	result.outer().clear();
	Point_2 upper_right, upper_left, down_right, down_left;
	double x, y;
	double length = diff_length / 2.0;  double width = diff_width / 2.0;
	x = length; y = width;
	if (std::abs(yaw - 1.57) < 0.001) {
		/*  
		upper_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		upper_right.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
		x = -length; y = width;
		upper_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		upper_left.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
		x = length; y = -width;
		down_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		down_right.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
		x = -length; y = -width;
		down_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		down_left.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
		*/
		upper_right.set<0>(center_x + y);
		upper_right.set<1>(center_y - x);

		upper_left.set<0>(center_x - y);
		upper_left.set<1>(center_y - x);

		down_right.set<0>(center_x + y);
		down_right.set<1>(center_y + x);

		down_left.set<0>(center_x - y);
		down_left.set<1>(center_y + x);
	}
	else if (std::abs(yaw - 0) < 0.0001) {
		upper_right.set<0>(center_x + x);
		upper_right.set<1>(center_y - y);
		
		upper_left.set<0>(center_x - x);
		upper_left.set<1>(center_y - y);
		
		down_right.set<0>(center_x + x);
		down_right.set<1>(center_y + y);

		down_left.set<0>(center_x - x);
		down_left.set<1>(center_y + y);
	}
	else {
		upper_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		upper_right.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
		x = -length; y = width;
		upper_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		upper_left.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
		x = length; y = -width;
		down_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		down_right.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
		x = -length; y = -width;
		down_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
		down_left.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
	}
	bg::append(result.outer(), down_left);
	bg::append(result.outer(), upper_left);
	bg::append(result.outer(), upper_right);
	bg::append(result.outer(), down_right);
	
	bg::correct(result);
}

void convertPolygon2Region(Polygon_2 poly, double& center_x, double& center_y, double& size_x, double& size_y) {
	//now I assume all the polygons are rectangles, following [0]--downLeft, [1]--upperLeft, [2]--upperRight, [3]--downRight
	double min_x = 0;
	double max_x = 0;
	double min_y = 0;
	double max_y = 0;
	min_x = poly.outer()[1].get<0>() - 30/1.414;
	max_x = poly.outer()[2].get<0>() + 30/1.414;
	min_y = poly.outer()[1].get<1>() - 30/1.414;
	max_y = poly.outer()[0].get<1>() + 30/1.414;
	if (min_x > max_x) {
		std::cerr << "Error: min_x > max_x" << std::endl;
	}
	if (min_y > max_y) {
		std::cerr << "Error: min_y > max_y" << std::endl;
	}
	center_x = (min_x + max_x) / 2.0;
	center_y = (min_y + max_y) / 2.0;
	size_x = max_x - min_x;
	size_y = max_y - min_y;

}

//from 0 
std::vector<std::vector<int>> generateComb(int K) {
	std::vector<std::vector<int>> result;
	std::vector<int> temp; 

	for (int i = K-1; i >= 0; i--) {
		temp.push_back(i);
	}
	int num = 0;
	do {
		for (int i = 0; i < K; i++) {
			std::cout << temp[i] << ","; 
		}
		std::cout << std::endl;
		result.push_back(temp);
		num++;
	} while (std::prev_permutation(temp.begin(), temp.end()));
	std::cout << "num:" << num << std::endl;
	return result;
						  // print integers and permute bitmask
	/*do {
		std::vector<int> temp;
		for (int i = 0; i < K; ++i) // [0..N-1] integers
		{
			if (bitmask[i]) {
				std::cout << " " << i;
				temp.push_back(i);
			}
		}
		std::cout << std::endl;
	} while (std::prev_permutation(bitmask.begin(), bitmask.end()));
	*/
}

// bool boundaryInterset(ECPolygon_2 &poly1, ECPolygon_2 &poly2){
// 	// Check edge by edge
// 	for(int i = 0; i < poly1.size(); i++){
// 		ECSegment_2 seg1 = poly1.edge(i);
// 		for(int j = 0; j < poly2.size(); j ++){
// 			ECSegment_2 seg2 = poly2.edge(j);
// 			if(CGAL::do_intersect(seg1, seg2)){return true;}
// 		}
// 	}
// 	return false;
// }

// ECPoint_2 convertToExactPoint(Point_2 &p){
// 	static CGAL::Cartesian_converter<K,ICK> K_ICK_converter; 
// 	ICPoint_2 pp = K_ICK_converter(p);
// 	return ECPoint_2(pp.x(), pp.y());
// }


// ECPolygon_2 convertToExactPolygon(Polygon_2 &poly){
// 	ECPolygon_2 ecPoly;
// 	for(Polygon_2::Vertex_iterator vi = poly.vertices_begin(); vi != poly.vertices_end(); vi ++){
// 		static CGAL::Cartesian_converter<K,ICK> K_ICK_converter; 
// 		ICPoint_2 pp = K_ICK_converter(*vi);
// 		ecPoly.push_back(ECPoint_2(pp.x(), pp.y()));
// 	}
// 	return ecPoly;
// }

// double getTriangleArea(const Point_2& p1, Point_2 p2, Point_2 p3){
// 	return 1/2*fabs((p1.x()-p3.x())*(p2.y()-p1.y())-(p1.x()-p2.x())*(p3.y()-p1.y()));
// }

// double getShortestDistance(Polygon_2 p1, Polygon_2 p2, Point_2 & p, Segment_2& s){	
// 	double distance, minDistance;
// 	distance = 0;
// 	minDistance = 0;
// 	int k = 0;
// 	int i, j;
// 	for(i = 0;i < p1.size(); i++){              // traverse p1's vertex
// 		for(j = 0; j < p2.size(); j ++){        // traverse p2's edge
// 			if(k == 0){
// 				minDistance = getTriangleArea(p1.vertex(i), p2.edge(j).source(), p2.edge(j).target());
// 				p = p1.vertex(i);
// 				s = p2.edge(j);
// 				k++;
// 			}else{
// 				distance = getTriangleArea(p1.vertex(i), p2.edge(j).source(), p2.edge(j).target());
// 				if(distance < minDistance){
// 					minDistance = distance;
// 					p = p1.vertex(i);
// 					s = p2.edge(j);
// 				}
// 			}
		
// 		}
// 	}
// 	for(int i = 0;i < p2.size(); i++){              // traverse p1's vertex
// 		for(int j = 0; j < p1.size(); j ++){        // traverse p2's edge
// 			distance = getTriangleArea(p1.vertex(i), p2.edge(j).source(), p2.edge(j).target());
// 			if(distance < minDistance){
// 				minDistance = distance;
// 				p = p2.vertex(i);
// 				s = p1.edge(j);
// 			}
// 		}
// 	}
// 	return minDistance;
// }

