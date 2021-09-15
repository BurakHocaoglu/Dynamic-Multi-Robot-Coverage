#pragma once

#ifndef DIST_COVERAGE_UTILS_H
#define DIST_COVERAGE_UTILS_H

#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <coverage_control2/Polygon.h>
#include <coverage_control2/AgentState.h>
#include <coverage_control2/SetInitialPose.h>

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cmath>
#include <chrono>
#include <map>
#include <typeinfo>
#include <list>
#include <vector>
#include <limits>
#include <random>
#include <utility>
#include <algorithm>
#include <unordered_map>

#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>

#include <boost/graph/properties.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/eccentricity.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/arithmetic/arithmetic.hpp> 

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>

// #include "coverage_control2/geodesic_center.hpp"

#include "dump_to_eps.h"

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Point_2<K> Point_2;
typedef CGAL::Vector_2<K> Vector_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
typedef Polygon_2::Vertex_iterator VertexIterator;
typedef Polygon_2::Edge_const_iterator EdgeIterator;

typedef K::Line_2 Line_2;
typedef K::Segment_2 Segment_2;
typedef CGAL::Arr_segment_traits_2<K> Traits_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef Traits_2::Point_2 Arr_Point_2;
typedef Arrangement_2::Face_handle Face_handle;

typedef Arrangement_2::Vertex_const_handle Vertex_const_handle;
typedef Arrangement_2::Halfedge_const_handle Halfedge_const_handle;
typedef Arrangement_2::Face_const_handle Face_const_handle;

typedef Arrangement_2::Edge_const_iterator Edge_const_iterator;
typedef Arrangement_2::Ccb_halfedge_circulator cb_halfedge_circulator;

typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2>  TEV;

typedef CGAL::Straight_skeleton_2<K>  Ss;
typedef boost::shared_ptr<Ss> SsPtr;
typedef boost::shared_ptr<Polygon_2> Polygon_2_ptr;
typedef std::vector<Polygon_2_ptr> Polygon_2_ptr_vector;

typedef std::vector<Polygon_2> Polygon_2_Array;

typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;
typedef Eigen::Matrix<double, Eigen::Dynamic, 2> ConstraintMatrix2d;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

typedef std::pair<double, Vector2d> UtilityPair;

typedef coverage_control2::AgentState AgentStateMsg;
typedef coverage_control2::SetInitialPose SetInitialPose;

enum CentroidAlgorithm {
	UNKNOWN=0,
	GEOMETRIC=1,
	GEODESIC_APPROXIMATE=2,
	GEODESIC_EXACT=3
};

struct MotionParameters {
	double delta_t;
	double attraction_const;
	double repulsion_const;
	double vmax;
	double umax;
	double wmax;
	double physical_radius;
	double safe_radius;
	double K_linear;
	double K_angular;
	double K_repulsion;
	double K_attraction;
	double K_goal;
};

struct SensingParameters {
	double sigma_local;
	double sense_radius;
	double hfov_range;
};

struct AgentState {
	double heading;
	double workload;
	Vector2d position;
	Vector2d velocity;
};

struct BehaviourSettings {
	CentroidAlgorithm centroid_alg;
	bool stationary;
	bool limited_sensing;
	bool flocking;
	bool planning;
	bool visibility;
	bool voronoi;
};

struct BoundarySegment {
	Vector2d normal;
	Vector2d middle;

	double self_product() { 
		return normal.dot(middle);
	}
};

struct DebugLogConfig {
	bool exec_time_log;
};

struct SkeletalNode {
	int id;
	Vector2d point;

	// SkeletalNode() : id(0), point(0, 0) {}
	// SkeletalNode(int _id, Vector2d _p) : id(_id), point(_p) {}
};

// struct SkeletalEdge {
// 	double weight;
// };

// typedef boost::undirected_graph<SkeletalNode> Graph;
// typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
// typedef boost::graph_traits<Graph>::edge_descriptor Edge;

// typedef boost::property_map<Graph, uint32_t SkeletalNode::*>::type NameMap;
// typedef boost::constant_property_map<Edge, double> WeightMap;

// typedef boost::exterior_vertex_property<Graph, double> DistanceProperty;
// typedef DistanceProperty::matrix_type DistanceMatrix;
// typedef DistanceProperty::matrix_map_type DistanceMatrixMap;

typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, 
							  SkeletalNode, EdgeWeightProperty> Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

typedef boost::property_map<Graph, boost::edge_weight_t>::type WeightMap;

typedef boost::exterior_vertex_property<Graph, double> DistanceProperty;
typedef DistanceProperty::matrix_type DistanceMatrix;
typedef DistanceProperty::matrix_map_type DistanceMatrixMap;

typedef boost::exterior_vertex_property<Graph, double> EccentricityProperty;
typedef EccentricityProperty::container_type EccentricityContainer;
typedef EccentricityProperty::map_type EccentricityMap;

class SkeletalBoostGraph {
	public:
		SkeletalBoostGraph();

		Vertex createVertex(int vid, Point_2 p);

		void addEdge(int vid1, Point_2 p1, int vid2, Point_2 p2);
		void clear();
		Vector2d getCentroid();
		uint32_t getCount();

	private:
		Graph g;
		uint32_t count;
		std::map<uint32_t, Vertex> vertex_map;
};

class SkeletalGraph {
	public:
		SkeletalGraph(uint32_t c);

		int getVertexId(int vid, Point_2 p);
		void addEdge(int vid1, Point_2 p1, int vid2, Point_2 p2);

		Vector2d getCentroid();
		uint32_t getCount();

	private:
		MatrixXd graph;
		uint32_t count;
		int next_id_available;
		std::unordered_map<int, int> id_map;
		std::unordered_map<int, SkeletalNode> vertex_map;
};

inline void create_poly_from_raw(XmlRpc::XmlRpcValue& data, Polygon_2& out_poly) {
	size_t v_count = data.size();
	for (size_t i = 0; i < v_count; i++) {
		out_poly.push_back(Point_2((double)(data[i][0]), (double)(data[i][1])));
	}
}

inline void angular_sort(std::vector<Point_2>& V, Point_2& ref) {
	std::sort(V.begin(), V.end(), 
			[&](Point_2 p1, Point_2 p2) {
				return atan2(CGAL::to_double(p1.y() - ref.y()), 
							 CGAL::to_double(p1.x() - ref.x())) < 
					   atan2(CGAL::to_double(p2.y() - ref.y()), 
					   		 CGAL::to_double(p2.x() - ref.x()));
			});
}

// ---------------------------------------------------------------------------------------------
// Copied from 
// https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_ros_pkgs/include/math_common.h
template <typename T>
inline T rad2deg(const T radians) {
	return (radians / M_PI) * 180.0;
}

template <typename T>
inline T deg2rad(const T degrees) {
	return (degrees / 180.0) * M_PI;
}

template <typename T>
inline T wrapToPi(T radians) {
	int m = (int)(radians / (2.0 * M_PI));
	radians -= 2.0 * m * M_PI;

	if (radians > M_PI)
		radians -= 2.0 * M_PI;

	else if (radians < -M_PI)
		radians += 2.0 * M_PI;

	return radians;
}

template <typename T>
inline void wrap_to_pi_inplace(T& a) {
	a = wrap_to_pi(a);
}
// ---------------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------------
// Copied from 
// https://stackoverflow.com/questions/9323903/most-efficient-elegant-way-to-clip-a-number
template <typename T>
inline T clip_inplace(const T& n, const T& lower, const T& upper) {
	return std::max(lower, std::min(n, upper));
}
// ---------------------------------------------------------------------------------------------

inline CentroidAlgorithm getAlgFromString(const std::string& s) {
	if (s.compare("geometric") == 0)
		return CentroidAlgorithm::GEOMETRIC;

	else if (s.compare("geodesic_approximate") == 0) 
		return CentroidAlgorithm::GEODESIC_APPROXIMATE;

	else if (s.compare("geodesic_exact") == 0) 
		return CentroidAlgorithm::GEODESIC_EXACT;

	else 
		return CentroidAlgorithm::UNKNOWN;
}

// ---------------------------------------------------------------------------------------------

namespace bg = boost::geometry;

typedef bg::model::point<double, 2, bg::cs::cartesian> point_t;
typedef bg::model::polygon<point_t> polygon_t;
typedef bg::model::box<point_t> box_t;

double getSegDistSq(const point_t& p, const point_t& a, const point_t& b) ;

double pointToPolygonDist(const point_t& point, const polygon_t& polygon);

struct Cell {
    Cell(const point_t& c_, double h_, const polygon_t& polygon)
        : c(c_), h(h_), d(pointToPolygonDist(c, polygon)), max(d + h * std::sqrt(2)) { }

    point_t c; // cell center
    double h; // half the cell size
    double d; // distance from cell center to polygon
    double max; // max distance to polygon within a cell
};

Cell getCentroidCell(const polygon_t& polygon);

point_t polylabel(const polygon_t& polygon, double precision = 1., bool debug = false);

#endif // DIST_COVERAGE_UTILS_H