#pragma once

#ifndef DIST_COVERAGE_UTILS_H
#define DIST_COVERAGE_UTILS_H

#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <coverage_control2/Polygon.h>
#include <coverage_control2/HistoryStep.h>
#include <coverage_control2/PolygonWithHoles.h>
#include <coverage_control2/AgentState.h>
#include <coverage_control2/SetInitialPose.h>
#include <coverage_control2/UtilityDebug.h>
#include <coverage_control2/GeodesicPartition.h>

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cmath>
#include <cassert>
#include <chrono>
#include <map>
#include <typeinfo>
#include <list>
#include <vector>
#include <deque>
#include <limits>
#include <random>
#include <utility>
#include <algorithm>
#include <set>
#include <unordered_map>
#include <functional>

#include <Eigen/Dense>

// #include "coverage_control2/BFSAgentImproved.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "dump_to_eps.h"

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Point_2<K> Point_2;
typedef CGAL::Vector_2<K> Vector_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;
typedef Polygon_2::Vertex_iterator VertexIterator;
typedef Polygon_2::Edge_const_iterator EdgeIterator;
typedef Polygon_with_holes_2::Hole_iterator HoleIterator;

typedef K::Line_2 Line_2;
typedef K::Segment_2 Segment_2;
typedef CGAL::Bbox_2 Bbox_2;
typedef CGAL::Arr_segment_traits_2<K> Traits_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef Traits_2::Point_2 Arr_Point_2;
typedef Arrangement_2::Face_handle Face_handle;

struct FaceInfo2 {
	int nesting_level;

	FaceInfo2() {}
	bool in_domain() {
		return nesting_level % 2 == 1;
	}
};

typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K> Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> TDS;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
typedef CDT::Point CDT_Point;
typedef CDT::Face_handle CDT_Face_handle;

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
typedef std::pair<Vector2d, double> MoveAction;

typedef coverage_control2::AgentState AgentStateMsg;
typedef coverage_control2::SetInitialPose SetInitialPose;

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 2, bg::cs::cartesian> MGPoint;
typedef bg::model::box<MGPoint> MGBox;
typedef std::pair<MGPoint, unsigned> MGValue;
typedef bgi::rtree<MGValue, bgi::quadratic<16> > MGRTree;

typedef std::pair<std::pair<double, double>, uint8_t> DeletionUpdate;

enum CentroidAlgorithm {
	UNKNOWN_ALG=0,
	GEOMETRIC=1,
	GEODESIC_APPROXIMATE=2,
	GEODESIC_EXACT=3,
	FRONTIER_FOCUSED=4,
	GRID_BASED=5,
	CONTINUOUS_DIJKSTRA=6
};

enum CentroidalMetric {
	UNKNOWN_MTR=0,
	RADIUS_MIN_SUM=1,
	RADIUS_MIN_MAX=2,
	CLOSENESS=3,
	K_AVG_RADIUS=4,
	BETWEENNESS=5
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
	double sense_fp_phys_rad_scale;
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
	CentroidalMetric centroid_mtr;
	int k_avg_count;
	bool stationary;
	bool limited_sensing;
	bool flocking;
	bool planning;
	bool visibility;
	bool voronoi;
};

struct BoundarySegment {
	uint8_t mirror_id;
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
	bool contour;
	int id;
	Vector2d point;
	double weight;
};

struct BFSAgent {
	uint8_t id;
	uint32_t step_count;
	double step_size;
	double discrete_mass;
	Vector2d p, gp, gv;
	std::set<std::pair<double, double> > visited;
	std::set<std::pair<double, double> > frontier;
	std::map<uint8_t, std::set<std::pair<double, double> > > borders;
	std::map<std::pair<double, double>, std::pair<double, double> > parents;
	std::map<std::pair<double, double>, std::set<std::pair<double, double> > > edges;
	std::map<std::pair<double, double>, Vector2d> normals;
	std::map<std::pair<double, double>, uint32_t> step_counts;

	BFSAgent();
	BFSAgent(uint8_t _id, Vector2d& pos, Vector2d& gpos, double _step_size);

	bool is_root(std::pair<double, double>& q);

	void add_border_info(std::pair<double, double> border_vertex, uint8_t border_to);

	std::set<std::pair<double, double> > frontier_expand(std::vector<MoveAction>& actions, 
														 Polygon_with_holes_2& context);
};

// ---------------------------------------------------------------------------------------------
// Copied from 
// https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
template<typename T>
struct Vector2dHash : std::unary_function<T, size_t> {
	std::size_t operator()(T const& matrix) const {
		// Note that it is oblivious to the storage order of Eigen matrix (column- or
		// row-major). It will give you the same hash value for two different matrices if they
		// are the transpose of each other in different storage order.
		size_t seed = 0;

		for (size_t i = 0; i < matrix.size(); ++i) {
			auto elem = *(matrix.data() + i);
			seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}

		return seed;
	}
};
// ---------------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------------
// Copied from
// https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> 
inline int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
// ---------------------------------------------------------------------------------------------

struct Vector2dComp {
	bool operator() (const Vector2d& lhs, const Vector2d& rhs) const {
		if (lhs(0) != rhs(0))
			return lhs(0) < rhs(0);
		else {
			return lhs(1) < rhs(1);
		}
	}
};

inline double metric_rounding(double value, double factor) {
	return value - std::remainder(value, factor);
}

inline Vector2d real_2_grid(Vector2d real_pos, Bbox_2 bbox, double resolution) {
	return Vector2d(std::round((real_pos(0) - bbox.xmin()) / resolution), 
					std::round((real_pos(1) - bbox.ymin()) / resolution));
}

// -------------------------------------------------------------------------------------------------
inline Vector2d grid_2_real(std::pair<double, double> grid_pos, Bbox_2 bbox, double resolution) {
	return Vector2d(grid_pos.first * resolution + bbox.xmin(), 
					grid_pos.second * resolution + bbox.ymin());
}

inline Vector2d grid_2_real(Vector2d grid_pos, Bbox_2 bbox, double resolution) {
	return Vector2d(grid_pos(0) * resolution + bbox.xmin(), 
					grid_pos(1) * resolution + bbox.ymin());
}
// -------------------------------------------------------------------------------------------------

inline double rate_derivative_coefficient(double rate, double work) {
	return - (1. + log(rate)) * rate / work;
}

inline double saturation(double x) {
	return (x <= 0.) ? 0. : std::exp(- 1. / (9. * x * x));
}

void get_metric_graph(Polygon_with_holes_2& polygon, double resolution, 
					  std::vector<Point_2>& outPoints);

double a_star_search(Point_2& start, Point_2& goal, double step_size, 
					 Polygon_with_holes_2& environment, bool prune, bool debug=false, 
					 bool with_path=false, std::vector<Vector2d>* outPathR=nullptr);

void mark_domains(CDT& ct, Face_handle start, int index, std::list<CDT::Edge>& border);
void mark_domains(CDT& cdt);

void get_cdt_of_polygon_with_holes(Polygon_with_holes_2& pwh, CDT& outCdt);

class SkeletalGraph {
	public:
		SkeletalGraph(uint32_t c, uint32_t offset=0);

		int getVertexId(int vid, Point_2 p, double w, bool c);
		void addEdge(int vid1, Point_2 p1, double w1, bool c1, 
					 int vid2, Point_2 p2, double w2, bool c2);

		double getTotalWork();
		void refineEdges();
		Vector2d getLargestNode(std::vector<std::pair<double, size_t> >& outStats);
		Vector2d getCentroid(std::vector<std::pair<double, size_t> >& outStats, 
							 CentroidalMetric metric=CentroidalMetric::RADIUS_MIN_SUM, 
							 uint8_t k=2);

		uint32_t getCount();
		const std::unordered_map<int, SkeletalNode>& getVertexMap() const;
		Vector2d getVertexById(int id);
		std::vector<Point_2> getVerticesAsCgalPoints();
		std::vector<UtilityPair> getNextToVertexFrom(Vector2d& fromV, Vector2d& toV);
		void assignWeightToVertex(int vid, double w);
		Vector2d getNext(Vector2d& start, Vector2d& goal, double& outDist);
		std::vector<Vector2d> getPathToVertex(Vector2d& start, Vector2d& goal, bool debug=false);

	private:
		MatrixXd graph;
		MatrixXd paths;
		uint32_t count;
		double total_work;
		int next_id_available;
		std::unordered_map<int, int> id_map;
		std::unordered_map<int, SkeletalNode> vertex_map;
		std::map<std::pair<double, double>, int> rid_map;
};

inline void create_poly_from_raw(XmlRpc::XmlRpcValue& data, Polygon_2& out_poly, 
								 bool is_outer_boundary=false) {
	size_t v_count = data.size();
	for (size_t i = 0; i < v_count; i++) {
		out_poly.push_back(Point_2((double)(data[i][0]), (double)(data[i][1])));
	}

	if (is_outer_boundary && out_poly.is_clockwise_oriented()) 
		out_poly.reverse_orientation();
}

inline void create_offset_poly_naive(Polygon_2& in_poly, double l, Polygon_2& out_poly) {
	assert(out_poly.size() == 0);

	int nVertices = in_poly.size();
	for (int i = 0; i < nVertices; i++) {
		int j = (i - 1 + nVertices) % nVertices;
		int k = (i + 1) % nVertices;

		Vector_2 v1(in_poly[i], in_poly[k]); // forward
		Vector_2 v2(in_poly[i], in_poly[j]); // backward
		double coeff = CGAL::to_double(v1.x() * v2.y()) - CGAL::to_double(v2.x() * v1.y());

		Vector_2 d = v1 + v2;
		double norm = sqrt(pow(CGAL::to_double(d.x()), 2) + pow(CGAL::to_double(d.y()), 2));
		d /= norm;

		out_poly.push_back(in_poly[i] + l * d * sgn<double>(coeff));
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

inline void get_nearest_neighbours(Vector2d& q, double r, MGRTree& inTree, 
								   std::vector<Vector2d>& outValues) {
	MGPoint qpoint(q(0), q(1));
	MGBox qbox(MGPoint(q(0) - r, q(1) - r), MGPoint(q(0) + r, q(1) + r));

	std::vector<MGValue> neighbours;
	inTree.query(bgi::within(qbox) && 
				 bgi::satisfies([&](MGValue const& v) {
				 	return bg::distance(v.first, qpoint) < r;
				 }), 
				 std::back_inserter(neighbours));

	size_t nNeighbours = neighbours.size();
	for (size_t i = 0; i < nNeighbours; i++) {
		outValues.push_back(Vector2d(neighbours[i].first.get<0>(), neighbours[i].first.get<1>()));
	}
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

	else if (s.compare("frontier_focused") == 0) 
		return CentroidAlgorithm::FRONTIER_FOCUSED;

	else if (s.compare("grid_based") == 0) 
		return CentroidAlgorithm::GRID_BASED;

	else if (s.compare("continuous_dijkstra") == 0) 
		return CentroidAlgorithm::CONTINUOUS_DIJKSTRA;

	else 
		return CentroidAlgorithm::UNKNOWN_ALG;
}

inline CentroidalMetric getMtrFromString(const std::string& s) {
	if (s.compare("radius_min_sum") == 0)
		return CentroidalMetric::RADIUS_MIN_SUM;

	else if (s.compare("radius_min_max") == 0)
		return CentroidalMetric::RADIUS_MIN_MAX;

	else if (s.compare("closeness") == 0) 
		return CentroidalMetric::CLOSENESS;

	else if (s.compare("k_avg_radius") == 0) 
		return CentroidalMetric::K_AVG_RADIUS;

	else if (s.compare("betweenness") == 0) 
		return CentroidalMetric::BETWEENNESS;

	else 
		return CentroidalMetric::UNKNOWN_MTR;
}

#endif // DIST_COVERAGE_UTILS_H