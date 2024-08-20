#ifndef DIST_COVERAGE_AGENT_H
#define DIST_COVERAGE_AGENT_H

#include "coverage_control2/coverage_utils.h"
#include <boost/functional/hash.hpp>

# define nice(os) ((os == CGAL::ON_ORIENTED_BOUNDARY) ? "on boundary" :  \
                   (os == CGAL::POSITIVE) ? "inside" : "outside")

// struct hash_pair {
// 	template <class T1, class T2>
// 	size_t operator()(const pair<T1, T2>& p) const {
// 		auto hash1 = std::hash<T1>{}(p.first);
// 		auto hash2 = std::hash<T2>{}(p.second);

// 		if (hash1 != hash2) {
// 			return hash1 ^ hash2;
// 		}

// 		// If hash1 == hash2, their XOR is zero.
// 		return hash1;
// 	}
// };

class Agent {
	public:
		Agent(ros::NodeHandle& nh_, const std::string& name_, uint8_t _id);
		~Agent();

		void set_motion_params(const MotionParameters& mp);
		void set_sensing_params(const SensingParameters& sp);
		void set_behaviour_settings(const BehaviourSettings& bs);
		void set_task_region_from_raw(Polygon_2& c_bounds, Polygon_2_Array& c_holes);

		uint8_t get_id();
		bool ready();
		bool is_alive();
		void set_alive(bool inAlive);
		bool is_point_valid(const Point_2& p);
		bool is_point_valid(const Vector2d& p);
		bool is_point_valid_compact(const Point_2& p);
		bool is_point_valid_compact(const Vector2d& p);

		void broadcast();
		double calculate_workload();
		void select_goal_from_local_frontier(std::vector<UtilityPair>& frontier);
		Vector2d calculate_geodesic_orientation(std::pair<double, double> v, BFSAgent& agent, double& d);

		void check_neighbours();
		void step();
		void control_step(uint32_t recusrion_count=0);
		void control_step(const Vector2d& force, uint32_t recursion_count=0);

		bool ready_for_geodesic_step();

	private:
		void visibility_polygon();
		void visibility_limited_voronoi(std::vector<BoundarySegment>& bisectors, 
										std::vector<UtilityPair>& outFrontier);

		void build_local_skeleton(std::vector<BoundarySegment>& bisectors, bool frontierFocus=false);
		void compute_geometric_centroid();
		Vector2d compute_center_of_mass(Polygon_2& poly, double& outArea);
		double workload_utility(Point_2& p, std::vector<BoundarySegment>& bisectors);
		double calculate_non_uniform_utility(Vector2d& major, double r, std::vector<Vector2d>& minor, 
											 std::vector<BoundarySegment>& bisectors);

		void get_voronoi_cell_raw(std::vector<BoundarySegment>& segments, 
								  uint8_t neighbour_count, 
								  ConstraintMatrix2d& A, 
								  VectorXd& b, 
								  std::vector<BoundarySegment>& outRelevantBisectors);

		BFSAgent geodesic_voronoi_partition_discrete(std::unordered_map<uint8_t, BFSAgent>& bfs_agents);

		void debug_cb(const std_msgs::Empty::ConstPtr& msg);
		void state_cb(const AgentStateMsg::ConstPtr& msg);

		bool handle_SetInitialPose(SetInitialPose::Request& req, SetInitialPose::Response& res);
		bool handle_SetReady(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool handle_DumpSkeleton(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool handle_SelfTermination(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

		ros::NodeHandle nh;
		ros::Publisher vpoly_pub;
		ros::Publisher cvx_vor_pub;
		ros::Publisher vl_voronoi_pub;
		ros::Publisher state_pub;
		ros::Publisher utility_debug_pub;
		ros::Publisher geodesic_partition_pub;
		ros::Subscriber debug_sub;
		ros::Subscriber state_sub;
		ros::ServiceServer initPose_service;
		ros::ServiceServer ready_service;
		ros::ServiceServer dump_skeleton_service;
		ros::ServiceServer self_termination_service;

		std::unordered_map<uint8_t, AgentState> neighbours;

		bool is_ready;
		bool self_dijkstra_state;
		bool debug_step;
		bool alive;
		uint8_t id;
		uint8_t process_steps;
		uint32_t iteration_count;
		int self_seq;
		std::string name;
		Vector2d position;
		Vector2d velocity;
		Vector2d goal;
		// std::deque<Vector2d> goal_history;
		Vector2d force_exerted_on_self;
		Vector2d target;
		double heading;
		double current_workload;
		double largest_workload;

		std::vector<double> angle_offsets;
		size_t n_offsets;

		MotionParameters m_params;
		SensingParameters s_params;
		BehaviourSettings b_settings;
		uint16_t sample_count;
		std::vector<MoveAction> valid_actions;

		Polygon_2 region_boundary;
		Polygon_2 region_boundary_bbox;
		Polygon_2 inflated_outer_boundary;
		Polygon_2 inflated_outer_boundary_bbox;

		Polygon_2_Array region_holes;
		Polygon_2_Array inflated_region_holes;

		Arrangement_2 environment_arr;
		CGAL::Arr_naive_point_location<Arrangement_2> environment_pl;

		Polygon_with_holes_2 actual_region;

		std::vector<BoundarySegment> segments_to_avoid;
		std::vector<Segment_2> vis_segments;
		std::vector<Segment_2> real_vis_segments;
		Polygon_2 current_cvx_voronoi;
		Polygon_2 current_visibility_poly;

		Polygon_with_holes_2 current_work_region;
		SsPtr current_skeleton;

		BFSAgent itself;
		std::vector<Vector2d> centroid_path;

		Bbox_2 actual_region_bbox;

		// std::vector<Point_2> global_metric_grid;
		// MGRTree global_metric_nn_rtree;
};

#endif // DIST_COVERAGE_AGENT_H