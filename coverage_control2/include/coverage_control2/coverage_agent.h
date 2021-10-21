#ifndef DIST_COVERAGE_AGENT_H
#define DIST_COVERAGE_AGENT_H

#include "coverage_control2/coverage_utils.h"

# define nice(os) ((os == CGAL::ON_ORIENTED_BOUNDARY) ? "on boundary" :  \
                   (os == CGAL::POSITIVE) ? "inside" : "outside")

class Agent {
	public:
		Agent(ros::NodeHandle& nh_, const std::string& name_, uint8_t _id);
		~Agent();

		void set_motion_params(const MotionParameters& mp);
		void set_sensing_params(const SensingParameters& sp);
		void set_behaviour_settings(const BehaviourSettings& bs);
		void set_task_region_from_raw(Polygon_2& c_bounds, Polygon_2_Array& c_holes);

		bool ready();
		bool is_point_valid(const Point_2& p);
		bool is_point_valid(const Vector2d& p);

		void broadcast();
		double calculate_workload();
		double local_utility(const Vector2d& p);
		void select_goal_from_local_frontier(std::vector<UtilityPair>& frontier);

		void step();

	private:
		void visibility_polygon();
		void visibility_limited_voronoi(std::vector<BoundarySegment>& bisectors, 
										std::vector<UtilityPair>& outFrontier);

		void build_local_skeleton(std::vector<UtilityPair>& inFrontier, bool frontierFocus=false);
		void compute_geometric_centroid();
		double workload_utility(Point_2& p, std::vector<BoundarySegment>& bisectors);

		void get_voronoi_cell_raw(std::vector<BoundarySegment>& segments, 
								  uint8_t neighbour_count, 
								  ConstraintMatrix2d& A, 
								  VectorXd& b, 
								  std::vector<BoundarySegment>& outRelevantBisectors);

		void state_cb(const AgentStateMsg::ConstPtr& msg);

		bool handle_SetInitialPose(SetInitialPose::Request& req, SetInitialPose::Response& res);
		bool handle_SetReady(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool handle_DumpSkeleton(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

		ros::NodeHandle nh;
		ros::Publisher vpoly_pub;
		ros::Publisher cvx_vor_pub;
		ros::Publisher vl_voronoi_pub;
		ros::Publisher state_pub;
		ros::Publisher utility_debug_pub;
		ros::Subscriber state_sub;
		ros::ServiceServer initPose_service;
		ros::ServiceServer ready_service;
		ros::ServiceServer dump_skeleton_service;

		std::unordered_map<uint8_t, AgentState> neighbours;

		bool is_ready;
		uint8_t id;
		std::string name;
		Vector2d position;
		Vector2d velocity;
		Vector2d goal;
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

		Polygon_2 region_boundary;
		Polygon_2 inflated_outer_boundary;

		Polygon_2_Array region_holes;
		Polygon_2_Array inflated_region_holes;

		Polygon_with_holes_2 actual_region;
		Arrangement_2 environment_arr;

		std::vector<BoundarySegment> segments_to_avoid;
		std::vector<Segment_2> vis_segments;
		Polygon_2 current_cvx_voronoi;
		Polygon_2 current_visibility_poly;

		Polygon_with_holes_2 current_work_region;
		SsPtr current_skeleton;
		// SkeletalBoostGraph skeletal_map;
};

#endif // DIST_COVERAGE_AGENT_H