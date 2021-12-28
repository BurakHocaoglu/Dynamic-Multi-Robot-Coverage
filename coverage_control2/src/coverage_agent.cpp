#include "coverage_control2/coverage_agent.h"

Agent::Agent(ros::NodeHandle& nh_, const std::string& name_, uint8_t id_) : 
	nh(nh_), 
	name(name_), 
	id(id_), 
	is_ready(false), 
	debug_step(false), 
	sample_count(30), 
	process_steps(0)
{
	position << 0.0, 0.0;
	velocity << 0.0, 0.0;
	goal << 0.0, 0.0;

	vpoly_pub = nh.advertise<coverage_control2::Polygon>("/visibility_polys", 1);
	cvx_vor_pub = nh.advertise<coverage_control2::Polygon>("/convex_voronoi", 1);
	vl_voronoi_pub = nh.advertise<coverage_control2::HistoryStep>("/visibility_limited_voronoi", 1);
	state_pub = nh.advertise<AgentStateMsg>("/states", 1);
	utility_debug_pub = nh.advertise<coverage_control2::UtilityDebug>("/" + name + "/utility_debug", 1);
	geodesic_partition_pub = nh.advertise<coverage_control2::GeodesicPartition>("/geodesic_partition", 1);
	debug_sub = nh.subscribe("/debug_continue", 1, &Agent::debug_cb, this);
	state_sub = nh.subscribe("/states", 10, &Agent::state_cb, this);
	initPose_service = nh.advertiseService("/" + name + "/set_initial_pose", &Agent::handle_SetInitialPose, this);
	ready_service = nh.advertiseService("/" + name + "/set_ready", &Agent::handle_SetReady, this);
	dump_skeleton_service = nh.advertiseService("/" + name + "/dump_skeleton", &Agent::handle_DumpSkeleton, this);
}

Agent::~Agent() { }

void Agent::set_motion_params(const MotionParameters& mp) {
	m_params = mp;
}

void Agent::set_sensing_params(const SensingParameters& sp) {
	s_params = sp;

	double fov_ang = - s_params.hfov_range / 2.;
	double fov_end = s_params.hfov_range / 2.;
	double angle_increment = s_params.hfov_range / sample_count;

	while (fov_ang <= fov_end) {
		angle_offsets.push_back(fov_ang);
		fov_ang += angle_increment;
	}

	n_offsets = angle_offsets.size();
}

void Agent::set_behaviour_settings(const BehaviourSettings& bs) {
	b_settings = bs;
}

void Agent::set_task_region_from_raw(Polygon_2& c_bounds, Polygon_2_Array& c_holes) {
	region_boundary = c_bounds;
	region_holes = c_holes;

	double metric_resolution = m_params.physical_radius * s_params.sense_fp_phys_rad_scale;

	if (b_settings.centroid_alg == CentroidAlgorithm::CONTINUOUS_DIJKSTRA) {
		create_offset_poly_naive(region_boundary, metric_resolution / 2., inflated_outer_boundary);
	} else {
		create_offset_poly_naive(region_boundary, -m_params.physical_radius, inflated_outer_boundary);
	}

	// double metric_resolution = m_params.physical_radius * s_params.sense_fp_phys_rad_scale;
	// create_offset_poly_naive(region_boundary, metric_resolution / 2., inflated_outer_boundary);
	// create_offset_poly_naive(region_boundary, -metric_resolution / 2., inflated_outer_boundary);

	K::FT outer_area, deflated_outer_area;
	CGAL::area_2(region_boundary.vertices_begin(), region_boundary.vertices_end(), outer_area, K());
	CGAL::area_2(inflated_outer_boundary.vertices_begin(), 
				 inflated_outer_boundary.vertices_end(), 
				 deflated_outer_area, K());

	if (id == 1) 
		ROS_INFO("Region boundary area: %.3f - Deflated: %.3f", CGAL::to_double(outer_area), 
																CGAL::to_double(deflated_outer_area));

	if (inflated_outer_boundary.is_clockwise_oriented()) 
		inflated_outer_boundary.reverse_orientation();

	Bbox_2 rb_box = region_boundary.bbox();
	region_boundary_bbox.push_back(Point_2(rb_box.xmin(), rb_box.ymin()));
	region_boundary_bbox.push_back(Point_2(rb_box.xmax(), rb_box.ymin()));
	region_boundary_bbox.push_back(Point_2(rb_box.xmax(), rb_box.ymax()));
	region_boundary_bbox.push_back(Point_2(rb_box.xmin(), rb_box.ymax()));

	Bbox_2 ob_box = inflated_outer_boundary.bbox();
	inflated_outer_boundary_bbox.push_back(Point_2(ob_box.xmin() - 1., ob_box.ymin() - 1.));
	inflated_outer_boundary_bbox.push_back(Point_2(ob_box.xmax() + 1., ob_box.ymin() - 1.));
	inflated_outer_boundary_bbox.push_back(Point_2(ob_box.xmax() + 1., ob_box.ymax() + 1.));
	inflated_outer_boundary_bbox.push_back(Point_2(ob_box.xmin() - 1., ob_box.ymax() + 1.));

	int rbnd_real_v_count = region_boundary.size();
	for (int i = 0; i < rbnd_real_v_count; i++) {
		int j = (i + 1) % rbnd_real_v_count;
		real_vis_segments.emplace_back(region_boundary[i], region_boundary[j]);
	}

	int rbnd_v_count = region_boundary_bbox.size();
	for (int i = 0; i < rbnd_v_count; i++) {
		int j = (i + 1) % rbnd_v_count;

		Vector2d normal(CGAL::to_double(region_boundary_bbox[i][1] - region_boundary_bbox[j][1]), 
						CGAL::to_double(region_boundary_bbox[j][0] - region_boundary_bbox[i][0]));

		Vector2d middle(CGAL::to_double(region_boundary_bbox[i][0] + region_boundary_bbox[j][0]), 
						CGAL::to_double(region_boundary_bbox[i][1] + region_boundary_bbox[j][1]));

		BoundarySegment seg = {0, normal, middle * 0.5};
		segments_to_avoid.push_back(seg);
		vis_segments.emplace_back(region_boundary_bbox[i], region_boundary_bbox[j]);
	}

	int h_count = region_holes.size();
	for (int i = 0; i < h_count; i++) {
		int hbnd_v_count = region_holes[i].size();

		for (int j = 0; j < hbnd_v_count; j++) {
			int k = (j + 1) % hbnd_v_count;
			real_vis_segments.emplace_back(region_holes[i][j], region_holes[i][k]);
			vis_segments.emplace_back(region_holes[i][j], region_holes[i][k]);
		}

		Polygon_2 inflated_region_hole_i;
		// create_offset_poly_naive(region_holes[i], metric_resolution / 2., inflated_region_hole_i);
		// create_offset_poly_naive(region_holes[i], -metric_resolution / 2., inflated_region_hole_i);

		if (b_settings.centroid_alg == CentroidAlgorithm::CONTINUOUS_DIJKSTRA) {
			create_offset_poly_naive(region_holes[i], -metric_resolution / 2., inflated_region_hole_i);
		} else {
			create_offset_poly_naive(region_holes[i], m_params.physical_radius, inflated_region_hole_i);
		}

		inflated_region_holes.push_back(inflated_region_hole_i);

		K::FT hole_area, inflated_hole_area;

		CGAL::area_2(region_holes[i].vertices_begin(), 
					 region_holes[i].vertices_end(), 
					 hole_area, K());

		CGAL::area_2(inflated_region_hole_i.vertices_begin(), 
					 inflated_region_hole_i.vertices_end(), 
					 inflated_hole_area, K());

		if (id == 1) 
			ROS_INFO("Hole area: %.3f - Inflated: %.3f", CGAL::to_double(hole_area), 
														 CGAL::to_double(inflated_hole_area));
	}

	for (int i = 0; i < h_count; i++) {
		if (!inflated_region_holes[i].is_clockwise_oriented()) 
			inflated_region_holes[i].reverse_orientation();
	}

	actual_region = Polygon_with_holes_2(inflated_outer_boundary, 
										 inflated_region_holes.begin(), 
										 inflated_region_holes.end());

	CGAL::insert_non_intersecting_curves(environment_arr, real_vis_segments.begin(), 
														real_vis_segments.end());

	environment_pl.attach(environment_arr);
	// environment_pl = new CGAL::Arr_naive_point_location<Arrangement_2>(environment_arr);

	std::cout << name << " - RB BBox: (" << rb_box.xmin() << ", " << rb_box.ymin() << ") - (" 
										 << rb_box.xmax() << ", " << rb_box.ymax() << ")\n";

	std::cout << name << " - OB BBox: (" << ob_box.xmin() << ", " << ob_box.ymin() << ") - (" 
										 << ob_box.xmax() << ", " << ob_box.ymax() << ")\n";

	Bbox_2 bbox = actual_region.outer_boundary().bbox();
	std::cout << name << " - BBox: (" << bbox.xmin() << ", " << bbox.ymin() << ") - (" 
									  << bbox.xmax() << ", " << bbox.ymax() << ")\n";

	// get_metric_graph(actual_region, m_params.physical_radius * 8., global_metric_grid);
	// get_metric_graph(actual_region, metric_resolution, global_metric_grid);

	valid_actions.emplace_back(Vector2d(-1., 0.), 1.);
	valid_actions.emplace_back(Vector2d(0., -1.), 1.);
	valid_actions.emplace_back(Vector2d(0., 1.), 1.);
	valid_actions.emplace_back(Vector2d(1., 0.), 1.);
}

bool Agent::ready() {
	return is_ready;
}

bool Agent::is_point_valid(const Point_2& p) {
	auto inside_check = CGAL::oriented_side(p, inflated_outer_boundary);
	if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) 
		return false;

	Polygon_2_Array::iterator itr = inflated_region_holes.begin();
	for (; itr != inflated_region_holes.end(); itr++) {
		inside_check = CGAL::oriented_side(p, *itr);
		if (inside_check == CGAL::ON_ORIENTED_BOUNDARY || inside_check == CGAL::POSITIVE) 
			return false;
	}

	return true;
}

bool Agent::is_point_valid(const Vector2d& p) {
	return is_point_valid(Point_2(p(0), p(1)));
}

bool Agent::is_point_valid_compact(const Point_2& p) {
	auto inside_check = CGAL::oriented_side(p, actual_region);
	if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) 
		return false;

	return true;
	// return inside_check == CGAL::ON_ORIENTED_BOUNDARY || inside_check == CGAL::POSITIVE;
}

bool Agent::is_point_valid_compact(const Vector2d& p) {
	return is_point_valid_compact(Point_2(p(0), p(1)));
}

void Agent::broadcast() {
	AgentStateMsg current_state;
	current_state.id = id;
	current_state.heading = heading;
	current_state.position.x = position(0);
	current_state.position.y = position(1);
	current_state.position.z = 0.;
	current_state.goal.x = goal(0);
	current_state.goal.y = goal(1);
	current_state.goal.z = 0.;
	current_state.velocity.x = velocity(0);
	current_state.velocity.y = velocity(1);
	current_state.velocity.z = 0.;
	current_state.largest_workload_piece = largest_workload;
	current_state.workload = current_workload;

	state_pub.publish(current_state);
}

double Agent::calculate_workload() {
	double negative_workload = 0.;
	K::FT outer_area, hole_area;

	CGAL::area_2(current_work_region.outer_boundary().vertices_begin(), 
				 current_work_region.outer_boundary().vertices_end(), 
				 outer_area, K());

	HoleIterator cwrh_itr = current_work_region.holes_begin();
	for (; cwrh_itr != current_work_region.holes_end(); cwrh_itr++) {
		CGAL::area_2(cwrh_itr->vertices_begin(), cwrh_itr->vertices_end(), hole_area, K());
		negative_workload += CGAL::to_double(hole_area);
	}

	return CGAL::to_double(outer_area) + negative_workload;
}

void Agent::select_goal_from_local_frontier(std::vector<UtilityPair>& frontier) {
	if (frontier.empty()) {
		ROS_WARN("%s has no valid frontier vertices!", name.c_str());
		return;
	}

	std::vector<UtilityPair>::iterator min_elem_itr = std::min_element(frontier.begin(), frontier.end(), 
														[&](UtilityPair p1, UtilityPair p2){
															return p1.first < p2.first;
														});
	goal = min_elem_itr->second;
}

Vector2d Agent::calculate_geodesic_orientation(std::pair<double, double> v, BFSAgent& agent) {
	bool geodesically_found = false;
	auto geodesic_visible = CGAL::oriented_side(Point_2(v.first, v.second), current_visibility_poly);
	std::pair<double, double> subgoal_key = std::make_pair(v.first, v.second);

	while (true) {
		geodesic_visible = CGAL::oriented_side(Point_2(v.first, v.second), current_visibility_poly);

		// if (geodesic_visible == CGAL::ON_ORIENTED_BOUNDARY || 
		// 	geodesic_visible == CGAL::POSITIVE) {
		if (geodesic_visible == CGAL::POSITIVE) {
			geodesically_found = true;
			break;
		}

		if (agent.is_root(v)) {
			geodesically_found = false;
			break;
		}

		subgoal_key = v;
		v = agent.parents[v];
	}

	if (geodesically_found) {
		// Direct
		// geodesic_orientation = Vector2d(subgoal_key.first - position(0), 
		// 								subgoal_key.second - position(1));

		// Visibility-based
		return Vector2d(v.first - position(0), v.second - position(1));
	} else {
		ROS_WARN("%s - could not geodesically find it's target.", name.c_str());
		// Direct
		// geodesic_orientation = Vector2d(v_temp.first - position(0), 
		// 								v_temp.second - position(1));

		// Visibility-based
		return Vector2d(subgoal_key.first - position(0), subgoal_key.second - position(1));
	}
}

void Agent::step() {
	visibility_polygon();

	if (b_settings.centroid_alg == CentroidAlgorithm::CONTINUOUS_DIJKSTRA) {
		std::unordered_map<uint8_t, BFSAgent> bfs_agents;
		BFSAgent self = geodesic_voronoi_partition_discrete(bfs_agents);
		return;
	}

	Vector2d total_force(0, 0);
	int agents_to_consider = 0;
	std::vector<double> values;
	std::vector<Vector2d> constraints;
	std::vector<BoundarySegment> neighbour_segments;

	// visibility_polygon();

	std::unordered_map<uint8_t, AgentState>::iterator itr = neighbours.begin();
	for (; itr != neighbours.end(); itr++) {
		if (itr->first == id) 
			continue;

		Vector2d r_ij = itr->second.position - position;
		Vector2d m_ij = (position + itr->second.position) * 0.5;
		double norm = r_ij.norm();

		if (b_settings.limited_sensing) {
			if (norm <= s_params.sense_radius) {
				// total_force += m_params.K_repulsion * -r_ij * m_params.repulsion_const / pow(norm - 2. * m_params.safe_radius, 2);
				// total_force += m_params.K_attraction * r_ij * m_params.attraction_const / norm;

				BoundarySegment n_seg = {itr->first, r_ij, m_ij};
				neighbour_segments.push_back(n_seg);

				constraints.push_back(r_ij);
				values.push_back(r_ij.dot(m_ij) - m_params.physical_radius);
				agents_to_consider++;
			}
		} else {
			// total_force += m_params.K_repulsion * -r_ij * m_params.repulsion_const / pow(norm - 2. * m_params.safe_radius, 2);
			// total_force += m_params.K_attraction * r_ij * m_params.attraction_const / norm;

			double total_workload = itr->second.workload + current_workload;
			double workload_diff = itr->second.workload - current_workload;

			if (itr->second.workload > current_workload) {
				// total_force += m_params.K_attraction * r_ij * m_params.attraction_const / norm;

				total_force -= (workload_diff / total_workload) * m_params.K_attraction * r_ij * 
								m_params.attraction_const / norm;
			} else if (itr->second.workload < current_workload) {
				// total_force -= m_params.K_attraction * r_ij * m_params.attraction_const / norm;
				// total_force -= m_params.K_repulsion * r_ij * m_params.repulsion_const / norm;

				total_force += (workload_diff / total_workload) * m_params.K_repulsion * r_ij * 
								m_params.repulsion_const / norm;
			}

			BoundarySegment n_seg = {itr->first, r_ij, m_ij};
			neighbour_segments.push_back(n_seg);

			constraints.push_back(r_ij);
			values.push_back(r_ij.dot(m_ij) - m_params.physical_radius);
			agents_to_consider++;
		}
	}

	VectorXd b(agents_to_consider);
	ConstraintMatrix2d A(agents_to_consider, 2);

	for (int i = 0; i < agents_to_consider; i++) {
		b(i) = values[i];

		for (int j = 0; j < 2; j++) {
			A(i, j) = constraints[i](j);
		}
	}

	std::vector<BoundarySegment> relevantBisectors;
	std::vector<UtilityPair> theFrontier;

	get_voronoi_cell_raw(neighbour_segments, agents_to_consider, A, b, relevantBisectors);

	visibility_limited_voronoi(relevantBisectors, theFrontier);
	bool frontier_focused = false;

	if (b_settings.centroid_alg == CentroidAlgorithm::UNKNOWN) {
		b_settings.stationary = true;
		ROS_WARN("%s - does not know a valid centroid algorithm!", name.c_str());
	} else {
		if (b_settings.centroid_alg == CentroidAlgorithm::GEOMETRIC) {
			compute_geometric_centroid();
		} else if (b_settings.centroid_alg == CentroidAlgorithm::GEODESIC_APPROXIMATE) {
			build_local_skeleton(relevantBisectors);
		} else if (b_settings.centroid_alg == CentroidAlgorithm::GEODESIC_EXACT) {
			ROS_WARN("%s - does not have a valid exact geodesic centroid algorithm!", name.c_str());
		} else if (b_settings.centroid_alg == CentroidAlgorithm::FRONTIER_FOCUSED) {
			frontier_focused = true;
			build_local_skeleton(relevantBisectors, frontier_focused);
		} else if (b_settings.centroid_alg == CentroidAlgorithm::GRID_BASED) {
			// Now what?
		}
	}

	force_exerted_on_self = total_force;
}

void Agent::control_step() {
	control_step(force_exerted_on_self);
}

void Agent::control_step(Vector2d& force) {
	double mag = force.norm();
	ROS_INFO("%s - Force magnitude: %.5f", name.c_str(), mag);

	// double ang_diff = wrapToPi(atan2(force[1], force[0]) - heading);
	// double u = m_params.K_linear * mag * cos(ang_diff);
	// double w = m_params.K_angular * mag * sin(ang_diff);

	// if (fabs(w) > m_params.wmax) 
	// 	w = copysign(m_params.wmax, w);

	if (b_settings.stationary) 
		return;

	if (mag <= 1e-1) 
		return;

	// double u = mag;

	// clip_inplace(u, 0., m_params.umax);

	double u = std::min({mag, m_params.umax});

	Vector2d hdgVector(cos(heading), sin(heading));
	// velocity = u * hdgVector;
	velocity = u * force / mag;

	// ************
	goal = position + velocity * m_params.delta_t;
	if (!is_point_valid_compact(goal)) {
		ROS_WARN("%s - would move towards an invalid goal!", name.c_str());
		return;
	}
	// ************

	position += velocity * m_params.delta_t;
	// position += velocity * 0.2;
	// heading = wrapToPi(heading + w * m_params.delta_t);
	heading = atan2(velocity(1), velocity(0));

	// if (std::fabs(ang_diff) <= m_params.wmax)
	// 	position += velocity * 0.1;

	// heading = wrapToPi(heading + w * 0.1);
}

void Agent::visibility_polygon() {
	Polygon_2 new_visibility_poly;
	Arrangement_2 local_env_context, current_visibility;
	CGAL::insert_non_intersecting_curves(local_env_context, real_vis_segments.begin(), 
															real_vis_segments.end());

	Point_2 query(position(0), position(1));
	CGAL::Arr_naive_point_location<Arrangement_2> pl(local_env_context);
	CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(query);

	TEV tev(local_env_context);
	Face_handle fh;

	if (obj.which() == 0) {
		return;
	} else if (obj.which() == 1) {
		fh = tev.compute_visibility(query, *(boost::get<Halfedge_const_handle>(&obj)), current_visibility);
	} else if (obj.which() == 2) {
		fh = tev.compute_visibility(query, *(boost::get<Face_const_handle>(&obj)), current_visibility);
	}

	coverage_control2::Polygon current_vpoly;
	current_vpoly.id = id;

	Arrangement_2::Ccb_halfedge_circulator curr = fh->outer_ccb();
	Point_2 vpoly_vertex = curr->source()->point();

	new_visibility_poly.push_back(vpoly_vertex);

	geometry_msgs::Point vpoly_point;
	vpoly_point.z = 0.;
	vpoly_point.x = CGAL::to_double(vpoly_vertex.x());
	vpoly_point.y = CGAL::to_double(vpoly_vertex.y());

	current_vpoly.points.push_back(vpoly_point);

	while (++curr != fh->outer_ccb()) {
		vpoly_vertex = curr->source()->point();
		new_visibility_poly.push_back(vpoly_vertex);

		vpoly_point.x = CGAL::to_double(vpoly_vertex.x());
		vpoly_point.y = CGAL::to_double(vpoly_vertex.y());
		current_vpoly.points.push_back(vpoly_point);
	}

	vpoly_pub.publish(current_vpoly);
	current_visibility_poly = new_visibility_poly;
}

void Agent::visibility_limited_voronoi(std::vector<BoundarySegment>& bisectors, 
									   std::vector<UtilityPair>& outFrontier) {
	std::list<Polygon_with_holes_2> intersection_pieces;

	// CGAL::intersection(current_visibility_poly, 
	// 				   current_cvx_voronoi, 
	// 				   std::back_inserter(intersection_pieces));

	CGAL::intersection(actual_region, current_cvx_voronoi, 
					   std::back_inserter(intersection_pieces));

	int n_pieces = intersection_pieces.size();

	if (n_pieces == 0) {
		coverage_control2::HistoryStep hist_step;
		hist_step.id = id;
		hist_step.position.x = position(0);
		hist_step.position.y = position(1);

		VertexIterator v_itr = current_cvx_voronoi.vertices_begin();
		for (; v_itr != current_cvx_voronoi.vertices_end(); v_itr++) {
			geometry_msgs::Point vor_point;
			vor_point.x = CGAL::to_double(v_itr->x());
			vor_point.y = CGAL::to_double(v_itr->y());
			hist_step.cell.outer_boundary.points.push_back(vor_point);
		}

		vl_voronoi_pub.publish(hist_step);

		ROS_WARN("%s has no visibility and convex voronoi intersection! Sending voronoi...", name.c_str());
	} else {
		bool located = false;
		Polygon_with_holes_2 intr_piece;
		Point_2 position_cgal(position(0), position(1));

		if (n_pieces > 1) {
			std::list<Polygon_with_holes_2>::iterator pwh_itr = intersection_pieces.begin();
			for (; pwh_itr != intersection_pieces.end(); pwh_itr++) {
				auto inside_check = CGAL::oriented_side(position_cgal, *pwh_itr);
				if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) 
					continue;

				else {
					intr_piece = *pwh_itr;
					located = true;
					break;
				}
			}

			if (!located) {
				intr_piece = intersection_pieces.front();
				ROS_WARN("%s - could not be located in one of the intersections!", name.c_str());
			}
		} else {
			intr_piece = intersection_pieces.front();
		}

		coverage_control2::HistoryStep hist_step;
		hist_step.id = id;
		hist_step.position.x = position(0);
		hist_step.position.y = position(1);

		VertexIterator v_itr = intr_piece.outer_boundary().vertices_begin();
		for (; v_itr != intr_piece.outer_boundary().vertices_end(); v_itr++) {
			geometry_msgs::Point vlvp_point;
			vlvp_point.x = CGAL::to_double(v_itr->x());
			vlvp_point.y = CGAL::to_double(v_itr->y());
			hist_step.cell.outer_boundary.points.push_back(vlvp_point);
		}

		HoleIterator h_itr = intr_piece.holes_begin();
		for (; h_itr != intr_piece.holes_end(); h_itr++) {
			coverage_control2::Polygon hole_i;
			hole_i.id = id;

			VertexIterator hv_itr = h_itr->vertices_begin();
			for (; hv_itr != h_itr->vertices_end(); hv_itr++) {
				geometry_msgs::Point hole_point;
				hole_point.x = CGAL::to_double(hv_itr->x());
				hole_point.y = CGAL::to_double(hv_itr->y());
				hole_i.points.push_back(hole_point);
			}

			hist_step.cell.holes.push_back(hole_i);
		}

		vl_voronoi_pub.publish(hist_step);
		current_work_region = intr_piece;
	}
}

void Agent::build_local_skeleton(std::vector<BoundarySegment>& inBisectors, bool frontierFocus) {
	if (current_work_region.outer_boundary().is_clockwise_oriented()) {
		ROS_WARN("%s - Work region is not CCW oriented!", name.c_str());
		// current_work_region.outer_boundary().reverse_orientation();
		goal = position;
		return;
	}

	current_skeleton = CGAL::create_interior_straight_skeleton_2(current_work_region, K());
	if (current_skeleton->size_of_vertices() < 3) {
		ROS_WARN("%s - Too low skeletal node count!!", name.c_str());
		goal = position;
		return;
	}

	// ----------------------------------------------------------------------------------------------
	// Metric graph construction
	MGRTree metric_nn_rtree;
	std::vector<Point_2> metric_graph_points;

	if (frontierFocus) {
		get_metric_graph(current_work_region, 
						 m_params.physical_radius * s_params.sense_fp_phys_rad_scale, 
						 metric_graph_points);

		size_t i = 0;
		for (Point_2& mgp : metric_graph_points) {
			metric_nn_rtree.insert(std::make_pair(MGPoint(CGAL::to_double(mgp.x()), 
														  CGAL::to_double(mgp.y())), i));
		}
	}
	// ----------------------------------------------------------------------------------------------

	SkeletalGraph skeletal_map(current_skeleton->size_of_vertices());
	std::unordered_map<int, bool> seen_edges;

	double total_workload = 0.;
	total_workload += current_workload;

	std::unordered_map<uint8_t, AgentState>::iterator a_itr = neighbours.begin();
	for (; a_itr != neighbours.end(); a_itr++) {
		total_workload += a_itr->second.workload;
	}

	auto htr = current_skeleton->halfedges_begin();
	for (; htr != current_skeleton->halfedges_end(); htr++) {
		auto shtr = seen_edges.find(htr->id());
		if (shtr != seen_edges.end()) 
			continue;

		if (!htr->is_bisector()) 
			continue;

		int vid1 = htr->vertex()->id();
		int vid2 = htr->opposite()->vertex()->id();

		Point_2 p1 = htr->vertex()->point();
		Point_2 p2 = htr->opposite()->vertex()->point();

		double u1 = 0., u2 = 0.;
		if (!frontierFocus) {
			u1 = workload_utility(p1, inBisectors) / total_workload;
			u2 = workload_utility(p2, inBisectors) / total_workload;
		} else {
			u1 = CGAL::to_double(htr->vertex()->time());
			u2 = CGAL::to_double(htr->opposite()->vertex()->time());
		}

		bool c1 = htr->vertex()->is_contour();
		bool c2 = htr->opposite()->vertex()->is_contour();

		if (c1 && c2) 
			ROS_WARN("%s - Contour halfedge detected. This should not be executed!", name.c_str());

		skeletal_map.addEdge(vid1, htr->vertex()->point(), u1, c1, 
							 vid2, htr->opposite()->vertex()->point(), u2, c2);

		seen_edges[htr->id()] = true;
		seen_edges[htr->opposite()->id()] = true;
	}

	if (frontierFocus) {
		std::vector<UtilityPair> super_nodes;

		for (auto& entry : skeletal_map.getVertexMap()) {

			if (entry.second.contour)
				continue;

			Vector2d node = entry.second.point;
			std::vector<Vector2d> local_mg_points;

			get_nearest_neighbours(node, entry.second.weight, metric_nn_rtree, local_mg_points);
			super_nodes.emplace_back(calculate_non_uniform_utility(node, entry.second.weight, 
																   local_mg_points, 
																   inBisectors), node);
		}

		std::vector<UtilityPair>::iterator t_itr = std::max_element(super_nodes.begin(), super_nodes.end(), 
																	[&](UtilityPair p1, UtilityPair p2){
																		return p1.first < p2.first;
																	});

		target = t_itr->second;
		current_workload = calculate_workload();
		largest_workload = t_itr->first;


		// How about A* here ???


		std::vector<UtilityPair> heuristics = skeletal_map.getNextToVertexFrom(position, target);

		bool candidate_found = false;
		std::vector<UtilityPair>::iterator g_itr = heuristics.begin();
		for (; g_itr != heuristics.end(); g_itr++) {
			auto inside_check = CGAL::oriented_side(Point_2(g_itr->second(0), g_itr->second(1)), 
													current_visibility_poly);

			if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) 
				continue;
			else {
				candidate_found = true;
				break;
			}
		}

		if (!candidate_found) {
			ROS_WARN("%s - Could not find a suitable goal candidate!", name.c_str());
			goal = position;
		} else {
			goal = g_itr->second;
			if (goal_history.size() == 5)
				goal_history.pop_front();

			goal_history.push_back(goal);
		}

	} else {
		// skeletal_map.refineEdges();

		std::vector<std::pair<double, size_t> > stats(skeletal_map.getCount());
		target = skeletal_map.getCentroid(stats, CentroidalMetric::K_AVG_RADIUS, 2);
		// target = skeletal_map.getLargestNode(stats);

		current_workload = calculate_workload();
		largest_workload = stats[0].first;
		// largest_workload = 0.; // Wrong, but does not matter for now...

		for (size_t j = 0; j < skeletal_map.getCount(); j++) {
			goal = skeletal_map.getVertexById(stats[j].second);

			Point_2 cgal_goal_candidate(goal(0), goal(1));
			auto inside_check = CGAL::oriented_side(cgal_goal_candidate, current_visibility_poly);
			if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) 
				continue;

			else 
				break;
		}
	}
}

void Agent::compute_geometric_centroid() {
	int count = 0;
	Vector2d cm_total(0., 0.);
	double outer_area;
	Vector2d outerCm = compute_center_of_mass(current_work_region.outer_boundary(), outer_area);
	cm_total += outerCm * outer_area;
	current_workload = outer_area;

	HoleIterator cwrh_itr = current_work_region.holes_begin();
	for (; cwrh_itr != current_work_region.holes_end(); cwrh_itr++) {
		double hole_area;
		Vector2d holeCm = compute_center_of_mass(*cwrh_itr, hole_area);
		cm_total += holeCm * hole_area;
		current_workload += hole_area;
	}

	goal = cm_total / current_workload;
}

Vector2d Agent::compute_center_of_mass(Polygon_2& poly, double& outArea) {
	double pcx = 0., pcy = 0.;
	outArea = CGAL::to_double(poly.area());

	size_t nVertices = poly.size();
	for (size_t i = 0; i < nVertices; i++) {
		size_t j = (i + 1) % nVertices;

		Point_2 p1 = poly.vertex(i);
		Point_2 p2 = poly.vertex(j);

		pcx += (CGAL::to_double(p1.x()) + CGAL::to_double(p2.x())) * 
		        (CGAL::to_double(p1.x()) * CGAL::to_double(p2.y()) - 
		         CGAL::to_double(p2.x()) * CGAL::to_double(p1.y()));

		pcy += (CGAL::to_double(p1.y()) + CGAL::to_double(p2.y())) * 
		        (CGAL::to_double(p1.x()) * CGAL::to_double(p2.y()) - 
		         CGAL::to_double(p2.x()) * CGAL::to_double(p1.y()));
	}

	pcx /= 6.0 * outArea;
	pcy /= 6.0 * outArea;

	return Vector2d(pcx, pcy);
}

double Agent::workload_utility(Point_2& p, std::vector<BoundarySegment>& bisectors) {
	double utility = 0.;
	Vector2d fv(CGAL::to_double(p.x()), CGAL::to_double(p.y()));

	std::vector<BoundarySegment>::iterator b_itr = bisectors.begin();
	for (; b_itr != bisectors.end(); b_itr++) {
		double d_u = (fv - b_itr->middle).dot(b_itr->normal) / b_itr->normal.norm();

		if (std::fabs(d_u) <= m_params.physical_radius) {
			utility += neighbours[b_itr->mirror_id].workload - current_workload;
		}
	}

	return utility;
}

double Agent::calculate_non_uniform_utility(Vector2d& major, double r, std::vector<Vector2d>& minor, 
											std::vector<BoundarySegment>& bisectors) {
	double total_importance = 0.;
	size_t nMinors = minor.size();
	std::vector<BoundarySegment>::iterator b_itr;

	for (size_t i = 0; i < nMinors; i++) {
		double minor_importance = 0.;

		b_itr = bisectors.begin();
		for (; b_itr != bisectors.end(); b_itr++) {
			double d_b = std::fabs((minor[i] - b_itr->middle).dot(b_itr->normal) / b_itr->normal.norm());

			if (d_b <= r * 2.)
				minor_importance += (neighbours[b_itr->mirror_id].workload - current_workload) / d_b;
		}

		total_importance += minor_importance;
	}

	return total_importance;
}

void Agent::get_voronoi_cell_raw(std::vector<BoundarySegment>& segments, 
								 uint8_t neighbour_count, 
								 ConstraintMatrix2d& A, 
								 VectorXd& b, 
								 std::vector<BoundarySegment>& outRelevantBisectors) {

	std::unordered_map<int, bool> relevantSegmentIndices;
	std::vector<Point_2> cvx_voronoi_vertices;
	int n_segments = segments.size();

	// Voronoi bisectors vs. Voronoi bisectors
	for (int i = 0; i < n_segments - 1; i++) {
		double d_i = segments[i].self_product();

		for (int j = i + 1; j < n_segments; j++) {
			double d_j = segments[j].self_product();

			ConstraintMatrix2d A_intr(2, 2);
			VectorXd b_intr(2);

			A_intr(0, 0) = segments[i].normal(0);
			A_intr(0, 1) = segments[i].normal(1);
			A_intr(1, 0) = segments[j].normal(0);
			A_intr(1, 1) = segments[j].normal(1);
			b_intr(0) = d_i;
			b_intr(1) = d_j;

			Vector2d p_intr = A_intr.colPivHouseholderQr().solve(b_intr);
			Point_2 cgal_point(p_intr(0), p_intr(1));

			auto inside_check = CGAL::oriented_side(cgal_point, inflated_outer_boundary_bbox);
			if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) 
				continue;

			if (neighbour_count > 0) {
				VectorXd feasibility(neighbour_count);
				feasibility = A * p_intr - b;

				if ((feasibility.array() < 1.).all()) {
					cvx_voronoi_vertices.emplace_back(p_intr(0), p_intr(1));
					relevantSegmentIndices[i] = true;
					relevantSegmentIndices[j] = true;
				}
			}
		}
	}

	for (std::pair<int, bool> entry : relevantSegmentIndices) {
		if (!entry.second) {
			ROS_WARN("%s - How did it get false!?", name.c_str());
			continue;
		}

		outRelevantBisectors.push_back(segments[entry.first]);
	}

	// Voronoi bisectors vs. Boundary segments
	size_t n_seg_avoid = segments_to_avoid.size();
	for (int i = 0; i < n_segments; i++) {
		double d_i = segments[i].self_product();

		for (int j = 0; j < n_seg_avoid; j++) {
			double d_j = segments_to_avoid[j].self_product();

			ConstraintMatrix2d A_intr(2, 2);
			VectorXd b_intr(2);

			A_intr(0, 0) = segments[i].normal(0);
			A_intr(0, 1) = segments[i].normal(1);
			A_intr(1, 0) = segments_to_avoid[j].normal(0);
			A_intr(1, 1) = segments_to_avoid[j].normal(1);
			b_intr(0) = d_i;
			b_intr(1) = d_j;

			Vector2d p_intr = A_intr.colPivHouseholderQr().solve(b_intr);
			Point_2 cgal_point(p_intr(0), p_intr(1));

			auto inside_check = CGAL::oriented_side(cgal_point, inflated_outer_boundary_bbox);
			if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) 
				continue;

			if (neighbour_count > 0) {
				VectorXd feasibility(neighbour_count);
				feasibility = A * p_intr - b;

				if ((feasibility.array() < 1.).all()) {
					cvx_voronoi_vertices.emplace_back(p_intr(0), p_intr(1));
				}
			}
		}
	}

	// Outer boundary vertices
	VertexIterator bnd_v_itr = region_boundary_bbox.vertices_begin();
	for (; bnd_v_itr != region_boundary_bbox.vertices_end(); bnd_v_itr++) {
		Vector2d vertex(CGAL::to_double(bnd_v_itr->x()), 
						CGAL::to_double(bnd_v_itr->y()));

		if (neighbour_count > 0) {
			VectorXd feasibility(neighbour_count);
			feasibility = A * vertex - b;

			if ((feasibility.array() < 1.).all()) {
				cvx_voronoi_vertices.emplace_back(vertex(0), vertex(1));
			}
		} else {
			cvx_voronoi_vertices.emplace_back(vertex(0), vertex(1));
		}
	}

	std::vector<Point_2> pruned;
	size_t n_intersections = cvx_voronoi_vertices.size();
	for (int i = 0; i < n_intersections; i++) {
		bool valid = true;

		size_t n_pruned = pruned.size();
		for (int j = 0; j < n_pruned; j++) {
			Vector_2 cgal_vec(cvx_voronoi_vertices[i], pruned[j]);
			double norm = sqrt(pow(CGAL::to_double(cgal_vec.x()), 2) + 
							   pow(CGAL::to_double(cgal_vec.y()), 2));

			if (norm <= m_params.physical_radius) {
				valid = false;
				break;
			}
		}

		if (valid)
			pruned.push_back(cvx_voronoi_vertices[i]);
	}

	Point_2 pos(position(0), position(1));

	if (cvx_voronoi_vertices.size() > 2) {
		angular_sort(cvx_voronoi_vertices, pos);
		current_cvx_voronoi = Polygon_2(cvx_voronoi_vertices.begin(), cvx_voronoi_vertices.end());

		if (current_cvx_voronoi.is_clockwise_oriented()) {
			current_cvx_voronoi.reverse_orientation();
		}

		if (!current_cvx_voronoi.is_simple()) {
			ROS_WARN("%s - CVX Voronoi is somehow (!) not simple!", name.c_str());
		}

		coverage_control2::Polygon current_vor_poly;
		current_vor_poly.id = id;

		VertexIterator v_itr = current_cvx_voronoi.vertices_begin();
		for (; v_itr != current_cvx_voronoi.vertices_end(); v_itr++) {
			geometry_msgs::Point vor_point;
			vor_point.x = CGAL::to_double(v_itr->x());
			vor_point.y = CGAL::to_double(v_itr->y());
			current_vor_poly.points.push_back(vor_point);
		}

		cvx_vor_pub.publish(current_vor_poly);
	} else {
		ROS_WARN("%s - Not enough valid voronoi vertices!", name.c_str());
	}
}

BFSAgent Agent::geodesic_voronoi_partition_discrete(std::unordered_map<uint8_t, BFSAgent>& bfs_agents) {
	std::map<std::pair<double, double>, uint8_t> metric_assignment;

	double metric_resolution = m_params.physical_radius * s_params.sense_fp_phys_rad_scale;

	for (std::pair<uint8_t, AgentState> nb_entry : neighbours) {
		Vector2d nb_grid_pos(metric_rounding(nb_entry.second.position(0), metric_resolution), 
							 metric_rounding(nb_entry.second.position(1), metric_resolution));

		bfs_agents[nb_entry.first] = BFSAgent(nb_entry.first, 
											  nb_entry.second.position, 
											  nb_grid_pos, 
											  metric_resolution);
	}

	Vector2d self_grid_pos(metric_rounding(position(0), metric_resolution), 
						   metric_rounding(position(1), metric_resolution));

	bfs_agents[id] = BFSAgent(id, position, self_grid_pos, metric_resolution);

	auto check_emptiness = [&bfs_agents] () {
		for (auto& entry : bfs_agents) {
			if (entry.second.frontier.size() > 0) 
				return false;
		}

		return true;
	};

	size_t nAgents = bfs_agents.size();
	while (!check_emptiness()) {
		std::set<DeletionUpdate> deletion_updates;

		for (auto& entry : bfs_agents) {
			std::set<std::pair<double, double> > expansions = entry.second.frontier_expand(valid_actions, actual_region);
			std::set<std::pair<double, double> >::iterator e_itr = expansions.begin();
			for (; e_itr != expansions.end(); e_itr++) {
				std::map<std::pair<double, double>, uint8_t>::iterator ma_itr = metric_assignment.find(*e_itr);

				if (ma_itr == metric_assignment.end()) {
					metric_assignment[*e_itr] = entry.first;
				} else {
					deletion_updates.insert(std::make_pair(ma_itr->first, entry.first));

					std::pair<double, double> parent_of_deletion = entry.second.parents[*e_itr];

					Vector2d outward_normal(e_itr->first - parent_of_deletion.first, 
											e_itr->second - parent_of_deletion.second);

					entry.second.add_border_info(parent_of_deletion, ma_itr->second);
					entry.second.normals[parent_of_deletion] = outward_normal;

					bfs_agents[ma_itr->second].add_border_info(*e_itr, entry.first);
					bfs_agents[ma_itr->second].normals[*e_itr] = - outward_normal;
				}
			}
		}

		for (size_t i = 1; i < nAgents; i++) {
			for (size_t j = i + 1; j <= nAgents; j++) {
				std::set<std::pair<double, double> > frontier_intr;

				std::set_intersection(bfs_agents[i].frontier.begin(), bfs_agents[i].frontier.end(), 
									  bfs_agents[j].frontier.begin(), bfs_agents[j].frontier.end(), 
									  std::inserter(frontier_intr, frontier_intr.begin()));

				std::set<std::pair<double, double> >::iterator mg_itr = frontier_intr.begin();
				for (; mg_itr != frontier_intr.end(); mg_itr++) {
					deletion_updates.insert(std::make_pair(*mg_itr, i));
					deletion_updates.insert(std::make_pair(*mg_itr, j));
				}
			}
		}

		std::set<DeletionUpdate>::iterator del_itr = deletion_updates.begin();
		for (; del_itr != deletion_updates.end(); del_itr++) {
			bfs_agents[del_itr->second].frontier.erase(del_itr->first);
		}
	}

	int mass_discrete = 0, global_mass_to_share = metric_assignment.size();
	coverage_control2::GeodesicPartition partition_msg;
	partition_msg.id = id;

	BFSAgent self = bfs_agents[id];

	for (std::pair<std::pair<double, double>, uint8_t> entry : metric_assignment) {
		if (entry.second == id) {
			mass_discrete++;
			partition_msg.xcoords.push_back(entry.first.first);
			partition_msg.ycoords.push_back(entry.first.second);
		}

		bfs_agents[entry.second].discrete_mass += 1.;
	}

	geodesic_partition_pub.publish(partition_msg);
	ROS_INFO("%s - Discrete mass = %d/%.2f", name.c_str(), mass_discrete, self.discrete_mass);

	if (self.discrete_mass < mass_discrete)
		self.discrete_mass = mass_discrete;

	if (process_steps < 5) {
		process_steps++;
		return self;
	}

	SkeletalGraph skeletal_map(self.discrete_mass);
	Vector2dHash<Vector2d> vertexHasher;

	for (std::pair<std::pair<double, double>, uint8_t> entry : metric_assignment) {
		if (entry.second != id) 
			continue;

		if (self.is_root(entry.first))
			continue;

		Vector2d v1(entry.first.first, entry.first.second);
		int vid1 = vertexHasher(v1);
		Point_2 p1(v1(0), v1(1));

		for (std::pair<double, double> nb : self.edges[entry.first]) {
			if (metric_assignment[nb] != entry.second)
				continue;

			Vector2d nbp(nb.first, nb.second);
			int vid2 = vertexHasher(nbp);
			Point_2 p2(nbp(0), nbp(1));

			skeletal_map.addEdge(vid1, p1, 1., false, vid2, p2, 1., false);
		}
	}

	std::vector<std::pair<double, size_t> > stats(skeletal_map.getCount());
	// target = skeletal_map.getCentroid(stats, true);
	target = skeletal_map.getCentroid(stats, CentroidalMetric::RADIUS);
	current_workload = self.discrete_mass;
	largest_workload = 0.; // Symbolic, has no use

	std::pair<double, double> goal_key = std::make_pair(target(0), target(1));
	Vector2d geodesic_vector_direct = target - position;
	Vector2d geodesic_centroid_orientation = calculate_geodesic_orientation(goal_key, self);

	try {
		Vector2d workload_sensitive_vector(0., 0.);
		double work_i = self.discrete_mass;
		// double rate_i = work_i / global_mass_to_share;
		double rate_i = - global_mass_to_share / work_i;
		// double rate_i = pow(global_mass_to_share / work_i, 2);
		// double rate_i = pow(1. / work_i, 2);

		for (std::pair<uint8_t, std::set<std::pair<double, double> > > border : self.borders) {
			if (border.first == id)
				continue;

			double work_j = bfs_agents[border.first].discrete_mass;
			// double rate_j = work_j / global_mass_to_share;
			double rate_j = - global_mass_to_share / work_j;
			// double rate_j = pow(global_mass_to_share / work_j, 2);
			// double rate_j = pow(1. / work_j, 2);

			for (std::pair<double, double> v_border : border.second) {
				Vector2d geodesic_orientation = calculate_geodesic_orientation(v_border, self);
				geodesic_orientation /= geodesic_orientation.norm();
				workload_sensitive_vector += (rate_j - rate_i) * geodesic_orientation / self.normals[v_border].norm();
				// workload_sensitive_vector += (work_j - work_i) * geodesic_orientation / self.normals[v_border].norm();
			}
		}

		workload_sensitive_vector /= workload_sensitive_vector.norm();
		geodesic_vector_direct /= geodesic_vector_direct.norm();

		// force_exerted_on_self = workload_sensitive_vector;
		// force_exerted_on_self = m_params.K_goal * workload_sensitive_vector;
		// force_exerted_on_self = workload_sensitive_vector.dot(geodesic_vector_direct) * geodesic_centroid_orientation;
		force_exerted_on_self = m_params.K_goal * saturation(workload_sensitive_vector.dot(geodesic_vector_direct)) * geodesic_centroid_orientation;
		// force_exerted_on_self = workload_sensitive_vector + geodesic_centroid_orientation;
		// force_exerted_on_self = geodesic_centroid_orientation;

	} catch(std::exception& e) {
		ROS_ERROR("%s - got the error: %s", name.c_str(), e.what());
	}

	return self;
}

void Agent::debug_cb(const std_msgs::Empty::ConstPtr& msg) {
	if (!debug_step)
		debug_step = true;
}

void Agent::state_cb(const AgentStateMsg::ConstPtr& msg) {
	if (msg->id == id) 
		return;

	AgentState new_state;
	new_state.heading = msg->heading;
	new_state.workload = msg->workload;
	new_state.position << msg->position.x, msg->position.y;
	new_state.velocity << msg->velocity.x, msg->velocity.y;

	neighbours[msg->id] = new_state;
}

bool Agent::handle_SetInitialPose(SetInitialPose::Request& req, SetInitialPose::Response& res) {
	try {
		position << req.x, req.y;
		heading = req.theta;

		ROS_INFO("%s is set to (%.3f, %.3f, %.3f).", name.c_str(), 
													 position[0], 
													 position[1], 
													 heading);

		res.success = true;
		return true;
	} catch(std::exception& e) {
		ROS_ERROR("%s could not process initial pose request - %s !", name.c_str(), e.what());
		res.success = false;
		return false;
	}
}

bool Agent::handle_SetReady(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	try {
		is_ready = true;
		ROS_INFO("%s is ready!", name.c_str());
		res.success = true;
		res.message = "OK";
		return true;
	} catch(std::exception& e) {
		ROS_ERROR("%s could not get ready - %s !", name.c_str(), e.what());
		res.success = false;
		res.message = "FAIL";
		return false;
	}
}

bool Agent::handle_DumpSkeleton(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	try {
		std::string eps_name = name + "_skeleton.eps";
		std::ofstream eps(eps_name.c_str());

		dump_to_eps(current_work_region, *current_skeleton, eps);

		ROS_INFO("%s has dumped skeleton!", name.c_str());
		res.success = true;
		res.message = "OK";
		return true;
	} catch(std::exception& e) {
		ROS_ERROR("%s could not dump skeleton - %s !", name.c_str(), e.what());
		res.success = false;
		res.message = "FAIL";
		return false;
	}
}
