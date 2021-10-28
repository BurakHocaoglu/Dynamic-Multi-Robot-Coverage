#include "coverage_control2/coverage_agent.h"

Agent::Agent(ros::NodeHandle& nh_, const std::string& name_, uint8_t id_) : 
	nh(nh_), 
	name(name_), 
	id(id_), 
	is_ready(false), 
	debug_step(false), 
	sample_count(30)
{
	position << 0.0, 0.0;
	velocity << 0.0, 0.0;
	goal << 0.0, 0.0;

	vpoly_pub = nh.advertise<coverage_control2::Polygon>("/visibility_polys", 1);
	cvx_vor_pub = nh.advertise<coverage_control2::Polygon>("/convex_voronoi", 1);
	vl_voronoi_pub = nh.advertise<coverage_control2::HistoryStep>("/visibility_limited_voronoi", 1);
	state_pub = nh.advertise<AgentStateMsg>("/states", 1);
	utility_debug_pub = nh.advertise<coverage_control2::UtilityDebug>("/" + name + "/utility_debug", 1);
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

	// Polygon_2_ptr_vector pieces = CGAL::create_exterior_skeleton_and_offset_polygons_2(-m_params.physical_radius, region_boundary, K());

	// if (pieces.size() < 1) {
	// 	ROS_WARN("%s - Offset has failed! %lu items...", name.c_str(), pieces.size());

	// 	for (auto p : pieces) {
	// 		std::cout << *p << std::endl;
	// 	}
	// 	std::cout << std::endl;

	// 	exit(EXIT_FAILURE);
	// }

	// inflated_outer_boundary = *(pieces[0]);

	create_offset_poly_naive(region_boundary, -m_params.physical_radius, inflated_outer_boundary);

	if (inflated_outer_boundary.is_clockwise_oriented()) 
		inflated_outer_boundary.reverse_orientation();

	ROS_INFO("%s - Inflated outer boundary has %d vertices", name.c_str(), (int)inflated_outer_boundary.size());

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

	// int rbnd_v_count = region_boundary.size();
	// for (int i = 0; i < rbnd_v_count; i++) {
	// 	int j = (i + 1) % rbnd_v_count;

	// 	Vector2d normal(CGAL::to_double(region_boundary[i][1] - region_boundary[j][1]), 
	// 					CGAL::to_double(region_boundary[j][0] - region_boundary[i][0]));

	// 	Vector2d middle(CGAL::to_double(region_boundary[i][0] + region_boundary[j][0]), 
	// 					CGAL::to_double(region_boundary[i][1] + region_boundary[j][1]));

	// 	BoundarySegment seg = {0, normal, middle * 0.5};
	// 	segments_to_avoid.push_back(seg);
	// 	vis_segments.emplace_back(region_boundary[i], region_boundary[j]);
	// }

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

			// Vector2d normal(region_holes[i][k][1] - region_holes[i][j][1], 
			// 				region_holes[i][j][0] - region_holes[i][k][0]);

			// Vector2d middle(region_holes[i][j][0] + region_holes[i][k][0], 
			// 				region_holes[i][j][1] + region_holes[i][k][1]);

			// BoundarySegment seg = {normal, middle * 0.5};
			// segments_to_avoid.push_back(seg);
			vis_segments.emplace_back(region_holes[i][j], region_holes[i][k]);
		}

		// Polygon_2_ptr_vector hole_pieces = CGAL::create_exterior_skeleton_and_offset_polygons_2(-m_params.physical_radius, region_holes[i], K());

		// if (hole_pieces.size() < 1) {
		// 	ROS_WARN("%s - Hole offset has failed! %lu items...", name.c_str(), hole_pieces.size());

		// 	for (auto hp : hole_pieces) {
		// 		std::cout << *hp << std::endl;
		// 	}
		// 	std::cout << std::endl;

		// 	exit(EXIT_FAILURE);
		// }

		// inflated_region_holes.push_back(*(hole_pieces[0]));

		Polygon_2 inflated_region_hole_i;
		create_offset_poly_naive(region_holes[i], -m_params.physical_radius, inflated_region_hole_i);
		inflated_region_holes.push_back(inflated_region_hole_i);
	}

	for (int i = 0; i < h_count; i++) {
		if (!inflated_region_holes[i].is_clockwise_oriented()) 
			inflated_region_holes[i].reverse_orientation();
	}

	actual_region = Polygon_with_holes_2(inflated_outer_boundary, 
										 inflated_region_holes.begin(), 
										 inflated_region_holes.end());

	// CGAL::draw(actual_region);
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

	return CGAL::to_double(outer_area) - negative_workload;
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

void Agent::step() {
	int agents_to_consider = 0;
	Vector2d total_force(0, 0);
	std::vector<double> values;
	std::vector<Vector2d> constraints;
	std::vector<BoundarySegment> neighbour_segments;

	visibility_polygon();

	std::unordered_map<uint8_t, AgentState>::iterator itr = neighbours.begin();
	for (; itr != neighbours.end(); itr++) {
		if (itr->first == id) 
			continue;

		// Point_2 cgal_point(itr->second.position(0), itr->second.position(1));
		// auto inside_check = CGAL::oriented_side(cgal_point, current_visibility_poly);
		// if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) 
		// 	continue;

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

	if (b_settings.centroid_alg == CentroidAlgorithm::UNKNOWN) {
		b_settings.stationary = true;
		ROS_WARN("%s - does not know a valid centroid algorithm!", name.c_str());
	} else {
		if (b_settings.centroid_alg == CentroidAlgorithm::GEOMETRIC) {
			compute_geometric_centroid();
		} else if (b_settings.centroid_alg == CentroidAlgorithm::GEODESIC_APPROXIMATE) {
			// build_local_skeleton(theFrontier);
			build_local_skeleton(relevantBisectors);
		} else if (b_settings.centroid_alg == CentroidAlgorithm::GEODESIC_EXACT) {
			ROS_WARN("%s - does not have a valid exact geodesic centroid algorithm!", name.c_str());
		} else if (b_settings.centroid_alg == CentroidAlgorithm::FRONTIER_FOCUSED) {
			// build_local_skeleton(theFrontier, true);
			build_local_skeleton(relevantBisectors, true);
		}
	}

	if (b_settings.stationary) 
		return;

	if (!is_point_valid(goal)) {
		ROS_WARN("%s - would move towards an invalid goal!", name.c_str());
	}

	total_force += m_params.K_goal * (goal - position);

	double mag = total_force.norm();
	double ang_diff = wrapToPi(atan2(total_force[1], total_force[0]) - heading);
	double u = m_params.K_linear * mag * cos(ang_diff);
	double w = m_params.K_angular * mag * sin(ang_diff);

	if (fabs(w) > m_params.wmax) 
		w = copysign(m_params.wmax, w);

	// double u = mag;

	clip_inplace(u, 0., m_params.umax);

	Vector2d hdgVector(cos(heading), sin(heading));
	velocity = u * hdgVector;
	// velocity = u * total_force / mag;

	// position += velocity * m_params.delta_t;
	// heading = wrapToPi(heading + w * m_params.delta_t);
	// heading = atan2(velocity(1), velocity(0));

	if (std::fabs(ang_diff) <= m_params.wmax)
		position += velocity * 0.1;

	heading = wrapToPi(heading + w * 0.1);
}

void Agent::visibility_polygon() {
	Polygon_2 new_visibility_poly;
	Arrangement_2 local_env_context, current_visibility;
	CGAL::insert_non_intersecting_curves(local_env_context, vis_segments.begin(), vis_segments.end());

	Point_2 query(position(0), position(1));
	CGAL::Arr_naive_point_location<Arrangement_2> pl(local_env_context);
	CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(query);

	TEV tev(local_env_context);
	Face_handle fh;

	if (obj.which() == 0) {
		// Vertex_const_handle : 0
		// ROS_WARN("%s - Got vertex const handle!", name.c_str());
		return;
	} else if (obj.which() == 1) {
		// Halfedge_const_handle : 1
		// ROS_INFO("%s - Got halfedge const handle.", name.c_str());
		fh = tev.compute_visibility(query, *(boost::get<Halfedge_const_handle>(&obj)), current_visibility);
	} else if (obj.which() == 2) {
		// Face_const_handle : 2
		// ROS_INFO("%s - Got face const handle!", name.c_str());
		fh = tev.compute_visibility(query, *(boost::get<Face_const_handle>(&obj)), current_visibility);
	}
	// ROS_INFO("%s - [Face_handle] Got face handle!", name.c_str());

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

	// CGAL::intersection(current_cvx_voronoi, actual_region, 
	// 				   std::back_inserter(intersection_pieces));

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
			ROS_WARN("%s has multiple visibility and convex voronoi intersection!", name.c_str());

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

		double u1 = workload_utility(p1, inBisectors) / total_workload;
		double u2 = workload_utility(p2, inBisectors) / total_workload;

		bool c1 = htr->vertex()->is_contour();
		bool c2 = htr->opposite()->vertex()->is_contour();

		if (c1 && c2) {
			ROS_WARN("%s - Contour halfedge detected. This should not be executed!", name.c_str());
		}

		skeletal_map.addEdge(vid1, htr->vertex()->point(), u1, c1, 
							 vid2, htr->opposite()->vertex()->point(), u2, c2);

		seen_edges[htr->id()] = true;
		seen_edges[htr->opposite()->id()] = true;
	}

	if (frontierFocus) {
		MGRTree metric_nn_rtree;
		std::vector<Point_2> metric_graph_points;

		get_metric_graph(current_work_region, m_params.physical_radius * 4., metric_graph_points);

		size_t i = 0;
		for (Point_2& mgp : metric_graph_points) {
			metric_nn_rtree.insert(std::make_pair(MGPoint(CGAL::to_double(mgp.x()), 
														  CGAL::to_double(mgp.y())), i));
		}

		std::vector<UtilityPair> super_nodes;

		// auto htr = current_skeleton->halfedges_begin();
		// for (; htr != current_skeleton->halfedges_end(); htr++) {
		// 	auto shtr = seen_edges.find(htr->id());
		// 	if (shtr != seen_edges.end()) 
		// 		continue;

		// 	if (!htr->is_bisector()) 
		// 		continue;

		// 	int vid1 = htr->vertex()->id();
		// 	int vid2 = htr->opposite()->vertex()->id();

		// 	Point_2 p1 = htr->vertex()->point();
		// 	Point_2 p2 = htr->opposite()->vertex()->point();

		// 	double u1 = workload_utility(p1, inBisectors) / total_workload;
		// 	double u2 = workload_utility(p2, inBisectors) / total_workload;

		// 	bool c1 = htr->vertex()->is_contour();
		// 	bool c2 = htr->opposite()->vertex()->is_contour();

		// 	std::vector<MGValue> local_mg_points;

		// 	if (c1 && c2) {
		// 		ROS_WARN("%s - Contour halfedge detected. This should not be executed!", name.c_str());
		// 	} else if (!c1) {
		// 		get_nearest_neighbours(p1, htr->vertex()->time(), metric_nn_rtree, local_mg_points);
		// 	} else if (!c2) {
		// 		get_nearest_neighbours(p2, htr->opposite()->vertex()->time(), 
		// 							   metric_nn_rtree, local_mg_points);
		// 	} else {
		// 		ROS_WARN("%s - WTF!?", name.c_str());
		// 	}

		// 	skeletal_map.addEdge(vid1, htr->vertex()->point(), u1, c1, 
		// 						 vid2, htr->opposite()->vertex()->point(), u2, c2);

		// 	seen_edges[htr->id()] = true;
		// 	seen_edges[htr->opposite()->id()] = true;
		// }

	} else {
		// auto htr = current_skeleton->halfedges_begin();
		// for (; htr != current_skeleton->halfedges_end(); htr++) {
		// 	auto shtr = seen_edges.find(htr->id());
		// 	if (shtr != seen_edges.end()) 
		// 		continue;

		// 	if (!htr->is_bisector()) 
		// 		continue;

		// 	int vid1 = htr->vertex()->id();
		// 	int vid2 = htr->opposite()->vertex()->id();

		// 	Point_2 p1 = htr->vertex()->point();
		// 	Point_2 p2 = htr->opposite()->vertex()->point();

		// 	double u1 = workload_utility(p1, inBisectors) / total_workload;
		// 	double u2 = workload_utility(p2, inBisectors) / total_workload;

		// 	bool c1 = htr->vertex()->is_contour();
		// 	bool c2 = htr->opposite()->vertex()->is_contour();

		// 	if (c1 && c2) {
		// 		ROS_WARN("%s - Contour halfedge detected. This should not be executed!", name.c_str());
		// 	}

		// 	skeletal_map.addEdge(vid1, htr->vertex()->point(), u1, c1, 
		// 						 vid2, htr->opposite()->vertex()->point(), u2, c2);

		// 	seen_edges[htr->id()] = true;
		// 	seen_edges[htr->opposite()->id()] = true;
		// }

		skeletal_map.refineEdges();

		std::vector<std::pair<double, size_t> > stats(skeletal_map.getCount());
		target = skeletal_map.getCentroid(stats);
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
	double cx = 0.0, cy = 0.0;
	double area = CGAL::to_double(current_work_region.outer_boundary().area());

	size_t nVertices = current_work_region.outer_boundary().size();
	for (size_t i = 0; i < nVertices; i++) {
		size_t j = (i + 1) % nVertices;

		Point_2 p1 = current_work_region.outer_boundary().vertex(i);
		Point_2 p2 = current_work_region.outer_boundary().vertex(j);

		cx += (CGAL::to_double(p1.x()) + CGAL::to_double(p2.x())) * 
		        (CGAL::to_double(p1.x()) * CGAL::to_double(p2.y()) - 
		         CGAL::to_double(p2.x()) * CGAL::to_double(p1.y()));

		cy += (CGAL::to_double(p1.y()) + CGAL::to_double(p2.y())) * 
		        (CGAL::to_double(p1.x()) * CGAL::to_double(p2.y()) - 
		         CGAL::to_double(p2.x()) * CGAL::to_double(p1.y()));
	}

	cx /= 6.0 * area;
	cy /= 6.0 * area;

	goal = Vector2d(cx, cy);
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

			// auto inside_check = CGAL::oriented_side(cgal_point, inflated_outer_boundary);
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

			// auto inside_check = CGAL::oriented_side(cgal_point, inflated_outer_boundary);
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
	// VertexIterator bnd_v_itr = region_boundary.vertices_begin();
	// for (; bnd_v_itr != region_boundary.vertices_end(); bnd_v_itr++) {
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

void Agent::debug_cb(const std_msgs::Empty::ConstPtr& msg) {
	if (!debug_step)
		debug_step = true;

	// CGAL::draw(actual_region);
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