#include "coverage_control2/coverage_agent.h"

Agent::Agent(ros::NodeHandle& nh_, const std::string& name_, uint8_t id_) : 
	nh(nh_), 
	name(name_), 
	id(id_), 
	is_ready(false), 
	sample_count(30)
{
	position << 0.0, 0.0;
	velocity << 0.0, 0.0;
	goal << 0.0, 0.0;

	vpoly_pub = nh.advertise<coverage_control2::Polygon>("/visibility_polys", 1);
	cvx_vor_pub = nh.advertise<coverage_control2::Polygon>("/convex_voronoi", 1);
	vl_voronoi_pub = nh.advertise<coverage_control2::Polygon>("/visibility_limited_voronoi", 1);
	state_pub = nh.advertise<AgentStateMsg>("/states", 1);
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

	Polygon_2_ptr_vector pieces = CGAL::create_exterior_skeleton_and_offset_polygons_2(-0.5, region_boundary, K());

	if (pieces.size() < 1) {
		ROS_WARN("%s - Offset has failed! %lu items...", name.c_str(), pieces.size());

		for (auto p : pieces) {
			std::cout << *p << std::endl;
		}
		std::cout << std::endl;

		exit(EXIT_FAILURE);
	}

	inflated_outer_boundary = *(pieces[0]);

	int rbnd_v_count = region_boundary.size();
	for (int i = 0; i < rbnd_v_count; i++) {
		int j = (i + 1) % rbnd_v_count;

		Vector2d normal(CGAL::to_double(region_boundary[i][1] - region_boundary[j][1]), 
						CGAL::to_double(region_boundary[j][0] - region_boundary[i][0]));

		Vector2d middle(CGAL::to_double(region_boundary[i][0] + region_boundary[j][0]), 
						CGAL::to_double(region_boundary[i][1] + region_boundary[j][1]));

		BoundarySegment seg = {normal, middle * 0.5};
		segments_to_avoid.push_back(seg);
		vis_segments.emplace_back(region_boundary[i], region_boundary[j]);
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
			// exact_vis_segments.emplace_back(region_holes[i][j], region_holes[i][k]);
			vis_segments.emplace_back(region_holes[i][j], region_holes[i][k]);
		}

		Polygon_2_ptr_vector hole_pieces = CGAL::create_exterior_skeleton_and_offset_polygons_2(-0.5, region_holes[i], K());

		if (hole_pieces.size() < 1) {
			ROS_WARN("%s - Hole offset has failed! %lu items...", name.c_str(), hole_pieces.size());

			for (auto hp : hole_pieces) {
				std::cout << *hp << std::endl;
			}
			std::cout << std::endl;

			exit(EXIT_FAILURE);
		}

		inflated_region_holes.push_back(*(hole_pieces[0]));

		actual_region = Polygon_with_holes_2(c_bounds, 
											 c_holes.begin(), 
											 c_holes.end());

		CGAL::insert_non_intersecting_curves(environment_arr, vis_segments.begin(), vis_segments.end());
	}
}

bool Agent::ready() {
	return is_ready;
}

bool Agent::is_point_valid(const Point_2& p) {
	auto inside_check = CGAL::oriented_side(p, inflated_outer_boundary);
	if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && 
		inside_check != CGAL::POSITIVE) {
		return false;
	}

	// if (inflated_outer_boundary.bounded_side(p) == CGAL::ON_UNBOUNDED_SIDE) 
	// 	return false;

	// Polygon_2_Array::iterator itr = region_holes.begin();
	Polygon_2_Array::iterator itr = inflated_region_holes.begin();
	// for (; itr != region_holes.end(); itr++) {
	for (; itr != inflated_region_holes.end(); itr++) {
		// if (itr->bounded_side(p) != CGAL::ON_UNBOUNDED_SIDE) 
		// 	return false;

		inside_check = CGAL::oriented_side(p, *itr);
		if (inside_check == CGAL::ON_ORIENTED_BOUNDARY || 
			inside_check == CGAL::POSITIVE) {
			return false;
		}
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

	K::FT the_workload;
	CGAL::area_2(current_work_region.outer_boundary().vertices_begin(), 
				 current_work_region.outer_boundary().vertices_end(), 
				 the_workload, K());

	current_workload = CGAL::to_double(the_workload);

	// ROS_INFO("%s - current workload: %.3f", name.c_str(), current_workload);

	current_state.workload = current_workload;

	state_pub.publish(current_state);
}

double Agent::local_utility(const Vector2d& p) {
	double value = 0.0;

	Vector2d hvec(cos(heading), sin(heading));
	value += hvec.dot(p - position);

	// ...

	return value;
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
	// ROS_INFO("%s has computed visibility!", name.c_str());

	std::unordered_map<uint8_t, AgentState>::iterator itr = neighbours.begin();
	for (; itr != neighbours.end(); itr++) {
		if (itr->first == id) 
			continue;

		// Point_2 cgal_point(itr->second.position(0), itr->second.position(1));
		// auto inside_check = CGAL::oriented_side(cgal_point, current_visibility_poly);
		// if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && 
		// 	inside_check != CGAL::POSITIVE) {
		// 	continue;
		// }

		// ROS_INFO("%s - processing ID %u ...", name.c_str(), itr->first);
		Vector2d r_ij = itr->second.position - position;
		Vector2d m_ij = (position + itr->second.position) * 0.5;
		double norm = r_ij.norm();

		if (b_settings.limited_sensing) {
			if (norm <= s_params.sense_radius) {
				// total_force += m_params.K_repulsion * -r_ij * m_params.repulsion_const / pow(norm - 2. * m_params.safe_radius, 2);
				// total_force += m_params.K_attraction * r_ij * m_params.attraction_const / norm;

				BoundarySegment n_seg = {r_ij, m_ij};
				neighbour_segments.push_back(n_seg);
				// ROS_INFO("%s - got (%.3f, %.3f) - (%.3f, %.3f)", name.c_str(), 
				// 												 r_ij(0), r_ij(1), 
				// 												 m_ij(0), m_ij(1));

				constraints.push_back(r_ij);
				values.push_back(r_ij.dot(m_ij) - m_params.physical_radius);
				agents_to_consider++;
			}
		} else {
			// total_force += m_params.K_repulsion * -r_ij * m_params.repulsion_const / pow(norm - 2. * m_params.safe_radius, 2);
			// total_force += m_params.K_attraction * r_ij * m_params.attraction_const / norm;

			if (itr->second.workload > current_workload) {
				total_force += m_params.K_attraction * r_ij * m_params.attraction_const / norm;
			} else if (itr->second.workload < current_workload) {
				// total_force -= m_params.K_attraction * r_ij * m_params.attraction_const / norm;
				total_force -= m_params.K_repulsion * r_ij * m_params.attraction_const / norm;
			}

			BoundarySegment n_seg = {r_ij, m_ij};
			neighbour_segments.push_back(n_seg);
			// ROS_INFO("%s - got (%.3f, %.3f) - (%.3f, %.3f)", name.c_str(), 
			// 												 r_ij(0), r_ij(1), 
			// 												 m_ij(0), m_ij(1));

			constraints.push_back(r_ij);
			values.push_back(r_ij.dot(m_ij) - m_params.physical_radius);
			agents_to_consider++;
		}
	}

	std::vector<BoundarySegment>::iterator seg_itr = segments_to_avoid.begin();
	for (; seg_itr != segments_to_avoid.end(); seg_itr++) {
		Vector2d vec = seg_itr->middle - position;

		// if (seg_itr->normal.dot(vec) <= m_params.safe_radius) {
		// 	total_force += m_params.K_repulsion * seg_itr->normal * m_params.repulsion_const / pow(vec.norm() - 2. * m_params.safe_radius, 2);
		// }
	}

	VectorXd b(agents_to_consider);
	ConstraintMatrix2d A(agents_to_consider, 2);

	for (int i = 0; i < agents_to_consider; i++) {
		b(i) = values[i];

		for (int j = 0; j < 2; j++) {
			A(i, j) = constraints[i](j);
		}
	}

	// visibility_polygon();
	// // ROS_INFO("%s has computed visibility!", name.c_str());

	get_voronoi_cell_raw(neighbour_segments, agents_to_consider, A, b);
	// ROS_INFO("%s converted visibility into polygon!", name.c_str());

	visibility_limited_voronoi();
	// ROS_INFO("%s has computed limited sensing voronoi cell!", name.c_str());

	if (b_settings.centroid_alg == CentroidAlgorithm::UNKNOWN) {
		b_settings.stationary = true;
		ROS_WARN("%s - does not know a valid centroid algorithm!", name.c_str());
	} else {
		if (b_settings.centroid_alg == CentroidAlgorithm::GEOMETRIC) {
			compute_geometric_centroid();
		} else if (b_settings.centroid_alg == CentroidAlgorithm::GEODESIC_APPROXIMATE) {
			build_local_skeleton();
		} else if (b_settings.centroid_alg == CentroidAlgorithm::GEODESIC_EXACT) {
			ROS_WARN("%s - does not have a valid exact geodesic centroid algorithm!", name.c_str());
		}
	}

	// build_local_skeleton();
	// ROS_INFO("%s has built local skeleton structure!", name.c_str());

	if (b_settings.stationary) 
		return;

	// std::vector<UtilityPair> local_valid_frontier;

	// for (int j = 0; j < n_offsets; j++) {
	// 	double ang_step = wrapToPi(heading + angle_offsets[j]);
	// 	Vector2d d(cos(ang_step), sin(ang_step));
	// 	Vector2d cand_vertex = position + s_params.sense_radius * d;

	// 	if (!is_point_valid(cand_vertex)) 
	// 		continue;

	// 	if (agents_to_consider > 0) {
	// 		VectorXd feasibility(agents_to_consider);
	// 		feasibility = A * cand_vertex - b;

	// 		if ((feasibility.array() < 0.1).all()) {
	// 			local_valid_frontier.emplace_back(local_utility(cand_vertex), cand_vertex);
	// 		}
	// 	} else {
	// 		local_valid_frontier.emplace_back(local_utility(cand_vertex), cand_vertex);
	// 	}
	// }

	// select_goal_from_local_frontier(local_valid_frontier);

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

	clip_inplace(u, 0., m_params.umax);

	Vector2d hdgVector(cos(heading), sin(heading));
	velocity = u * hdgVector;

	// position += velocity * m_params.delta_t;
	// heading = wrapToPi(heading + w * m_params.delta_t);
	// heading = atan2(velocity(1), velocity(0));

	if (ang_diff <= m_params.wmax)
		position += velocity * 0.1;

	// if (velocity.norm() <= 0.01) 
	// 	ROS_WARN("%s - TOO SLOW!", name.c_str());

	heading = wrapToPi(heading + w * 0.1);
}

void Agent::visibility_polygon() {
	Polygon_2 new_visibility_poly;
	// Arrangement_2 current_visibility;
	Arrangement_2 local_env_context, current_visibility;
	CGAL::insert_non_intersecting_curves(local_env_context, vis_segments.begin(), vis_segments.end());

	Point_2 query(position(0), position(1));
	CGAL::Arr_naive_point_location<Arrangement_2> pl(local_env_context);
	// CGAL::Arr_naive_point_location<Arrangement_2> pl(environment_arr);
	CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(query);

	// TEV tev(local_env_context);
	TEV tev(environment_arr);

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

void Agent::visibility_limited_voronoi() {
	std::list<Polygon_with_holes_2> intersection_pieces;

	// ROS_INFO("%s - Intersecting - %d items with %d items", name.c_str(), 
	// 													   (int)current_visibility_poly.size(), 
	// 													   (int)current_cvx_voronoi.size());

	CGAL::intersection(current_visibility_poly, 
					   current_cvx_voronoi, 
					   std::back_inserter(intersection_pieces));

	// CGAL::intersection(actual_region, 
	// 				   current_cvx_voronoi, 
	// 				   std::back_inserter(intersection_pieces));

	int n_pieces = intersection_pieces.size();

	if (n_pieces == 0) {
		coverage_control2::Polygon current_vor_poly;
		current_vor_poly.id = id;
		current_vor_poly.position.x = position(0);
		current_vor_poly.position.y = position(1);

		VertexIterator v_itr = current_cvx_voronoi.vertices_begin();
		for (; v_itr != current_cvx_voronoi.vertices_end(); v_itr++) {
			geometry_msgs::Point vor_point;
			vor_point.x = CGAL::to_double(v_itr->x());
			vor_point.y = CGAL::to_double(v_itr->y());
			current_vor_poly.points.push_back(vor_point);
		}

		vl_voronoi_pub.publish(current_vor_poly);
		ROS_WARN("%s has no visibility and convex voronoi intersection! Sending voronoi...", name.c_str());
	} else if (n_pieces > 1) {
		ROS_WARN("%s has multiple visibility and convex voronoi intersection!", name.c_str());
	} else {
		// ROS_INFO("%s has computed visibility and convex voronoi intersection.", name.c_str());

		Polygon_with_holes_2 intr_piece = intersection_pieces.front();

		coverage_control2::Polygon current_vlv_poly;
		current_vlv_poly.id = id;
		current_vlv_poly.position.x = position(0);
		current_vlv_poly.position.y = position(1);

		VertexIterator v_itr = intr_piece.outer_boundary().vertices_begin();
		for (; v_itr != intr_piece.outer_boundary().vertices_end(); v_itr++) {
			geometry_msgs::Point vlvp_point;
			vlvp_point.x = CGAL::to_double(v_itr->x());
			vlvp_point.y = CGAL::to_double(v_itr->y());
			current_vlv_poly.points.push_back(vlvp_point);
		}

		vl_voronoi_pub.publish(current_vlv_poly);
		current_work_region = intr_piece;
	}
}

void Agent::build_local_skeleton() {
	if (current_work_region.outer_boundary().is_clockwise_oriented()) {
		// poly.reverse_orientation();
		ROS_WARN("%s - Work region is not CCW oriented!", name.c_str());
	}

	current_skeleton = CGAL::create_interior_straight_skeleton_2(current_work_region, K());
	// ROS_INFO("%s - created inner skeleton.", name.c_str());

	if (current_skeleton->size_of_vertices() < 3)
		ROS_WARN("%s - Too low skeletal node count!!", name.c_str());

	SkeletalGraph skeletal_map(current_skeleton->size_of_vertices());
	// ROS_INFO("%s - Constructed skeletal map for %lu vertices!", name.c_str(), current_skeleton->size_of_vertices());
	// std::cout << name << " will have " << skeletal_map.getCount() << " vertices!\n";

	std::unordered_map<int, bool> seen_edges;

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

		skeletal_map.addEdge(vid1, htr->vertex()->point(), 
							 vid2, htr->opposite()->vertex()->point());

		seen_edges[htr->id()] = true;
		seen_edges[htr->opposite()->id()] = true;
	}

	goal = skeletal_map.getCentroid();
	// goal = (skeletal_map.getCentroid() + position) / 3.;

	Point_2 cgal_point(goal(0), goal(1));
	auto inside_check = CGAL::oriented_side(cgal_point, inflated_outer_boundary);
	if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && 
		inside_check != CGAL::POSITIVE) {
		ROS_WARN("%s was going outside!", name.c_str());
		goal = position;
	}

	// polygon_t cwr;
	// VertexIterator v_itr = current_work_region.outer_boundary().vertices_begin();
	// for (; v_itr != current_work_region.outer_boundary().vertices_end(); v_itr++) {
	// 	bg::append(cwr.outer(), point_t(CGAL::to_double(v_itr->x()), CGAL::to_double(v_itr->y())));
	// }

	// ROS_INFO("%s - Work region polygon is constructed.", name.c_str());

	// Cell goal_cell = getCentroidCell(cwr);
	// ROS_INFO("%s - Calculated approximate geodesic centroid.", name.c_str());

	// goal = Vector2d(goal_cell.c.get<0>(), goal_cell.c.get<1>());
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

void Agent::get_voronoi_cell_raw(std::vector<BoundarySegment>& segments, 
								 uint8_t neighbour_count, 
								 ConstraintMatrix2d& A, 
								 VectorXd& b) {
	std::vector<Point_2> cvx_voronoi_vertices;
	int n_segments = segments.size();

	// Voronoi bisectors vs. Voronoi bisectors
	// ROS_INFO("%s - Intersecting Voronoi vs. Voronoi, %d segments..., %d neighbours", name.c_str(), n_segments, neighbour_count);
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

			auto inside_check = CGAL::oriented_side(cgal_point, inflated_outer_boundary);
			if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && 
				inside_check != CGAL::POSITIVE) {
				// std::cout << "--- " << name << " --- VxV ---\n";
				// std::cout << p_intr << " not inside!\n";
				// std::cout << region_boundary << std::endl;
				// std::cout << nice(inside_check) << std::endl;
				continue;
			}

			if (neighbour_count > 0) {
				VectorXd feasibility(neighbour_count);
				feasibility = A * p_intr - b;

				if ((feasibility.array() < 1.).all()) {
					cvx_voronoi_vertices.emplace_back(p_intr(0), p_intr(1));
					// ROS_INFO("%s - Voronoi vs. Voronoi: (%.3f, %.3f)", name.c_str(), 
					// 												   p_intr(0), 
					// 												   p_intr(1));
				}
				else {
					// std::cout << "--- " << name << " --- VxV ---\n";
					// std::cout << p_intr << " is not feasible with: A\n" << A << std::endl;
					// std::cout << "and b\n" << b << std::endl;
				}
			}
		}
	}

	// Voronoi bisectors vs. Boundary segments
	// ROS_INFO("%s - Intersecting Voronoi vs. Outer Boundary", name.c_str());
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

			auto inside_check = CGAL::oriented_side(cgal_point, inflated_outer_boundary);
			if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && 
				inside_check != CGAL::POSITIVE) {
				// std::cout << "--- " << name << " --- VxB ---\n";
				// std::cout << p_intr << " not inside!\n";
				// std::cout << region_boundary << std::endl;
				// std::cout << nice(inside_check) << std::endl;
				continue;
			}

			if (neighbour_count > 0) {
				VectorXd feasibility(neighbour_count);
				feasibility = A * p_intr - b;

				if ((feasibility.array() < 1.).all()) {
					cvx_voronoi_vertices.emplace_back(p_intr(0), p_intr(1));
					// ROS_INFO("%s - Voronoi vs. Boundary: (%.3f, %.3f)", name.c_str(), 
					// 													p_intr(0), 
					// 													p_intr(1));
				}
				else {
					// std::cout << "--- " << name << " --- VxB ---\n";
					// std::cout << p_intr << " is not feasible with: A\n" << A << std::endl;
					// std::cout << "and b\n" << b << std::endl;
				}
			}
		}
	}

	// Outer boundary vertices
	// ROS_INFO("%s - Checking Outer Boundary vertices", name.c_str());
	VertexIterator bnd_v_itr = region_boundary.vertices_begin();
	for (; bnd_v_itr != region_boundary.vertices_end(); bnd_v_itr++) {
		Vector2d vertex(CGAL::to_double(bnd_v_itr->x()), 
						CGAL::to_double(bnd_v_itr->y()));

		if (neighbour_count > 0) {
			VectorXd feasibility(neighbour_count);
			feasibility = A * vertex - b;

			if ((feasibility.array() < 0.5).all()) {
				cvx_voronoi_vertices.emplace_back(vertex(0), vertex(1));
				// ROS_INFO("%s - Boundary vertices: (%.3f, %.3f)", name.c_str(), 
				// 												 vertex(0), 
				// 												 vertex(1));
			}
			// else {
			// 	std::cout << "--- " << name << " --- BV ---\n";
			// 	std::cout << vertex << " is not feasible with: A\n" << A << std::endl;
			// 	std::cout << "and b\n" << b << std::endl;
			// }
		} else {
			cvx_voronoi_vertices.emplace_back(vertex(0), vertex(1));
			// ROS_INFO("%s - Boundary vertices: (%.3f, %.3f)", name.c_str(), 
			// 												 vertex(0), 
			// 												 vertex(1));
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
		// ROS_INFO("%s - Sorting valid %d vertices", name.c_str(), (int)cvx_voronoi_vertices.size());
		angular_sort(cvx_voronoi_vertices, pos);
		current_cvx_voronoi = Polygon_2(cvx_voronoi_vertices.begin(), cvx_voronoi_vertices.end());

		coverage_control2::Polygon current_vor_poly;
		current_vor_poly.id = id;

		VertexIterator v_itr = current_cvx_voronoi.vertices_begin();
		for (; v_itr != current_cvx_voronoi.vertices_end(); v_itr++) {
			geometry_msgs::Point vor_point;
			vor_point.x = CGAL::to_double(v_itr->x());
			vor_point.y = CGAL::to_double(v_itr->y());
			current_vor_poly.points.push_back(vor_point);
		}

		// ROS_INFO("%s - Sending voronoi...", name.c_str());
		cvx_vor_pub.publish(current_vor_poly);
	} else {
		ROS_WARN("%s - Not enough valid voronoi vertices!", name.c_str());
	}
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