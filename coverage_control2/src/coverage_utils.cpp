#include "coverage_control2/coverage_utils.h"

void get_metric_graph(Polygon_with_holes_2& polygon, double resolution, 
					  std::vector<Point_2>& outPoints) {
	Bbox_2 bbox = polygon.outer_boundary().bbox();

	for (double xidx = bbox.xmin(); xidx < bbox.xmax(); xidx += resolution) {
		for (double yidx = bbox.ymin(); yidx < bbox.ymax(); yidx += resolution) {
			Point_2 p(xidx, yidx);

			if (CGAL::oriented_side(p, polygon) == CGAL::POSITIVE) 
				outPoints.push_back(p);
		}
	}
}

double a_star_search(Point_2& start, Point_2& goal, double step_size, 
					 Polygon_with_holes_2& environment, bool prune, bool debug, 
					 bool with_path, std::vector<Vector2d>* outPathR) {
	auto inside_check = CGAL::oriented_side(start, environment);
	if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) {
		if (debug)
			std::cout << "Invalid start location!\n";

		return -1;
	}

	inside_check = CGAL::oriented_side(goal, environment);
	if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) {
		if (debug) 
			std::cout << "Invalid goal location!\n";

		return -1;
	}

	if (with_path && outPathR != nullptr)
		assert(outPathR->size() == 0);

	bool found = false;
	double total_cost = 0.;
	auto UtilityPairCmp = [](UtilityPair p1, UtilityPair p2) { return p1.first < p2.first; };

	std::set<Vector2d, Vector2dComp> visited;
	std::priority_queue<UtilityPair, std::vector<UtilityPair>, 
						decltype(UtilityPairCmp)> frontier{UtilityPairCmp};
	std::unordered_map<Vector2d, UtilityPair, Vector2dHash<Vector2d> > branch;

	std::vector<std::pair<Vector2d, double> > actions;
	for (int i = -1; i < 2; i++) {
		for (int j = -1; j < 2; j++) {
			if (i == 0 && j == 0)
				continue;

			Vector2d action(i, j);
			actions.push_back(std::make_pair(action, action.norm()));
		}
	}

	Vector2d start_node(CGAL::to_double(start.x()), CGAL::to_double(start.y()));
	Vector2d goal_node(CGAL::to_double(goal.x()), CGAL::to_double(goal.y()));

	frontier.push(std::make_pair(0., start_node));
	visited.insert(start_node);

	while (!frontier.empty()) {
		std::pair<double, Vector2d> item = frontier.top();
		frontier.pop();

		if ((goal_node - item.second).norm() < step_size) {
		// if (goal_node == item.second) {
			found = true;
			break;
		} else {
			for (std::pair<Vector2d, double>& act : actions) {
				Vector2d next_node = item.second + act.first;

				inside_check = CGAL::oriented_side(Point_2(next_node(0), next_node(1)), environment);
				if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) 
					continue;

				std::set<Vector2d>::iterator v_itr = visited.find(next_node);
				if (v_itr != visited.end())
					continue;

				Vector2d diff = goal_node - next_node;
				double cost = item.first + act.second + std::fabs(diff(0)) + std::fabs(diff(1));
				UtilityPair entry = std::make_pair(cost, next_node);

				frontier.push(entry);
				branch[next_node] = entry;
				visited.insert(next_node);
			}
		}
	}

	if (found) {
		total_cost = branch[goal_node].first;

		if (with_path && outPathR != nullptr) {
			Vector2d temp = goal_node;
			outPathR->push_back(goal_node);

			// while (branch[temp].second != start_node) {
			while ((branch[temp].second - start_node).norm() > step_size) {
				outPathR->push_back(temp);
				temp = branch[temp].second;
			}

			outPathR->push_back(temp);
		}
	} else {
		if (debug) {
			std::cout << "There is no valid path from (" 
					  << start_node(0) << ", " << start_node(1) << ") to ("
					  << goal_node(0) << ", " << goal_node(1) << ")\n";
		}

		total_cost = -1;
	}

	return total_cost;
}

void mark_domains(CDT& ct, CDT_Face_handle start, int index, std::list<CDT::Edge>& border) {
	if (start->info().nesting_level != -1) {
		return;
	}

	std::list<CDT_Face_handle> queue;
	queue.push_back(start);

	while (!queue.empty()) {
		CDT_Face_handle fh = queue.front();
		queue.pop_front();

		if (fh->info().nesting_level == -1) {
			fh->info().nesting_level = index;

			for (int i = 0; i < 3; i++) {
				CDT::Edge e(fh, i);
				CDT_Face_handle n = fh->neighbor(i);

				if (n->info().nesting_level == -1) {
					if (ct.is_constrained(e)) 
						border.push_back(e);

					else 
						queue.push_back(n);
				}
			}
		}
	}
}

void mark_domains(CDT& cdt) {
	for (CDT::Face_handle f : cdt.all_face_handles()) {
		f->info().nesting_level = -1;
	}

	std::list<CDT::Edge> border;
	mark_domains(cdt, cdt.infinite_face(), 0, border);

	while (!border.empty()) {
		CDT::Edge e = border.front();
		border.pop_front();

		CDT::Face_handle n = e.first->neighbor(e.second);
		if (n->info().nesting_level == -1) {
			mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
		}
	}
}

void get_cdt_of_polygon_with_holes(Polygon_with_holes_2& pwh, CDT& outCdt) {
	outCdt.insert_constraint(pwh.outer_boundary().vertices_begin(), 
							 pwh.outer_boundary().vertices_end(), true);

	HoleIterator h_itr = pwh.holes_begin();
	for (; h_itr != pwh.holes_end(); h_itr++) {
		outCdt.insert_constraint(h_itr->vertices_begin(), h_itr->vertices_end(), true);
	}

	assert(outCdt.is_valid());
	mark_domains(outCdt);
}

BFSAgent::BFSAgent() {}
BFSAgent::BFSAgent(uint8_t _id, Vector2d& pos, Vector2d& gpos, double _step_size) : 
	id(_id), p(pos), gp(gpos), step_size(_step_size) {
		frontier.insert(std::make_pair(gp(0), gp(1)));
		visited.insert(std::make_pair(gp(0), gp(1)));
}

bool BFSAgent::is_root(std::pair<double, double>& q) {
	return (q.first == gp(0) && q.second == gp(1));
}

void BFSAgent::add_border_info(std::pair<double, double> border_vertex, uint8_t border_to) {
	std::map<uint8_t, std::set<std::pair<double, double> > >::iterator b_itr = borders.find(border_to);
	if (b_itr != borders.end()) {
		b_itr->second.insert(border_vertex);
	} else {
		borders.insert(std::make_pair(border_to, std::set<std::pair<double, double> >{border_vertex}));
	}
}

std::set<std::pair<double, double> > BFSAgent::frontier_expand(std::vector<MoveAction>& actions, 
															   Polygon_with_holes_2& context) {
	if (frontier.size() == 0)
		return frontier;

	std::set<std::pair<double, double> > next_wave, next_frontier;
	while (!frontier.empty()) {
		std::set<std::pair<double, double> >::iterator f_pos_itr = frontier.begin();
		std::pair<double, double> f_pos = *f_pos_itr;
		Vector2d f_pos_v(f_pos.first, f_pos.second);

		std::map<std::pair<double, double>, std::vector<std::pair<double, double> > >::iterator e_list_itr = edges.find(f_pos);
		if (e_list_itr == edges.end()) {
			edges.insert(std::make_pair(f_pos, std::vector<std::pair<double, double> >()));
		}

		// for (MoveAction& act : actions) {
		for (uint8_t act_i = 0; act_i < 4; act_i++) {
			// Vector2d next_pos = act.first * step_size + f_pos_v;
			Vector2d next_pos = actions[act_i].first * step_size + f_pos_v;

			if (CGAL::oriented_side(Point_2(next_pos(0), next_pos(1)), context) != CGAL::POSITIVE)
				continue;

			std::pair<double, double> next_pos_key = std::make_pair(next_pos(0), next_pos(1));
			edges[f_pos].push_back(next_pos_key);

			if (visited.find(next_pos_key) != visited.end())
				continue;

			next_wave.insert(next_pos_key);
			visited.insert(next_pos_key);
			parents[next_pos_key] = f_pos;
		}

		frontier.erase(f_pos_itr);
	}

	std::set_union(frontier.begin(), frontier.end(), 
				   next_wave.begin(), next_wave.end(), 
				   std::inserter(next_frontier, next_frontier.begin()));

	frontier = next_frontier;
	return frontier;
}

// ---------------------------------------------------------------------------------------------

SkeletalGraph::SkeletalGraph(uint32_t c) {
	count = c + 1;
	graph = MatrixXd::Constant(count, count, std::numeric_limits<int>::max());
	paths = MatrixXd::Constant(count, count, -1);
	next_id_available = 0;
}

int SkeletalGraph::getVertexId(int vid, Point_2 p, double w, bool c) {
	int the_id;

	std::unordered_map<int, int>::iterator itr = id_map.find(vid);

	if (itr == id_map.end()) {
		the_id = next_id_available;

		vertex_map[the_id] = SkeletalNode{c, vid, Vector2d(CGAL::to_double(p.x()), 
										 				   CGAL::to_double(p.y())), w};

		id_map[vid] = the_id;
		next_id_available++;
	} else {
		the_id = itr->second;
	}

	paths(the_id, the_id) = the_id;
	graph(the_id, the_id) = 0.;

	return the_id;
}

void SkeletalGraph::addEdge(int vid1, Point_2 p1, double w1, bool c1, 
							int vid2, Point_2 p2, double w2, bool c2) {
	int id1 = getVertexId(vid1, p1, w1, c1);
	int id2 = getVertexId(vid2, p2, w2, c2);
	assert(id1 < count && id2 < count);

	double D = sqrt(pow(CGAL::to_double(p1.x() - p2.x()), 2) + 
					pow(CGAL::to_double(p1.y() - p2.y()), 2));

	graph(id1, id2) = D;
	graph(id2, id1) = D;

	paths(id1, id2) = id2;
	paths(id2, id1) = id1;

	total_work += D;
}

double SkeletalGraph::getTotalWork() {
	return total_work;
}

void SkeletalGraph::refineEdges() {
	for (int i = 0; i < count; i++) {
		if (vertex_map[i].contour)
			continue;

		for (int j = i + 1; j < count; j++) {
			if (vertex_map[j].contour) {
				graph(i, j) *= (1. + vertex_map[j].weight);
				graph(j, i) *= (1. + vertex_map[j].weight);
				// vertex_map[i].weight += vertex_map[j].weight;
			} //else {}
		}
	}
}

Vector2d SkeletalGraph::getLargestNode(std::vector<std::pair<double, size_t> >& outStats) {
	std::unordered_map<int, SkeletalNode>::iterator vitr = vertex_map.begin();

	size_t i = 0;
	for (; vitr != vertex_map.end(); vitr++) {
		outStats[i] = std::make_pair(vitr->second.weight, vitr->first);
		i++;
	}

	std::sort(outStats.begin(), outStats.end(), [](std::pair<double, size_t> a, 
												   std::pair<double, size_t> b) {
		return a < b;
	});

	bool found = false;
	size_t k = 0;
	for (; k < count; k++) {
		if (!vertex_map[outStats[k].second].contour) {
			found = true;
			break;
		}
	}

	k = (found) ? k : 0;

	// return vertex_map[outStats[0].second].point;
	return vertex_map[outStats[k].second].point;
}

// ---------------------------------------------------------------------------------------------
// Copied from (and modified slightly, for Eigen::Matrix and extra functionality) 
// https://github.com/vsmolyakov/cpp/blob/master/graphs/apsp_floyd_warshall.cpp
Vector2d SkeletalGraph::getCentroid(std::vector<std::pair<double, size_t> >& outStats, 
									CentroidalMetric metric, uint8_t k_avg, bool immediate) {
	assert(count > 0);

	for (int k = 0; k < count; k++) {
		for (int i = 0; i < count; i++) {
			for (int j = 0; j < count; j++) {
				if (graph(i, k) + graph(k, j) < graph(i, j)) {
					graph(i, j) = graph(i, k) + graph(k, j);
					paths(i, j) = paths(i, k);
				}
			}

			if (graph(i, i) < 0.)
				std::cout << "NEGATIVE CYCLE FOUND!!!\n";
		}
	}

	auto dists = graph.rowwise().sum();

	// if (immediate) {
	if (metric == CentroidalMetric::RADIUS) {
		int min_idx = 0;
		double min_val = dists(0);
		for (size_t i = 1; i < count; i++) {
			if (min_val > dists(i)) {
				min_val = dists(i);
				min_idx = i;
			}
		}

		return vertex_map[min_idx].point;
	}
	// else if (metric == CentroidalMetric::DIAMETER) {}
	// else if (metric == CentroidalMetric::ECCENTRICITY) {}
	// else if (metric == CentroidalMetric::CLOSENESS) {}
	// else if (metric == CentroidalMetric::BETWEENNESS) {}
	// else if (metric == CentroidalMetric::K_AVG_RADIUS) {}
	// else if (metric == CentroidalMetric::K_FURTHEST) {}
	else {
		for (size_t i = 0; i < count; i++) {
			outStats[i] = std::make_pair(dists(i), i);
		}

		std::sort(outStats.begin(), outStats.end(), [](std::pair<double, size_t> a, 
													   std::pair<double, size_t> b) {
			return a < b;
		});

		// double pair_total = outStats[0].first + outStats[1].first;
		// double triple_total = outStats[0].first + outStats[1].first + outStats[2].first;

		// return (outStats[0].first * vertex_map[outStats[0].second].point + 
		// 		outStats[1].first * vertex_map[outStats[1].second].point) / pair_total;

		// return (outStats[0].first * vertex_map[outStats[0].second].point + 
		// 		outStats[1].first * vertex_map[outStats[1].second].point + 
		// 		outStats[2].first * vertex_map[outStats[2].second].point) / triple_total;

		double k_total = 0.;
		Vector2d point_total(0., 0.);
		for (size_t l = 0; l < k_avg; l++) {
			k_total += outStats[l].first;
			point_total += outStats[l].first * vertex_map[outStats[l].second].point;
		}

		return point_total / k_total;
	}
}
// ---------------------------------------------------------------------------------------------

uint32_t SkeletalGraph::getCount() {
	return count;
}

const std::unordered_map<int, SkeletalNode>& SkeletalGraph::getVertexMap() const {
	return vertex_map;
}

Vector2d SkeletalGraph::getVertexById(int id) {
	return vertex_map[id].point;
}

std::vector<Point_2> SkeletalGraph::getVerticesAsCgalPoints() {
	std::vector<Point_2> vertices(count);

	size_t i = 0;
	std::unordered_map<int, SkeletalNode>::iterator sn_itr = vertex_map.begin();
	for (; sn_itr != vertex_map.end(); sn_itr++) {
		vertices[i] = Point_2(sn_itr->second.point(0), sn_itr->second.point(1));
		i++;
	}

	return vertices;
}

std::vector<UtilityPair> SkeletalGraph::getNextToVertexFrom(Vector2d& fromV, Vector2d& toV) {
	std::vector<UtilityPair> distances;

	std::unordered_map<int, SkeletalNode>::iterator v_itr = vertex_map.begin();
	for (; v_itr != vertex_map.end(); v_itr++) {
		if (v_itr->second.contour)
			continue;

		double value = (v_itr->second.point - toV).norm() * (v_itr->second.point - fromV).norm();
		distances.push_back(std::make_pair(value, v_itr->second.point));
	}

	std::sort(distances.begin(), distances.end(), 
				[&](UtilityPair p1, UtilityPair p2){
					return p1.first < p2.first;
				});

	// return toV;
	return distances;
}

void SkeletalGraph::assignWeightToVertex(int vid, double w) {
	vertex_map[vid].weight = w;
}
