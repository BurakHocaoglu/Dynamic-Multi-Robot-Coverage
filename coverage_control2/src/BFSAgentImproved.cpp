#include "coverage_control2/BFSAgentImproved.h"

BFSAgentImproved::BFSAgentImproved() {}
BFSAgentImproved::BFSAgentImproved(uint8_t _id, Vector2d& pos, Vector2d& gpos, double _step_size) :
	id(_id), p(pos), gp(gpos), step_size(_step_size), next_available_id(0) {
		BFSVertex root(next_available_id, gp(0), gp(1));
		assigned_vertices.insert(std::make_pair(next_available_id, root));

		frontier.insert(next_available_id);
		visited.insert(next_available_id);

		next_available_id++;
}

bool BFSAgentImproved::is_root(std::pair<double, double>& q) {
	return (q.first == gp(0) && q.second == gp(1));
}

void BFSAgentImproved::add_border_info(std::pair<double, double> border_vertex, uint8_t border_to) {
	//
}

std::set<std::pair<double, double> > BFSAgentImproved::frontier_expand(std::vector<MoveAction>& actions, 
													 					Polygon_with_holes_2& context) {
	if (frontier.size() == 0)
		return frontier;

	std::set<uint32_t> next_wave, next_frontier;
	while (!frontier.empty()) {
		std::set<std::pair<double, double> >::iterator f_pos_itr = frontier.begin();
		std::pair<double, double> f_pos = *f_pos_itr;
		// visited.insert(f_pos);

		bool expanded = false;
		Vector2d f_pos_v(f_pos.first, f_pos.second);
		for (MoveAction& act : actions) {
			Vector2d next_pos = act.first * step_size + f_pos_v;

			if (CGAL::oriented_side(Point_2(next_pos(0), next_pos(1)), context) != CGAL::POSITIVE)
				continue;

			std::pair<double, double> next_pos_key = std::make_pair(next_pos(0), next_pos(1));
			if (visited.find(next_pos_key) != visited.end())
				continue;

			next_wave.insert(next_pos_key);
			visited.insert(next_pos_key);
			parents[next_pos_key] = f_pos;
			expanded = true;
		}

		frontier.erase(f_pos_itr);
	}

	std::set_union(frontier.begin(), frontier.end(), 
				   next_wave.begin(), next_wave.end(), 
				   std::inserter(next_frontier, next_frontier.begin()));

	frontier = next_frontier;
	return frontier;
}
