#pragma once

#ifndef DIST_COVERAGE_BFS_AGENT_IMPROVED_H
#define DIST_COVERAGE_BFS_AGENT_IMPROVED_H

#include "coverage_control2/BFSVertex.h"

struct BFSAgentImproved {
	uint8_t id;
	uint32_t next_available_id;
	Vector2d p, gp, gv;
	double step_size;
	double discrete_mass;

	std::map<uint32_t, BFSVertex> assigned_vertices;
	std::map<uint8_t, std::set<uint32_t> > borders;

	std::set<uint32_t> frontier;
	std::set<uint32_t> visited;

	BFSAgentImproved();
	BFSAgentImproved(uint8_t _id, Vector2d& pos, Vector2d& gpos, double _step_size);

	bool is_root(std::pair<double, double>& q);

	void add_border_info(std::pair<double, double> border_vertex, uint8_t border_to);

	std::set<std::pair<double, double> > frontier_expand(std::vector<MoveAction>& actions, 
														 Polygon_with_holes_2& context);
};

#endif // DIST_COVERAGE_BFS_AGENT_IMPROVED_H