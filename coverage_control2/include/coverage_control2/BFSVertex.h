#pragma once

#ifndef DIST_COVERAGE_BFS_VERTEX_H
#define DIST_COVERAGE_BFS_VERTEX_H

#include "coverage_control2/coverage_utils.h"

struct BFSVertex {
	bool is_border;
	bool is_orphan;
	uint32_t vid;
	// double x, y;
	std::vector<uint32_t> neighbours;
	Vector2d point;
	Vector2d outward_normal;

	BFSVertex();
	BFSVertex(uint32_t _vid, double _x, double _y);

	void set_borderness(bool b_val);
	void set_orphanness(bool o_val);
	void add_neighbour(uint32_t n_id);
	void set_outward_normal(Vector2d n);
};

#endif // DIST_COVERAGE_BFS_VERTEX_H