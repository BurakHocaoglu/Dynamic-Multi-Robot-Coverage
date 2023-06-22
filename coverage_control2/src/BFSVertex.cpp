#include "coverage_control2/BFSVertex.h"

BFSVertex::BFSVertex() {}

BFSVertex::BFSVertex(uint32_t _vid, double _x, double _y) : 
	is_border(false), is_orphan(false), vid(_vid), point(_x, _y) {}

void BFSVertex::set_borderness(bool b_val) {
	is_border = b_val;
}

void BFSVertex::set_orphanness(bool o_val) {
	is_orphan = o_val;
}

void BFSVertex::add_neighbour(uint32_t n_id) {
	neighbours.push_back(n_id);
}

void BFSVertex::set_outward_normal(Vector2d n) {
	outward_normal = n;
}
