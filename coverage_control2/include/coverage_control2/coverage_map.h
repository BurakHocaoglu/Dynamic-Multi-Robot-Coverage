#ifndef DIST_COVERAGE_MAP_H
#define DIST_COVERAGE_MAP_H

#include "coverage_control2/coverage_utils.h"

class CoverageMap {
	public:
		CoverageMap(Polygon_2& m_bounds, Polygon_2_Array& m_holes);
		~CoverageMap();

		void construct_metric_graph();

		std::vector<Vector2d> get_partition(uint8_t caller_id, 
											std::unordered_map<uint8_t, AgentState>& agents);

	private:
		Polygon_2 map_boundary;
		Polygon_2_Array map_holes;

		Polygon_with_holes_2 map;

		MGRTree metric_nn_rtree;
		// std::vector<Point_2> metric_graph_points;
		std::vector<Vector2d> metric_graph_points;
};

#endif // DIST_COVERAGE_MAP_H