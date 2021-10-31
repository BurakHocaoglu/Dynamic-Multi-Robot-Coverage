#ifndef DIST_COVERAGE_MAP_H
#define DIST_COVERAGE_MAP_H

#include "coverage_control2/coverage_utils.h"

class CoverageMap {
	public:
		CoverageMap();
		~CoverageMap();

		void construct_metric_graph();

	private:
		Polygon_2 map_boundary;
		Polygon_2_Array map holes;

		Polygon_with_holes_2 map;

		MGRTree metric_nn_rtree;
		std::vector<Point_2> metric_graph_points;
};

#endif // DIST_COVERAGE_MAP_H