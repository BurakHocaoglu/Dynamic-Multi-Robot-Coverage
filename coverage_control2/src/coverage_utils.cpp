#include "coverage_control2/coverage_utils.h"

SkeletalGraph::SkeletalGraph(uint32_t c) {
	count = c;
	// graph = MatrixXd::Zero(c, c);
	graph = MatrixXd::Constant(c, c, std::numeric_limits<int>::max());
	next_id_available = 0;
}

int SkeletalGraph::getVertexId(int vid, Point_2 p, double w) {
	int the_id;

	std::unordered_map<int, int>::iterator itr = id_map.find(vid);

	if (itr == id_map.end()) {
		the_id = next_id_available;

		vertex_map[the_id] = SkeletalNode{vid, Vector2d(CGAL::to_double(p.x()), 
										 				CGAL::to_double(p.y())), w};

		id_map[vid] = the_id;
		next_id_available++;
	} else {
		the_id = itr->second;
	}

	return the_id;
}

void SkeletalGraph::addEdge(int vid1, Point_2 p1, double w1, 
							int vid2, Point_2 p2, double w2) {
	int id1 = getVertexId(vid1, p1, w1);
	int id2 = getVertexId(vid2, p2, w2);

	double D = sqrt(pow(CGAL::to_double(p1.x() - p2.x()), 2) + 
					pow(CGAL::to_double(p1.y() - p2.y()), 2));

	graph(id1, id2) = D + w1 - w2;
	graph(id2, id1) = D + w2 - w1;
	total_work += D;
}

double SkeletalGraph::getTotalWork() {
	return total_work;
}

Vector2d SkeletalGraph::getLargestNode(std::vector<std::pair<double, size_t> >& outStats) {
	std::unordered_map<int, SkeletalNode>::iterator vitr = vertex_map.begin();
	// double max_weight = vitr->second.weight;
	// Vector2d max_point = vitr->second.point;
	// vitr++;

	size_t i = 0;
	for (; vitr != vertex_map.end(); vitr++) {
		// if (vitr->second.weight > max_weight) {
		// 	max_weight = vitr->second.weight;
		// 	max_point = vitr->second.point;
		// }

		outStats[i] = std::make_pair(vitr->second.weight, vitr->first);
		i++;
	}

	// return max_point;

	std::sort(outStats.begin(), outStats.end(), [](std::pair<double, size_t> a, 
												   std::pair<double, size_t> b) {
		return a > b;
	});

	return vertex_map[outStats[0].second].point;
}

// ---------------------------------------------------------------------------------------------
// Copied from (and modified slightly, for Eigen::Matrix and extra functionality) 
// https://github.com/vsmolyakov/cpp/blob/master/graphs/apsp_floyd_warshall.cpp
// Vector2d SkeletalGraph::getCentroid(bool immediate=false) {
Vector2d SkeletalGraph::getCentroid(std::vector<std::pair<double, size_t> >& outStats, 
									bool immediate) {
	for (int k = 0; k < count; k++) {
		for (int i = 0; i < count; i++) {
			for (int j = 0; j < count; j++) {
				if (graph(i, k) + graph(k, j) < graph(i, j)) 
					graph(i, j) = graph(i, k) + graph(k, j);
			}
		}
	}

	auto dists = graph.rowwise().sum();

	if (immediate) {
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

	// std::vector<std::pair<double, size_t> > outStats(count);
	for (size_t i = 0; i < count; i++) {
		outStats[i] = std::make_pair(dists(i), i);
	}

	std::sort(outStats.begin(), outStats.end(), [](std::pair<double, size_t> a, 
												   std::pair<double, size_t> b) {
		return a < b;
	});

	// double pair_total = outStats[0].first + outStats[1].first;
	double triple_total = outStats[0].first + outStats[1].first + outStats[2].first;

	// return (outStats[0].first * vertex_map[outStats[0].second].point + 
	// 		outStats[1].first * vertex_map[outStats[1].second].point) / pair_total;

	return (outStats[0].first * vertex_map[outStats[0].second].point + 
			outStats[1].first * vertex_map[outStats[1].second].point + 
			outStats[2].first * vertex_map[outStats[2].second].point) / triple_total;
}
// ---------------------------------------------------------------------------------------------

uint32_t SkeletalGraph::getCount() {
	return count;
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

// void SkeletalGraph::spreadFromVertex(Vector2d v, std::vector<std::pair<double, size_t> >& outStats) {
Vector2d SkeletalGraph::spreadSearchFromVertex(Vector2d v, Polygon_2& vispoly) {
	bool found = false;

	int gv_idx;
	std::unordered_map<int, SkeletalNode>::iterator sn_itr = vertex_map.begin();
	for (; sn_itr != vertex_map.end(); sn_itr++) {
		if ((v - sn_itr->second.point).norm() < 0.01) {
			gv_idx = sn_itr->first;
			found = true;
			break;
		}
	}

	if (!found) {
		std::cout << "The requested vertex is not in the graph!!!\n";
		return v;
	}

	std::vector<bool> visited(count);
	std::queue<int> gvq;
	gvq.push(gv_idx);
	Vector2d goal;

	while (!gvq.empty()) {
		found = false;
		int i = gvq.back();
		gvq.pop();

		for (int j = 0; j < count; j++) {
			if (!graph(i, j)) 
				continue;

			Vector2d vertex = vertex_map[j].point;
			Point_2 cand(vertex(0), vertex(1));
			auto inside_check = CGAL::oriented_side(cand, vispoly);
			if (inside_check != CGAL::ON_ORIENTED_BOUNDARY && inside_check != CGAL::POSITIVE) 
				continue;

			else {
				goal = vertex;
				found = true;
				break;
			}
		}

		if (found)
			break;
	}

	if (!found) {
		std::cout << "Could not find the vertex to the highest utility target! What to do?\n";
		goal = v;
	}

	return goal;
}

// ------------------------------------------------------------------------------------------

// SkeletalGraphGeneric::SkeletalGraphGeneric(uint32_t c) {
// 	count = c;
// 	// graph = MatrixXd::Constant(c, c, std::numeric_limits<int>::max());
// 	next_vid_available = 0;
// 	next_eid_available = 0;
// }

// int SkeletalGraphGeneric::getVertexId(int vid, Point_2 p, double w) {
// 	int the_id;

// 	std::unordered_map<int, int>::iterator itr = id_map.find(vid);

// 	if (itr == id_map.end()) {
// 		the_id = next_vid_available;

// 		vertex_map[the_id] = SkeletalNode{vid, w, 
// 										  Vector2d(CGAL::to_double(p.x()), CGAL::to_double(p.y())), 
// 										  std::vector<int>()};

// 		id_map[vid] = the_id;
// 		next_vid_available++;
// 	} else {
// 		the_id = itr->second;
// 	}

// 	return the_id;
// }

// void SkeletalGraphGeneric::addEdge(int vid1, Point_2 p1, double w1, 
// 								   int vid2, Point_2 p2, double w2) {
// 	int id1 = getVertexId(vid1, p1, w1);
// 	int id2 = getVertexId(vid2, p2, w2);

// 	double D = sqrt(pow(CGAL::to_double(p1.x() - p2.x()), 2) + 
// 					pow(CGAL::to_double(p1.y() - p2.y()), 2));

// 	// graph(id1, id2) = D;
// 	// graph(id2, id1) = D;

// 	int the_eid = next_eid_available;
// 	edge_map[the_eid] = SkeletalEdgeGeneric{the_eid, id1, id2, D};

// 	vertex_map[id1].edge_ids.push_back(the_eid);
// 	vertex_map[id2].edge_ids.push_back(the_eid);

// 	next_eid_available++;
// 	total_work += D;
// }

// double SkeletalGraphGeneric::getTotalWork() {
// 	return total_work;
// }

// Vector2d SkeletalGraphGeneric::getCentroid(std::vector<std::pair<double, size_t> >& outStats, 
// 										   bool immediate=false) {
// 	return Vector2d(0., 0.);
// }

// uint32_t SkeletalGraphGeneric::getCount() {
// 	return count;
// }

// Vector2d SkeletalGraphGeneric::getVertexById(int id) {
// 	return vertex_map[id].point;
// }

// std::vector<Point_2> SkeletalGraphGeneric::getVerticesAsCgalPoints() {
// 	std::vector<Point_2> vertices(count);

// 	size_t i = 0;
// 	std::unordered_map<int, SkeletalNode>::iterator sn_itr = vertex_map.begin();
// 	for (; sn_itr != vertex_map.end(); sn_itr++) {
// 		vertices[i] = Point_2(sn_itr->second.point(0), sn_itr->second.point(1));
// 		i++;
// 	}

// 	return vertices;
// }

// ------------------------------------------------------------------------------------------

// double getSegDistSq(const point_t& p, const point_t& a, const point_t& b) {
//     double x = a.get<0>();
//     double y = a.get<1>();
//     double dx = b.get<0>() - x;
//     double dy = b.get<1>() - y;

//     if (dx != 0 || dy != 0) {
//         double t = ((p.get<0>() - x) * dx + (p.get<1>() - y) * dy) / (dx * dx + dy * dy);

//         if (t > 1) {
//             x = b.get<0>();
//             y = b.get<1>();

//         } else if (t > 0) {
//             x += dx * t;
//             y += dy * t;
//         }
//     }

//     dx = p.get<0>() - x;
//     dy = p.get<1>() - y;

//     return dx * dx + dy * dy;
// }

// double pointToPolygonDist(const point_t& point, const polygon_t& polygon) {
//     bool inside = false;
//     auto minDistSq = std::numeric_limits<double>::infinity();

//     std::size_t i = 0, j, len;
//     for (std::size_t i = 0, len = polygon.outer().size(), j = len - 1; i < len; j = i++) {
//         const auto& a = polygon.outer()[i];
//         const auto& b = polygon.outer()[j];

//         if ((a.get<1>() > point.get<1>()) != (b.get<1>() > point.get<1>()) && 
//             (point.get<0>() < (b.get<0>() - a.get<0>()) * (point.get<1>() - a.get<1>()) / (b.get<1>() - a.get<1>()) + a.get<0>())) inside = !inside;

//         minDistSq = std::min(minDistSq, getSegDistSq(point, a, b));
//     }

//     return (inside ? 1 : -1) * std::sqrt(minDistSq);
// }

// Cell getCentroidCell(const polygon_t& polygon) {
//     double area = 0.;
//     point_t c(0., 0.);

//     for (std::size_t i = 0, len = polygon.outer().size(), j = len - 1; i < len; j = i++) {
//         const point_t& a = polygon.outer()[i];
//         const point_t& b = polygon.outer()[j];
//         auto f = a.get<0>() * b.get<1>() - b.get<1>() * a.get<1>();
//         c.set<0>(c.get<0>() + (a.get<0>() + b.get<0>()) * f);
//         c.set<1>(c.get<1>() + (a.get<1>() + b.get<1>()) * f);
//         area += f * 3;
//     }

//     bg::divide_value(c, area);
//     return Cell(area == 0 ? polygon.outer()[0] : c, 0, polygon);
// }

// point_t polylabel(const polygon_t& polygon, double precision, bool debug) {
//     // find the bounding box of the outer ring
//     box_t envelope;
//     bg::envelope(polygon.outer()[0], envelope);

//     point_t size = envelope.max_corner();
//     bg::subtract_point(size, envelope.min_corner());

//     const double cellSize = std::min(size.get<0>(), size.get<1>());
//     double h = cellSize / 2.;

//     // a priority queue of cells in order of their "potential" (max distance to polygon)
//     auto compareMax = [] (const Cell& a, const Cell& b) {
//         return a.max < b.max;
//     };

//     using Queue = std::priority_queue<Cell, std::vector<Cell>, decltype(compareMax) >;
//     Queue cellQueue(compareMax);

//     if (cellSize == 0) {
//         return envelope.min_corner();
//     }

//     // cover polygon with initial cells
//     for (double x = envelope.min_corner().get<0>(); x < envelope.max_corner().get<0>(); x += cellSize) {
//         for (double y = envelope.min_corner().get<1>(); y < envelope.max_corner().get<1>(); y += cellSize) {
//             cellQueue.push(Cell(point_t(x + h, y + h), h, polygon));
//         }
//     }

//     // take centroid as the first best guess
//     auto bestCell = getCentroidCell(polygon);

//     point_t temp = envelope.min_corner();
//     bg::divide_value(size, 2.);
//     bg::add_point(temp, size);

//     // second guess: bounding box centroid
//     Cell bboxCell(temp, 0, polygon);
//     if (bboxCell.d > bestCell.d) {
//         bestCell = bboxCell;
//     }

//     auto numProbes = cellQueue.size();
//     while (!cellQueue.empty()) {
//         // pick the most promising cell from the queue
//         auto cell = cellQueue.top();
//         cellQueue.pop();

//         // update the best cell if we found a better one
//         if (cell.d > bestCell.d) {
//             bestCell = cell;

//             if (debug) 
//                 std::cout << "found best " << ::round(1e4 * cell.d) / 1e4 << " after " << numProbes << " probes" << std::endl;
//         }

//         // do not drill down further if there's no chance of a better solution
//         if (cell.max - bestCell.d <= precision) 
//             continue;

//         // split the cell into four cells
//         h = cell.h / 2.;
//         cellQueue.push(Cell(point_t(cell.c.get<0>() - h, cell.c.get<1>() - h), h, polygon));
//         cellQueue.push(Cell(point_t(cell.c.get<0>() + h, cell.c.get<1>() - h), h, polygon));
//         cellQueue.push(Cell(point_t(cell.c.get<0>() - h, cell.c.get<1>() + h), h, polygon));
//         cellQueue.push(Cell(point_t(cell.c.get<0>() + h, cell.c.get<1>() + h), h, polygon));
//         numProbes += 4;
//     }

//     if (debug) {
//         std::cout << "num probes: " << numProbes << std::endl;
//         std::cout << "best distance: " << bestCell.d << std::endl;
//     }

//     return bestCell.c;
// }