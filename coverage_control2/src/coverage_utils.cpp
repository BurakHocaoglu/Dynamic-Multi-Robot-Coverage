#include "coverage_control2/coverage_utils.h"

SkeletalGraph::SkeletalGraph(uint32_t c) {
	count = c;
	// graph = MatrixXd::Zero(c, c);
	graph = MatrixXd::Constant(c, c, std::numeric_limits<int>::max());
	next_id_available = 0;
}

int SkeletalGraph::getVertexId(int vid, Point_2 p) {
	int the_id;

	std::unordered_map<int, int>::iterator itr = id_map.find(vid);

	if (itr == id_map.end()) {
		the_id = next_id_available;

		vertex_map[the_id] = SkeletalNode{vid, 
										  Vector2d(CGAL::to_double(p.x()), CGAL::to_double(p.y()))};

		id_map[vid] = the_id;
		next_id_available++;

		// std::cout << "Allocating " << the_id << " for VID " << vid << " | next: " << next_id_available << std::endl;
	} else {
		the_id = itr->second;
		// std::cout << "*** Allocated " << the_id << " for VID " << vid << " | next: " << next_id_available << std::endl;
	}

	// std::cout << std::endl;
	// for (auto entry : id_map) {
	// 	std::cout << "Entry: " << entry.first << " - " << entry.second << std::endl;
	// }
	// std::cout << std::endl;

	return the_id;
}

void SkeletalGraph::addEdge(int vid1, Point_2 p1, int vid2, Point_2 p2) {
	int id1 = getVertexId(vid1, p1);
	// std::cout << "Allocating vertex id (1): " << id1 << "...\n";
	int id2 = getVertexId(vid2, p2);
	// std::cout << "Allocating vertex id (2): " << id2 << "...\n";

	double D = sqrt(pow(CGAL::to_double(p1.x() - p2.x()), 2) + 
					pow(CGAL::to_double(p1.y() - p2.y()), 2));

	// printf("Edge [VID: %d, GID: %d] - [VID: %d, GID: %d]: %.5f\n", vid1, id1, vid2, id2, D);

	// std::cout << "Adding edge...\n";
	graph(id1, id2) = D;
	graph(id2, id1) = D;
}

// ---------------------------------------------------------------------------------------------
// Copied from (and modified slightly, for Eigen::Matrix and extra functionality) 
// https://github.com/vsmolyakov/cpp/blob/master/graphs/apsp_floyd_warshall.cpp
Vector2d SkeletalGraph::getCentroid() {
	// std::cout << graph << std::endl;
	// std::cout << std::endl;

	for (int k = 0; k < count; k++) {
		for (int i = 0; i < count; i++) {
			for (int j = 0; j < count; j++) {
				if (graph(i, k) + graph(k, j) < graph(i, j)) 
					graph(i, j) = graph(i, k) + graph(k, j);
			}
		}
	}

	auto dists = graph.rowwise().sum();

	// int min_idx = 0;
	// double min_val = dists(0);
	// for (size_t i = 1; i < count; i++) {
	// 	if (min_val > dists(i)) {
	// 		min_val = dists(i);
	// 		min_idx = i;
	// 	}
	// }

	// return vertex_map[min_idx].point;

	std::vector<std::pair<double, size_t> > distInfo(count);
	for (size_t i = 0; i < count; i++) {
		distInfo[i] = std::make_pair(dists(i), i);
	}

	std::sort(distInfo.begin(), distInfo.end(), [](std::pair<double, size_t> a, 
												   std::pair<double, size_t> b) {
		return a < b;
	});

	return (vertex_map[distInfo[0].second].point + vertex_map[distInfo[1].second].point) / 2.;
	// return vertex_map[distInfo[0].second].point + vertex_map[distInfo[1].second].point;
	// return (vertex_map[distInfo[0].second].point + 
	// 		vertex_map[distInfo[1].second].point + 
	// 		vertex_map[distInfo[2].second].point) / 3.;
}
// ---------------------------------------------------------------------------------------------

uint32_t SkeletalGraph::getCount() {
	return count;
}

// =============================================================================================

SkeletalBoostGraph::SkeletalBoostGraph() {
	// this->names = NameMap(get(&SkeletalNode::id, this->g));
}

Vertex SkeletalBoostGraph::createVertex(int vid, Point_2 p) {
	// return boost::add_vertex(SkeletalNode(vid, p), this->g);

	SkeletalNode new_node;
	new_node.id = vid;
	new_node.point = Vector2d(CGAL::to_double(p.x()), CGAL::to_double(p.y()));

	Vertex v;
	bool inserted;
	std::map<uint32_t, Vertex>::iterator itr;

	boost::tie(itr, inserted) = this->vertex_map.insert(std::make_pair(vid, Vertex()));
	if (inserted) {
		v = add_vertex(new_node, this->g);
		itr->second = v;
	} else {
		v = itr->second;
	}

	return v;
}

void SkeletalBoostGraph::addEdge(int vid1, Point_2 p1, int vid2, Point_2 p2) {
	Vertex v1 = this->createVertex(vid1, p1);
	Vertex v2 = this->createVertex(vid2, p2);

	double D = sqrt(pow(CGAL::to_double(p1.x() - p2.x()), 2) + 
					pow(CGAL::to_double(p1.y() - p2.y()), 2));

	boost::add_edge(v1, v2, D, g);
}

void SkeletalBoostGraph::clear() {
	//
}

Vector2d SkeletalBoostGraph::getCentroid() {
	DistanceMatrix distances(num_vertices(this->g));
	DistanceMatrixMap dm(distances, this->g);

	WeightMap weight_pmap = boost::get(boost::edge_weight, this->g);

	bool valid = floyd_warshall_all_pairs_shortest_paths(this->g, dm, boost::weight_map(weight_pmap));

	if (valid) {
		ROS_INFO("Valid Floyd-Warshall!");
	} else {
		ROS_WARN("Invalid Floyd-Warshall! Negative cycle exists!");
		return Vector2d(0, 0);
	}

	EccentricityContainer eccs(num_vertices(this->g));
	EccentricityMap em(eccs, this->g);

	int r, d, min_ecc;
	Vector2d centroid(0, 0);

	boost::tie(r, d) = all_eccentricities(g, dm, em);

	boost::graph_traits<Graph>::vertex_iterator i, end;
	for(boost::tie(i, end) = vertices(g); i != end; ++i) {
		int ecc_i = get(em, *i);
		if (min_ecc > ecc_i) {
			centroid = g[*i].point;
			min_ecc = ecc_i;
		}
	}

	return centroid;
}

uint32_t SkeletalBoostGraph::getCount(){
	return this->count;
}

// ------------------------------------------------------------------------------------------

double getSegDistSq(const point_t& p, const point_t& a, const point_t& b) {
    double x = a.get<0>();
    double y = a.get<1>();
    double dx = b.get<0>() - x;
    double dy = b.get<1>() - y;

    if (dx != 0 || dy != 0) {
        double t = ((p.get<0>() - x) * dx + (p.get<1>() - y) * dy) / (dx * dx + dy * dy);

        if (t > 1) {
            x = b.get<0>();
            y = b.get<1>();

        } else if (t > 0) {
            x += dx * t;
            y += dy * t;
        }
    }

    dx = p.get<0>() - x;
    dy = p.get<1>() - y;

    return dx * dx + dy * dy;
}

double pointToPolygonDist(const point_t& point, const polygon_t& polygon) {
    bool inside = false;
    auto minDistSq = std::numeric_limits<double>::infinity();

    std::size_t i = 0, j, len;
    for (std::size_t i = 0, len = polygon.outer().size(), j = len - 1; i < len; j = i++) {
        const auto& a = polygon.outer()[i];
        const auto& b = polygon.outer()[j];

        if ((a.get<1>() > point.get<1>()) != (b.get<1>() > point.get<1>()) && 
            (point.get<0>() < (b.get<0>() - a.get<0>()) * (point.get<1>() - a.get<1>()) / (b.get<1>() - a.get<1>()) + a.get<0>())) inside = !inside;

        minDistSq = std::min(minDistSq, getSegDistSq(point, a, b));
    }

    return (inside ? 1 : -1) * std::sqrt(minDistSq);
}

Cell getCentroidCell(const polygon_t& polygon) {
    double area = 0.;
    point_t c(0., 0.);

    for (std::size_t i = 0, len = polygon.outer().size(), j = len - 1; i < len; j = i++) {
        const point_t& a = polygon.outer()[i];
        const point_t& b = polygon.outer()[j];
        auto f = a.get<0>() * b.get<1>() - b.get<1>() * a.get<1>();
        c.set<0>(c.get<0>() + (a.get<0>() + b.get<0>()) * f);
        c.set<1>(c.get<1>() + (a.get<1>() + b.get<1>()) * f);
        area += f * 3;
    }

    bg::divide_value(c, area);
    return Cell(area == 0 ? polygon.outer()[0] : c, 0, polygon);
}

point_t polylabel(const polygon_t& polygon, double precision, bool debug) {
    // find the bounding box of the outer ring
    box_t envelope;
    bg::envelope(polygon.outer()[0], envelope);

    point_t size = envelope.max_corner();
    bg::subtract_point(size, envelope.min_corner());

    const double cellSize = std::min(size.get<0>(), size.get<1>());
    double h = cellSize / 2.;

    // a priority queue of cells in order of their "potential" (max distance to polygon)
    auto compareMax = [] (const Cell& a, const Cell& b) {
        return a.max < b.max;
    };

    using Queue = std::priority_queue<Cell, std::vector<Cell>, decltype(compareMax) >;
    Queue cellQueue(compareMax);

    if (cellSize == 0) {
        return envelope.min_corner();
    }

    // cover polygon with initial cells
    for (double x = envelope.min_corner().get<0>(); x < envelope.max_corner().get<0>(); x += cellSize) {
        for (double y = envelope.min_corner().get<1>(); y < envelope.max_corner().get<1>(); y += cellSize) {
            cellQueue.push(Cell(point_t(x + h, y + h), h, polygon));
        }
    }

    // take centroid as the first best guess
    auto bestCell = getCentroidCell(polygon);

    point_t temp = envelope.min_corner();
    bg::divide_value(size, 2.);
    bg::add_point(temp, size);

    // second guess: bounding box centroid
    Cell bboxCell(temp, 0, polygon);
    if (bboxCell.d > bestCell.d) {
        bestCell = bboxCell;
    }

    auto numProbes = cellQueue.size();
    while (!cellQueue.empty()) {
        // pick the most promising cell from the queue
        auto cell = cellQueue.top();
        cellQueue.pop();

        // update the best cell if we found a better one
        if (cell.d > bestCell.d) {
            bestCell = cell;

            if (debug) 
                std::cout << "found best " << ::round(1e4 * cell.d) / 1e4 << " after " << numProbes << " probes" << std::endl;
        }

        // do not drill down further if there's no chance of a better solution
        if (cell.max - bestCell.d <= precision) 
            continue;

        // split the cell into four cells
        h = cell.h / 2.;
        cellQueue.push(Cell(point_t(cell.c.get<0>() - h, cell.c.get<1>() - h), h, polygon));
        cellQueue.push(Cell(point_t(cell.c.get<0>() + h, cell.c.get<1>() - h), h, polygon));
        cellQueue.push(Cell(point_t(cell.c.get<0>() - h, cell.c.get<1>() + h), h, polygon));
        cellQueue.push(Cell(point_t(cell.c.get<0>() + h, cell.c.get<1>() + h), h, polygon));
        numProbes += 4;
    }

    if (debug) {
        std::cout << "num probes: " << numProbes << std::endl;
        std::cout << "best distance: " << bestCell.d << std::endl;
    }

    return bestCell.c;
}