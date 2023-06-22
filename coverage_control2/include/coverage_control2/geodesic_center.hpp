#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/arithmetic/arithmetic.hpp> 

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>

namespace bg = boost::geometry;

typedef bg::model::point<double, 2, bg::cs::cartesian> point_t;
typedef bg::model::polygon<point_t> polygon_t;
typedef bg::model::box<point_t> box_t;

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

struct Cell {
    Cell(const point_t& c_, double h_, const polygon_t& polygon)
        : c(c_), h(h_), d(pointToPolygonDist(c, polygon)), max(d + h * std::sqrt(2)) { }

    point_t c; // cell center
    double h; // half the cell size
    double d; // distance from cell center to polygon
    double max; // max distance to polygon within a cell
};

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

point_t polylabel(const polygon_t& polygon, double precision = 1., bool debug = false) {
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
