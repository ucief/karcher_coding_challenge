#pragma once

#include <boost/geometry/geometries/point_xy.hpp>

#include <array>
#include <limits>
#include <vector>

namespace path_analysis
{

    using Point = boost::geometry::model::d2::point_xy<double>;

    // A single 2D pose. An unknown heading is NaN.
    struct Pose
    {
        Point position;
        double heading = std::numeric_limits<double>::quiet_NaN(); // Radians, CCW from +x.
    };

    // Ordered poses.
    using Trajectory = std::vector<Pose>;

    struct Recording
    {
        std::vector<Point> path;              // World coordinates, [m]: Path samples.
        std::vector<Point> robot;             // Robot-frame, [m]: Robot-footprint polygon.
        std::array<Point, 2> cleaning_gadget; // Robot-frame, [m]: Cleaning gadget line segment.
    };

} // namespace path_analysis
