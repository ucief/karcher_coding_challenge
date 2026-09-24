#include "path_analysis/path.h"

#include <boost/geometry.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

#include <algorithm>
#include <utility>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <limits>
#include <fstream>
#include <iomanip>
#include <string>
namespace path_analysis
{
    namespace
    {

        namespace bg = boost::geometry;
        using Polygon = bg::model::polygon<Point>;
        using MultiPolygon = bg::model::multi_polygon<Polygon>;

        double angle_change(double from, double to)
        {
            return std::atan2(std::sin(to - from), std::cos(to - from));
        }

        void add_sweep(std::vector<Polygon> &sweeps, Point a, Point b, Point c, Point d)
        {
            bg::model::multi_point<Point> endpoints;
            // Round to five decimal places; coordinates remain in metres.
            for (Point point : {a, b, c, d})
            {
                endpoints.emplace_back(std::round(point.x() * 100000.0) / 100000.0,
                                       std::round(point.y() * 100000.0) / 100000.0);
            }
            Polygon sweep;
            bg::convex_hull(endpoints, sweep);
            if (bg::area(sweep) > 1e-15)
                sweeps.push_back(std::move(sweep));
        }

        MultiPolygon union_sweeps(const std::vector<Polygon> &sweeps)
        {
            MultiPolygon coverage;
            for (const auto &sweep : sweeps)
            {
                if (coverage.empty())
                {
                    coverage.push_back(sweep);
                }
                else
                {
                    MultiPolygon merged;
                    bg::union_(coverage, sweep, merged);
                    coverage = std::move(merged);
                }
            }
            return coverage;
        }

    } // namespace

    double path_length(const std::vector<Point> &path)
    {
        const bg::model::linestring<Point> line(path.begin(), path.end());
        return bg::length(line);
    }

    std::vector<Point> preprocess_path(const std::vector<Point> &path, double min_movement)
    {
        if (!std::isfinite(min_movement) || min_movement < 0)
        {
            throw std::invalid_argument("Minimum movement must be finite and nonnegative");
        }
        if (path.empty() || min_movement == 0)
            return path;

        std::vector<Point> result;
        std::size_t first = 0;
        while (first < path.size())
        {
            std::size_t last = first;
            while (last + 1 < path.size() && bg::distance(path[first], path[last + 1]) < min_movement)
            {
                ++last;
            }
            result.push_back(path[first]);
            if (last != first)
                result.push_back(path[last]);
            first = last + 1;
        }
        return result;
    }

    double estimate_heading(
        const std::vector<Point> &path,
        std::size_t index,
        double minimum_distance)
    {
        if (!std::isfinite(minimum_distance) || minimum_distance <= 0.0)
        {
            throw std::invalid_argument(
                "Minimum distance for heading calculation must be finite and positive");
        }

        if (index >= path.size())
        {
            throw std::out_of_range(
                "Heading point index is outside the path");
        }

        std::size_t left = index;
        std::size_t right = index;

        while (true)
        {
            // Stop if the current window is large enough
            const double window_distance =
                bg::distance(path[left], path[right]);

            if (window_distance >= minimum_distance)
            {
                return std::atan2(
                    path[right].y() - path[left].y(),
                    path[right].x() - path[left].x());
            }

            // Check whether we can expand the window
            const bool can_expand_left = left > 0;
            const bool can_expand_right = right + 1 < path.size();

            // Entire path exhausted
            if (!can_expand_left && !can_expand_right)
            {
                std::cerr
                    << "Warning: could not estimate heading at path index "
                    << index
                    << ": available path window is shorter than minimum_distance="
                    << minimum_distance
                    << " m\n";

                return std::numeric_limits<double>::quiet_NaN();
            }

            // Expand wherever possible
            if (can_expand_left)
                --left;

            if (can_expand_right)
                ++right;
        }
    }

    Trajectory estimate_trajectory(const std::vector<Point> &path, double minimum_distance)
    {
        if (!std::isfinite(minimum_distance) || minimum_distance <= 0)
        {
            throw std::invalid_argument("Minimum distance for heading calculation must be finite and positive");
        }
        Trajectory trajectory;
        trajectory.reserve(path.size());
        for (std::size_t i = 0; i < path.size(); ++i)
        {
            trajectory.push_back({path[i], estimate_heading(path, i, minimum_distance)});
        }
        return trajectory;
    }

    Point to_world(Point local, const Pose &pose)
    {
        const double c = std::cos(pose.heading);
        const double s = std::sin(pose.heading);
        const bg::strategy::transform::matrix_transformer<double, 2, 2> transform(
            c, -s, pose.position.x(),
            s, c, pose.position.y(),
            0, 0, 1);
        Point world;
        bg::transform(local, world, transform);
        return world;
    }

    std::vector<double> estimate_curvatures(const Trajectory &trajectory)
    {
        std::vector<double> curvatures;
        for (std::size_t i = 1; i < trajectory.size(); ++i)
        {
            const Pose &a = trajectory[i - 1];
            const Pose &b = trajectory[i];
            const double length = bg::distance(a.position, b.position);
            double curvature = std::numeric_limits<double>::quiet_NaN();
            if (length > 0 && std::isfinite(a.heading) && std::isfinite(b.heading))
            {
                curvature = std::abs(angle_change(a.heading, b.heading)) / length;
            }
            curvatures.push_back(curvature);
        }
        return curvatures;
    }

    double speed_model(double curvature)
    {
        if (!std::isfinite(curvature) || curvature < 0)
        {
            throw std::invalid_argument("Curvature must be finite and nonnegative");
        }
        if (curvature < 0.5)
            return 1.10;
        if (curvature >= 10.0)
            return 0.15;
        return 1.10 - (1.10 - 0.15) * (curvature - 0.5) / (10.0 - 0.5);
    }

    double traversal_time(const std::vector<Point> &path,
                          const std::vector<double> &segment_curvatures)
    {
        const std::size_t segments = path.empty() ? 0 : path.size() - 1;
        if (segment_curvatures.size() != segments)
        {
            throw std::invalid_argument("Expected one curvature per path segment");
        }
        double seconds = 0;
        for (std::size_t i = 0; i < segments; ++i)
        {
            seconds += bg::distance(path[i], path[i + 1]) / speed_model(segment_curvatures[i]);
        }
        return seconds;
    }

    static MultiPolygon compute_cleaned_region(
        const Trajectory &trajectory,
        const std::array<Point, 2> &gadget)
    {
        if (trajectory.size() < 2)
            return {};

        std::vector<Polygon> sweeps;
        const double max_turn = 2.0 * std::acos(-1.0) / 180.0;

        for (std::size_t i = 1; i < trajectory.size(); ++i)
        {
            const Pose &a = trajectory[i - 1];
            const Pose &b = trajectory[i];

            if (!std::isfinite(a.heading) || !std::isfinite(b.heading))
                continue;

            const double turn = angle_change(a.heading, b.heading);

            const int steps = std::max({1,
                                        static_cast<int>(
                                            std::ceil(bg::distance(a.position, b.position) / 0.02)),
                                        static_cast<int>(
                                            std::ceil(std::abs(turn) / max_turn))});

            Point previous0 = to_world(gadget[0], a);
            Point previous1 = to_world(gadget[1], a);

            for (int step = 1; step <= steps; ++step)
            {
                const double t =
                    static_cast<double>(step) / steps;

                const Pose pose{
                    {(1 - t) * a.position.x() + t * b.position.x(),
                     (1 - t) * a.position.y() + t * b.position.y()},
                    a.heading + t * turn};

                const Point current0 =
                    to_world(gadget[0], pose);

                const Point current1 =
                    to_world(gadget[1], pose);

                add_sweep(
                    sweeps,
                    previous0,
                    previous1,
                    current1,
                    current0);

                previous0 = current0;
                previous1 = current1;
            }
        }

        return union_sweeps(sweeps);
    }

    double cleaned_area(
        const Trajectory &trajectory,
        const std::array<Point, 2> &gadget)
    {
        return bg::area(
            compute_cleaned_region(trajectory, gadget));
    }

    void export_cleaned_area_csv(
        const Trajectory &trajectory,
        const std::array<Point, 2> &gadget,
        const std::string &filename)
    {
        const MultiPolygon coverage =
            compute_cleaned_region(trajectory, gadget);

        std::ofstream output(filename);

        if (!output)
            throw std::runtime_error(
                "Cannot open cleaned area CSV output");

        output
            << "polygon_id,ring_id,is_hole,x_m,y_m\n"
            << std::setprecision(17);

        for (std::size_t polygon_id = 0;
             polygon_id < coverage.size();
             ++polygon_id)
        {
            const auto &polygon = coverage[polygon_id];

            for (const auto &point : polygon.outer())
            {
                output
                    << polygon_id << ','
                    << 0 << ','
                    << 0 << ','
                    << point.x() << ','
                    << point.y() << '\n';
            }

            for (std::size_t hole_id = 0;
                 hole_id < polygon.inners().size();
                 ++hole_id)
            {
                for (const auto &point :
                     polygon.inners()[hole_id])
                {
                    output
                        << polygon_id << ','
                        << hole_id + 1 << ','
                        << 1 << ','
                        << point.x() << ','
                        << point.y() << '\n';
                }
            }
        }

        if (!output)
            throw std::runtime_error(
                "Failed to write cleaned area CSV output");
    }

} // namespace path_analysis
