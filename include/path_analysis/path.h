#pragma once

#include "path_analysis/recording.h"

#include <cstddef>
#include <string>

namespace path_analysis
{

    // Sum of consecutive distances in metres. Empty/single-point paths return zero.
    double path_length(const std::vector<Point> &path);

    // Collapse stationary clusters, keeping their first and last samples.
    // Compare with the cluster's first point so small movements can accumulate.
    // min_movement = 0 disables filtering; negative/nonfinite values are invalid.
    std::vector<Point> preprocess_path(const std::vector<Point> &path, double min_movement = 0.01);

    // Heading at index in an already preprocessed path, radians CCW from world +x.
    // Expand equally before/after the point until the endpoint chord spans
    // minimum_distance; at a boundary continue on the available side.
    // If unreachable, use the longest nonzero chord; if none exists, return NaN.
    // Invalid index throws out_of_range; invalid distance throws invalid_argument.
    double estimate_heading(const std::vector<Point> &path, std::size_t index,
                            double minimum_distance = 0.09);

    // Combine each preprocessed point with estimate_heading(); no filtering here.
    // Preserves position/order; empty input returns empty. Distance must be positive.
    Trajectory estimate_trajectory(const std::vector<Point> &path,
                                   double minimum_distance = 0.09);

    // Transform a robot-frame point into world coordinates using a pose.
    Point to_world(Point local, const Pose &pose);

    // One unsigned curvature per segment: shortest heading change / segment length.
    // Zero-length segments have zero curvature. Undefined headings give zero here,
    // a deliberate fallback for stationary/ambiguous samples. Units: 1/metre.
    std::vector<double> estimate_curvatures(const Trajectory &trajectory);

    // Challenge speed model, m/s: 1.10 below 0.5/m, linear down to 0.15 at 10/m.
    // Curvature is unsigned; negative/nonfinite values throw invalid_argument.
    double speed_model(double curvature);

    // Sum segment_length / speed_model(segment_curvature), seconds.
    // Requires exactly max(path.size() - 1, 0) curvatures, otherwise invalid_argument.
    // Empty/single-point paths return zero.
    double traversal_time(const std::vector<Point> &path,
                          const std::vector<double> &segment_curvatures);

    // Area swept by the gadget, square metres; count overlapping regions only once.
    // Interpolate position and shortest-angle rotation between consecutive poses.
    // Empty/single-point paths sweep zero area. Skip segments with undefined headings.
    // Boost.Geometry unions swept triangles; pose steps <= 2 cm and 2 degrees.
    // Union coordinates are rounded to 10 micrometres; rotation is approximated.
    // Polygon area includes holes and disconnected regions.
    double cleaned_area(const Trajectory &trajectory,
                        const std::array<Point, 2> &cleaning_gadget);

    // Export the cleaned area polygon to a JSON file for visualization. The polygon is the union of all gadget sweeps.                    
    void export_cleaned_area_csv(
        const Trajectory &trajectory,
        const std::array<Point, 2> &gadget,
        const std::string &filename);
} // namespace path_analysis
