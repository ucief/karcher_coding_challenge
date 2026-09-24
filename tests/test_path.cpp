#include "path_analysis/path.h"

#include <boost/core/lightweight_test.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace path_analysis;
const double pi = std::acos(-1.0);

void check_near(double actual, double expected, double tolerance = 1e-10) {
    BOOST_TEST(std::isfinite(actual));
    BOOST_TEST_LE(std::abs(actual - expected), tolerance);
}

void test_length() {
    check_near(path_length({}), 0);
    check_near(path_length({{1, 2}}), 0);
    check_near(path_length({{0, 0}, {3, 4}, {3, 4}, {6, 4}}), 8);
    // Raw jitter remains part of the recorded distance.
    check_near(path_length({{0, 0}, {0.001, 0}, {0, 0}}), 0.002);
}

void test_preprocessing() {
    BOOST_TEST(preprocess_path({}).empty());
    const std::vector<Point> raw{{0, 0}, {0.001, 0}, {0.002, 0},
                                 {0.1, 0}, {0.101, 0}, {0.102, 0}};
    const auto filtered = preprocess_path(raw);
    if (BOOST_TEST_EQ(filtered.size(), 4u)) {
        check_near(filtered[0].x(), 0);
        check_near(filtered[1].x(), 0.002);
        check_near(filtered[2].x(), 0.1);
        check_near(filtered[3].x(), 0.102);
    }
    const auto unchanged = preprocess_path(raw, 0);
    BOOST_TEST_EQ(unchanged.size(), raw.size());
    for (std::size_t i = 0; i < unchanged.size() && i < raw.size(); ++i) {
        BOOST_TEST_EQ(unchanged[i].x(), raw[i].x());
        BOOST_TEST_EQ(unchanged[i].y(), raw[i].y());
    }
    BOOST_TEST_EQ(preprocess_path({{0, 0}}).size(), 1u);
    BOOST_TEST_EQ(preprocess_path({{0, 0}, {0, 0}, {0, 0}}).size(), 2u);
    // Small consecutive movements must not merge the entire path into one cluster.
    const auto slow = preprocess_path({{0, 0}, {0.004, 0}, {0.008, 0},
                                      {0.012, 0}, {0.016, 0}, {0.020, 0}});
    if (BOOST_TEST_EQ(slow.size(), 4u)) check_near(slow[2].x(), 0.012);
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()}) {
        BOOST_TEST_THROWS(preprocess_path(raw, invalid), std::invalid_argument);
    }
}

void test_single_heading() {
    const auto path = preprocess_path({{-0.06, 0}, {-0.02, 0}, {0, 0},
                                       {0.02, 0.02}, {0.06, 0.08}}, 0);
    check_near(estimate_heading(path, 2), std::atan2(0.08, 0.12));
    check_near(estimate_heading(path, 0), std::atan2(0.08, 0.12));
    check_near(estimate_heading(path, 4), std::atan2(0.08, 0.06));
    check_near(estimate_heading(path, 2, 0.03), std::atan2(0.02, 0.04));
    BOOST_TEST(std::isnan(estimate_heading({{0, 0}}, 0)));
    BOOST_TEST_THROWS(estimate_heading({}, 0), std::out_of_range);
    BOOST_TEST_THROWS(estimate_heading(path, path.size()), std::out_of_range);
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()}) {
        BOOST_TEST_THROWS(estimate_heading(path, 0, invalid), std::invalid_argument);
    }
    const auto filtered = preprocess_path({{0, 0}, {0.001, 0}, {0.002, 0}, {0.1, 0}});
    const auto trajectory = estimate_trajectory(filtered);
    BOOST_TEST_EQ(trajectory.size(), filtered.size());
    for (const auto& pose : trajectory) check_near(pose.heading, 0);
}

void test_heading_window() {
    // The nearest neighbours span < 0.09 m; the centre needs two expansions.
    const std::vector<Point> path{{-0.06, 0}, {-0.02, 0}, {0, 0},
                                  {0.02, 0.02}, {0.06, 0.08}};
    const auto trajectory = estimate_trajectory(path);
    BOOST_TEST_EQ(trajectory.size(), path.size());
    if (trajectory.size() != path.size()) return;
    check_near(trajectory[2].heading, std::atan2(0.08, 0.12));
    check_near(trajectory.front().heading, std::atan2(0.08, 0.12));
    check_near(trajectory.back().heading, std::atan2(0.08, 0.06));

    const auto shorter = estimate_trajectory(path, 0.03);
    BOOST_TEST_EQ(shorter.size(), path.size());
    if (shorter.size() == path.size()) {
        check_near(shorter[2].heading, std::atan2(0.02, 0.04));
    }
    // A sufficient local chord must stop expanding before a later turn.
    const auto local = estimate_trajectory({{-0.05, 0}, {0, 0}, {0.05, 0}, {0.05, 1}});
    if (BOOST_TEST_EQ(local.size(), 4u)) check_near(local[1].heading, 0);
}

void test_heading_edge_cases() {
    BOOST_TEST(estimate_trajectory({}).empty());
    for (const auto& path : std::vector<std::vector<Point>>{
             {{0, 0}}, {{0, 0}, {0, 0}, {0, 0}}}) {
        const auto trajectory = estimate_trajectory(path);
        BOOST_TEST_EQ(trajectory.size(), path.size());
        for (const auto& pose : trajectory) BOOST_TEST(std::isnan(pose.heading));
    }
    // Short paths use the available baseline and retain their direction.
    const auto short_path = estimate_trajectory({{0, 0.02}, {0, 0}});
    BOOST_TEST_EQ(short_path.size(), 2u);
    for (const auto& pose : short_path) check_near(pose.heading, -pi / 2);

    // Raw duplicates and the stationary tail are retained, with a stable heading.
    const std::vector<Point> raw{{0, 0}, {0, 0}, {0.1, 0}, {0.2, 0},
                                 {0.201, 0.001}, {0.199, -0.001}, {0.2, 0}};
    const auto trajectory = estimate_trajectory(raw);
    BOOST_TEST_EQ(trajectory.size(), raw.size());
    for (const auto& pose : trajectory) check_near(pose.heading, 0, 0.02);
    for (std::size_t i = 0; i < trajectory.size() && i < raw.size(); ++i) {
        BOOST_TEST_EQ(trajectory[i].position.x(), raw[i].x());
        BOOST_TEST_EQ(trajectory[i].position.y(), raw[i].y());
    }

    const auto reversal = estimate_trajectory({{0, 0}, {0.02, 0}, {0, 0}});
    if (BOOST_TEST_EQ(reversal.size(), 3u)) {
        check_near(reversal[0].heading, 0);
        BOOST_TEST(std::isnan(reversal[1].heading));
    }
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()}) {
        BOOST_TEST_THROWS(estimate_trajectory({{0, 0}}, invalid), std::invalid_argument);
    }
}

void test_transform() {
    const auto translated = to_world({0.15, 0.3}, {{2, 3}, 0});
    check_near(translated.x(), 2.15);
    check_near(translated.y(), 3.3);
    const auto rotated = to_world({0.15, 0.3}, {{2, 3}, pi / 2});
    check_near(rotated.x(), 1.7);
    check_near(rotated.y(), 3.15);
}

void test_curvature() {
    BOOST_TEST(estimate_curvatures({}).empty());
    BOOST_TEST(estimate_curvatures({{{0, 0}, 0}}).empty());
    const auto straight = estimate_curvatures({{{0, 0}, 0}, {{1, 0}, 0}, {{3, 0}, 0}});
    BOOST_TEST_EQ(straight.size(), 2u);
    for (double value : straight) check_near(value, 0);
    const auto turn = estimate_curvatures({{{0, 0}, 0}, {{0.5, 0}, -0.25}});
    if (BOOST_TEST_EQ(turn.size(), 1u)) check_near(turn[0], 0.5);
    const auto wrap = estimate_curvatures({{{0, 0}, pi - 0.01}, {{1, 0}, -pi + 0.01}});
    if (BOOST_TEST_EQ(wrap.size(), 1u)) check_near(wrap[0], 0.02);
    const auto duplicate = estimate_curvatures({{{0, 0}, 0}, {{0, 0}, 1}});
    if (BOOST_TEST_EQ(duplicate.size(), 1u)) check_near(duplicate[0], 0);
    const auto unknown = estimate_curvatures({{{0, 0}, std::numeric_limits<double>::quiet_NaN()}, {{1, 0}, 0}});
    if (BOOST_TEST_EQ(unknown.size(), 1u)) check_near(unknown[0], 0);
}

void test_speed_and_time() {
    check_near(speed_model(0), 1.10);
    check_near(speed_model(0.49), 1.10);
    check_near(speed_model(0.5), 1.10);
    check_near(speed_model(5.25), 0.625);
    check_near(speed_model(10), 0.15);
    check_near(speed_model(20), 0.15);
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()}) {
        BOOST_TEST_THROWS(speed_model(invalid), std::invalid_argument);
    }
    check_near(traversal_time({}, {}), 0);
    check_near(traversal_time({{0, 0}}, {}), 0);
    check_near(traversal_time({{0, 0}, {1.1, 0}}, {0}), 1);
    check_near(traversal_time({{0, 0}, {1.1, 0}, {1.25, 0}}, {0, 10}), 2);
    check_near(traversal_time({{0, 0}, {0, 0}}, {0}), 0);
    BOOST_TEST_THROWS(traversal_time({{0, 0}, {1, 0}}, {}), std::invalid_argument);
}

void test_cleaned_area() {
    const std::array<Point, 2> gadget{{{0.15, -0.3}, {0.15, 0.3}}};
    check_near(cleaned_area({}, gadget), 0);
    check_near(cleaned_area({{{0, 0}, 0}}, gadget), 0);
    check_near(cleaned_area({{{0, 0}, 0}, {{0, 0}, 0}}, gadget), 0);
    // A 2 m straight sweep with a 0.6 m gadget is a 1.2 m² rectangle.
    check_near(cleaned_area({{{0, 0}, 0}, {{2, 0}, 0}}, gadget), 1.2, 0.01);
    check_near(cleaned_area({{{0, 0}, 0}, {{1, 0}, 0}, {{2, 0}, 0}}, gadget), 1.2, 0.01);
    // Retracing with the same orientation must not double-count coverage.
    check_near(cleaned_area({{{0, 0}, 0}, {{2, 0}, 0}, {{0, 0}, 0}}, gadget), 1.2, 0.01);
    // Rotating/translating the entire scene preserves its area.
    check_near(cleaned_area({{{3, 4}, pi / 2}, {{3, 6}, pi / 2}}, gadget), 1.2, 0.01);
    // Undefined poses break the sweep: disconnected regions both count.
    const double unknown = std::numeric_limits<double>::quiet_NaN();
    const Trajectory separated{{{0, 0}, 0}, {{2, 0}, 0}, {{3, 0}, unknown},
                               {{4, 0}, 0}, {{6, 0}, 0}};
    check_near(cleaned_area(separated, gadget), 2.4, 0.01);
    // Arbitrary orientation should preserve area within the numerical tolerance.
    const double diagonal = std::sqrt(2.0);
    check_near(cleaned_area({{{0, 0}, pi / 4}, {{diagonal, diagonal}, pi / 4}}, gadget), 1.2, 0.01);
    // A rotating offset segment leaves a hole: annulus area is pi * (2² - 1²).
    const std::array<Point, 2> offset{{{1, 0}, {2, 0}}};
    const Trajectory revolution{{{0, 0}, 0}, {{0, 0}, pi / 2}, {{0, 0}, pi},
                                {{0, 0}, -pi / 2}, {{0, 0}, 0}};
    check_near(cleaned_area(revolution, offset), 3 * pi, 0.03);
    // A radial unit gadget turning 90 degrees sweeps a quarter-disc.
    const std::array<Point, 2> radial{{{0, 0}, {1, 0}}};
    check_near(cleaned_area({{{0, 0}, 0}, {{0, 0}, pi / 2}}, radial), pi / 4, 0.01);
    // Wraparound must use the short rotation, not a nearly full revolution.
    check_near(cleaned_area({{{0, 0}, pi - 0.1}, {{0, 0}, -pi + 0.1}}, radial), 0.1, 0.005);
    check_near(cleaned_area({{{0, 0}, std::numeric_limits<double>::quiet_NaN()}, {{1, 0}, 0}}, gadget), 0);
}

int main() {
    test_length();
    test_preprocessing();
    test_single_heading();
    test_heading_window();
    test_heading_edge_cases();
    test_transform();
    test_curvature();
    test_speed_and_time();
    test_cleaned_area();
    return boost::report_errors();
}
