#include "path_analysis/json_file.h"
#include "path_analysis/path.h"

#include <boost/core/lightweight_test.hpp>
#include <sstream>
#include <cmath>
#include <stdexcept>

using namespace path_analysis;

Recording parse(const char *text)
{
    std::istringstream input(text);
    return read_recording(input);
}

void test_valid_input() {
    const auto recording = parse(R"({
        "path": [[0, 0], [0, 0], [1.5, -2]],
        "robot": [[-1, -1], [1, -1], [1, 1], [-1, 1]],
        "cleaning_gadget": [[0.15, 0.3], [0.15, -0.36]],
        "extra": "ignored"
    })");
    BOOST_TEST_EQ(recording.path.size(), 3u); // Keep duplicates and order.
    if (recording.path.size() == 3) {
        BOOST_TEST_EQ(recording.path[1].x(), 0);
        BOOST_TEST_EQ(recording.path[2].x(), 1.5);
        BOOST_TEST_EQ(recording.path[2].y(), -2);
    }
    BOOST_TEST_EQ(recording.robot.size(), 4u);
    BOOST_TEST_EQ(recording.cleaning_gadget[0].x(), 0.15);
    BOOST_TEST_EQ(recording.cleaning_gadget[1].y(), -0.36);
    const auto empty = parse(R"({"path": [], "robot": [[0,0],[1,0],[0,1]],
                                 "cleaning_gadget": [[0,0],[0,1]]})");
    BOOST_TEST(empty.path.empty());
}

void test_invalid_input() {
    for (const char* json : {
        "not json", "{}", "[]",
        R"({"path": [], "robot": [[0,0],[1,0],[0,1]]})",
        R"({"robot": [[0,0],[1,0],[0,1]], "cleaning_gadget": [[0,0],[0,1]]})",
        R"({"path": [], "cleaning_gadget": [[0,0],[0,1]]})",
        R"({"path": [[0]], "robot": [[0,0],[1,0],[0,1]], "cleaning_gadget": [[0,0],[0,1]]})",
        R"({"path": [[0,1,2]], "robot": [[0,0],[1,0],[0,1]], "cleaning_gadget": [[0,0],[0,1]]})",
        R"({"path": [["0",1]], "robot": [[0,0],[1,0],[0,1]], "cleaning_gadget": [[0,0],[0,1]]})",
        R"({"path": [[null,1]], "robot": [[0,0],[1,0],[0,1]], "cleaning_gadget": [[0,0],[0,1]]})",
        R"({"path": [[1e999,1]], "robot": [[0,0],[1,0],[0,1]], "cleaning_gadget": [[0,0],[0,1]]})",
        R"({"path": [], "robot": [[0,0],[1,0]], "cleaning_gadget": [[0,0],[0,1]]})",
        R"({"path": [], "robot": [[0,0],[1,0],[0,1]], "cleaning_gadget": [[0,0]]})",
        R"({"path": [], "robot": [[0,0],[1,0],[0,1]], "cleaning_gadget": [[0,0],[0,1],[0,2]]})",
        R"({"path": [], "robot": [[0,0],[1,0],[0,1]], "cleaning_gadget": [[0,0],[0,0]]})"
    }) {
        BOOST_TEST_THROWS(parse(json), std::runtime_error);
    }
}

void test_files() {
    const auto recording = load_recording(TEST_DATA_FILE);
    BOOST_TEST_EQ(recording.path.size(), 162u);
    BOOST_TEST_EQ(recording.robot.size(), 4u);
    BOOST_TEST_EQ(recording.cleaning_gadget[0].x(), 0.15);
    // Independent 2.5 mm raster reference was about 8.088 m². Catch lost regions
    // when merging the many small sweep polygons in the real recording.
    const double area = cleaned_area(estimate_trajectory(recording.path), recording.cleaning_gadget);
    BOOST_TEST_LE(std::abs(area - 8.088), 0.02);
    // A regular file cannot have children: guaranteed invalid without temp files.
    BOOST_TEST_THROWS(load_recording(std::filesystem::path(TEST_DATA_FILE) / "missing.json"),
                      std::runtime_error);
}

int main() {
    test_valid_input();
    test_invalid_input();
    test_files();
    return boost::report_errors();
}