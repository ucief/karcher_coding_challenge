#include "path_analysis/json_file.h"

#include "test_helpers.h"
#include <sstream>
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
    CHECK_EQ(recording.path.size(), 3u); // Keep duplicates and order.
    if (recording.path.size() == 3) {
        CHECK_EQ(recording.path[1].x, 0);
        CHECK_EQ(recording.path[2].x, 1.5);
        CHECK_EQ(recording.path[2].y, -2);
    }
    CHECK_EQ(recording.robot.size(), 4u);
    CHECK_EQ(recording.cleaning_gadget[0].x, 0.15);
    CHECK_EQ(recording.cleaning_gadget[1].y, -0.36);
    const auto empty = parse(R"({"path": [], "robot": [[0,0],[1,0],[0,1]],
                                 "cleaning_gadget": [[0,0],[0,1]]})");
    CHECK(empty.path.empty());
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
        CHECK_THROWS(parse(json), std::runtime_error);
    }
}

void test_files() {
    const auto recording = load_recording(TEST_DATA_FILE);
    CHECK_EQ(recording.path.size(), 162u);
    CHECK_EQ(recording.robot.size(), 4u);
    CHECK_EQ(recording.cleaning_gadget[0].x, 0.15);
    // A regular file cannot have children: guaranteed invalid without temp files.
    CHECK_THROWS(load_recording(std::filesystem::path(TEST_DATA_FILE) / "missing.json"),
                      std::runtime_error);
}

int main() {
    test_valid_input();
    test_invalid_input();
    test_files();
    return report_errors();
}