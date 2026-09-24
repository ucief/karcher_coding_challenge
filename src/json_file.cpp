#include "path_analysis/json_file.h"

#include <boost/json.hpp>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace path_analysis {
namespace {

std::vector<Point> read_points(const boost::json::object& document, const char* name) {
    const auto* value = document.if_contains(name);
    if (!value || !value->is_array()) {
        throw std::runtime_error(std::string(name) + " must be an array");
    }
    std::vector<Point> points;
    for (const auto& entry : value->as_array()) {
        if (!entry.is_array() || entry.as_array().size() != 2) {
            throw std::runtime_error(std::string(name) + ": each point needs two numbers");
        }
        const auto& pair = entry.as_array();
        if (!pair[0].is_number() || !pair[1].is_number()) {
            throw std::runtime_error(std::string(name) + ": each point needs two numbers");
        }
        const Point point{pair[0].to_number<double>(), pair[1].to_number<double>()};
        if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
            throw std::runtime_error(std::string(name) + ": coordinates must be finite");
        }
        points.push_back(point);
    }
    return points;
}

} // namespace

Recording read_recording(std::istream& input) {
    std::ostringstream text;
    text << input.rdbuf();
    if (input.bad()) throw std::runtime_error("Failed to read recording");
    boost::system::error_code error;
    const auto document = boost::json::parse(text.str(), error);
    if (error) throw std::runtime_error("Invalid recording JSON: " + error.message());
    if (!document.is_object()) throw std::runtime_error("Expected a JSON object");

    Recording recording;
    recording.path = read_points(document.as_object(), "path");
    recording.robot = read_points(document.as_object(), "robot");
    const auto gadget = read_points(document.as_object(), "cleaning_gadget");
    if (recording.robot.size() < 3) {
        throw std::runtime_error("Robot footprint needs at least three vertices");
    }
    if (gadget.size() != 2 ||
        (gadget[0].x() == gadget[1].x() && gadget[0].y() == gadget[1].y())) {
        throw std::runtime_error("Cleaning gadget needs two distinct endpoints");
    }
    recording.cleaning_gadget = {gadget[0], gadget[1]};
    return recording;
}

Recording load_recording(const std::filesystem::path& filename) {
    std::ifstream input(filename);
    if (!input) throw std::runtime_error("Cannot open recording: " + filename.string());
    return read_recording(input);
}

} // namespace path_analysis
