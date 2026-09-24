#include "path_analysis/json_file.h"
#include "path_analysis/path.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <boost/geometry.hpp>
#include <boost/json.hpp>

namespace
{
    // -----------------------------------------------------------------------------
    // Analysis parameters
    // -----------------------------------------------------------------------------
    constexpr double PREPROCESSING_MINIMUM_DISTANCE_M = 0.01;
    constexpr double HEADING_MINIMUM_DISTANCE_M = 0.09;
    constexpr double CURVATURE_MINIMUM_DISTANCE_M = 0.09;

    constexpr int CSV_PRECISION = 17;
    constexpr int CONSOLE_PRECISION = 3;
}

int main(int argc, char *argv[])
{
    if (argc == 2 && std::string(argv[1]) == "--help")
    {
        std::cout << "Usage: analyze_path recording.json [trajectory.csv] [cleaned_area.csv] [results.json]\n";
        return 0;
    }
    if (argc < 2 || argc > 5)
    {
        std::cerr << "Usage: analyze_path recording.json [trajectory.csv] [cleaned_area.csv] [results.json]\n";
        return 1;
    }
    try
    {
        using namespace path_analysis;
        const auto recording = load_recording(argv[1]);
        // Keep raw samples by default; set a positive threshold to filter stationary clusters.
        const auto path = preprocess_path(recording.path, PREPROCESSING_MINIMUM_DISTANCE_M);
        const auto trajectory = estimate_trajectory(path, HEADING_MINIMUM_DISTANCE_M);
        const auto curvatures = estimate_curvatures(trajectory, CURVATURE_MINIMUM_DISTANCE_M);
        const double length = path_length(path);
        const double area = cleaned_area(trajectory, recording.cleaning_gadget);
        const double seconds = traversal_time(path, curvatures);

        if (argc >= 3)
        {
            // Avoid overwriting the input with the optional CSV output.
            if (std::filesystem::weakly_canonical(argv[1]) == std::filesystem::weakly_canonical(argv[2]) ||
                (std::filesystem::exists(argv[2]) && std::filesystem::equivalent(argv[1], argv[2])))
            {
                throw std::runtime_error("CSV output must differ from the input recording");
            }
            std::ofstream output(argv[2]);
            if (!output)
                throw std::runtime_error("Cannot open CSV output");
            output << "s_m,x_m,y_m,heading_rad,curvature_per_m,speed_m_per_s\n"
                   // output << "x_m,y_m,heading_rad,curvature_per_m,speed_m_per_s\n"
                   << std::setprecision(17);
            double s = 0.0;
            for (std::size_t i = 0; i < trajectory.size(); ++i)
            {
                const auto &pose = trajectory[i];
                output << s << ',';
                output << pose.position.x() << ',' << pose.position.y() << ',';
                if (std::isfinite(pose.heading))
                    output << pose.heading;
                output << ',';
                // Curvature and speed refer to the outgoing segment; last row is blank.
                if (i < curvatures.size())
                    output << curvatures[i] << ',' << speed_model(curvatures[i]);
                else
                    output << ',';
                output << '\n';
                if (i + 1 < path.size())
                {
                    s += boost::geometry::distance(
                        path[i],
                        path[i + 1]);
                }
            }
            output.close();
            if (!output)
                throw std::runtime_error("Failed to write CSV output");
        }

        if (argc >= 4)
        {
            export_cleaned_area_csv(
                trajectory,
                recording.cleaning_gadget,
                argv[3]);
        }

        if (argc >= 5)
        {
            boost::json::object results;

            results["path_length_m"] = length;
            results["cleaned_area_m2"] = area;
            results["traversal_time_s"] = seconds;

            std::ofstream output(argv[4]);

            if (!output)
                throw std::runtime_error("Cannot open results JSON output");

            output << boost::json::serialize(results) << '\n';

            if (!output)
                throw std::runtime_error("Failed to write results JSON output");
        }

        std::cout << std::fixed << std::setprecision(3)
                  << "Path length:    " << length << " m\n"
                  << "Cleaned area:   " << area << " m^2 (approximate polygon sweep)\n"
                  << "Traversal time: " << seconds << " s\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
