#include "path_analysis/json_file.h"
#include "path_analysis/path.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char* argv[]) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: analyze_path recording.json [trajectory.csv]\n";
        return 0;
    }
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: analyze_path recording.json [trajectory.csv]\n";
        return 1;
    }
    try {
        using namespace path_analysis;
        const auto recording = load_recording(argv[1]);
        // Keep raw samples by default; set a positive threshold to filter stationary clusters.
        const auto path = preprocess_path(recording.path, 0.0);
        const auto trajectory = estimate_trajectory(path);
        const auto curvatures = estimate_curvatures(trajectory);
        const double length = path_length(path);
        const double area = cleaned_area(trajectory, recording.cleaning_gadget);
        const double seconds = traversal_time(path, curvatures);

        if (argc == 3) {
            // Avoid overwriting the input with the optional CSV output.
            if (std::filesystem::weakly_canonical(argv[1]) == std::filesystem::weakly_canonical(argv[2]) ||
                (std::filesystem::exists(argv[2]) && std::filesystem::equivalent(argv[1], argv[2]))) {
                throw std::runtime_error("CSV output must differ from the input recording");
            }
            std::ofstream output(argv[2]);
            if (!output) throw std::runtime_error("Cannot open CSV output");
            output << "x_m,y_m,heading_rad,curvature_per_m,speed_m_per_s\n" << std::setprecision(17);
            for (std::size_t i = 0; i < trajectory.size(); ++i) {
                const auto& pose = trajectory[i];
                output << pose.position.x() << ',' << pose.position.y() << ',';
                if (std::isfinite(pose.heading)) output << pose.heading;
                output << ',';
                // Curvature and speed refer to the outgoing segment; last row is blank.
                if (i < curvatures.size()) output << curvatures[i] << ',' << speed_model(curvatures[i]);
                else output << ',';
                output << '\n';
            }
            output.close();
            if (!output) throw std::runtime_error("Failed to write CSV output");
        }
        std::cout << std::fixed << std::setprecision(3)
                  << "Path length:    " << length << " m\n"
                  << "Cleaned area:   " << area << " m^2 (approximate polygon sweep)\n"
                  << "Traversal time: " << seconds << " s\n";
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
