#include <iostream>
#include <sstream>
#include <string>
#include <cstdio>
#include <random>
#include <cmath>
#include <chrono>

#include <data_types/vector.h>
#include <robot/controls.h>

int python_pipeline(){
    
    FILE* pipe = popen("python3 scripts/vision.py 2>/dev/null", "r");
    if (!pipe) {
        std::cerr << "Failed to open pipe to Python process\n";
        return 1;
    }

    char buffer[4096];

    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        std::string line(buffer);

        // Remove trailing newline
        if (!line.empty() && line.back() == '\n') {
            line.pop_back();
        }

        if (line.empty()) {
            continue;
        }

        // Ignore NO_HAND lines
        if (line == "NO_HAND") {
            //std::cout << "No hand detected\n";
            continue;
        }

        // Split CSV line by commas
        std::vector<std::string> fields;
        std::stringstream ss(line);
        std::string item;

        while (std::getline(ss, item, ',')) {
            fields.push_back(item);
        }

        // Expected format:
        // HAND,timestamp,handedness,score,
        // NAME,x,y,z,
        // NAME,x,y,z,...
        if (fields.size() < 8 || fields[0] != "HAND") {
            //std::cerr << "bad line: " << line << "\n";
            continue;
        }

        std::string timestamp  = fields[1];
        std::string handedness = fields[2];
        std::string score_str  = fields[3];

        try {
            float score = std::stof(score_str);

            // std::cout << "timestamp = " << timestamp
            //           << ", handedness = " << handedness
            //           << ", score = " << score << "\n";

            // Landmark records begin at index 4 in groups of 4:
            // [name, x, y, z]
            bool found_wrist = false;

            for (size_t i = 4; i + 3 < fields.size(); i += 4) {
                const std::string& name = fields[i];

                float x = std::stof(fields[i + 1]);
                float y = std::stof(fields[i + 2]);
                float z = std::stof(fields[i + 3]);

                if (name == "WRIST") {
                    // std::cout << "WRIST -> x = " << x
                    //           << ", y = " << y
                    //           << ", z = " << z << "\n";
                    //std::cout << "c " << x << " " << y << " 0.5" << "\n";
                    //std::cout << "F\n";
                    found_wrist = true;
                    break;
                }
            }

            if (!found_wrist) {
                //std::cerr << "WRIST not found in line: " << line << "\n";
            }

        } catch (const std::exception& e) {
            std::cerr << "parse error: " << e.what()
                      << " | line: " << line << "\n";
        }
    }

    int status = pclose(pipe);
    std::cout << "Python process exited with status: " << status << "\n";
}

int main() {
    std::cout.setf(std::ios::unitbuf);
    //python_pipeline();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_angle(0.0, 2*M_PI);
    std::uniform_real_distribution<double> dist_radius(0.0, 1.0);

    double R = 2.0;
    control_unit* arm_control = new control_unit();

    using clock = std::chrono::steady_clock;
    auto last_update = clock::now();

    while(true){
        auto now = clock::now();
        
        if(now - last_update >= std::chrono::seconds(3)){
            last_update = now;

            double theta = dist_angle(gen);
            double r = R * std::sqrt(dist_radius(gen));

            double x = r * std::cos(theta);
            double y = r * std::sin(theta);
            std::cout << "!x: " << x << ", y: " << y << std::endl;
            arm_control->controls(vector(x, y, 0));
            animFlush();
        }
        
    }

    
    
    return 0;
}