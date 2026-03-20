#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdio>



int main() {
    std::cout.setf(std::ios::unitbuf);
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

    return 0;
}