#include <iostream>
#include <sstream>
#include <string>
#include <cstdio>
#include <random>
#include <cmath>
#include <chrono>

#include <data_types/vector.h>
#include <robot/controls.h>

int arm_tests(){

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_angle(0.0, 2*M_PI);
    std::uniform_real_distribution<double> dist_radius(0.0, 1.0);

    double R = 2.0;
    control_unit* arm_control = new control_unit();

    using clock = std::chrono::steady_clock;
    auto last_update = clock::now();


    auto now = clock::now();
    
    if(now - last_update >= std::chrono::seconds(3)){
        last_update = now;

        double theta = dist_angle(gen);
        double r = R * std::sqrt(dist_radius(gen));

        double x = r * std::cos(theta);
        double y = r * std::sin(theta);
        std::cout << "!target x: " << x << ", y: " << y << std::endl;
        arm_control->controls(vector(x, y, 0));
        // debug

    }
    setColor(0, 255, 0);
    drawSphere(vector(0,0,0), 0.1);
    drawLine(arm_control->getBase()->getPos(), arm_control->getElbow()->getPos());
    setColor(255, 255, 255);
    drawSphere(arm_control->getElbow()->getPos(), 0.1);
    drawLine(arm_control->getElbow()->getPos(), arm_control->TARGET_POS);
    setColor(255, 0, 0);
    drawSphere(arm_control->getWrist()->getPos(), 0.1);
    animFlush();
        

}

void arm_control(){
    arm_tests();
}

int python_pipeline(){
    
    FILE* pipe = popen("python3 -u scripts/vision.py 2>/dev/null", "r");
    if (!pipe) {
        std::cerr << "Failed to open pipe to Python process\n";
        return 1;
    }

    char buffer[4096];

    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        arm_control();
        std::string line(buffer);

        if (!line.empty() && line.back() == '\n') {
            line.pop_back();
        }

        if (line.empty()) {
            continue;
        }

        if (line == "NO_HAND") {
            continue;
        }

        std::stringstream ss(line);
        std::string name, x_str, y_str;

        if (!std::getline(ss, name, ',') ||
            !std::getline(ss, x_str, ',') ||
            !std::getline(ss, y_str, ',')) {
            std::cerr << "bad line: " << line << "\n";
            continue;
        }

        if (name != "WRIST") {
            std::cerr << "bad line: " << line << "\n";
            continue;
        }

        try {
            float x = std::stof(x_str);
            float y = std::stof(y_str);

            std::cout << "!VISION_WRIST -> x = " << x
                      << ", y = " << y << "\n";

        } catch (const std::exception& e) {
            std::cerr << "parse error: " << e.what()
                      << " | line: " << line << "\n";
        }
    }

    int status = pclose(pipe);
    std::cout << "Python process exited with status: " << status << "\n";
    return status;
}

int main() {
    std::cout.setf(std::ios::unitbuf);
    python_pipeline();
    //
    
    
    return 0;
}