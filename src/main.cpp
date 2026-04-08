#include <iostream>
#include <sstream>
#include <string>
#include <cstdio>
#include <cstring>
#include <random>
#include <cmath>
#include <chrono>

#include <data_types/vector.h>
#include <robot/controls.h>

control_unit arm_control_unit;

constexpr int GRID_W = 41;
constexpr int GRID_H = 41;

constexpr double FRAME_W = 640.0;
constexpr double FRAME_H = 480.0;

constexpr double ARM_LENGTH = 1.0; // 1 m
constexpr double R = 2.0 * ARM_LENGTH;

int DEBUG_MICRO = 0;
int DEBUG_CONTROL = 0;
int DEBUG_VISION = 0;

int PINCH_JOYSTICK = 0;

vector grid_to_world(int gx, int gy) {
    // normalize grid → [-0.5, 0.5]
    double u = (static_cast<double>(gx) / (GRID_W - 1)) - 0.5;
    double v = (static_cast<double>(gy) / (GRID_H - 1)) - 0.5;

    // world rectangle (aspect-correct)
    double world_h = 2.0 * R;
    double world_w = world_h * (FRAME_W / FRAME_H);

    // map to world coords
    double x = u * world_w;
    double y = -v * world_h; // flip y (image down → world up)

    return vector(x, y, 0);
}

int arm_tests(){

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_angle(0.0, 2*M_PI);
    std::uniform_real_distribution<double> dist_radius(0.0, 1.0);

    double R = 2.0;

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
        arm_control_unit.controls(vector(x, y, 0));
        // debug

    }
    // setColor(0, 255, 0);
    // drawSphere(vector(0,0,0), 0.1);
    // drawLine(arm_control->getBase()->getPos(), arm_control->getElbow()->getPos());
    // setColor(255, 255, 255);
    // drawSphere(arm_control->getElbow()->getPos(), 0.1);
    // drawLine(arm_control->getElbow()->getPos(), arm_control->TARGET_POS);
    // setColor(255, 0, 0);
    // drawSphere(arm_control->getWrist()->getPos(), 0.1);
    // animFlush();
    return 1;

}

void arm_control(float x, float y){
    //arm_tests();
    vector next_target_pos = grid_to_world(x, y);
    arm_control_unit.controls(next_target_pos);
    
    setColor(0, 255, 0);
    drawSphere(vector(0,0,0), 0.1);
    drawLine(arm_control_unit.getBase()->getPos(), arm_control_unit.getElbow()->getPos());
    setColor(255, 255, 255);
    drawSphere(arm_control_unit.getElbow()->getPos(), 0.1);
    drawLine(arm_control_unit.getElbow()->getPos(), arm_control_unit.TARGET_POS);
    setColor(255, 0, 0);
    drawSphere(arm_control_unit.getWrist()->getPos(), 0.1);
    animFlush();

    std::cout.flush();
    fflush(stdout);
}

bool parse_joystick(const std::string& line,
                              int& x,
                              int& y,
                              float& pressed) {
    if (line.rfind("Rotation:", 0) != 0) return false;

    const char* ptr = line.c_str() + 9;
    char* end;

    // Parse X
    x = std::strtol(ptr, &end, 10);
    if (*end != ',') return false;

    // Parse Y
    ptr = end + 1;
    y = std::strtol(ptr, &end, 10);
    if (*end != ',') return false;

    // Parse pressed
    ptr = end + 1;
    pressed = (std::strcmp(ptr, "PRESSED") == 0);

    return true;
}

int python_pipeline(){

    serial_sender* joystick = new serial_sender("/dev/tty.usbmodem2101");

    
    FILE* pipe = popen("python3 -u scripts/vision.py 2>/dev/null", "r");
    if (!pipe) {
        if(DEBUG_VISION){
            std::cerr << "Failed to open pipe to Python process\n";
        }
        return 1;
    }

    char buffer[4096];

    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        std::string joy_response = joystick->read_response();
        int joyx, joyy = 0;
        float pressed;
    
        if(parse_joystick(joy_response, joyx, joyy, pressed)){
            if(PINCH_JOYSTICK){
                arm_control_unit.head_control(joyx, joyy, pressed);
            }
            else{
                arm_control_unit.head_control(joyx, joyy);
            }
        }

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
            if(DEBUG_VISION){
                std::cerr << "!bad line: " << line << "\n";
            }
            continue;
        }

        try {
            if(name == "WRIST"){
                float x = std::stof(x_str);
                float y = std::stof(y_str);
                if(DEBUG_VISION){
                    std::cout << "!VISION_WRIST -> x = " << x
                            << ", y = " << y << "\n";
                }
                arm_control(x, y);
            }
            else{
                float pinch = std::stof(x_str);
                if(DEBUG_VISION){
                    std::cout << "!VISION_PINCH -> pinch = " << pinch << "\n";
                }
                if(!PINCH_JOYSTICK){
                    arm_control_unit.pinch_control(pinch);
                }
            }


        } catch (const std::exception& e) {
            if(DEBUG_VISION){
                std::cerr << "parse error: " << e.what()
                        << " | line: " << line << "\n";
            }
        }
    }

    int status = pclose(pipe);
    std::cout << "Python process exited with status: " << status << "\n";
    return status;
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    int i = 0;
    while(i < argc){
        if(!std::strcmp(argv[i], "-dm")){
            DEBUG_MICRO = 1;
            arm_control_unit.set_debug_response(DEBUG_MICRO);
        }
        else if(!std::strcmp(argv[i], "-dc")){
            DEBUG_CONTROL = 1;
            arm_control_unit.set_debug_control(DEBUG_CONTROL);
        }
        else if(!std::strcmp(argv[i], "-dv")){
            DEBUG_VISION = 1;
        }
        else if(!std::strcmp(argv[i], "-pj")){
            PINCH_JOYSTICK = 1;
        }
        i++;
    }
    std::cout.setf(std::ios::unitbuf);
    python_pipeline();    
    return 0;
}