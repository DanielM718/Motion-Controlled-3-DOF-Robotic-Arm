#include "robot/controls.h"


const float control_unit::ARM_LENGTH = 1; // 1 m fake for sim purposes
vector control_unit::TARGET_POS = vector(0.0, 2*ARM_LENGTH, 0.0);

int control_unit::DEBUG_RESPONSE = 0;
int control_unit::DEBUG_CONTROL = 0;

control_unit::control_unit(){
    base = new joint(vector(0.0,0.0,0.0), 90, 0, 360);
    shoulder = new joint(vector(0.0, 0.0, 0.0), 0.0f, 0.0f, 180.0f);
    elbow = new joint(vector(0.0, ARM_LENGTH, 0.0), 0.0f, 0.0f, 180.0f);
    wrist = new joint(vector(0.0, ARM_LENGTH*2, 0.0), 0.0f, 0.0f, 180.0f);

    robot = new serial_sender("/dev/tty.usbmodem4");
}

void control_unit::set_debug_response(int DEBUG_RESPONSE){
    this->DEBUG_RESPONSE = DEBUG_RESPONSE;
}

void control_unit::set_debug_control(int DEBUG_CONTROL){
    this->DEBUG_CONTROL = DEBUG_CONTROL;
}

int control_unit::base_control(){
    // compute angle
    const float x = TARGET_POS.x;
    const float y = TARGET_POS.y;
    float rad = std::atan2(y, x);
    float degree = rad * (180*M_1_PI); // convert to degree
    degree += 180;
    return static_cast<int>(degree);
}

int control_unit::shoulder_control(){
    // compute angle
    vector r = TARGET_POS;
    r.z = 0.0f;
    vector half_r = r/2;
    float mag_half_r = mag(half_r);
    float rad = std::acosf(mag_half_r / ARM_LENGTH);
    float degree = rad * (180*M_1_PI);

    // update the pos of the elbow joint
    float height = ARM_LENGTH*sinf(rad); // need the z component
    elbow->setPos(vector(half_r.x, half_r.y, height));
    return static_cast<int>(degree);
}

int control_unit::elbow_control(){
    vector r = TARGET_POS;
    r.z = 0.0f;
    float rad = std::acosf(elbow->getPos().z / ARM_LENGTH);
    float degree = rad * (180*M_1_PI) * 2.0f;

    // update the pos of the wrist
    vector e = elbow->getPos();
    float heading = std::atan2f(TARGET_POS.y, TARGET_POS.x);
    float planar_len = std::sqrtf(ARM_LENGTH * ARM_LENGTH - e.z * e.z);

    wrist->setPos(vector(
        e.x + planar_len * std::cosf(heading),
        e.y + planar_len * std::sinf(heading),
        0.0f
    ));
    return static_cast<int>(degree);
}

void control_unit::controls(const vector NEW_TARGET_POS){
    TARGET_POS = NEW_TARGET_POS;

    int base_target = base_control();
    int shoulder_target = shoulder_control();
    int elbow_target = elbow_control();

    vector wrist_pos = wrist->getPos();
    //printf("!wrist x: %f y: %f, z: %f\n", wrist_pos.x, wrist_pos.y, wrist_pos.z);
    if(DEBUG_RESPONSE){
        std::string robot_response = robot->read_response();
        std::cout << "!" << robot_response << "\n";
    }
    if(DEBUG_CONTROL){
        printf("!angles base: %d shoulder: %d elbow: %d\n", base_target, 180 - shoulder_target, 100 + elbow_target);
    }
    //robot->send_angles(base_target, shoulder_target, elbow_target, 180, 180);
    //robot->send_angles_arm(base_target, 180 - shoulder_target, 120 + elbow_target);
    robot->send_angles_arm(base_target, 170 - shoulder_target, 120 + elbow_target);
}   

void control_unit::head_control(int x, int y, float pressed){
    robot->send_angles_wrist(x, y);
    robot->send_angles_grip(pressed);
}

void control_unit::head_control(int x, int y){
    robot->send_angles_wrist(x, y);
}

void control_unit::pinch_control(float pressed){
    robot->send_angles_grip(pressed);
}

joint* control_unit::getBase(){
    return base;
}

joint* control_unit::getShoulder(){
    return shoulder;
}

joint* control_unit::getElbow(){
    return elbow;
}

joint* control_unit::getWrist(){
    return wrist;
}