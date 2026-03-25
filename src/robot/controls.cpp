#include "robot/controls.h"


const float control_unit::ARM_LENGTH = 1; // 1 m fake for sim purposes
vector control_unit::TARGET_POS = vector(0.0, 2*ARM_LENGTH, 0.0);

control_unit::control_unit(){
    base = new joint(vector(0.0,0.0,0.0), 90, 0, 360);
    shoulder = new joint(vector(0.0, 0.0, 0.0), 0.0f, 0.0f, 180.0f);
    elbow = new joint(vector(0.0, ARM_LENGTH, 0.0), 0.0f, 0.0f, 360.0f);
}

int control_unit::base_control(){
    // compute angle
    const float x = TARGET_POS.x;
    const float y = TARGET_POS.y;
    float rad = std::atan2(y, x);
    float degree = rad * (180*M_1_PI); // convert to degree
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
    vector half_r = r/2;
    float mag_half_r = mag(half_r);
    float rad = std::acosf(elbow->getPos().z / ARM_LENGTH);
    float degree = rad * (180*M_1_PI) * 2.0f;
    return static_cast<int>(degree);
}

void control_unit::controls(const vector NEW_TARGET_POS){
    TARGET_POS = NEW_TARGET_POS;

    int base_target = base_control();
    int shoulder_target = shoulder_control();
    int elbow_target = elbow_control();
    
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
