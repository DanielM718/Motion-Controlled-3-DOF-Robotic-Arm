#include "robot/controls.h"

const float control_unit::ARM_LENGTH = 0.25; // 0.25 m
vector control_unit::TARGET_POS = vector(0.0, 2*ARM_LENGTH, 0.0);

control_unit::control_unit(){
    base = new joint(vector(0.0,0.0,0.0), 90, 0, 360);
    shoulder = new joint(vector(0.0, 0.0, 0.0), 0.0f, 0.0f, 180.0f);
    elbow = new joint(vector(0.0, ARM_LENGTH, 0.0), 0.0f, 0.0f, 360.0f);
}

void control_unit::controls(const vector NEW_TARGET_POS){
    TARGET_POS = NEW_TARGET_POS;
}

