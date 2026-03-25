#ifndef CONTROLS_H
#define CONTROLS_H

#include "data_types/vector.h"
#include "robot/joint.h"

#include "anim/anim.h"
#include <cmath>

class control_unit {
    public:
        static const float ARM_LENGTH;
        static vector TARGET_POS;
        control_unit();
        void controls(const vector NEW_TARGET_POS);
        int base_control();
        int shoulder_control();
        int elbow_control();

        joint* getBase();
        joint* getShoulder();
        joint* getElbow();

    private:
        joint* base;
        joint* shoulder;
        joint* elbow;
};

#endif