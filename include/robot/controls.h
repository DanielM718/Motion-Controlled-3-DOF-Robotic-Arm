#ifndef CONTROLS_H
#define CONTROLS_H

#include "data_types/vector.h"
#include "robot/joint.h"

class control_unit {
    static const float ARM_LENGTH;
    static vector TARGET_POS;
    public:
        control_unit();
        void controls(const vector NEW_TARGET_POS);
    private:
        joint* base;
        joint* shoulder;
        joint* elbow;
};

#endif