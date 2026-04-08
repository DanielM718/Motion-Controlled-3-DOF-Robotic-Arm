#ifndef CONTROLS_H
#define CONTROLS_H

#include "data_types/vector.h"
#include "robot/joint.h"
#include "robot/serial_sender.h"

#include "anim/anim.h"
#include <cmath>
#include <iostream>

class control_unit {
    public:
        static const float ARM_LENGTH;
        static vector TARGET_POS;

        static int DEBUG_RESPONSE;
        static int DEBUG_CONTROL;

        control_unit();

        void controls(const vector NEW_TARGET_POS);
        void controls(const vector NEW_TARGET_POS, float pinch);

        int base_control();
        int shoulder_control();
        int elbow_control();

        void set_debug_response(int DEBUG_RESPONSE);
        void set_debug_control(int DEBUG_CONTROL);

        void head_control(int x, int y, float pressed);
        void head_control(int x, int y);

        void pinch_control(float pressed);

        joint* getBase();
        joint* getShoulder();
        joint* getElbow();
        joint* getWrist();

    private:
        joint* base;
        joint* shoulder;
        joint* elbow;
        joint* wrist;

        serial_sender* robot;
};

#endif