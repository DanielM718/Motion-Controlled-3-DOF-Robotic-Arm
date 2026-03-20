#ifndef JOINT_H
#define JOINT_H

#include "data_types/vector.h"

class joint {
    public:
        joint(vector pos, float angle, int MIN_ANGLE, int MAX_ANGLE);

        const int MAX_ANGLE;
        const int MIN_ANGLE;

        void setAngle(float angle);
        void setPos(vector pos);

        float getAngle();
        vector getPos();

    private:
        char name[32];
        
        float angle;
        vector pos;
};

#endif