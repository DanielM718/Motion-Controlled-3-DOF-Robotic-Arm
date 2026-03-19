#ifndef JOINT_H
#define JOINT_H

class joint {
    public:
        int MAX_ANGLE;
        char name[32];
        
        float angle;

        void setAngle(float angle);


};

#endif