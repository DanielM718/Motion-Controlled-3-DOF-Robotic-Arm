#include "robot/joint.h"

joint::joint(vector pos, float angle, int MIN_ANGLE, int MAX_ANGLE): 
    pos(pos),
    angle(angle),
    MIN_ANGLE(MIN_ANGLE),
    MAX_ANGLE(MAX_ANGLE)
    {}

void joint::setAngle(float angle){
    if(angle < MIN_ANGLE) angle = MIN_ANGLE;
    if(angle > MAX_ANGLE) angle = MAX_ANGLE;
    this->angle = angle;
}

void joint::setPos(vector pos){
    this->pos = pos;
}

float joint::getAngle(){
    return angle;
}
vector joint::getPos(){
    return pos;
}
