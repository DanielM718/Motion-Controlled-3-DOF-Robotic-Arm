#include <anim/anim.h>

void drawLine(vector pos1, vector pos2){
    printf("l3 %f %f %f %f %f %f\n", pos1.x, pos1.y, pos1.z 
                                   , pos2.x, pos2.y, pos2.z);
}

void drawSphere(vector pos, float radius){
    printf("c3 %f %f %f %f\n", pos.x, pos.y, pos.z, radius);
}

void setColor(int r, int g, int b){
    printf("C %d %d %d\n", r, g, b);
}

void animFlush(){
    printf("F\n");
}
