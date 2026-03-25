#ifndef ANIM_H
#define ANIM_H

#include <data_types/vector.h>
#include <stdio.h>

void drawLine(vector pos1, vector pos2);

void drawSphere(vector pos, float radius);

void setColor(int r, int g, int b);

void animFlush();

#endif