#include <math.h>

#ifndef VECTOR_H
#define VECTOR_H

struct vector
{
    double x;
    double y;
    double z;
    vector(double x, double y, double z) : x(x), y(y), z(z) {}   // constructor
    vector() {}                                                  // destructor
};

vector operator+(const vector& a, const vector& b);
double operator*(const vector& a, const vector& b);
vector operator^(const vector& a, const vector& b);
vector operator*(const double& a, const vector& b);
vector operator*(const vector& a, const double& b);
vector operator-(const vector& a, const vector& b);
vector operator/(const vector& a, const double& b);
vector operator-(const vector &a);
vector operator+=(vector& a, const vector b);
vector operator-=(vector& a, const vector b);
vector operator*=(vector& a, const double b);
vector operator/=(vector& a, const double b);

double mag (vector v);
double magnitude (vector v);
double norm (vector v);

vector normalize (const vector v);

#endif