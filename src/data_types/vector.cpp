#include "data_types/vector.h"

vector operator+(const vector& a, const vector& b)           // vector plus vector
{
    return vector(a.x+b.x, a.y+b.y, a.z+b.z);         
}

double operator*(const vector& a, const vector& b)           // dot product 
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

vector operator^(const vector& a, const vector& b)           // what's this do?
{
    vector c;
    c.x=a.y*b.z - a.z*b.y;
    c.y=a.z*b.x - a.x*b.z;
    c.z=a.x*b.y - a.y*b.x;
    return c;
}

vector operator*(const double& a, const vector& b)           // double times vector
{
    vector c;
    c.x = a*b.x;
    c.y = a*b.y;
    c.z = a*b.z;
    return c;
}

vector operator*(const vector& a, const double& b)          // vector times double
{
    vector c;
    c.x=a.x*b;
    c.y=a.y*b;
    c.z=a.z*b;
    return c;
}

vector operator-(const vector& a, const vector& b)         // vector minus vector
{
    vector c;
    c.x = a.x-b.x;
    c.y = a.y-b.y;
    c.z = a.z-b.z;
    return c;
}

vector operator/(const vector& a, const double& b)         // vector divided by double
{
    vector c;
    c.x = a.x/b;
    c.y = a.y/b;
    c.z = a.z/b;
    return c;
}

vector operator-(const vector &a)                              // unary minus
{
    return a * -1;
}

vector operator+=(vector& a, const vector b)              // add-in-place to vector
{
    a=a+b;
    return a;
}

vector operator-=(vector& a, const vector b)             // subtract-in-place to vector
{
    a=a-b;
    return a;
}

vector operator*=(vector& a, const double b)             // multiply-in-place to vector
{
    a=a*b;
    return a;
}

vector operator/=(vector& a, const double b)             // divide-in-place to vector
{
    a=a/b;
    return a;
}


double mag (vector v)                                   // magnitude of vector
{
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
} 
double magnitude (vector v)                             // with different names
{
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
} 
double norm (vector v)                                  // since I can never remember what I call it
{
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}
vector normalize (const vector v)
{
    double m = mag(v);
    if(m == 0) return vector(0,0,0);
    return vector(v.x / m, v.y / m, v.z / m);
    //return v/mag(v);
}