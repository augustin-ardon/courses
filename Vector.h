#pragma once
#include <math.h> 

class Vector {
public:
    double coords[3];
    Vector(double x = 0, double y = 0, double z = 0) {
        coords[0] = x;
        coords[1] = y;
        coords[2] = z;
    };

    double operator[] (int i) const {
        return coords[i];
    };

    double& operator[] (int i) {
        return coords[i];
    };

    Vector operator- (Vector a) const {
        return Vector(coords[0] - a[0],
            coords[1] - a[1],
            coords[2] - a[2]);
    };

    Vector operator+ (Vector a) const {
        return Vector(coords[0] + a[0],
            coords[1] + a[1],
            coords[2] + a[2]);
    };

    Vector operator* (double a) const {
        return Vector(coords[0] * a,
            coords[1] * a,
            coords[2] * a);
    }

    Vector operator* (Vector a) const {
        return Vector(coords[0] * a[0],
            coords[1] * a[1],
            coords[2] * a[2]);
    }

    Vector operator+= (Vector a) {
        coords[0] += a[0];
        coords[1] += a[1];
        coords[2] += a[2];
        return *this;
    }

    double getNorm2() const {
        double result = 0;
        for (int i = 0; i < 3; i++) result += coords[i] * coords[i];
        return result;
    };

    Vector normalize() const {
        double norm = sqrt(getNorm2());
        return Vector(coords[0] / norm,
            coords[1] / norm,
            coords[2] / norm);
    };
};

double dot(const Vector A, const Vector B) {
    return A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
};

Vector cross(const Vector A, const Vector B) {
    return Vector(A[1] * B[2] - A[2] * B[1], A[2] * B[0] - A[0] * B[2], A[0] * B[1] - A[1] * B[0]);
};

double sqr(double a) {
    double result = a * a;
    return result;
};

class Ray {
public:
    Ray(const Vector& paramC, Vector paramu) : C(paramC), u(paramu) {};
    Vector C, u;
};

