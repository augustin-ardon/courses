#pragma once
#include "Vector.h"

class Intersection {
public:
    Intersection(bool inter, Vector paramN, Vector paramP, Vector col, double paramt, int paramid=0) : intersection(inter), N(paramN), P(paramP), color(col), t(paramt), obj_id(paramid) {};

    bool intersection;
    Vector N, P, color;
    double t;
    int obj_id;
};

class Object {
public:
    Object() {};
    virtual Intersection intersect(const Ray& r) const = 0;

    Vector albedo;
    bool is_speculaire;
    bool is_transparent;
    double n;
};

class Triangle : public Object {
public:
    Triangle(const Vector A, const Vector B, const Vector C, const Vector& couleur, bool speculaire = false, bool transparent = false, double indice = 1) : A(A), B(B), C(C) {
        albedo = couleur;
        is_speculaire = speculaire;
        is_transparent = transparent;
        n = indice;
    };

    Intersection intersect(const Ray& r) const;
    Intersection intersect(const Ray& r, double& alpha, double& beta, double& gamma) const;

    Vector A, B, C;
};

class Sphere : public Object {
public:
    Sphere(const Vector& paramO, double paramR, const Vector& couleur, bool speculaire = false, bool transparent = false, double indice = 1) : O(paramO), R(paramR) {
        albedo = couleur;
        is_speculaire = speculaire;
        is_transparent = transparent;
        n = indice;
    };

    Intersection intersect(const Ray& r) const;

    Vector O;
    double R;
};