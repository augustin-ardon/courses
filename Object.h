#pragma once
#include "Vector.h"

class Object {
public:
    Object() {};
    virtual bool intersect(const Ray& r, Vector& N, Vector& P, double& t, Vector& color) const = 0;

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

    bool intersect(const Ray& r, Vector& N, Vector& P, double& t, Vector& color) const;
    bool intersect(const Ray& r, Vector& N, Vector& P, double& t, Vector& color, double& alpha, double& beta, double& gamma) const;

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

    bool intersect(const Ray& r, Vector& N, Vector& P, double& t, Vector& color) const;

    Vector O;
    double R;
};