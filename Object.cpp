#include "Object.h"
#include "Vector.h"
#include <algorithm>

Intersection Triangle::intersect(const Ray& r) const {
    double alpha, beta, gamma;
    return intersect(r, alpha, beta, gamma);
}

Intersection Triangle::intersect(const Ray& r, double& alpha, double& beta, double& gamma) const {
    Intersection inter(false, Vector(0., 0., 0.), Vector(0., 0., 0.), Vector(0., 0., 0.), 0.); // init intersection
    inter.N = cross(C - A, B - A).normalize();

    double denom = dot(r.u, inter.N);
    if (abs(denom) < 1E-5) return inter;
    inter.t = dot(C - r.C, inter.N) / denom;
    if (inter.t < 0) return inter;

    inter.P = r.C + r.u * inter.t;
    // vérification que le point P est dans le triangle
    Vector u = B - A;
    Vector v = C - A;
    Vector w = inter.P - A;
    double u2 = u.getNorm2();
    double v2 = v.getNorm2();

    double detm = u2 * v2 - sqr(dot(u, v));
    double detb = dot(w, u) * v2 - dot(w, v) * dot(u, v);
    double detg = u2 * dot(w, v) - dot(w, u) * dot(u, v);

    beta = detb / detm;
    gamma = detg / detm;
    alpha = 1 - beta - gamma;

    if (alpha < 0 || beta < 0 || gamma < 0) return inter;
    if (alpha > 1 || beta > 1 || gamma > 1) return inter;

    inter.intersection = true;
    inter.color = albedo;
    return inter;
};


Intersection Sphere::intersect(const Ray& r) const {
    Intersection inter(false, Vector(0., 0., 0.), Vector(0., 0., 0.), Vector(0., 0., 0.), 0.); // init intersection
    // solves a*t² + b*t +c
    double a = 1;
    double b = 2 * dot(r.u, r.C - O);
    double c = (r.C - O).getNorm2() - R * R;

    double delta = b * b - 4 * a * c;
    if (delta < 0) return inter;

    double t1 = (-b - sqrt(delta)) / (2 * a);
    double t2 = (-b + sqrt(delta)) / (2 * a);

    if (t2 < 0) return inter;
    if (t1 > 0) inter.t = t1;
    else inter.t = t2;

    inter.P = r.C + r.u * inter.t;
    inter.N = (inter.P - O).normalize();
    inter.color = albedo;
    inter.intersection = true;
    return inter;
};
