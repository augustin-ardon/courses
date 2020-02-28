#include "Object.h"
#include "Vector.h"
#include <algorithm>

bool Triangle::intersect(const Ray& r, Vector& N, Vector& P, double& t) const {
    double alpha, beta, gamma;
    return intersect(r, N, P, t, alpha, beta, gamma);
}

bool Triangle::intersect(const Ray& r, Vector& N, Vector& P, double& t, double& alpha, double& beta, double& gamma) const {
    N = cross(C - A, B - A).normalize();

    double denom = dot(r.u, N);
    if (abs(denom) < 1E-5) return false;
    t = dot(C - r.C, N) / denom;
    if (t < 0) return false;

    P = r.C + r.u * t;
    // vérification que le point P est dans le triangle
    Vector u = B - A;
    Vector v = C - A;
    Vector w = P - A;
    double u2 = u.getNorm2();
    double v2 = v.getNorm2();

    double detm = u2 * v2 - sqr(dot(u, v));
    double detb = dot(w, u) * v2 - dot(w, v) * dot(u, v);
    double detg = u2 * dot(w, v) - dot(w, u) * dot(u, v);

    beta = detb / detm;
    gamma = detg / detm;
    alpha = 1 - beta - gamma;

    if (alpha < 0 || beta < 0 || gamma < 0) return false;
    if (alpha > 1 || beta > 1 || gamma > 1) return false;

    return true;
};


bool Sphere::intersect(const Ray& r, Vector& N, Vector& P, double& t) const {
    // solves a*t² + b*t +c
    double a = 1;
    double b = 2 * dot(r.u, r.C - O);
    double c = (r.C - O).getNorm2() - R * R;

    double delta = b * b - 4 * a * c;
    if (delta < 0) return false;

    double t1 = (-b - sqrt(delta)) / (2 * a);
    double t2 = (-b + sqrt(delta)) / (2 * a);

    if (t2 < 0) return false;
    if (t1 > 0) t = t1;
    else t = t2;

    P = r.C + r.u * t;
    N = (P - O).normalize();
    return true;
};
