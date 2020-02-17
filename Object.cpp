#include "Object.h"
#include "Vector.h"

class Object {
public:
    Object() {};
    virtual bool intersect(const Ray& r, Vector& N, Vector& P, double& t) const = 0;

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

    bool intersect(const Ray& r, Vector& N, Vector& P, double& t) const {
        N = cross(B - A, C - A).normalize();
        t = dot(C - r.C, N) / dot(r.u, N);
        if (t < 0) return false;

        P = r.C + r.u * t;
        // vérification que le point P est dans le triangle
        Vector u = B - A;
        Vector v = C - A;
        Vector w = P - A;

        double detm = u.getNorm2() * v.getNorm2() - sqr(dot(u, v));
        double detb = dot(w, u) * v.getNorm2() - dot(w, v) * dot(u, v);
        double detg = u.getNorm2() * dot(w, v) - dot(w, u) * dot(u, v);

        double beta = detb / detm;
        double gamma = detg / detm;
        double alpha = 1 - beta - gamma;

        if (alpha < 0 || beta < 0 || gamma < 0) return false;
        if (alpha > 1 || beta > 1 || gamma > 1) return false;

        return true;
    };

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

    bool intersect(const Ray& r, Vector& N, Vector& P, double& t) const {
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

    Vector O;
    double R;
};
