#pragma once
#include "Object.h"
#include <vector>


class Scene {
public:
    Scene(std::vector<Object*> paramObjects, Sphere& paramLumiere, double paramIntensite_lumiere) : Objects(paramObjects), lumiere(&paramLumiere), intensite_lumiere(paramIntensite_lumiere) {};
    std::vector<Object*> Objects;
    Sphere* lumiere;
    double intensite_lumiere;

    Intersection intersect(const Ray& r);
};

Ray reflect(Ray r, Vector normal, Vector intersect);

Ray refract(Ray r, Vector normal, Vector intersect, double n1, double n2);

Vector random_cos(const Vector& n);

Vector getColor(Scene scene, Ray ray, int numero_rebond);

Vector reflect_vect(Vector inc, Vector norm);

double get_random();
