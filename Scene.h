#pragma once
#include "Object.h"
#include <vector>


class Scene {
public:
    Scene(std::vector<Object*> paramObjects, Sphere& paramLumiere, double paramIntensite_lumiere) : Objects(paramObjects), lumiere(&paramLumiere), intensite_lumiere(paramIntensite_lumiere) {};
    std::vector<Object*> Objects;
    Sphere* lumiere;
    double intensite_lumiere;

    bool intersect(const Ray& r, Vector& N, Vector& P, int& sphere_id, Vector& color);
};

Ray reflect(Ray r, Vector normal, Vector intersect);

Ray refract(Ray r, Vector normal, Vector intersect, double n1, double n2, bool& is_refracted);

Vector random_cos(const Vector& n);

Vector getColor(Scene scene, Ray ray, int numero_rebond);

double get_random();
