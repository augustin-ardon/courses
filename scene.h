#include "Vector.h"
#include "Object.h"


class Scene {
public:
    Scene(std::vector<Object*> paramObjects, Sphere& paramLumiere, double paramIntensite_lumiere) : Objects(paramObjects), lumiere(&paramLumiere), intensite_lumiere(paramIntensite_lumiere) {};
    std::vector<Object*> Objects;
    Sphere* lumiere;
    double intensite_lumiere;

    bool intersect(const Ray& r, Vector& N, Vector& P, int& sphere_id) {};
};

