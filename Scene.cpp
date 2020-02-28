#include <iostream>
#define _USE_MATH_DEFINES
#include <algorithm> 
#include <math.h> 

#include "Vector.h"
#include "Object.h"
#include "Scene.h"
#include "ompsetup.h"

bool Scene::intersect(const Ray& r, Vector& N, Vector& P, int& obj_id, Vector& color) {
    double t = 1E99;
    bool intersection = false;
    for (int i = 0; i < Objects.size(); i++) {
        Vector localN, localP, localColor;
        double localt;
        bool local_intersection = Objects[i]->intersect(r, localN, localP, localt, localColor);
        if (local_intersection) {
            intersection = true;
            if (localt < t) {
                t = localt;
                P = localP;
                N = localN;
                obj_id = i;
                color = localColor;
            }
        }
    }
    return intersection;
};

Ray reflect(Ray r, Vector normal, Vector intersect) {
    return Ray(intersect, r.u - normal * 2 * dot(r.u, normal));
};

Ray refract(Ray r, Vector normal, Vector intersect, double n1, double n2, bool& is_refracted) {
    double prod = dot(r.u, normal);
    double n = n1 / n2;
    Vector new_norm(normal);
    if (prod > 0) {
        n = 1 / n;
        new_norm = normal * -1;
    }
    double radical = 1 - sqr(n) * (1 - sqr(dot(new_norm, r.u)));
    if (radical > 0) {
        // le rayon est refracté
        is_refracted = true;
        Vector epsilon = new_norm * 1E-10;

        Vector new_ray_dir = (r.u - new_norm * dot(r.u, new_norm)) * n - new_norm * sqrt(radical);
        return Ray(intersect - epsilon, new_ray_dir);
    }
    // le rayon est refléchi
    is_refracted = false;
    return reflect(r, normal, intersect);
};

Vector random_cos(const Vector& n) {
    Vector t1;
    if (n[0] <= n[1] && n[0] <= n[2]) {
        t1 = Vector(0, -n[2], n[1]);
    }
    else {
        if (n[1] <= n[0] && n[1] <= n[2]) {
            t1 = Vector(-n[2], 0, n[0]);
        }
        else {
            t1 = Vector(-n[1], n[0], 0);
        }
    }
    t1 = t1.normalize();
    Vector t2 = cross(n, t1);

    double r1 = get_random();
    double r2 = get_random();

    Vector random_u_loc(cos(2 * M_PI * r1) * sqrt(1 - r2), sin(2 * M_PI * r1) * sqrt(1 - r2), sqrt(r2));

    return n * random_u_loc[2] + t1 * random_u_loc[0] + t2 * random_u_loc[1];
};

Vector getColor(Scene scene, Ray ray, int numero_rebond) {
    if (numero_rebond == 0) return Vector(0, 0, 0);

    Vector color(0., 0., 0.); // noir
    Vector L = scene.lumiere->O;
    //double intensiteL = scene.intensite_lumiere;

    int obj_id; // sphere la plus proche intersectée par le rayon
    Vector n, p, albedo; // point d'intersection et normale à la sphère en ce point
    if (scene.intersect(ray, n, p, obj_id, albedo)) {
        // partie réflechie
        if (scene.Objects[obj_id]->is_speculaire) {
            Vector epsilon = n * 1E-10;

            Ray reflected_ray = reflect(ray, n, p + epsilon);
            return getColor(scene, reflected_ray, numero_rebond - 1);
        }
        // partie transparente
        else if (scene.Objects[obj_id]->is_transparent) {
            double n_air = 1;
            bool is_refracted;

            Ray refracted_ray = refract(ray, n, p, n_air, scene.Objects[obj_id]->n, is_refracted);
            return getColor(scene, refracted_ray, numero_rebond - 1);
        }

        else {
            //// éclairage direct
            Vector epsilon = n * 1E-10;

            Vector l = p - L;  // direction allant du centre de la source lumineuse vers le point P
            l = l.normalize();
            Vector randSdir = random_cos(l);
            Vector xi = L + randSdir * scene.lumiere->R; // point sur la sphere lumineuse
            Vector wi = xi - p;
            double d2 = wi.getNorm2();
            wi = wi.normalize();
            double costheta = std::max(0., dot(n, wi));
            double costhetaprime = std::max(0., dot(randSdir, wi * (-1)));
            double costhetaseconde = std::max(0., dot(randSdir, l));

            Ray new_ray(p + epsilon, wi); // nouveau rayon issu du point P allant vers la source de lumière
            Vector new_n, new_p, new_color;
            int new_id;

            if (!(scene.intersect(new_ray, new_n, new_p, new_id, new_color)) or (d2 * 0.99 < (new_p - p).getNorm2())) {
                //std::cout << albedo[0] << ";" << albedo[1] << ";" << albedo[2] << std::endl;
                color = albedo * (1 / (4 * M_PI * d2 * costhetaseconde)) * costheta * costhetaprime * scene.intensite_lumiere;
            }

            // éclairage indirect
            Vector random_u = random_cos(n);
            Ray random_ray(p + epsilon, random_u);

            color += getColor(scene, random_ray, numero_rebond - 1) * albedo;
        }
    }
    return color;
};

double get_random() {
    return uniform(engine[omp_get_thread_num()]);
}
