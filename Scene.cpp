#include <iostream>
#define _USE_MATH_DEFINES
#include <algorithm> 
#include <math.h> 

#include "Vector.h"
#include "Object.h"
#include "Scene.h"
#include "ompsetup.h"

Intersection Scene::intersect(const Ray& r) {
    Intersection inter(false, Vector(0., 0., 0.), Vector(0., 0., 0.), Vector(0., 0., 0.), 1E99);
    
    for (int i = 0; i < Objects.size(); i++) {
        Intersection local_intersection = Objects[i]->intersect(r);
        if (local_intersection.intersection) {
            inter.intersection = true;
            if (local_intersection.t < inter.t) {
                inter.t = local_intersection.t;
                inter.P = local_intersection.P;
                inter.N = local_intersection.N;
                inter.obj_id = i;
                inter.color = local_intersection.color;
            }
        }
    }
    return inter;
};

Ray reflect(Ray r, Vector normal, Vector intersect) {
    return Ray(intersect, r.u - normal * 2 * dot(r.u, normal));
};

Ray refract(Ray r, Vector normal, Vector intersect, double n1, double n2) {
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
        Vector epsilon = new_norm * 1E-10;

        Vector new_ray_dir = (r.u - new_norm * dot(r.u, new_norm)) * n - new_norm * sqrt(radical);
        return Ray(intersect - epsilon, new_ray_dir);
    }
    // le rayon est refléchi
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

Vector random_phong(const Vector& n, double phong) {
    double r1 = get_random();
    double r2 = get_random();
    double fac = sqrt(1 - std::pow(r2, 2. / (phong + 1)));
    Vector random_u_loc(cos(2 * M_PI * r1) * fac, sin(2 * M_PI * r1), std::pow(r2, 1. / (phong + 1)));

    Vector random(get_random() - 0.5, get_random() - 0.5, get_random() - 0.5);
    Vector t1 = cross(n, random).normalize();
    Vector t2 = cross(t1, n);

    return n * random_u_loc[2] + t1 * random_u_loc[0] + t2 * random_u_loc[1];
}

double phong_brdf(const Vector& wi, const Vector& wo, const Vector& N, double phong) {
    Vector reflechi = reflect_vect(wo, N);
    double lobe = std::pow(dot(reflechi, wi), phong) * (phong + 2) / (2. * M_PI);
    return lobe;
}

Vector getColor(Scene scene, Ray ray, int numero_rebond) {
    if (numero_rebond == 0) return Vector(0, 0, 0);

    Vector color(0., 0., 0.); // noir
    Vector L = scene.lumiere->O;
    //double intensiteL = scene.intensite_lumiere;

    Intersection scene_intersect = scene.intersect(ray);
    if (scene_intersect.intersection) {
        // partie réflechie
        if (scene.Objects[scene_intersect.obj_id]->is_speculaire) {
            Vector epsilon = scene_intersect.N * 1E-10;

            Ray reflected_ray = reflect(ray, scene_intersect.N, scene_intersect.P + epsilon);
            return getColor(scene, reflected_ray, numero_rebond - 1) * scene.Objects[scene_intersect.obj_id]->albedo;
        }
        // partie transparente
        else if (scene.Objects[scene_intersect.obj_id]->is_transparent) {
            double n_air = 1;

            Ray refracted_ray = refract(ray, scene_intersect.N, scene_intersect.P, n_air, scene.Objects[scene_intersect.obj_id]->n);
            return getColor(scene, refracted_ray, numero_rebond - 1) * scene.Objects[scene_intersect.obj_id]->albedo;
        }

        else {
            //// éclairage direct
            Vector epsilon = scene_intersect.N * 1E-10;

            Vector l = (scene_intersect.P - L).normalize();  // direction allant du centre de la source lumineuse vers le point 
            Vector randSdir = random_cos(l);
            Vector xi = L + randSdir * scene.lumiere->R; // point sur la sphere lumineuse
            Vector wi = xi - scene_intersect.P;
            double d2 = wi.getNorm2();
            wi = wi.normalize();
            double costheta = std::max(0., dot(scene_intersect.N, wi));
            double costhetaprime = std::max(0., dot(randSdir, wi * (-1)));
            double costhetaseconde = std::max(0., dot(randSdir, l));

            Ray new_ray(scene_intersect.P + epsilon, wi); // nouveau rayon issu du point P allant vers la source de lumière

            Intersection ombre_intersect = scene.intersect(new_ray);
            if (!ombre_intersect.intersection or d2 * 0.99 < (ombre_intersect.P - scene_intersect.P).getNorm2()) {
                color = scene_intersect.color * (1. / (4 * M_PI * d2 * costhetaseconde)) * costheta * costhetaprime * scene.intensite_lumiere;
            }

            // éclairage indirect
            Vector random_u = random_cos(scene_intersect.N);
            
            Ray random_ray(scene_intersect.P + epsilon, random_u);
            color += getColor(scene, random_ray, numero_rebond - 1) * scene_intersect.color;
            
        }
    }
    return color;
};

Vector reflect_vect(Vector inc, Vector norm) {
    Vector result = inc - 2 * dot(inc, norm);
    return result;
}

double get_random() {
    return uniform(engine[omp_get_thread_num()]);
}
