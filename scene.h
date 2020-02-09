#include <iostream>
#define _USE_MATH_DEFINES
#include <algorithm> 
#include <math.h> 
#include <random>
#include <omp.h>

std::default_random_engine engine[8];
std::uniform_real_distribution<double> uniform(0,1);

class Vector {
public:
    double coords[3];
    Vector(double x = 0, double y = 0, double z = 0) {
        coords[0] = x;
        coords[1] = y;
        coords[2] = z;
    };

    double operator[] (int i) const {
        return coords[i];
    };

    double& operator[] (int i) {
        return coords[i];
    };

    Vector operator- (Vector a) const {
        return Vector(coords[0] - a[0],
            coords[1] - a[1],
            coords[2] - a[2]);
    };

    Vector operator+ (Vector a) const {
        return Vector(coords[0] + a[0],
            coords[1] + a[1],
            coords[2] + a[2]);
    };

    Vector operator* (double a) const {
        return Vector(coords[0] * a,
            coords[1] * a,
            coords[2] * a);
    }

    Vector operator* (Vector a) const {
        return Vector(coords[0] * a[0],
            coords[1] * a[1],
            coords[2] * a[2]);
    }

    Vector operator+= (Vector a) {
        coords[0] += a[0];
        coords[1] += a[1];
        coords[2] += a[2];
        return *this;
    }

    double getNorm2() const {
        double result = 0;
        for (int i = 0; i < 3; i++) result += coords[i] * coords[i];
        return result;
    };

    Vector normalize() const {
        double norm = sqrt(getNorm2());
        return Vector(coords[0] / norm,
            coords[1] / norm,
            coords[2] / norm);
    };
};

double dot(const Vector A, const Vector B) {
    return A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
}

Vector cross(const Vector A, const Vector B) {
    return Vector(A[1] * B[2] - A[2] * B[1], A[2] * B[0] - A[0] * B[2], A[0] * B[1] - A[1] * B[0]);
}

class Ray {
public:
    Ray(const Vector& paramC, Vector paramu) : C(paramC), u(paramu) {};
    Vector C, u;
};

class Sphere {
public:
    Sphere(const Vector& paramO, double paramR, const Vector& couleur, bool speculaire, bool is_transparent, double indice = 1) : O(paramO), R(paramR), albedo(couleur) , is_speculaire(speculaire), is_transparent(is_transparent), n(indice) {};
    Vector O;
    double R;
    Vector albedo;
    bool is_speculaire;
    bool is_transparent;
    double n;

    bool intersect(const Ray& r, Vector& N, Vector& P, double& t) {
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
    }
};

class Scene {
public:
    Scene(std::vector<Sphere> paramSpheres, Sphere paramLumiere, double paramIntensite_lumiere) : Spheres(paramSpheres), lumiere(&paramLumiere), intensite_lumiere(paramIntensite_lumiere){};
    std::vector<Sphere> Spheres;
    Sphere *lumiere;
    double intensite_lumiere;

    bool intersect(const Ray& r, Vector& N, Vector& P, int& sphere_id) {
        double t = 1E99;
        bool intersection = false;
        for (int i = 0; i < Spheres.size(); ++i) {
            Vector localN, localP;
            double localt;
            bool local_intersection = Spheres[i].intersect(r, localN, localP, localt);
            if (local_intersection) {
                intersection = true;
                if (localt < t) {
                    t = localt;
                    P = localP;
                    N = localN;
                    sphere_id = i;
                }
            }
        }
        return intersection;
    }

    Sphere operator[] (int i) const {
        return Spheres[i];
    }
};

double sqr(double a) {
    double result = a * a;
    return result;
}

Ray reflect(Ray r, Vector normal, Vector intersect) {
    return Ray(intersect, r.u - normal * 2 * dot(r.u, normal));
}

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
}

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

    double r1 = uniform(engine[omp_get_thread_num()]);
    double r2 = uniform(engine[omp_get_thread_num()]);

    Vector random_u_loc(cos(2 * M_PI * r1) * sqrt(1 - r2), sin(2 * M_PI * r1) * sqrt(1 - r2), sqrt(r2));

    return n * random_u_loc[2] + t1 * random_u_loc[0] + t2 * random_u_loc[1];
}

Vector getColor(Scene scene, Ray ray, int numero_rebond) {
    if (numero_rebond == 0) return Vector(0, 0, 0);

    Vector color(0., 0., 0.); // noir
    Vector L = scene.lumiere->O;
    //double intensiteL = scene.intensite_lumiere;

    int sphere_id; // sphere la plus proche intersectée par le rayon
    Vector n, p; // point d'intersection et normale à la sphère en ce point
    if (scene.intersect(ray, n, p, sphere_id)) {
        // partie réflechie
        if (scene[sphere_id].is_speculaire) {
            Vector epsilon = n * 1E-10;

            Ray reflected_ray = reflect(ray, n, p + epsilon);
            return getColor(scene, reflected_ray, numero_rebond - 1);
        } 
        // partie transparente
        else if (scene[sphere_id].is_transparent) {
            double n_air = 1;
            bool is_refracted;

            Ray refracted_ray = refract(ray, n, p, n_air, scene[sphere_id].n, is_refracted);
            return getColor(scene, refracted_ray, numero_rebond - 1);
        }
        
        else {
            //// éclairage direct
            //Vector l = L - p; // chemin inverse de la lumière allant du point d'intersection vers la source de lumière

            Vector epsilon = n * 1E-10;
            //Ray new_ray(p + epsilon, l.normalize()); // nouveau rayon issu du point P allant vers la source de lumière
            //Vector new_n, new_p;
            //int new_id;

            //if (!(scene.intersect(new_ray, new_n, new_p, new_id)) or (l.getNorm2() < (new_p - p).getNorm2())) {
            //    double intensite_norm = dot(l.normalize(), n) * intensiteL / l.getNorm2();
            //    color = scene.Spheres[sphere_id].albedo * (intensite_norm / M_PI);
            //}  
            
            //color += scene.Spheres[sphere_id].albedo * scene.Spheres[sphere_id].I;

            Vector l = p - L;  // direction allant du centre de la source lumineuse vers le point P
            l = l.normalize();
            Vector randSdir = random_cos(l);
            Vector xi = L + randSdir * scene.Spheres[0].R; // point sur la sphere lumineuse
            Vector wi = xi - p;
            double d2 = wi.getNorm2();
            wi = wi.normalize();
            double costheta = std::max(0., dot(n, wi));
            double costhetaprime = std::max(0., dot(randSdir, wi*(-1)));
            double costhetaseconde = std::max(0., dot(randSdir, l));

            Ray new_ray(p + epsilon, wi); // nouveau rayon issu du point P allant vers la source de lumière
            Vector new_n, new_p;
            int new_id;

            if (!(scene.intersect(new_ray, new_n, new_p, new_id)) or (d2 * 0.99 < (new_p - p).getNorm2())) {
                color = scene[sphere_id].albedo * (1 / (4 * M_PI * d2 * costhetaseconde)) * costheta * costhetaprime * scene.intensite_lumiere;
            }

            // éclairage indirect
            Vector random_u = random_cos(n);
            Ray random_ray(p + epsilon, random_u);

            color += getColor(scene, random_ray, numero_rebond - 1) * scene.Spheres[sphere_id].albedo;
        }
    }
    return color;
}

