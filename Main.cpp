#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <iostream>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "Vector.h"
#include "Object.h"
#include "Scene.h"
#include "Mesh.h"

int main() {
    // taille de l'image
    int W = 512;
    int H = 512;
    double n_ray = 200;

    // source de la lumière
    Vector L(-10., 20., 40);
    double intensiteL = 1000000000;

    // origine de la camera
    Vector C(0., 0., 55.);

    // initialisation de la scene
    double R = 2;
    // Sphere(position, rayon, couleur(R, G, B), is_speculaire, is_transparent, indice)
    Sphere slum(L, R, Vector(1., 1., 1.));
    //Sphere s1(Vector(-10, 0., -25.), 15, Vector(0.7, 0.3, 0.5), false, true, 1.5); // centre
    Sphere s1bis(Vector(-15., 15., -10), 10, Vector(1, 1, 1), false, true, 1.7); // centre2
    Sphere s2(Vector(0., -1000., 0.), 980, Vector(1, 0, 0)); // sol
    Sphere s3(Vector(0., 1000., 0.), 940, Vector(0.5, 1, 1)); // plafond
    Sphere s4(Vector(0., 0., -1000.), 940, Vector(0, 1, 0)); // fond
    Sphere s5(Vector(1000., 0., 0.), 950, Vector(0, 0, 1)); // droite
    Sphere s6(Vector(-1000., 0., 0.), 950, Vector(0, 0.5, 0.5)); // gauche
    Sphere s7(Vector(0., 0., 1000), 940, Vector(0.8, 0.8, 0.1)); // arrière

    //Triangle tri(Vector(-5, -5, 0), Vector(5, -5, 0), Vector(0, 5, 0), Vector(1, 1, 1));
    
    Mesh girl("Model/Beautiful Girl.obj", 25., Vector(0., -20, 0.), Vector(1., 1., 1.));
    Mesh cube("cube/cube.obj", 0.5, Vector(15., 10, 30), Vector(1., 1., 1.), false, true);

    std::vector<Object*> vec;

    //vec.push_back(&slum);
    //vec.push_back(&s1);
    vec.push_back(&s1bis);
    vec.push_back(&s2);
    vec.push_back(&s3);
    vec.push_back(&s4);
    vec.push_back(&s5);
    vec.push_back(&s6);
    vec.push_back(&s7);

    vec.push_back(&girl);
    vec.push_back(&cube);
    
    Scene s(vec, slum, intensiteL);

    // plan de visualisation
    double focus_distance = 55;
    double fov = 60 * M_PI / 180;
    double d = W / (2 * tan(fov / 2.));

    // construction de l'image
    std::vector<unsigned char> image(W * H * 3, 0);
#pragma omp parallel for
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {

            Vector color(0., 0., 0.);
            for (int k = 0; k < n_ray; k++) {
                // anti aliasing
                double r1 = get_random();
                double r2 = get_random();
                double R = sqrt(-2 * log(r1));
                double dx = R * cos(r2);
                double dy = R * sin(r2);

                // profondeur de champ
                double dx_ouverture = get_random()*2;
                double dy_ouverture = get_random()*2;

                // rayon partant de la camera et allant jusqu'au pixel (i, j)
                Vector u(j - H / 2 + 0.5 + dx, -i + W / 2 + 0.5 + dy, -d);
                u = u.normalize();

                Vector destination = C + u * focus_distance;
                Vector new_origin = C + Vector(dx_ouverture, dy_ouverture, 0);
                Ray ray(new_origin, (destination - new_origin).normalize());

                color += getColor(s, ray, 5) * (1 / n_ray);
            }

            image[(i * W + j) * 3 + 0] = std::min(255., std::max(0., std::pow(color[0], 1 / 2.2)));
            image[(i * W + j) * 3 + 1] = std::min(255., std::max(0., std::pow(color[1], 1 / 2.2)));
            image[(i * W + j) * 3 + 2] = std::min(255., std::max(0., std::pow(color[2], 1 / 2.2)));
        }
    }
    stbi_write_png("image.png", W, H, 3, &image[0], 0);

    return 0;
}