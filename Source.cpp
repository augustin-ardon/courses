#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

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

double dot(const Vector& A, const Vector& B) {
    return A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
}

class Ray {
public:
    Ray(const Vector& paramC, Vector paramu) : C(paramC), u(paramu) {};
    Vector C, u;
};

class Sphere {
public:
    Sphere(const Vector& paramO, double paramR) : O(paramO), R(paramR) {};
    Vector O;
    double R;

    bool intersect(const Ray& r) {
        // solves a*t² + b*t +c
        double a = 1;
        double b = 2 * dot(r.u, r.C - O);
        double c = (r.C - O).getNorm2() - R*R;

        double delta = b * b - 4 * a * c;
        if (delta < 0) return false;

        double t1 = (-b + sqrt(delta)) / (2 * a);
        if (t1 < 0) return false;

        // double t0 = (-b - sqrt(delta)) / (2 * a);
        return true;
    }
};

int main() {
    int W = 512;
    int H = 512;

    Sphere s(Vector(0., 0., 0.), 10);
    Vector C(0., 0., 55.);

    Vector L(-10, 20, 40);
    double intensiteL = 1000000;

    double fov = 60 * M_PI / 180;
    double d = W / (2 * tan(fov / 2.));

    std::vector<unsigned char> image(W * H * 3, 0);
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {

            Vector u(j - W / 2, -i + H / 2, -d);
            Ray ray(C, u.normalize());

            if (s.intersect(ray)) {
                
                image[(i * W + j) * 3 + 0] = 255;
                image[(i * W + j) * 3 + 1] = 255;
                image[(i * W + j) * 3 + 2] = 255;
            }
            else {
                image[(i * W + j) * 3 + 0] = 0;
                image[(i * W + j) * 3 + 1] = 0;
                image[(i * W + j) * 3 + 2] = 0;
            }
        }
    }
    stbi_write_png("image.png", W, H, 3, &image[0], 0);

    return 0;
}