#pragma once
#include <vector>
#include <string>
#include <map>
#include "scene.h"
#include <iostream>

class BBox {
public:
    BBox() {};

    bool intersect(const Ray &r) const;

    Vector bmin, bmax;
};

class BVH {
public:
    int i0, i1;

    BBox bbox;
    BVH* node_l, * node_r;
};

class TriangleIndices {
public:
    TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk) {
    };
    int vtxi, vtxj, vtxk;
    int uvi, uvj, uvk;
    int ni, nj, nk;
    int faceGroup;
};

class Mesh : public Object {
public:
    Mesh() {};
    Mesh(const char* obj, double scaling, const Vector& offset, const Vector& couleur, bool textures=true, bool speculaire = false, bool transparent = false, double indice = 1) : has_textures(textures) {
        albedo = couleur;
        is_speculaire = speculaire;
        is_transparent = transparent;
        n = indice;

        readOBJ(obj);
        for (int i = 0; i < vertices.size(); i++) {
            vertices[i] = vertices[i] * scaling + offset;
        }

        build_bvh(&bvh, 0, indices.size());
    };
    bool has_textures;

    Intersection intersect(const Ray& r) const;
    BBox build_bbox(int i0, int i1);
    void build_bvh(BVH* bvh, int i0, int i1);

    void readOBJ(const char* obj);
    void add_texture(const char* filename);

    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs; // Vector en 3D mais on n'utilise que 2 composantes
    std::vector<Vector> vertexcolors;

    std::vector<std::vector<unsigned char> > textures;
    std::vector<int> w, h;

private:
    BVH bvh;
};
