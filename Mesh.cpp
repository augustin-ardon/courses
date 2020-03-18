#include "Mesh.h"
#include <algorithm>
#include <list>


bool BBox::intersect(const Ray& r) const {
    double t1_x = (bmin[0] - r.C[0]) / r.u[0];
    double t1_y = (bmin[1] - r.C[1]) / r.u[1];
    double t1_z = (bmin[2] - r.C[2]) / r.u[2];

    double t2_x = (bmax[0] - r.C[0]) / r.u[0];
    double t2_y = (bmax[1] - r.C[1]) / r.u[1];
    double t2_z = (bmax[2] - r.C[2]) / r.u[2];

    double tmin_x = std::min(t1_x, t2_x);
    double tmax_x = std::max(t1_x, t2_x);
    double tmin_y = std::min(t1_y, t2_y);
    double tmax_y = std::max(t1_y, t2_y);
    double tmin_z = std::min(t1_z, t2_z);
    double tmax_z = std::max(t1_z, t2_z);

    double min = std::min(std::min(tmax_x, tmax_y), tmax_z);
    if (min > 0 && min - std::max(std::max(tmin_x, tmin_y), tmin_z) > 0) return true;
    return false;
}


Intersection Mesh::intersect(const Ray& r) const {
    Intersection inter(false, Vector(0., 0., 0.), Vector(0., 0., 0.), Vector(0., 0., 0.), 1E99);
    if (!bvh.bbox.intersect(r)) return inter;

    std::list<const BVH*> l;
    l.push_front(&bvh);

    while (!l.empty()) {
        const BVH* current = l.front();
        l.pop_front();

        if (current->node_l && current->node_l->bbox.intersect(r)) {
            l.push_back(current->node_l);
        }
        if (current->node_r && current->node_r->bbox.intersect(r)) {
            l.push_back(current->node_r);
        }

        if (!current->node_r) {
            for (int i = current->i0; i < current->i1; ++i) {
                int i1 = indices[i].vtxi;
                int i2 = indices[i].vtxj;
                int i3 = indices[i].vtxk;
                Triangle tri(vertices[i1], vertices[i2], vertices[i3], albedo);
                 
                double alpha, beta, gamma;
                Intersection local_intersect = tri.intersect(r, alpha, beta, gamma);
                if (local_intersect.intersection) {
                    inter.intersection = true;
                    if (local_intersect.t < inter.t) {
                        inter.t = local_intersect.t;
                        inter.P = local_intersect.P;
                        inter.N = (normals[indices[i].ni] * alpha + normals[indices[i].nj] * beta + normals[indices[i].nk] * gamma).normalize();
                        
                        if (has_textures) {
                            int texture_id = indices[i].faceGroup;
                            int x = (uvs[indices[i].uvi][0] * alpha + uvs[indices[i].uvj][0] * beta + uvs[indices[i].uvk][0] * gamma) * (w[texture_id] - 1);
                            int y = (uvs[indices[i].uvi][1] * alpha + uvs[indices[i].uvj][1] * beta + uvs[indices[i].uvk][1] * gamma) * (h[texture_id] - 1);

                            double comp_red = (textures[texture_id][(y * w[texture_id] + x) * 3]) / 255.;
                            double comp_green = (textures[texture_id][(y * w[texture_id] + x) * 3 + 1]) / 255.;
                            double comp_blue = (textures[texture_id][(y * w[texture_id] + x) * 3 + 2]) / 255.;

                            inter.color = Vector(comp_red, comp_green, comp_blue);
                        } else {
                            inter.color = albedo;
                        }
                    }
                }
            }
        }
    }
    return inter;
};

BBox Mesh::build_bbox(int i0, int i1) {
    BBox result;
    result.bmin = vertices[indices[i0].vtxi];
    result.bmax = vertices[indices[i0].vtxi];
    for (int i = i0; i < i1; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.bmin[j] = std::min(result.bmin[j], vertices[indices[i].vtxi][j]);
            result.bmin[j] = std::min(result.bmin[j], vertices[indices[i].vtxj][j]);
            result.bmin[j] = std::min(result.bmin[j], vertices[indices[i].vtxk][j]);

            result.bmax[j] = std::max(result.bmax[j], vertices[indices[i].vtxi][j]);
            result.bmax[j] = std::max(result.bmax[j], vertices[indices[i].vtxj][j]);
            result.bmax[j] = std::max(result.bmax[j], vertices[indices[i].vtxk][j]);
        }
    }
    return result;
};

void Mesh::build_bvh(BVH* bvh, int i0, int i1) {
    bvh->bbox = build_bbox(i0, i1);
    bvh->i0 = i0;
    bvh->i1 = i1;
    bvh->node_l = NULL;
    bvh->node_r = NULL;

    // Trouver la dimension avec la plus grande amplitude
    Vector diag = bvh->bbox.bmax - bvh->bbox.bmin;
    int split_dim;
    if ((diag[0] > diag[1]) && (diag[0] > diag[2])) {
        split_dim = 2;
    }
    else {
        if ((diag[1] > diag[0]) && (diag[1] > diag[2])) {
            split_dim = 1;
        }
        else {
            split_dim = 2;
        }
    }

    double mid_dim = bvh->bbox.bmin[split_dim] + diag[split_dim] * 0.5;
    int pivot = i0;
    for (int i = i0; i < i1; ++i) {
        double center = (vertices[indices[i].vtxi][split_dim] + vertices[indices[i].vtxj][split_dim] + vertices[indices[i].vtxk][split_dim]) / 3.;
        if (center < mid_dim) {
            std::swap(indices[pivot], indices[i]);
            pivot++;
        }
    }

    if (pivot <= i0 || pivot >= i1 || i1 == i0+1) return;

    bvh->node_l = new BVH();
    build_bvh(bvh->node_l, i0, pivot);

    bvh->node_r = new BVH();
    build_bvh(bvh->node_r, pivot, i1);
};

void Mesh::readOBJ(const char* obj) {

    char matfile[255];
    char grp[255];

    FILE* f;
    errno_t err;
    err = fopen_s(&f, (std::string("models/") + std::string(obj)).c_str(), "rb");
    f = fopen(obj, "r");

    std::map<std::string, int> groupNames;
    int curGroup = -1;
    while (!feof(f)) {
        char line[255];
        if (!fgets(line, 255, f)) break;

        std::string linetrim(line);
        linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
        strcpy(line, linetrim.c_str());

        if (line[0] == 'u' && line[1] == 's') {
            sscanf(line, "usemtl %[^\n]\n", grp);
            if (groupNames.find(std::string(grp)) != groupNames.end()) {
                curGroup = groupNames[std::string(grp)];
            }
            else {
                curGroup = groupNames.size();
                groupNames[std::string(grp)] = curGroup;
            }
        }
        if (line[0] == 'm' && line[1] == 't' && line[2] == 'l') {
            sscanf(line, "mtllib %[^\n]\n", matfile);
        }
        if (line[0] == 'v' && line[1] == ' ') {
            Vector vec;
            Vector col;
            if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[2], &vec[1], &col[0], &col[1], &col[2]) == 6) {
                vertices.push_back(vec);
                vertexcolors.push_back(col);
            }
            else {
                sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[2], &vec[1]);  // helmet
                                                                             //vec[2] = -vec[2]; //car2
                vertices.push_back(vec);
            }
        }
        if (line[0] == 'v' && line[1] == 'n') {
            Vector vec;
            sscanf_s(line, "vn %lf %lf %lf\n", &vec[0], &vec[2], &vec[1]); //girl
            normals.push_back(vec);
        }
        if (line[0] == 'v' && line[1] == 't') {
            Vector vec;
            sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
            uvs.push_back(vec);
        }
        if (line[0] == 'f') {
            TriangleIndices t;
            int i0, i1, i2, i3;
            int j0, j1, j2, j3;
            int k0, k1, k2, k3;
            int nn;

            char* consumedline = line + 1;
            int offset;
            t.faceGroup = curGroup;
            nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
            if (nn == 9) {
                if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;

                indices.push_back(t);
            }
            else {
                nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
                if (nn == 6) {
                    if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                    if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                    if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                    if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                    if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                    if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                    indices.push_back(t);
                }
                else {
                    nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
                    if (nn == 3) {
                        if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                        if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                        if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                        indices.push_back(t);
                    }
                    else {
                        nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
                        if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                        if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                        if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                        if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                        if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                        if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                        indices.push_back(t);
                    }
                }
            }


            consumedline = consumedline + offset;

            while (true) {
                if (consumedline[0] == '\n') break;
                if (consumedline[0] == '\0') break;
                nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
                TriangleIndices t2;
                t2.faceGroup = curGroup;
                if (nn == 3) {
                    if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                    if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                    if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                    if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                    if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                    if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                    if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                    if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                    if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;
                    indices.push_back(t2);
                    consumedline = consumedline + offset;
                    i2 = i3;
                    j2 = j3;
                    k2 = k3;
                }
                else {
                    nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
                    if (nn == 2) {
                        if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                        if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                        if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                        if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                        if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                        if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                        consumedline = consumedline + offset;
                        i2 = i3;
                        j2 = j3;
                        indices.push_back(t2);
                    }
                    else {
                        nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
                        if (nn == 2) {
                            if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                            if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                            if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                            if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                            if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                            if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;
                            consumedline = consumedline + offset;
                            i2 = i3;
                            k2 = k3;
                            indices.push_back(t2);
                        }
                        else {
                            nn = sscanf(consumedline, "%u%n", &i3, &offset);
                            if (nn == 1) {
                                if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                consumedline = consumedline + offset;
                                i2 = i3;
                                indices.push_back(t2);
                            }
                            else {
                                consumedline = consumedline + 1;
                            }
                        }
                    }
                }
            }

        }


    }
    fclose(f);
    err = fopen_s(&f, (std::string("Model/") + std::string(matfile)).c_str(), "r");
    if (err == 0) {
        while (!feof(f))
        {
            char line[255];
            fgets(line, 255, f);
            if (line[0] == 'm' && line[4] == 'K' && line[5] == 'd')
            {
                char texturefile[255];
                sscanf_s(line, "map_Kd %100s\r\n", texturefile, 255);
                add_texture((std::string("Model/textures/") + std::string(texturefile).replace(std::strlen(texturefile) - 4, 4, ".bmp")).c_str());
            }
        }
        fclose(f);
    }
};

void Mesh::add_texture(const char* filename) {

    textures.resize(textures.size() + 1);
    w.resize(w.size() + 1);
    h.resize(h.size() + 1);

    FILE* f;
    errno_t err;
    err = fopen_s(&f, filename, "rb");
    unsigned char info[54];
    fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header

    w[w.size() - 1] = *(int*)&info[18]; // extract image height and width from header
    h[h.size() - 1] = *(int*)&info[22];

    int size = 3 * w[w.size() - 1] * h[h.size() - 1];
    textures[textures.size() - 1].resize(size); // allocate 3 bytes per pixel
    fread(&textures[textures.size() - 1][0], sizeof(unsigned char), size, f); // read the rest of the data at once
    fclose(f);

    for (int i = 0; i < size; i += 3) {
        std::swap(textures[textures.size() - 1][i], textures[textures.size() - 1][i + 2]);
    }
};