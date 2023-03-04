#include "mesh.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>
#include "constant.h"

using namespace std;

float now_center;

bool Pel_min_x(Pel x, Pel y) {
    return x.minxyz.x() < y.minxyz.x();
}

bool Pel_min_y(Pel x, Pel y) {
    return x.minxyz.y() < y.minxyz.y();
}

bool Pel_min_z(Pel x, Pel y) {
    return x.minxyz.z() < y.minxyz.z();
}

bool Pel_max_x(Pel x, Pel y) {
    return x.maxxyz.x() < y.maxxyz.x();
}

bool Pel_max_y(Pel x, Pel y) {
    return x.maxxyz.y() < y.maxxyz.y();
}

bool Pel_max_z(Pel x, Pel y) {
    return x.maxxyz.z() < y.maxxyz.z();
}

bool Pel_cen_x(Pel x) {
    return x.centroid.x() < now_center;
}

bool Pel_cen_y(Pel x) {
    return x.centroid.y() < now_center;
}

bool Pel_cen_z(Pel x) {
    return x.centroid.z() < now_center;
}

BVH_TreeNode::BVH_TreeNode(BVH_TreeNode* a, vector<Pel>::iterator it, int _num) {
        parent = a;
        lc = nullptr;
        rc = nullptr;
        first_pel = it;
        pel_num = _num;

        if(_num) {
            float minx = (*min_element(it, it + _num, Pel_min_x)).centroid.x();
            float miny = (*min_element(it, it + _num, Pel_min_y)).centroid.y();
            float minz = (*min_element(it, it + _num, Pel_min_z)).centroid.z();
            float maxx = (*max_element(it, it + _num, Pel_max_x)).centroid.x();
            float maxy = (*max_element(it, it + _num, Pel_max_y)).centroid.y();
            float maxz = (*max_element(it, it + _num, Pel_max_z)).centroid.z();
            minxyz = Vector3f(minx, miny, minz);
            maxxyz = Vector3f(maxx, maxy, maxz);
        }
    }

bool Ray_hit_AABB(BVH_TreeNode* e, const Ray &r, float &the_t) {

    if(e->pel_num == 0)
        return false;

    Vector3f origin = r.getOrigin();
    Vector3f direction = r.getDirection().normalized();

    float t_xmin = -1e38;
    float t_xmax = 1e38;
    float t_ymin = -1e38;
    float t_ymax = 1e38;
    float t_zmin = -1e38;
    float t_zmax = 1e38;

    if(direction.x() > 1e-4) {
        t_xmin = (e->minxyz.x() - origin.x()) / direction.x();
        t_xmax = (e->maxxyz.x() - origin.x()) / direction.x();
    } else if(direction.x() < -1e-4) {
        t_xmax = (e->minxyz.x() - origin.x()) / direction.x();
        t_xmin = (e->maxxyz.x() - origin.x()) / direction.x();
    } else if(origin.x() > e->maxxyz.x() || origin.x() < e->minxyz.x())
        return false;

    if(t_xmax <= 0)
        return false;

    if(direction.y() > 1e-4) {
        t_ymin = (e->minxyz.y() - origin.y()) / direction.y();
        t_ymax = (e->maxxyz.y() - origin.y()) / direction.y();
    } else if(direction.y() < -1e-4) {
        t_ymax = (e->minxyz.y() - origin.y()) / direction.y();
        t_ymin = (e->maxxyz.y() - origin.y()) / direction.y();
    } else if(origin.y() > e->maxxyz.y() || origin.y() < e->minxyz.y())
        return false;

    if(t_ymax <= 0)
        return false;

    if(direction.z() > 1e-4) {
        t_zmin = (e->minxyz.z() - origin.z()) / direction.z();
        t_zmax = (e->maxxyz.z() - origin.z()) / direction.z();
    } else if(direction.z() < -1e-4) {
        t_zmax = (e->minxyz.z() - origin.z()) / direction.z();
        t_zmin = (e->maxxyz.z() - origin.z()) / direction.z();
    } else if(origin.z() > e->maxxyz.z() || origin.z() < e->minxyz.z())
        return false;

    if(t_zmax <= 0)
        return false;
    
    float t0, t1;
    t0 = max(t_zmin, max(t_xmin, t_ymin));
    t1 = min(t_zmax, min(t_xmax, t_ymax));
    
    if(t0 < t1) {
        the_t = (t0 > 0) ? t0 : t1;
        return true;
    } else {
        return false;
    }

};

bool Mesh::hit_intersect(const Ray &r, Hit &h, float tmin, BVH_TreeNode* e) {//已经通过AABB包围盒测试的
    if (e->rc == nullptr && e->lc == nullptr) {//叶子节点
        bool result = false;
        for (auto it = e->first_pel; it != e->first_pel + e->pel_num; ++it) {
            result |= ((Triangle *)triangles[(*it).index])->intersect(r, h, tmin, 0);
        }
        return result;
    }
    float lc_t;
    float rc_t;
    bool lc_hit = Ray_hit_AABB(e->lc, r, lc_t);
    bool rc_hit = Ray_hit_AABB(e->rc, r, rc_t);

    if(!(lc_hit || rc_hit))
        return false;

    bool real_hit;

    if(lc_hit && rc_hit) {
        if(lc_t < rc_t) {
            real_hit = hit_intersect(r, h, tmin, e->lc);
            if(real_hit)
                return true;
            else
                real_hit = hit_intersect(r, h, tmin, e->rc);
            return real_hit;
        } else {
            real_hit = hit_intersect(r, h, tmin, e->rc);
            if(real_hit)
                return true;
            else
                real_hit = hit_intersect(r, h, tmin, e->lc);
            return real_hit;
        }
    } else if(lc_hit)
        return hit_intersect(r, h, tmin, e->lc);
    else
        return hit_intersect(r, h, tmin, e->rc);

};


bool Mesh::intersect(const Ray &r, Hit &h, float tmin, float time_) {

    // Optional: Change this brute force method into a faster one.
    bool result = false;
    // for (int triId = 0; triId < (int) triangles.size(); ++triId) {
    //     result |= ((Triangle *)triangles[triId])->intersect(r, h, tmin);
    // }
    // return result;
    float t;
    if(!Ray_hit_AABB(root, r, t))
        return false;

    if(!is_moving)
        result = hit_intersect(r, h, tmin, root);
    else
        result = hit_intersect(Ray(r.origin - time_ * velocity, r.direction), h, tmin, root);
    return result;
}

Mesh::Mesh(const char *filename, Material *material, const Vector3f& ve) : Object3D(material, ve) {
    // this->is_AABB = true;
    // Optional: Use tiny obj loader to replace this simple one.
    // std::cout << "mesh begin" << std::endl;
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    std::string line;
    std::string vTok("v");
    std::string fTok("f");
    std::string texTok("vt");
    std::string vnTok("vn");

    char bslash = '/', space = ' ';
    std::string tok;
    bool texture_and_normal = false;
    while (true) {
        std::getline(f, line);
        if (f.eof()) {
            break;
        }
        if (line.size() < 3) {
            continue;
        }
        if (line.at(0) == '#') {
            continue;
        }
        std::stringstream ss(line);
        ss >> tok;

        if (tok == vTok) {
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            v.push_back(vec);
        } else if (tok == fTok) {
            TriangleIndex vrig, trig, nrig;
            if (line.find(bslash) != std::string::npos) {  //限定两种输入方式f 2 3 4与f 2/3/4 3/4/5 4/5/6
                texture_and_normal = true;
                std::replace(line.begin(), line.end(), bslash, space);

                std::stringstream facess(line);
                facess >> tok;
                for (int ii = 0; ii < 3; ii++) {
                    facess >> vrig[ii] >> trig[ii] >> nrig[ii];
                    trig[ii]--;
                    vrig[ii]--;
                    nrig[ii]--;
                }
            } else {
                for (int ii = 0; ii < 3; ii++) {
                    ss >> vrig[ii];
                    vrig[ii]--;
                }
            }
            // std::cout << vrig[0] << ' ' << vrig[1] << ' ' << vrig[2] << std::endl;
            vIndex.push_back(vrig);
            tIndex.push_back(trig);
            nIndex.push_back(nrig);
        } else if (tok == texTok) {
            Vector2f texcoord;
            ss >> texcoord[0];
            ss >> texcoord[1];
            vt.push_back(texcoord);
        } else if (tok == vnTok) {
            Vector3f vnvec;
            ss >> vnvec[0] >> vnvec[1] >> vnvec[2];
            vn.push_back(vnvec);
        }
    }
    f.close();
    // std::cout <<  vIndex.size() << std::endl;
    if(!texture_and_normal)
        computeNormal();
    else
        computeNormal_and_texture();

    cout << "Normal and (possible) texture have been computed." << endl;

    computeAABB();

    cout << "AABB box has been computed." << endl;
    cout << "triangle num:" << triangles_info.size() << endl;

    setup_bvh_tree();

    cout << "BVH tree has set up." << endl;
    
}

void Mesh::computeAABB() {
    if((int) triangles.size() <= 0) {
        printf("Obj File Errors.\n");
        exit(0);
    }

    for (int i = 0; i < (int) triangles.size(); ++i) {
        float now_minx, now_maxx, now_miny, now_maxy, now_minz, now_maxz;
        for (int j = 0; j < 3; j++) {
            if(j == 0) {
                now_minx = now_maxx = ((Triangle*) triangles[i])->vertices[j].x();
                now_miny = now_maxy = ((Triangle*) triangles[i])->vertices[j].y();
                now_minz = now_maxz = ((Triangle*) triangles[i])->vertices[j].z();
            } else {
                if(((Triangle*) triangles[i])->vertices[j].x() < now_minx)
                    now_minx = ((Triangle*) triangles[i])->vertices[j].x();
                else if(((Triangle*) triangles[i])->vertices[j].x() > now_maxx)
                    now_maxx = ((Triangle*) triangles[i])->vertices[j].x();

                if(((Triangle*) triangles[i])->vertices[j].y() < now_miny)
                    now_miny = ((Triangle*) triangles[i])->vertices[j].y();
                else if(((Triangle*) triangles[i])->vertices[j].y() > now_maxy)
                    now_maxy = ((Triangle*) triangles[i])->vertices[j].y();

                if(((Triangle*) triangles[i])->vertices[j].z() < now_minz)
                    now_minz = ((Triangle*) triangles[i])->vertices[j].z();
                else if(((Triangle*) triangles[i])->vertices[j].z() > now_maxz)
                    now_maxz = ((Triangle*) triangles[i])->vertices[j].z();
            }
        }
        Pel t = Pel(now_minx, now_miny, now_minz, now_maxx, now_maxy, now_maxz, i);
        this->triangles_info.push_back(t);
    }
}

void Mesh::computeNormal() {
    vn.resize(vIndex.size());
    for (int triId = 0; triId < (int) vIndex.size(); ++triId) {
        TriangleIndex& triIndex = vIndex[triId];
        Vector3f a = v[triIndex[1]] - v[triIndex[0]];
        Vector3f b = v[triIndex[2]] - v[triIndex[0]];
        b = Vector3f::cross(a, b);
        vn[triId] = b / b.length();

        Triangle* triangle = new Triangle(v[triIndex[0]],
                          v[triIndex[1]], v[triIndex[2]], material, Vector3f::ZERO);
        triangle->normal = vn[triId];
        triangles.push_back(triangle);
    }
}

void Mesh::delete_node(BVH_TreeNode* e) {
    if(!e)
        return;
    if (e->rc == nullptr && e->lc == nullptr) {
        delete e;
        return;
    }
    delete_node(e->lc);
    delete_node(e->rc);
    delete e;
    return;
}

Mesh::~Mesh() {//记得写这个，删树节点
    for (int i = 0; i < triangles.size(); i++)
        delete triangles[i];
    delete_node(root);
}

void Mesh::computeNormal_and_texture() {
    for (int triId = 0; triId < (int) vIndex.size(); ++triId) {

        TriangleIndex& triIndex = vIndex[triId];
        // std::cout << triIndex[0] << ' ' << triIndex[1] << ' ' << triIndex[2] << std::endl;

        Triangle* triangle = new Triangle(v[triIndex[0]], v[triIndex[1]], v[triIndex[2]], material, Vector3f::ZERO);

        triangles.push_back(triangle);

        if(!vt.empty()) {
            TriangleIndex& tIdx = tIndex[triId];
            ((Triangle *)triangles.back())->set_vt(vt[tIdx[0]], vt[tIdx[1]], vt[tIdx[2]]);
        }
        
        if(!vn.empty()) {
            TriangleIndex& nIdx = nIndex[triId];
            ((Triangle *)triangles.back())->set_vn(vn[nIdx[0]], vn[nIdx[1]], vn[nIdx[2]]);
        }
        

    }
}

void Mesh::setup_bvh_tree() {
    root = new BVH_TreeNode(nullptr, triangles_info.begin(), (int) triangles_info.size());
    produce_child(root);
}

void Mesh::produce_child(BVH_TreeNode* e) {
    if(e->pel_num < leaf_max_pel)
        return;
    bool (*comp)(Pel x);
    float dx = e->maxxyz.x() - e->minxyz.x();
    float dy = e->maxxyz.y() - e->minxyz.y();
    float dz = e->maxxyz.z() - e->minxyz.z();
    if (dx > dy && dx > dz) {
        comp = Pel_cen_x;
        now_center = e->minxyz.x() + dx / 2;
    } else if (dy > dz){
        comp = Pel_cen_y;
        now_center = e->minxyz.y() + dy / 2;
    } else {
        comp = Pel_cen_z;
        now_center = e->minxyz.z() + dz / 2;
    }

    auto bound = partition(e->first_pel, e->first_pel + e->pel_num, comp);
    int distance1 = distance(e->first_pel, bound);
    int distance2 = e->pel_num - distance1;
    BVH_TreeNode* l_child = new BVH_TreeNode(e, e->first_pel, distance1);
    BVH_TreeNode* r_child = new BVH_TreeNode(e, bound, distance2);
    produce_child(l_child);
    produce_child(r_child);
}