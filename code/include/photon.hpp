#ifndef PHOTON_H
#define PHOTON_H
//优先堆找时间给他去了。。
#include "camera.hpp"
#include "constant.h"
#include "group.hpp"
#include "hit.hpp"
#include "image.hpp"
#include "light.hpp"
#include "ray.hpp"
#include "scene_parser.hpp"
#include "random_producer.hpp"
#include "photon.hpp"
#include <queue>
#include <algorithm>

using namespace std;

struct Photon {
    Vector3f position;
    Vector3f direction;
    Vector3f power;
    Photon(Vector3f _position, Vector3f _direction, Vector3f _power) {
        position = _position;
        direction = _direction;
        power = _power;
    }
};

struct KDTreeNode
{
    int depth;
    Photon* photon;
    KDTreeNode* lc;
    KDTreeNode* rc;
    KDTreeNode* parent;
    KDTreeNode(KDTreeNode* e) {
        parent = e;
        lc = nullptr;
        rc = nullptr;
        photon = nullptr;
    }
};

Vector3f Pos;

struct cmp
{
     bool operator()(const Photon* a, const Photon*b)
    {
         return (a->position - Pos).squaredLength() < (b->position - Pos).squaredLength();
    }
};


bool find_median_x(Photon* pp, Photon* qq) {
    return pp->position.x() < qq->position.x();
}

bool find_median_y(Photon* pp, Photon* qq) {
    return pp->position.y() < qq->position.y();
}

bool find_median_z(Photon* pp, Photon* qq) {
    return pp->position.z() < qq->position.z();
}

bool cmp_x(Vector3f& pp, Vector3f& qq) {
    return pp.x() < qq.x();
}

bool cmp_y(Vector3f& pp, Vector3f& qq) {
    return pp.y() < qq.y();
}

bool cmp_z(Vector3f& pp, Vector3f& qq) {
    return pp.z() < qq.z();
}




class Photon_KDtree {
    public:
    KDTreeNode* root;
    vector<Photon*>::iterator start;
    vector<Photon*>::iterator end;
    // priority_queue<Photon*, vector<Photon*>, cmp> k_near;
    vector<Photon*> k_near;

    void buildtree(vector<Photon*>::iterator _start, vector<Photon*>::iterator _end, int depth, KDTreeNode* e);
    void search(Vector3f pos, float& radius, int& M, Vector3f& phi_i);
    Photon_KDtree(vector<Photon*>::iterator _start, vector<Photon*>::iterator _end) : start(_start) , end(_end){
        buildtree(start, end, 0, root);
        KDTreeNode* p = root;
        // while(p) {
        //     cout << "dep:" << p->depth << endl;
        //     p->photon->direction.print();
        //     p->photon->position.print();
        //     p = p->lc;
        // }
    };

    void recurve(KDTreeNode* e, float radius);
    // ~Photon_KDtree() {
    //     delete_node(root);
    // };

    void delete_node(KDTreeNode* e) {
        if(!e)
            return;
        if(e->photon)
            delete e->photon;
        if (e->rc == nullptr && e->lc == nullptr) {
            delete e;
            return;
        }
        delete_node(e->lc);
        delete_node(e->rc);
        delete e;
        return;
    }

};

void Photon_KDtree::buildtree(vector<Photon*>::iterator _start, vector<Photon*>::iterator _end, int depth, KDTreeNode* e) {
    int nums = distance(_start, _end);
    e->depth = depth;
    if(nums == 0)
        return;
    int total = nums / 2;
    bool (*cmp_func) (Photon* p1, Photon* p2);
    switch (depth % 3)
    {
    case 0:
        cmp_func = find_median_z;
        break;
    case 1:
        cmp_func = find_median_y;
        break;
    default:
        cmp_func = find_median_x;
        break;
    }
    sort(_start, _end, cmp_func);
    // if(depth == 0 ) {
    //     for(auto it = _start; it != _end; ++ it) {
    //         (*it)->position.print();
    //     }
    // }
    auto it = _start + total;
    e->photon = *it;
    KDTreeNode* e1 = new KDTreeNode(e);
    KDTreeNode* e2 = new KDTreeNode(e);
    e->lc = e1;
    e->rc = e2;
    buildtree(_start, it, depth + 1, e1);
    buildtree(it + 1, _end, depth + 1, e2);
        // priority_queue<Photon*, vector<Photon*>, cmp_z> maxHeap;
        // int total = nums / 2 + 1;
        // for (auto it = _start; it != end; ++it) {
        //     if (maxHeap.size() < total) {
        //         maxHeap.push((*it));
        //     } else {
        //         if (maxHeap.top()->direction.z() > (*it)->direction.z()) {
        //             maxHeap.pop();
        //             maxHeap.push((*it));
        //         }
        //     }
        // }
        // e->photon = maxHeap.top();
        // median = e->photon->direction.z();
        // auto bound = partition(_start, _end, find_median_z);
}

void Photon_KDtree::search(Vector3f pos, float& radius, int& M, Vector3f& phi_i) {
    // while(!k_near.empty()) {
    //     k_near.pop();
    // }
    k_near.clear();
    Pos = pos;
    recurve(root, radius);
    // float new_radius = (k_near.top()->position - Pos).length();
    M = (int)k_near.size();
    //叠加power：
    for (auto it = k_near.begin(); it != k_near.end(); ++it)
        phi_i += (*it)->power;

}

void Photon_KDtree::recurve(KDTreeNode* e, float radius) {
    if(!e->photon)
        return;

    float axis_dist;
    bool (*cmp_func) (Vector3f& pp, Vector3f& qq);
    switch (e->depth % 3)
    {
    case 0:
        cmp_func = cmp_z;
        axis_dist = Pos.z() - e->photon->position.z();
        break;
    case 1:
        cmp_func = cmp_y;
        axis_dist = Pos.y() - e->photon->position.y();
        break;
    default:
        cmp_func = cmp_x;
        axis_dist = Pos.x() - e->photon->position.x();
        break;
    }

    if(cmp_func(Pos, e->photon->position))
        recurve(e->lc, radius);
    else
        recurve(e->rc, radius);


    // if(k != -1) {
    //     float now_max_dist = (k_near.top()->position - Pos).squaredLength();
    //     if((int)k_near.size() < k)
    //         k_near.push(e->photon);
    //     else {
    //         if(now_max_dist > (e->photon->position - Pos).squaredLength()) {
    //             k_near.pop();
    //             k_near.push(e->photon);
    //         }
    //     }
    //     if (now_max_dist > axis_dist * axis_dist) {
    //         if(axis_dist < 0)
    //             recurve(e->rc, radius);
    //         else
    //             recurve(e->lc, radius);
    //     }
    // } else {
        if((e->photon->position - Pos).squaredLength() < radius * radius)
            k_near.push_back(e->photon);
        if (radius > abs(axis_dist)) {
            if(axis_dist < 0)
                recurve(e->rc, radius);
            else
                recurve(e->lc, radius);
        }
        
    // }
}

#endif