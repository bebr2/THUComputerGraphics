#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"
#include <algorithm>

using namespace std;


struct Pel {
    Vector3f minxyz;
    Vector3f maxxyz;
    Vector3f centroid;
    int index;
    Pel(float minx, float miny, float minz, float maxx, float maxy, float maxz, int idx) {
        minxyz = Vector3f(minx, miny, minz);
        maxxyz = Vector3f(maxx, maxy, maxz);
        centroid = (minxyz + maxxyz) * 0.5;
        index = idx;
    }
};



struct BVH_TreeNode {
    BVH_TreeNode* parent;
    BVH_TreeNode* lc;
    BVH_TreeNode* rc;
    Vector3f minxyz;
    Vector3f maxxyz;
    int pel_num;
    vector<Pel>::iterator first_pel; //有这个的是叶子节点
    BVH_TreeNode(BVH_TreeNode* a, vector<Pel>::iterator it, int _num);
};


bool Ray_hit_AABB(BVH_TreeNode* e, const Ray &r, float& t);

class Mesh : public Object3D {

public:
    BVH_TreeNode* produce_child(int axis, BVH_TreeNode* now_root);//axis:0为x轴，1为y轴，2为z轴

    Mesh(const char *filename, Material *m, const Vector3f& v);
    ~Mesh();
    struct TriangleIndex {
        TriangleIndex() {
            x[0] = 0; x[1] = 0; x[2] = 0;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3]{};
    };

    vector<TriangleIndex> vIndex, tIndex, nIndex;
    vector<Vector3f> v, vn;
    vector<Vector2f> vt;
    vector<Object3D *> triangles;

    bool intersect(const Ray &r, Hit &h, float tmin, float time_) override;
    bool hit_intersect(const Ray &r, Hit &h, float tmin, BVH_TreeNode* e);
    BVH_TreeNode* root;
    vector<Pel> triangles_info;

    void delete_node(BVH_TreeNode* e);


private:

    // Normal can be used for light estimation
    void computeNormal();
    void computeNormal_and_texture();
    void computeAABB();

    void setup_bvh_tree();
    void produce_child(BVH_TreeNode* e);
};

#endif
