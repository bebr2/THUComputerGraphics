#ifndef OBJECT3D_H
#define OBJECT3D_H

#include "ray.hpp"
#include "hit.hpp"
#include "material.hpp"

// Base class for all 3d entities.
class Object3D {
public:
    Object3D() : material(nullptr) {}

    virtual ~Object3D() = default;

    explicit Object3D(Material *material, const Vector3f& v) {
        this->material = material;
        if(abs(v.x()) < 1e-3 && abs(v.y()) < 1e-3 && abs(v.z()) < 1e-3) {
            is_moving = false;
            velocity = Vector3f::ZERO;
        } else {
            is_moving = true;
            velocity = v;
        }
    }

    // Intersect Ray with this object. If hit, store information in hit structure.
    virtual bool intersect(const Ray &r, Hit &h, float tmin, float t) = 0;

    bool is_moving;

    Vector3f velocity;
    
protected:

    Material *material;
};

#endif

