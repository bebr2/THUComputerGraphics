#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <vecmath.h>
#include "object3d.hpp"

// transforms a 3D point using a matrix, returning a 3D point
static Vector3f transformPoint(const Matrix4f &mat, const Vector3f &point) {
    return (mat * Vector4f(point, 1)).xyz();
}

// transform a 3D directino using a matrix, returning a direction
static Vector3f transformDirection(const Matrix4f &mat, const Vector3f &dir) {
    return (mat * Vector4f(dir, 0)).xyz();
}

// TODO: implement this class so that the intersect function first transforms the ray
class Transform : public Object3D {
public:
    Transform() {}

    Transform(const Matrix4f &m, Object3D *obj, const Vector3f &v) : o(obj) {
        transform = m.inverse();
        if(abs(v.x()) < 1e-3 && abs(v.y()) < 1e-3 && abs(v.z()) < 1e-3) {
            is_moving = false;
            velocity = Vector3f::ZERO;
        } else {
            is_moving = true;
            velocity = v;
        }
    }

    ~Transform() {
    }

    virtual bool intersect(const Ray &r, Hit &h, float tmin, float time_) {
        Vector3f trSource;
        if(is_moving)
            trSource = transformPoint(transform, r.getOrigin() - time_ * velocity);
        else
            trSource = transformPoint(transform, r.getOrigin());
        Vector3f trDirection = transformDirection(transform, r.getDirection());
        Ray tr(trSource, trDirection);
        bool inter = o->intersect(tr, h, tmin, time_);
        if (inter) {
            h.set(h.getT(), h.getMaterial(), transformDirection(transform.transposed(), h.getNormal()).normalized(), h.color, h.p, h.is_texture);
        }
        return inter;
    }

protected:
    Object3D *o; //un-transformed object
    Matrix4f transform;
};

#endif //TRANSFORM_H
