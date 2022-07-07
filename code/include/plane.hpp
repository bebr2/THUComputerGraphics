#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    Plane() {
        
    }

    Plane(const Vector3f &normal, float d, Material *m, const Vector3f &v) : Object3D(m, v) {
        //this->offset = d;
        //this->normal = normal;
        this->offset = d / normal.length();
        this->normal = normal.normalized();
        if(normal.x() < 1e-3 && normal.z() < 1e-3) {
            this->main_tangent = Vector3f::RIGHT;
        } else {
            this->main_tangent = Vector3f::cross(Vector3f::UP, normal);
            this->main_tangent.normalize();
        }
        
        this->bi_normal =  Vector3f::cross(main_tangent, normal);
        this->bi_normal.normalize();
        // this->is_AABB = false;
    }

    ~Plane() override = default;

    void get_uv(const Vector3f& p, float& u, float& v) {
        v = Vector3f::dot(p - offset * normal, bi_normal) / 100;
        u = Vector3f::dot(p - offset * normal, main_tangent) / 100;
        
    }

    bool intersect(const Ray &r, Hit &h, float tmin, float time_) override {
        Vector3f origin = r.getOrigin();
        Vector3f direction = r.getDirection();
        if(is_moving)
            origin -= time_ * this->velocity;
        float dir_len = direction.length();
        direction.normalize();
        float a = Vector3f::dot(origin, normal);
        float b = offset - a;
        float c = Vector3f::dot(direction, normal);
        if(c == 0)
            return false;
        if(b > -1e-3 && b < 1e-3)
            return false;
        float t = b / c;
        t = t / dir_len;
        // printf("%f\n", t);
        // std::cout << "h.t: " << h.getT() << std::endl;
        if(t > 0 && t > tmin && t < h.getT()) {
            // printf("%f\n", t);
            Vector3f next_origin = origin + r.getDirection() * t;
            float v = next_origin.y();
            float u = Vector3f::dot(next_origin - r.getDirection() * normal, main_tangent);
            Vector2f grad = Vector2f::ZERO;
            float f = material->bump.get_disturb(u, v, grad);

            Vector3f new_normal = normal;

            if (!(f < 1e-4 && f > -1e-4)) {
                new_normal += main_tangent * grad[0];
                new_normal += bi_normal * grad[1];
                new_normal.normalize();
                // new_normal.print();
                // std::cout << grad[0] << ' ' << grad[1] << std::endl;
            }

            float uu = 0, vv = 0;
			get_uv(next_origin, uu, vv);
            
            h.set(t, this->material, new_normal, material->get_color(uu, vv), next_origin, material->texture.is_texture);
            return true;
        }
        return false;
    }

protected:
    Vector3f normal;
    Vector3f main_tangent;
    Vector3f bi_normal;
    float offset;

};

#endif //PLANE_H
		

