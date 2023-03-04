#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement functions and add more fields as necessary

class Sphere : public Object3D {
    
public:

    Sphere() {
        // unit ball at the center
        this->radius = 1.0;
        this->center = Vector3f::ZERO;
    }

    Sphere(const Vector3f &center, float radius, Material *material, const Vector3f &v) : Object3D(material, v) {
        // 
        // (radius < 400) ? this->is_AABB = true : this->is_AABB = false; //太大的是灯
        this->radius = radius;
        this->center = center;
        // minx = (this->center - radius * Vector3f(1, 0, 0)).x();
        // miny = (this->center - radius * Vector3f(0, 1, 0)).y();
        // minz = (this->center - radius * Vector3f(0, 0, 1)).z();
        // maxx = (this->center + radius * Vector3f(1, 0, 0)).x();
        // maxy = (this->center + radius * Vector3f(0, 1, 0)).y();
        // maxz = (this->center + radius * Vector3f(0, 0, 1)).z();
    }

    ~Sphere() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin, float time_) override {
        //
        Vector3f origin = r.getOrigin();
        if(is_moving)
            origin -= time_ * this->velocity;
        Vector3f direction = r.getDirection();
        float dir_len = direction.length();

        direction.normalize();
        Vector3f l = this->center - origin;
        float l_sqlength = l.squaredLength();
        float radius_sq = radius * radius;
        
        if(l_sqlength <= radius_sq + 0.01 && l_sqlength >= radius_sq - 0.01) {
            ///???????
            float t = 2 * Vector3f::dot(direction, l);
            t = t / dir_len;
            if (t <= 0)
                return false;
            Vector3f point2 = r.pointAtParameter(t);
            Vector3f p = point2 - center;
            
            if(t > tmin &&  t < h.getT()) {
                float u;
                u = 0.5 + atan2(p.x(), p.z()) / (2 * M_PI);
                float v = 0.5 - asin(p.y()) / M_PI;
                p.normalize();
                Vector3f new_color = material->get_color(u, v);

                h.set(t, this->material, p, new_color, point2, material->texture.is_texture);
                return true;
            }
        }

        float tp = Vector3f::dot(l, direction);

        if(tp < 0 && l_sqlength > radius_sq)
            return false;

        float d_sq = l_sqlength - tp * tp;
        if(d_sq > radius_sq)
            return false;
        float t_sq =  radius_sq - d_sq;
        float final_t;
        Vector3f p;
        if(l_sqlength > radius_sq) {
            final_t = tp - sqrt(t_sq);
            p = final_t * direction;
            p += origin;
            p -= this->center;
        } else {
            final_t = tp + sqrt(t_sq);
            p = final_t * direction;
            p += origin;
            p -= this->center;
        }
        Vector3f old_normal = p.normalized();
        // std::cout << final_t / dir_len << endl;
        // std::cout << "h.t: " << h.getT() << std::endl;
        if(final_t / dir_len > tmin &&  final_t / dir_len < h.getT()) {
            // std::cout << final_t / dir_len << endl;
            float u;
            u = 0.5 + atan2(old_normal.x(), old_normal.z()) / (2 * M_PI);
            float v = 0.5 - asin(old_normal.y()) / M_PI;

            Vector2f grad = Vector2f::ZERO;
            float f = material->bump.get_disturb(u, v, grad);
            
            Vector3f new_normal = p;
            if (!(f < 1e-4 && f > -1e-4)) {
                float phi = u * 2 * M_PI, theta = M_PI - v * M_PI;
                Vector3f pu(-p.z(), 0, p.x()), pv(p.y() * cos(phi), -radius * sin(theta), p.y() * sin(phi));

                new_normal = Vector3f::cross(pu + old_normal * grad[0] / (2 * M_PI), pv + old_normal * grad[1] / M_PI).normalized();

            }
            // printf("newc");
            Vector3f new_color = material->get_color(u, v);
            // new_color.print();
            h.set(final_t / dir_len, this->material, new_normal, new_color, p + this->center, material->texture.is_texture);
            return true;
        }


        return false;
    }

protected:
    float radius;
    Vector3f center;
};


#endif
