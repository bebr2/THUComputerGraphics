#ifndef LIGHT_H
#define LIGHT_H

#include <Vector3f.h>
#include "object3d.hpp"

class Light {
public:
    Light() = default;

    virtual ~Light() = default;

    virtual void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const = 0;

    virtual Ray get_ray(Vector3f& pp) const = 0;

    virtual Vector3f get_direction() const = 0;

    Vector3f emission;

    Vector3f position;

    float radius;
};


class DirectionalLight : public Light {
public:
    DirectionalLight() = delete;

    DirectionalLight(const Vector3f &d, const Vector3f &c) {
        direction = d.normalized();
        color = c;
    }

    ~DirectionalLight() override = default;

    ///@param p unsed in this function
    ///@param distanceToLight not well defined because it's not a point light
    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = -direction;
        col = color;
    }

    Vector3f get_direction() const {
        printf("Error!/n");
        return Vector3f::ZERO;
    }

    Ray get_ray(Vector3f & ini_power) const {
        printf("Error!/n");
        return Ray(Vector3f::ZERO, Vector3f::ZERO);

    }

private:

    Vector3f direction;
    Vector3f color;

};

class PointLight : public Light {
public:
    PointLight() = delete;

    PointLight(const Vector3f &p, const Vector3f &c) {
        position = p;
        color = c;
    }

    ~PointLight() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = (position - p);
        dir = dir / dir.length();
        col = color;
    }

    Vector3f get_direction() const {
        printf("Error!/n");
        return Vector3f::ZERO;
    }

    Ray get_ray(Vector3f & ini_power) const {
        printf("Error!/n");
        return Ray(Vector3f::ZERO, Vector3f::ZERO);

    }

private:

    
    Vector3f color;

};

class RoundDisk_Light : public Light {
public:
    RoundDisk_Light() = delete;

    RoundDisk_Light(const Vector3f &p, const Vector3f &dir, const Vector3f &c, float r, const Vector3f& em) {
        position = p;
        color = c;
        radius = r;
        emission = em;
        direction = dir.normalized();
        if(direction.x() < 1e-4 && direction.x() > -1e-4)
            if(direction.z() < 1e-4 && direction.z() > -1e-4)
                if(1 - direction.y() < 1e-4 || 1 + direction.y() < 1e-4) {
                    x_axis = Vector3f::RIGHT;
                    y_axis = Vector3f::FORWARD;
                    return;
                }
        if(abs(direction.x()) < 1e-3 && abs(direction.z()) < 1e-3) { //和y轴平行
            x_axis = Vector3f(1, 0, 0);
            y_axis = Vector3f(0, 0, 1);
        } else {
            x_axis = Vector3f::cross(direction, Vector3f::UP).normalized();
            y_axis = Vector3f::cross(direction, x_axis).normalized();
        }
        // x_axis.print();
        // y_axis.print();
        
    }

    ~RoundDisk_Light() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        dir = (position - p);
        dir = dir / dir.length();
        col = color;
    }

    Ray get_ray(Vector3f & ini_power) const {
        float alpha = RND2 * 2 * M_PI;
        Vector3f p = position + radius * cos(alpha) * x_axis + radius * sin(alpha) * y_axis;
        // float theta = RND2 * 2 * M_PI; //[0, 2pi]
        // float phi = RND2 * M_PI / 2; //[0, pi/2]
        // Vector3f new_dir = direction * cos(phi) + sin(phi) * cos(theta) * x_axis + sin(phi) * sin(theta) * y_axis;
        Vector3f new_dir = direction * RND2 + RND1 * x_axis + RND1 * y_axis;
        new_dir.normalize();
        if(new_dir.squaredLength() < 1e-4)
            new_dir = direction;
        ini_power = (M_PI * M_PI * radius * radius) * emission;
        // new_dir.print();
        // p.print();
        return Ray(p, new_dir);
    }

    Vector3f get_direction() const {
        return direction;
    }

private:

    Vector3f color;
    
    Vector3f direction;
    Vector3f x_axis, y_axis;

};

#endif // LIGHT_H
