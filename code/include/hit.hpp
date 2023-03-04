#ifndef HIT_H
#define HIT_H

#include <vecmath.h>
#include "ray.hpp"

class Material;

class Hit {
public:

    // constructors
    Hit() {
        material = nullptr;
        t = 1e38;
    }

    Hit(float _t, Material *m, const Vector3f &n) {
        t = _t;
        material = m;
        normal = n;
    }

    Hit(const Hit &h) {
        t = h.t;
        material = h.material;
        normal = h.normal;
    }

    // destructor
    ~Hit() = default;

    float getT() const {
        return t;
    }

    Material *getMaterial() const {
        return material;
    }

    const Vector3f &getNormal() const {
        return normal;
    }

    void set(float _t, Material *m, const Vector3f &n, const Vector3f &c = Vector3f::ZERO, const Vector3f &_p = Vector3f::ZERO, bool _is_texture = false) {
        t = _t;
        material = m;
        normal = n;
        color = c;
        is_texture = _is_texture;
        p = _p;
    }


    Vector3f get_color(bool& _is_texture) const {
        _is_texture = is_texture;
        return color;
    }
    Vector3f color, p;
    bool is_texture;
private:
    float t;
    Material *material;
    Vector3f normal;
    

};

inline std::ostream &operator<<(std::ostream &os, const Hit &h) {
    os << "Hit <" << h.getT() << ", " << h.getNormal() << ">";
    return os;
}

#endif // HIT_H
