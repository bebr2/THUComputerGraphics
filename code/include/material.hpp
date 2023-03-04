#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>
#include "texture.h"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>

// TODO: Implement Shade function that computes Phong introduced in class.
class Material {
public:

    explicit Material(const Vector3f &d_color, const Vector3f &color, const Vector3f &emission, const Vector3f& ty, float refr = 1, const Vector3f &s_color = Vector3f::ZERO, float s = 0, const char *textureFile = "", const char *bumpFile = "") :
            diffuseColor(d_color), specularColor(s_color), shininess(s), color(color), emission(emission), refractive_index(refr), texture(textureFile), bump(bumpFile){
        this->type = ty / (ty.x() + ty.y() + ty.z());
    }

    virtual ~Material() = default;

    virtual Vector3f getDiffuseColor() const {
        return diffuseColor;
    }

    Vector3f get_color(float u, float v) {
        if(!texture.is_texture)
            return color;

        return texture.get_color(u, v);
    }

    float get_disturb(float u, float v, Vector2f& grad) {

        return bump.get_disturb(u, v, grad);
    }


    Vector3f Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor) {
        Vector3f shaded = Vector3f::ZERO;
        // 
        Vector3f Li = dirToLight.normalized();
        Vector3f N = hit.getNormal().normalized();
        // Li.print();
        // N.print();
        float diffuse = Vector3f::dot(Li, N);
        // std:: cout << diffuse << std::endl;
        Vector3f R = (2 * diffuse) * N - Li;
        R.normalize();

        if(diffuse < 0)
            diffuse = 0;
        float specular = - Vector3f::dot(ray.getDirection(), R);
        
        if(specular < 0)
            specular = 0;
        specular = pow(specular, shininess);
        shaded += specular * specularColor;
        bool is_texture;
        Vector3f dfcolor = hit.get_color(is_texture);
        if(!is_texture)
            shaded += diffuse * diffuseColor;
        else
            shaded += diffuse * dfcolor;
        shaded = lightColor * shaded;
        // shaded.print();
        return shaded;
    }
    Vector3f color;
    Vector3f type; //（漫反射，镜面反射, 折射）
    Vector3f emission; //发光系数
    Vector3f diffuseColor; //
    Vector3f specularColor; //镜面反射
    float shininess;
    float refractive_index; //折射率

    Texture texture;//颜色纹理
    Texture bump; //凹凸纹理
    
};


#endif // MATERIAL_H
