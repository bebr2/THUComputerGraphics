#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// TODO: implement this class and add more fields as necessary,
class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m, const Vector3f &v) : Object3D(m, v) {
		this->vertices[0] = a;
		this->vertices[1] = b;
		this->vertices[2] = c;
		Vector3f x1 = b - a;
		Vector3f x2 = c - a;
		this->normal = Vector3f::cross(x1, x2);
		this->normal.normalize();
		this->v0v0 = Vector3f::dot(x2, x2);
		this->v0v1 = Vector3f::dot(x1, x2);
		this->v1v1 = Vector3f::dot(x1, x1);
		this->inverDeno = v0v0 * v1v1 - v0v1 * v0v1;
		this->d = Vector3f::dot(this->normal, a);
		this->is_norm = false;
		this->is_texture = false;
		// this->is_AABB = false;
	}

	bool intersect( const Ray& ray,  Hit& hit , float tmin, float time_) override {
		// vertices[0].print();
		// vertices[1].print();
		// vertices[2].print();

		//return false;
		Vector3f origin = ray.getOrigin();
        Vector3f direction = ray.getDirection();
		if(is_moving)
            origin -= time_ * this->velocity;
		float dir_len = direction.length();
        direction.normalize();
        float a = Vector3f::dot(origin, normal);
        float b = d - a;
        float c = Vector3f::dot(direction, normal);
        if(c == 0)
            return false;

		if(b < 1e-3 && b > -1e-3)
			return false;

        float t = b / c;
		t = t / dir_len;
        if(t > 0 && t > tmin && t < hit.getT()) {
			Vector3f v0 = vertices[2] - vertices[0];
    		Vector3f v1 = vertices[1] - vertices[0];
    		Vector3f v2 = ray.pointAtParameter(t) - vertices[0];
			float v0v2 = Vector3f::dot(v0, v2);
			float v1v2 = Vector3f::dot(v1, v2);
			if(inverDeno == 0)
				return false;
			float u = (v1v1 * v0v2 - v0v1 * v1v2) / inverDeno;
			if(u < 0 || u > 1)
				return false;
			float v = (v0v0 * v1v2 - v0v1 * v0v2) / inverDeno;
			if(v < 0 || v > 1)
				return false;
			if(u + v <= 1) {
				// std::cout << "yeah" << std::endl;
				Vector3f next_origin = origin + ray.getDirection() * t;
				float uu = 0, vv = 0;
				get_uv(next_origin, uu, vv);
				Vector3f new_normal = get_norm(next_origin);
            	hit.set(t, this->material, new_normal, material->get_color(uu, vv), next_origin, is_texture);
            	return true;
			}
        }

        return false;

		 
	}

	void set_vt(const Vector2f& _at, const Vector2f& _bt, const Vector2f& _ct) {
		at = _at;
		bt = _bt;
		ct = _ct;
		is_texture = true;
	}

	void set_vn(const Vector3f& _an, const Vector3f& _bn, const Vector3f& _cn) {
		an = _an;
		bn = _bn;
		cn = _cn;
		is_norm = true;
	}
	
	void get_uv(const Vector3f& p, float& u, float& v) {
        if (!is_texture)
			return;
        Vector3f va = (vertices[0] - p), vb = (vertices[1] - p), vc = (vertices[2] - p);
        float ra = Vector3f::cross(vb, vc).length();
    	float rb = Vector3f::cross(vc, va).length();
    	float rc = Vector3f::cross(va, vb).length();
        Vector2f uv = (ra * at + rb * bt + rc * ct) / (ra + rb + rc); //加权平均
        u = uv.x();
        v = uv.y();
    }

	Vector3f get_norm(const Vector3f& p) {
		if(!is_norm)
			return this->normal;
		Vector3f va = (vertices[0] - p), vb = (vertices[1] - p), vc = (vertices[2] - p);
		float ra = Vector3f::cross(vb, vc).length();
    	float rb = Vector3f::cross(vc, va).length();
    	float rc = Vector3f::cross(va, vb).length();
		return (ra * an + rb * bn + rc * cn).normalized();
	}


	Vector3f normal;
	Vector3f vertices[3];
	float minx, miny, minz, maxx, maxy, maxz;
protected:
	float d;
	float v0v0;
	float v0v1;
	float v1v1;
	float inverDeno;
	bool is_texture;
	bool is_norm;
	Vector2f at, bt, ct;
    Vector3f an, bn, cn;
};

#endif //TRIANGLE_H
