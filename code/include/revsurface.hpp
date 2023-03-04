#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include "object3d.hpp"
#include "curve.hpp"
#include <tuple>
#include "constant.h"

class RevSurface : public Object3D {

    Curve *pCurve;

    float x_max, y_max, y_min;

public:
    RevSurface(Curve *pCurve, Material* material) : pCurve(pCurve), Object3D(material, Vector3f::ZERO) { //默认旋转体表面套一个transform就好了
        // Check flat.
        float xmax = 0, ymax = -1e38, ymin = 1e38;
        for (const auto &cp : pCurve->getControls()) {
            if (cp.z() != 0.0) {
                printf("Profile of revSurface must be flat on xy plane.\n");
                exit(0);
            }
            if (abs(cp.x()) > xmax) xmax = abs(cp.x());
            if (cp.y() > ymax) ymax = cp.y();
            if (cp.y() < ymin) ymin = cp.y();
        }
        x_max = xmax;
        y_max = ymax;
        y_min = ymin;
    }

    ~RevSurface() override {
        delete pCurve;
    }

    float F_(const Vector3f& dir, const Vector3f& origin, float x_t, float y_t) {
        float x1 = dir.y() * dir.y() * x_t * x_t;
        float y1 = y_t - origin.y();
        float x2 = pow((y1 * dir.x() + dir.y() * origin.x()), 2);
        float x3 = pow((y1 * dir.z() + dir.y() * origin.z()), 2);
        return x2 + x3 - x1;
    }

    float F_grad(const Vector3f& dir, const Vector3f& origin, float x_t, float y_t, float x_grad_t, float y_grad_t) {
        float x1 = 2 * dir.y() * dir.y() * x_t * x_grad_t;
        float y1 = y_t - origin.y();
        float x2 = 2 * dir.x() * y_grad_t * (y1 * dir.x() + dir.y() * origin.x());
        float x3 = 2 * dir.z() * y_grad_t * (y1 * dir.z() + dir.y() * origin.z());
        return x2 + x3 - x1;
    }

    bool inter_AABB(const Ray &r, Hit &h, float & x_, float & y_) {
        Vector3f direction = r.direction;
        Vector3f origin = r.origin;
        if(abs(r.origin.x()) < x_max && abs(r.origin.z()) < x_max && y_min < r.origin.y() && r.origin.y() < y_max) {
            x_ = r.origin.x() * r.origin.x() + r.origin.z() * r.origin.z();
            x_ = - sqrt(x_);
            y_ = r.origin.y();
            return true;
        }

        float t_xmin = -1e38;
        float t_xmax = 1e38;
        float t_ymin = -1e38;
        float t_ymax = 1e38;
        float t_zmin = -1e38;
        float t_zmax = 1e38;

        if(direction.x() > 1e-4) {
            t_xmin = (-x_max - origin.x()) / direction.x();
            t_xmax = (x_max - origin.x()) / direction.x();
        } else if(direction.x() < -1e-4) {
            t_xmax = (-x_max - origin.x()) / direction.x();
            t_xmin = (x_max - origin.x()) / direction.x();
        } else if(origin.x() > x_max || origin.x() < -x_max)
            return false;

        if(t_xmax <= 0)
            return false;

        if(direction.y() > 1e-4) {
            t_ymin = (y_min - origin.y()) / direction.y();
            t_ymax = (y_max - origin.y()) / direction.y();
        } else if(direction.y() < -1e-4) {
            t_ymax = (y_min - origin.y()) / direction.y();
            t_ymin = (y_max - origin.y()) / direction.y();
        } else if(origin.y() > y_max || origin.y() < y_min)
            return false;

        if(t_ymax <= 0)
            return false;

        if(direction.z() > 1e-4) {
            t_zmin = (-x_max - origin.z()) / direction.z();
            t_zmax = (x_max - origin.z()) / direction.z();
        } else if(direction.z() < -1e-4) {
            t_zmax = (-x_max - origin.z()) / direction.z();
            t_zmin = (x_max - origin.z()) / direction.z();
        } else if(origin.z() > x_max || origin.z() < -x_max)
            return false;

        if(t_zmax <= 0)
            return false;
        
        float t0, t1;
        t0 = max(t_zmin, max(t_xmin, t_ymin));
        t1 = min(t_zmax, min(t_xmax, t_ymax));
        
        if(t0 < t1) {
            float the_t = (t0 > 0) ? t0 : t1;
            Vector3f next_ori = origin + the_t * direction;
            x_ =  next_ori.x() * next_ori.x() + next_ori.z() * next_ori.z();
            x_ =  - sqrt(x_);
            y_ = next_ori.y();
            return true;
        } else {
            return false;
        }
            
    }

    //对于curve上的点(x, y)，其旋转后为(xcos, y, xsin)
    bool intersect(const Ray &r, Hit &h, float tmin, float time_) override {
        // if(abs(r.direction.y()) < 1e-3) {
        //     // std::cout << "rdirection.ywei0" << std::endl;
        // }
        // (PA2 optional TODO): implement this for the ray-tracing routine using G-N iteration.
        float AABB_x, AABB_y;
        bool is_interAABB = inter_AABB(r, h, AABB_x, AABB_y);
        if(!is_interAABB) {
            // printf("Not hit aabb\n");
            return false;
        }
        //与包围盒相交，得到一个估计的平面点(x, y)，然后估计一下t。

        float estimate_t;
        float ini_ft;
        pCurve->get_t(AABB_x, AABB_y, estimate_t);
        //牛顿迭代
        int dep = 0;
        while(dep < newton_depth) {
            dep++;
            if(!pCurve->valid_t(estimate_t))
                return false;
            
            Vector3f now_point;
            Vector3f now_grad;
            pCurve->get_point(estimate_t, now_point, now_grad);
            float ft = F_(r.direction, r.origin, now_point.x(), now_point.y());
            float ft_grad = F_grad(r.direction, r.origin, now_point.x(), now_point.y(), now_grad.x(), now_grad.y());
            // printf("dep: %d", dep);
            // printf("ft: %f\n", ft);
            // printf("ft_grad: %f\n", ft_grad);
            // printf("estimate: %f\n", estimate_t);

            if(abs(ft) < 1e-5) {
                float tr;
                if(abs(r.direction.y()) > 1e-3) {
                    tr = (now_point.y() - r.origin.y()) / (r.direction.y());
                    // if(tr < 0) {
                
                    // }
                }
                else {
                    
                    float a = r.direction.z() * r.direction.z() + r.direction.x() + r.direction.x();
                    float b = 2 * (r.direction.z() * r.origin.z() + r.direction.x() * r.origin.x());
                    float c = pow(r.origin.x(), 2) + pow(r.origin.z(), 2) - pow(now_point.x(), 2);
                    tr = (sqrt(pow(b, 2) - 4 * a * c) - b) / (2 * a);
                    // std::cout << tr << " " << a  << " "<< b << " " << c << std::endl;
                    // now_point.print();
                    // r.origin.print();
                    // r.direction.print();
                }
                Vector3f next_origin = tr * r.direction + r.origin;
                
                if(tr > tmin && pCurve->is_on_curve(next_origin)) {
                    //  std::cout << AABB_x << " " << AABB_y << std::endl;
                    //     now_point.print();
                    //     r.origin.print();
                    //     r.direction.print();
                    
                    
                    Vector2f plane_normal(now_grad.y(), -now_grad.x());
                    plane_normal.normalize();

                    if(abs(now_point.x()) < 1e-4) {
                        h.set(tr, material, now_point.y() > 0 ? Vector3f(0, 1, 0) : Vector3f(0, -1, 0));
                        return true;
                    }
                    float costheta = (r.direction.x() * tr + r.origin.x()) / now_point.x();
                    float sintheta = (r.direction.z() * tr + r.origin.z()) / now_point.x();
                    Vector3f new_normal = Vector3f(costheta * plane_normal.x(), plane_normal.y(), sintheta * plane_normal.x());
                    // if(Vector3f::dot(new_normal, next_origin) < 0)
                    //     new_normal.negate();
                    // std::cout << "nextori";
                    // next_origin.print();
                    // std::cout << "normal";
                    // new_normal.print();
                    h.set(tr, material, new_normal);
                    return true;
                } 
            }
            float step = ft / ft_grad;
            if(step > 0.05)
                step = 0.05;
            else if(step < -0.05)
                step = -0.05;
            estimate_t -= step;
        }
        return false;
    }

    // void drawGL() override {
    //     Object3D::drawGL();

    //     // Definition for drawable surface.
    //     typedef std::tuple<unsigned, unsigned, unsigned> Tup3u;
    //     // Surface is just a struct that contains vertices, normals, and
    //     // faces.  VV[i] is the position of vertex i, and VN[i] is the normal
    //     // of vertex i.  A face is a triple i,j,k corresponding to a triangle
    //     // with (vertex i, normal i), (vertex j, normal j), ...
    //     // Currently this struct is computed every time when canvas refreshes.
    //     // You can store this as member function to accelerate rendering.

    //     struct Surface {
    //         std::vector<Vector3f> VV;
    //         std::vector<Vector3f> VN;
    //         std::vector<Tup3u> VF;
    //     } surface;

    //     std::vector<CurvePoint> curvePoints;
    //     pCurve->discretize(30, curvePoints);
    //     const int steps = 40;
    //     for (unsigned int ci = 0; ci < curvePoints.size(); ++ci) {
    //         const CurvePoint &cp = curvePoints[ci];
    //         for (unsigned int i = 0; i < steps; ++i) {
    //             float t = (float) i / steps;
    //             Quat4f rot;
    //             rot.setAxisAngle(t * 2 * 3.14159, Vector3f::UP);
    //             Vector3f pnew = Matrix3f::rotation(rot) * cp.V;
    //             Vector3f pNormal = Vector3f::cross(cp.T, -Vector3f::FORWARD);
    //             Vector3f nnew = Matrix3f::rotation(rot) * pNormal;
    //             surface.VV.push_back(pnew);
    //             surface.VN.push_back(nnew);
    //             int i1 = (i + 1 == steps) ? 0 : i + 1;
    //             if (ci != curvePoints.size() - 1) {
    //                 surface.VF.emplace_back((ci + 1) * steps + i, ci * steps + i1, ci * steps + i);
    //                 surface.VF.emplace_back((ci + 1) * steps + i, (ci + 1) * steps + i1, ci * steps + i1);
    //             }
    //         }
    //     }

    //     glBegin(GL_TRIANGLES);
    //     for (unsigned i = 0; i < surface.VF.size(); i++) {
    //         glNormal3fv(surface.VN[std::get<0>(surface.VF[i])]);
    //         glVertex3fv(surface.VV[std::get<0>(surface.VF[i])]);
    //         glNormal3fv(surface.VN[std::get<1>(surface.VF[i])]);
    //         glVertex3fv(surface.VV[std::get<1>(surface.VF[i])]);
    //         glNormal3fv(surface.VN[std::get<2>(surface.VF[i])]);
    //         glVertex3fv(surface.VV[std::get<2>(surface.VF[i])]);
    //     }
    //     glEnd();
    // }
};

#endif //REVSURFACE_HPP
