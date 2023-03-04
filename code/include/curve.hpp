#ifndef CURVE_HPP
#define CURVE_HPP

#include "object3d.hpp"
#include <vecmath.h>
#include <vector>
#include <utility>

#include <algorithm>

// TODO (PA2): Implement Bernstein class to compute spline basis function.
//       You may refer to the python-script for implementation.

// The CurvePoint object stores information about a point on a curve
// after it has been tesselated: the vertex (V) and the tangent (T)
// It is the responsiblility of functions that create these objects to fill in all the data.
struct CurvePoint {
    Vector3f V; // Vertex
    Vector3f T; // Tangent  (unit)
    float t;//对应的t
};

Vector3f now_xy;

bool dist(const CurvePoint& a, const CurvePoint& b) {
    return (a.V - now_xy).squaredLength() < (b.V - now_xy).squaredLength();
}

class Curve : public Object3D {
protected:
    std::vector<Vector3f> controls;
public:
    std::vector<CurvePoint> mydata;

    explicit Curve(std::vector<Vector3f> points) : controls(std::move(points)) {}

    bool intersect(const Ray &r, Hit &h, float tmin, float t) override {
        return false;
    }

    std::vector<Vector3f> &getControls() {
        return controls;
    }

    virtual void discretize(int resolution, std::vector<CurvePoint>& data) = 0;

    virtual bool valid_t(float t) = 0;

    void get_t(float x_, float y_, float& t) { //根据x，y估计t
        now_xy = Vector3f(x_, y_, 0.0);

        
        //以和估计点之间的距离最小来估计t
        t = (*min_element(mydata.begin(), mydata.end(), dist)).t;
    };

    bool is_on_curve(const Vector3f& vv) {
        now_xy = Vector3f(-sqrt(vv.x() * vv.x() + vv.z() * vv.z()), vv.y(), 0.0);
        auto it  = min_element(mydata.begin(), mydata.end(), dist);
        if(((*it).V - now_xy).length() > 1e-2)
            return false;
        return true;
    }

    virtual void get_point(float, Vector3f&, Vector3f&) = 0; //根据t获得精确的point和切线
    // void drawGL() override {
    //     Object3D::drawGL();
    //     glPushAttrib(GL_ALL_ATTRIB_BITS);
    //     glDisable(GL_LIGHTING);
    //     glColor3f(1, 1, 0);
    //     glBegin(GL_LINE_STRIP);
    //     for (auto & control : controls) { glVertex3fv(control); }
    //     glEnd();
    //     glPointSize(4);
    //     glBegin(GL_POINTS);
    //     for (auto & control : controls) { glVertex3fv(control); }
    //     glEnd();
    //     std::vector<CurvePoint> sampledPoints;
    //     discretize(30, sampledPoints);
    //     glColor3f(1, 1, 1);
    //     glBegin(GL_LINE_STRIP);
    //     for (auto & cp : sampledPoints) { glVertex3fv(cp.V); }
    //     glEnd();
    //     glPopAttrib();
    // }
};

class BezierCurve : public Curve {
public:
    explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4 || points.size() % 3 != 1) {
            printf("Number of control points of BezierCurve must be 3n+1!\n");
            exit(0);
        }
        this->discretize(1000, mydata);
    }

    bool valid_t(float t) {
        return (t >= 0 && t <= 1);
    }

    void get_point(float t_, Vector3f& point, Vector3f& grad) override {

        std::vector<Vector3f> A = controls;
        int n = this->controls.size() - 1;
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n - i; j++) {
                if(i == n - 1)
                    grad = A[1] - A[0];
                A[j] = (1 - t_) * A[j] + t_ * A[j + 1];                        
            }
        point = A[0];
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();
        // TODO (PA2): fill in data vector
        int n = this->controls.size() - 1;
        for(int p = 0; p < resolution; p++) {
            CurvePoint ft;
            std::vector<Vector3f> A = controls;
            float u = float(p) / resolution;
            for(int i = 0; i < n; i++)
                for(int j = 0; j < n - i; j++) {
                    if(i == n - 1)
                        ft.T = A[1] - A[0];
                    A[j] = (1 - u) * A[j] + u * A[j + 1];                        
                }
            ft.V = A[0];
            ft.T.normalize();
            ft.t = u;
            //ft.T.print();
            data.push_back(ft);
        }
    }


protected:

};

class BsplineCurve : public Curve {
public:
    inline float t(float i) {return i / (n + k + 1); };

    BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4) {
            printf("Number of control points of BspineCurve must be more than 4!\n");
            exit(0);
        }
        this->discretize(100, mydata);
    }

    bool valid_t(float t) {
        n = controls.size() - 1;
        return (t >= this->t(k) && t <= this->t(n));
    }

    void get_point(float t_, Vector3f& point, Vector3f& grad) override {

        n = controls.size() - 1;
        CurvePoint ft;
        ft.V = Vector3f::ZERO;
        ft.T = Vector3f::ZERO;
        float t = t_;
        float B[n + k + 1][k + 1]; // [0, n + k][0, k]
        //std::cout << t;
        int index = k;
        for(int i = k; i <= n; i++) {
            if(t > this->t(i))
                index = i;
            else
                break;
        }
        for(int i = 0; i < n + k + 1; i++)
            (i == index) ? B[i][0] = 1 : B[i][0] = 0;

        for(int p = 1; p <= k; p++)
            for(int i = 0; i + p + 1 <= n + k + 1; i++)
                B[i][p] = ((t - this->t(i)) / (this->t(i + p) - this->t(i))) * B[i][p - 1] + ((this->t(i + p + 1) - t) / (this->t(i + p + 1) - this->t(i + 1))) * B[i + 1][p - 1];

        for(int i = 0; i <= n; i++)
            ft.V += B[i][k] * controls.at(i);
        for(int i = 0; i <= n; i++)
            ft.T += (k * (B[i][k - 1] / (this->t(i + k) - this->t(i)) - B[i + 1][k - 1] / (this->t(i + k + 1) - this->t(i + 1)))) * controls.at(i);
        
        point = ft.V;
        grad = ft.T;
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();
        // TODO (PA2): fill in data vector
        n = controls.size() - 1;
        for(int index = k; index <= n; index++) {
            for(int j = 0; j < resolution; j++) {
                CurvePoint ft;
                ft.V = Vector3f::ZERO;
                ft.T = Vector3f::ZERO;
                float t = this->t(index) + (float(j) / resolution) * (this->t(index + 1) - this->t(index));
                float B[n + k + 1][k + 1]; // [0, n + k][0, k]
                //std::cout << t;
                for(int i = 0; i < n + k + 1; i++)
                    (i == index) ? B[i][0] = 1 : B[i][0] = 0;

                for(int p = 1; p <= k; p++)
                    for(int i = 0; i + p + 1 <= n + k + 1; i++)
                        B[i][p] = ((t - this->t(i)) / (this->t(i + p) - this->t(i))) * B[i][p - 1] + ((this->t(i + p + 1) - t) / (this->t(i + p + 1) - this->t(i + 1))) * B[i + 1][p - 1];

                for(int i = 0; i <= n; i++)
                    ft.V += B[i][k] * controls.at(i);
                for(int i = 0; i <= n; i++)
                    ft.T += (k * (B[i][k - 1] / (this->t(i + k) - this->t(i)) - B[i + 1][k - 1] / (this->t(i + k + 1) - this->t(i + 1)))) * controls.at(i);
                ft.T.normalize();
                ft.t = t;
                data.push_back(ft);
            }
        }
    }



protected:
    int n = 0;
    int k = 3;
};

#endif // CURVE_HPP
