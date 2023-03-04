#ifndef PT_H
#define PT_H

#include "camera.hpp"
#include "constant.h"
#include "group.hpp"
#include "hit.hpp"
#include "image.hpp"
#include "light.hpp"
#include "ray.hpp"
#include "scene_parser.hpp"
#include "random_producer.hpp"
using namespace std;

Vector3f getPtColor(Ray ray, Group* group, int depth, bool de = false) {
    
    Hit hit;
    bool isIntersect = group->intersect(ray, hit, 0, RND2);

    if(isIntersect) {

        Material* material = hit.getMaterial();
        bool is_texture;
        Vector3f hit_color = hit.get_color(is_texture);
        if(!is_texture)
            hit_color = material->color;

        if(material->emission != Vector3f::ZERO)
            return material->emission;

        float p = hit_color.x() > hit_color.y() && hit_color.x() > hit_color.z() ? hit_color.x() : hit_color.y() > hit_color.z() ? hit_color.y() : hit_color.z();
        p = p < 0.75 ? p : 0.75;
        if(depth > pt_max_depth) {
            if(RND2 < p) {
                hit_color = hit_color / p;
            } else {
                return material->emission;
            }
        }
        Vector3f hit_emission = material->emission;
        Vector3f hit_normal = hit.getNormal().normalized();
        Vector3f next_origin = ray.getOrigin() + hit.getT() * ray.getDirection();
        Vector3f ray_direction = ray.getDirection().normalized();

        float type_decision = RND2;
        float b = Vector3f::dot(ray_direction, hit_normal);
        if(type_decision < material->type.x()) {//漫反射
            Vector3f z_ = Vector3f::cross(ray_direction, hit_normal);
            Vector3f x_ = Vector3f::cross(z_, hit_normal);
            z_.normalize();
            x_.normalize();
            Vector3f next_direction;
            if(b < 0)
                next_direction = RND1 * z_ + RND1 * x_ + RND2 * hit_normal;
            else
                next_direction = RND1 * z_ + RND1 * x_ - RND2 * hit_normal;
            next_direction.normalize();

            Vector3f next_color = getPtColor(Ray(next_origin, next_direction), group, depth + 1, de);

            if(de) {
                
                cout << "depth:" << depth << endl << "ru_direct:";
                ray_direction.print();
                cout << "type:";
                material->type.print();
                cout << "matirial color:";
                material->color.print();
                cout << "normal:";
                hit_normal.print();
                cout << "chu_direct:" ;
                next_direction.print();
                cout << "origin:" ;
                next_origin.print();
                cout << "t: " << hit.getT() << endl;
                next_color.print();
            }

            return hit_emission + hit_color * next_color;

        } else if(type_decision < material->type.x() + material->type.y()) {//镜面反射
            Vector3f next_direction = ray_direction - hit_normal * (b * 2);
            next_direction.normalize();
            Vector3f next_color = getPtColor(Ray(next_origin, next_direction), group, depth + 1, de);
            if(de) {
                cout << "depth:" << depth << endl << "ru_direct:";
                ray_direction.print();
                cout << "normal:";
                hit_normal.print();
                cout << "chu_direct:" ;
                next_direction.print();
                next_origin.print();
                next_color.print();
            }
            return hit_emission + hit_color * next_color;

        } else {//折射
            float n = material->refractive_index;
            float R0 = ((1.0 -n) * (1.0 - n)) / ((1.0 + n) * (1.0 + n));
            if(b > 0) {//??存疑
                hit_normal.negate();
            } else {
                n = 1.0 / n;
            }
            
            float cos1 = -Vector3f::dot(hit_normal, ray_direction);
            float cos2 = 1.0 - n * n * (1.0 - cos1 * cos1); //cos(theta2)的平方
            Vector3f reflect = (ray_direction + hit_normal * (cos1 * 2));
            if(cos2 < 0) {
                return hit_emission + hit_color * getPtColor(Ray(next_origin, reflect), group, depth + 1, de);
            }
            //Schlick估计菲涅尔项
            float Rprob = R0 + (1.0 - R0) * pow(1.0 - cos1, 5.0);

            Vector3f refrac =  ((ray_direction * n) + (hit_normal * (n * cos1 - sqrt(cos2)))).normalized();
            

            float P = 0.25 + 0.5 * Rprob;
            if (depth <= 1)
                return hit_emission + hit_color * (Rprob * getPtColor(Ray(next_origin, reflect), group, depth + 1, de) + (1 - Rprob) * getPtColor(Ray(next_origin, refrac), group, depth + 1, de));

            if (RND2 < P) {
                return hit_emission + hit_color * ((Rprob / P) * getPtColor(Ray(next_origin, reflect), group, depth + 1, de));
            } else {
                return hit_emission + hit_color * (((1 - Rprob) / (1 - P)) * getPtColor(Ray(next_origin, refrac), group, depth + 1, de));
            }
        }
    }
    else {

        return Vector3f::ZERO; //递归终止，返回背景色 黑色
    }
}

class Path_Tracer {
    const SceneParser& scene;
    const char* output_file;
    bool debug;
    public:
    Path_Tracer(const SceneParser& scene, const char* output, bool debuger = false):scene(scene), output_file(output), debug(debuger){

    };

    void trace() {
        Camera *camera = scene.getCamera();
        int w = camera->getWidth();
        int h = camera->getHeight();
        Image myImage(w, h);
        Vector3f background = scene.getBackgroundColor();
        Group* group = scene.getGroup();
        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y) {
            
                Vector3f finalColor = Vector3f::ZERO;
                for (int s = 0; s < pt_samples; s++) {
                    
                    Vector2f ori = Vector2f(x + RND1 / 2, y + RND1 / 2);
                    Ray camRay = camera->generateRay(ori);

                    // if(s == 2) {
                    //     cout << "<<<<<<<<<<<<<" << endl;
                    finalColor += getPtColor(camRay, group, 0, debug);

                }
                finalColor = finalColor / float(pt_samples);
                finalColor = Vector3f(toFloat(finalColor.x()), toFloat(finalColor.y()), toFloat(finalColor.z()));
                // cout << "finalcolor: ";
                // finalColor.print();
                myImage.SetPixel(x, y, finalColor);
               


            }
            if (x % 5 == 0)
                cout << "Finished:" << (float(x) * 100) / w << "%" << endl; 
        }
         myImage.SaveImage(output_file);

    }
};

#endif