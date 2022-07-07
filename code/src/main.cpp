#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <sys/time.h>
#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "pt.hpp"
#include "ppm.hpp"
#include <string>

using namespace std;

int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 5) {
        cout << "Usage: ./bin/PA1 <method> <input scene file> <output bmp file><debug>" << endl;
        return 1;
    }
    string method = argv[1];
    string inputFile = argv[2];
    string outputFile = argv[3];  // only bmp is allowed.
    string is_debug = argv[4];
    bool debug = false;
    if(is_debug == "debug")
        debug = true;
    // TODO: Main RayCasting Logic
    // First, parse the scene using SceneParser.
    // Then loop over each pixel in the image, shooting a ray
    // through that pixel and finding its intersection with
    // the scene.  Write the color at the intersection to that
    // pixel in your output image.
    int length = outputFile.length();
    char ext[5] = "pmb.";
    for (int i = 1; i <= 4; ++i)
        if(outputFile[length - i] != ext[i - 1]) {
            printf("wrong file name extension\n");
            exit(0);
        }

    struct timeval start, end;
    gettimeofday(&start, NULL);

    SceneParser myParser(inputFile.c_str());
    // Group* the_group = myParser.getGroup();

    // the_group->setup_bvh_tree();
    if(method == "ppm") {
        printf("Method: progressive photon mapping\n");
        PPM my_ppm(myParser, argv[3]);
        my_ppm.trace();
    } else if (method == "pt") {
        printf("Method: path tracing\n");
        Path_Tracer my_pathtracer(myParser, argv[3], debug);
        my_pathtracer.trace();
   } else {
        printf("Method: ray castray\n");
        Camera *camera = myParser.getCamera();
        Image myImage(camera->getWidth(), camera->getHeight());

        Vector3f background = myParser.getBackgroundColor();
    //   for (auto it =  myParser.getGroup()->object_list.begin(); it !=  myParser.getGroup()->object_list.end(); it++) {
    //          Ray camRay = camera->generateRay(Vector2f(0, 0));
    //           Hit h;
    //         (*it)->intersect(camRay, h, 0);
    //     }
    //     exit(0);
    
        background.print();
        for (int x = 0; x < camera->getWidth(); ++x) {
            for (int y = 0; y < camera->getHeight(); ++y) {
                
                Ray camRay = camera->generateRay(Vector2f(x, y));
                Group* baseGroup = myParser.getGroup();
                Hit hit;
                bool isIntersect = baseGroup->intersect(camRay, hit, 0, 0);

                // std::cout << x << " " << y << std::endl;
                if(isIntersect) {
                    // myImage.SetPixel(x, y, Vector3f(1,1,1));
                    // continue;
                    Vector3f finalColor = Vector3f::ZERO;
                    for (int li = 0; li < myParser.getNumLights(); li++) {
                        Light* light = myParser.getLight(li);
                        Vector3f L, lightColor;
                        light->getIllumination(camRay.pointAtParameter(hit.getT()), L, lightColor);
                        //cout << x << ' ' << y << endl;
                        //cout << hit.getT() << endl;
                        finalColor += hit.getMaterial()->Shade(camRay, hit, L, lightColor);
                    }
                    
                    myImage.SetPixel(x, y, finalColor);
                }
                else {
                    myImage.SetPixel(x, y, background);
                    //cout << "back" << ' ' << x << ' ' << y << endl;
                }
            }
        }

        myImage.SaveImage(outputFile.c_str());
   }
    gettimeofday(&end, NULL);
    time_t sec = end.tv_sec - start.tv_sec;
    time_t minute = sec / 60;
    cout << "Total time: " << sec << "seconds, or " << minute << "mins" << endl;
    cout << "Hello! Computer Graphics!" << endl;
    return 0;
}

