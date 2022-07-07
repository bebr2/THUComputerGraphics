#ifndef RANDOMPRODUCER_H
#define RANDOMPRODUCER_H
#include <cmath>
#include <cstdlib>
#include <random>
#include <vecmath.h>

static std::mt19937 mersenneTwister;
static std::uniform_real_distribution<float> uniform;
#define RND1 (2.0 * uniform(mersenneTwister) - 1.0) //[-1, 1]均匀分布
#define RND2 (uniform(mersenneTwister))//[0,1]均匀分布
#define PI (3.1415926536)

inline float ReLU(float x) { return x < 0 ? 0 : x > 1 ? 1 : x; }

inline float toFloat(float x) { return float(int(pow(ReLU(x), 1 / 2.2) * 255 + .5)) / 255.0; }
#endif

