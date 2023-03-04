#define STB_IMAGE_IMPLEMENTATION
#include <vecmath.h>
#include "stb_image.h"
#include <cstring>
#include "texture.h"

float Texture::get_gray(int index) { return (picture[index] / 255.0 - 0.5) * 2; }

int Texture::get_index(float u, float v) {
    u -= int(u);
    v -= int(v);
    u = u < 0 ? 1 + u : u;
    v = v < 0 ? 1 + v : v;
    int x = u * width;
    int y = v * height;
    x = x < 0 ? 0 : x;
    x = x > width - 1 ? width - 1 : x;
    y = y < 0 ? 0 : y;
    y = y > height - 1 ? height - 1 : y;
    int index = (y * width + x) * channel;
    return index;
};

Texture::Texture(const char* filename) {
    if (std::strlen(filename) == 0) {
        is_texture = false;
        return;
    }
    this->picture = stbi_load(filename, &width, &height, &channel, 0);
    is_texture = true;
}

Vector3f Texture::get_color(float u, float v) {
    
    u -= int(u);
    v -= int(v);
    u = u < 0 ? 1 + u : u;
    v = v < 0 ? 1 + v : v;
    u = u * width;
    v = height * (1 - v);
    int iu = (int)u, iv = (int)v;
    Vector3f ret_color = Vector3f::ZERO;
    float s = u - iu;
    float t = v - iv;
    Vector3f color1 = (1 - s) * get_pic_color(iu, iv + 1) + s * get_pic_color(iu + 1, iv + 1);
    Vector3f color2 = (1 - s) * get_pic_color(iu, iv) + s * get_pic_color(iu + 1, iv);
    ret_color += (1 - t) * color2;
    ret_color += t * color1;
    return ret_color;
}

Vector3f Texture::get_pic_color(int x, int y) {

    x = x < 0 ? 0 : x;
    x = x > width - 1 ? width - 1 : x;
    y = y < 0 ? 0 : y;
    y = y > height - 1 ? height - 1 : y;
    int index = (y * width + x) * channel;
    return Vector3f(picture[index + 0], picture[index + 1], picture[index + 2]) / 255.0;
}


float Texture::get_disturb(float u, float v, Vector2f &grad) {
    
    if(!is_texture) {
        return 0.0;
    }

    int idx_ = get_index(u, v);
    float disturb = get_gray(idx_);
    float du = 1.0 / width, dv = 1.0 / height;
    //计算梯度
    grad[0] = width * (get_gray(get_index(u + du, v)) - get_gray(get_index(u - du, v))) * 0.5;
    grad[1] = height * (get_gray(get_index(u, v + dv)) - get_gray(get_index(u, v - dv))) * 0.5;
    return disturb;
}