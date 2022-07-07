#ifndef TEXTURE_H
#define TEXTURE_H

class Texture {
    public:
    unsigned char *picture;
    int width, height, channel;
    bool is_texture;
    Texture(const char *filename);
    Vector3f get_color(float u, float v);
    Vector3f get_pic_color(int x, int y);
    float get_gray(int index);
    float get_disturb(float u, float v, Vector2f &grad);
    int get_index(float u, float v);
};

#endif