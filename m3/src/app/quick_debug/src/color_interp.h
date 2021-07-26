//
// Created by gaoxiang on 2021/2/19.
//

#ifndef MAPPING_COLOR_INTERP_H
#define MAPPING_COLOR_INTERP_H

struct Color {
    unsigned char r = 0;
    unsigned char g = 0;
    unsigned char b = 0;
    unsigned char a = 0;

    Color() : r(0), g(0), b(0), a(255) {}

    Color(unsigned char rr, unsigned char gg, unsigned char bb) : r(rr), g(gg), b(bb), a(255) {}

    Color(unsigned char rr, unsigned char gg, unsigned char bb, unsigned char aa) : r(rr), g(gg), b(bb), a(aa) {}

    float redF() const { return ((float)r / 255.0f); }

    float greenF() const { return ((float)g / 255.0f); }

    float blueF() const { return ((float)b / 255.0f); }
};

#endif  // MAPPING_COLOR_INTERP_H
