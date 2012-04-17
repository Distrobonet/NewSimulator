//
// Filename:        "Color.h"
//
// Description:     This enumerated type defines a color.
//

// preprocessor directives
#ifndef COLOR_H
#define COLOR_H
#include <Simulator/GLIncludes.h>



// predefined colors
enum Color    // color array index values
{
    INVISIBLE = 0,
    BLACK,
    WHITE,
    RED,
    YELLOW,
    GREEN,
    CYAN,
    BLUE,
    MAGENTA
};  // Color
const int   N_COLORS           = 9;    // number of colors
const float COLOR[N_COLORS][3] =       // color array
{
    {-1.0f, -1.0f, -1.0f},    // INVISIBLE
    { 0.0f,  0.0f,  0.0f},    // BLACK
    { 1.0f,  1.0f,  1.0f},    // WHITE
    { 1.0f,  0.0f,  0.0f},    // RED
    { 1.0f,  1.0f,  0.0f},    // YELLOW
    { 0.0f,  1.0f,  0.0f},    // GREEN
    { 0.0f,  1.0f,  1.0f},    // CYAN
    { 0.0f,  0.0f,  1.0f},    // BLUE
    { 1.0f,  0.0f,  1.0f}     // MAGENTA
};  // COLOR[][]

#endif
