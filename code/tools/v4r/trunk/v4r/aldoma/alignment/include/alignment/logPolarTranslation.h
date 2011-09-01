#include <stdio.h>
#include <cutil_inline.h>

typedef float2 Complex;

extern "C"
void computeCrossCorrelations(Complex * img1, int size_img1, Complex * img2, int size_img2, int size_img, int SCALE_RANGE);
