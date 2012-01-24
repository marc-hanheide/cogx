#ifndef _SOMEKERNELS_H
#define _SOMEKERNELS_H

#include <stdio.h>
#include <stdlib.h>
#include "svm.h"
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

// chi-square kernel (see e.g. O.Chapelle99)
double chiSquared(const svm_node *px, const svm_node *py, double gamma);

// generalized gaussian kernel (see e.g. Caputo)
double generalizedGauss(const svm_node *px, const svm_node *py, double b, double a, double gamma);

// intersection kernel (see e.g. Odone2003)
double intersection(const svm_node *px, const svm_node *py);

#ifdef __cplusplus
}
#endif

#endif
