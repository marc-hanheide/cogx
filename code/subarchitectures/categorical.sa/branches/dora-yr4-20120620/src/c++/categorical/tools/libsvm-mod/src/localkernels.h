#ifndef _LOCALKERNELS_H
#define _LOCALKERNELS_H

#include "svm.h"

#ifdef __cplusplus
extern "C" {
#endif

// function that computes the local kernel
// (similar to Wallraven03)
double localkernel1(const svm_node *x, const svm_node *y,
        const svm_parameter& param);

// still non symmetric local kernel
// (was only required for local kernel of Wallraven,
// that didn't perform one-to-one matching and therefore
// was not symmetric)
double localkernel1_hat(const svm_node *x, const svm_node *y,
        const svm_parameter& param);

#ifdef __cplusplus
}
#endif

#endif
