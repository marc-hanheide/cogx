/**
 * Speeded up local kernel.
 * \file localkernel2.cpp
 * \author Andrzej Pronobis
 */

#ifndef _LOCALKERNEL2_H
#define _LOCALKERNEL2_H

#include "svm.h"


#ifdef __cplusplus
extern "C" {
#endif


double localkernel2(const svm_node *x, const svm_node *y,
                    const svm_parameter& param);

#ifdef __cplusplus
}
#endif

#endif
