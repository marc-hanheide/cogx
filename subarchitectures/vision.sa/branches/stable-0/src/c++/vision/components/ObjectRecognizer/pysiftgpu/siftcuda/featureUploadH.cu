// vim:set fileencoding=utf-8 sw=3 ts=3 et:vim
/*
 * @author:  Marko Mahnič
 * @created: jul 2009 
 */
   

#include <stdio.h>
#include <string.h>
#include <cutil.h>

#undef VERBOSE

#include "cudaImage.h"
#include "cudaSift.h"

void UploadSiftData(SiftData *data)
{
   if (data->h_data == NULL) return; // No data to upload to device
   if (data->maxPts < 1 || data->numPts < 1) return;
   if (data->d_data == NULL) {
      int sz = sizeof(SiftPoint)*data->maxPts;
      printf("allocating %d, %d, ", data->maxPts, sz);
      CUDA_SAFE_CALL(cudaMalloc((void **)&data->d_data, sz));
   }
   CUDA_SAFE_CALL(cudaMemcpy(data->d_data, data->h_data, 
      sizeof(SiftPoint)*data->numPts, cudaMemcpyHostToDevice));
}
