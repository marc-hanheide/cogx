// includes, project
#include <stdio.h>
#include <cutil_inline.h>
#include <cufft.h>

//define kernel...
//needs to multiply img_1 by img_2 in complex stuff
//perform the inverse of the cross spectrum... we can't do it in device code...

typedef float2 Complex;

__global__ void
cross_power_multiply (cufftComplex *img_1, cufftComplex * img_2, int size_img, int width)
{
  float threshold = 15 * 15;
  float tmp_r, tmp_c, tmp;
  tmp_r = tmp_c = 0;

  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int i2 = (i % size_img);

  //low pass filters
  int row = i2 / width;
  int col = i2 % width;
  if ((row * row + col * col) <= threshold)
  {
    tmp_r = (img_2[i2].x * img_1[i].x) - (-img_2[i2].y * (img_1[i].y));
    tmp_c = (img_2[i2].x * (img_1[i].y)) + (-img_2[i2].y * img_1[i].x);
    tmp = sqrt (tmp_r * tmp_r + tmp_c * tmp_c);
    tmp_r /= tmp;
    tmp_c /= tmp;
  }

  img_1[i].x = tmp_r;
  img_1[i].y = tmp_c;
}

__global__ void
normalize (cufftComplex *img_1, int scale_limit, int width, int n_elements_img2) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int idx_image = (i % (n_elements_img2));
  if ( (idx_image % width) <= (size_t)scale_limit || (idx_image % width) >= (size_t) (width - scale_limit) )
  {
    img_1[i].x /= 193.0;
  }
  else
  {
    img_1[i].x = 0;
  }
}


extern "C"
void computeCrossCorrelations(Complex * img1, int size_img1, Complex * img2, int size_img2, int size_img, int SCALE_RANGE)
{
  //allocate memory in device

  int n_imgs = size_img1 / size_img;
  int width = sqrt(size_img / sizeof(Complex));
  int n_elements_img1 = size_img1 / sizeof(Complex);
  int n_elements_img2 = width * width;

  ////printf("Inside of computeCrossCorrelations, number of images= %d %d %d %d\n", n_imgs, width, n_elements_img1, size_img1);

  Complex *d_img1, *d_img2;

  cudaMalloc((void**) &d_img1, sizeof(Complex) * n_elements_img1);
  //printf("Error alloc 1: %s\n", cudaGetErrorString(cudaGetLastError()));
  cudaMemcpy(d_img1, img1, sizeof(Complex) * n_elements_img1, cudaMemcpyHostToDevice);
  //printf("Error memcpy 1: %s\n", cudaGetErrorString(cudaGetLastError()));
  cudaMalloc((void**) &d_img2, sizeof(Complex) * n_elements_img2);
  //printf("Error alloc 2: %s\n", cudaGetErrorString(cudaGetLastError()));
  cudaMemcpy(d_img2, img2, sizeof(Complex) * n_elements_img2, cudaMemcpyHostToDevice);
  //printf("Error memcpy 2: %s\n", cudaGetErrorString(cudaGetLastError()));
  //Create FFT plans
  cufftHandle plan_1, plan_2;
  cufftPlan1d(&plan_1, n_elements_img2, CUFFT_C2C, n_imgs);
  //printf("Error cufftPlan1d 1: %s\n", cudaGetErrorString(cudaGetLastError()));
  cufftPlan1d(&plan_2, n_elements_img2, CUFFT_C2C, 1);
  //printf("Error cufftPlan1d 2: %s\n", cudaGetErrorString(cudaGetLastError()));

  //Transform in place!
  cufftExecC2C(plan_1, (cufftComplex *)d_img1, (cufftComplex *)d_img1, CUFFT_FORWARD);
  //printf("Error plan 1: %s\n", cudaGetErrorString(cudaGetLastError()));
  cufftExecC2C(plan_2, (cufftComplex *)d_img2, (cufftComplex *)d_img2, CUFFT_FORWARD);
  //printf("Error plan 2: %s\n", cudaGetErrorString(cudaGetLastError()));

  //call kernel that will compute the cross power multiplication between one chunk of data_1 and data_2
  int N_threads = 512;
  int N_blocks = n_elements_img1 / N_threads;
  ////printf("blocks: %d threads: %d", N_blocks, N_threads);

  cross_power_multiply <<< N_blocks , N_threads >>> (d_img1, d_img2, n_elements_img2, width);
  //printf("cross_power_multiply: %s\n", cudaGetErrorString(cudaGetLastError()));

  cufftExecC2C(plan_1, (cufftComplex *)d_img1, (cufftComplex *)d_img1, CUFFT_INVERSE);
  //printf("Error plan 1 inverse: %s\n", cudaGetErrorString(cudaGetLastError()));

  // Destroy the CUFFT plan.
  cufftDestroy(plan_1);
  //printf("Error cufftDestroy 1: %s\n", cudaGetErrorString(cudaGetLastError()));
  cufftDestroy(plan_2);
  //printf("Error cufftDestroy 2: %s\n", cudaGetErrorString(cudaGetLastError()));


  normalize <<< N_blocks , N_threads >>> (d_img1, SCALE_RANGE, width, n_elements_img2);
  //printf("Error normalize: %s\n", cudaGetErrorString(cudaGetLastError()));

  cudaMemcpy(img1, d_img1, n_elements_img1 * sizeof(Complex), cudaMemcpyDeviceToHost);
  //printf("Error cudaMemcpyDeviceToHost: %s\n", cudaGetErrorString(cudaGetLastError()));

  //free cuda memory
  cudaFree(d_img1);
  //printf("Error cudaFree 1: %s\n", cudaGetErrorString(cudaGetLastError()));
  cudaFree(d_img2);
  //printf("Error cudaFree 2: %s\n", cudaGetErrorString(cudaGetLastError()));

}
