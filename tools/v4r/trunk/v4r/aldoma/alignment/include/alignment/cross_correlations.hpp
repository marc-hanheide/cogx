#include "fftw3.h"
#include "cv.h"
#include "highgui.h"

using namespace std;

inline double
crossCorrelation (IplImage * im1, IplImage * im2, IplImage * im1Extended, vector<int> * rotRange, int scaleRange,
                  int * maxIP, int * maxJP)
{
  IplImage * im1_32 = cvCreateImage (cvGetSize (im1), IPL_DEPTH_32F, 1);
  IplImage * im2_32 = cvCreateImage (cvGetSize (im1), IPL_DEPTH_32F, 1);
  IplImage * im1Extended_32 = cvCreateImage (cvGetSize (im1Extended), IPL_DEPTH_32F, 1);

  cvConvertScale (im1, im1_32, 1 / 255.);
  cvConvertScale (im2, im2_32, 1 / 255.);
  cvConvertScale (im1Extended, im1Extended_32, 1 / 255.);

  IplImage * im1ScaleExtended = cvCreateImage (cvSize (im1->width + scaleRange * 2 + 1, im1->height), IPL_DEPTH_32F, 1);
  cvZero (im1ScaleExtended);

  double meanMask = cvSum (im2_32).val[0] / (im2->width * im2->height);
  double meanCircShift = cvSum (im1_32).val[0] / (im1->width * im1->height);
  IplImage * maskMinusAvgMask = cvCreateImage (cvGetSize (im1), IPL_DEPTH_32F, 1);
  cvSubS (im2_32, cvScalar (meanMask), maskMinusAvgMask);
  cvSubS (im1Extended_32, cvScalar (meanCircShift), im1Extended_32);

  IplImage * res = cvCreateImage (cvGetSize (im1), IPL_DEPTH_32F, 1);

  double maxSum, maxI, maxJ, suma;
  maxSum = maxI = maxJ = -1;

  int a, r;
  vector<int>::iterator it;
  for (it = rotRange->begin (); it != rotRange->end (); it++)
  {
    a = (*it);
    cvSetImageROI (im1Extended_32, cvRect (0, a, im1Extended_32->width, im1->height));
    cvSetImageROI (im1ScaleExtended, cvRect (scaleRange, 0, im1->width, im1->height));
    cvCopy (im1Extended_32, im1ScaleExtended);

    for (r = -scaleRange; r <= scaleRange; r++)
    {
      cvSetImageROI (im1ScaleExtended, cvRect (scaleRange - r, 0, im1->width, im1->height));

      //Normal cross correlation
      cvMul (im1ScaleExtended, maskMinusAvgMask, res, 1.0);
      suma = cvSum (res).val[0];
      if (suma > maxSum)
      {
        maxI = a;
        maxSum = suma;
        maxJ = r;
      }
    }
  }

  *maxIP = maxI;
  *maxJP = maxJ;
  return maxSum;
}

inline double
crossCorrelationAbsDifference (IplImage * im1, IplImage * im2, IplImage * im1Extended, vector<int> * rotRange,
                               int scaleRange, int * maxIP, int * maxJP)
{
  IplImage * im1_32 = cvCreateImage (cvGetSize (im1), IPL_DEPTH_32F, 1);
  IplImage * im2_32 = cvCreateImage (cvGetSize (im1), IPL_DEPTH_32F, 1);
  IplImage * im1Extended_32 = cvCreateImage (cvGetSize (im1Extended), IPL_DEPTH_32F, 1);

  cvConvertScale (im1, im1_32, 1 / 255.);
  cvConvertScale (im2, im2_32, 1 / 255.);
  cvConvertScale (im1Extended, im1Extended_32, 1 / 255.);

  IplImage * im1ScaleExtended = cvCreateImage (cvSize (im1->width + scaleRange * 2 + 1, im1->height), IPL_DEPTH_32F, 1);
  cvZero (im1ScaleExtended);

  double meanMask = cvSum (im2_32).val[0] / (im2->width * im2->height);
  double meanCircShift = cvSum (im1_32).val[0] / (im1->width * im1->height);
  IplImage * maskMinusAvgMask = cvCreateImage (cvGetSize (im1), IPL_DEPTH_32F, 1);
  cvSubS (im2_32, cvScalar (meanMask), maskMinusAvgMask);
  cvSubS (im1Extended_32, cvScalar (meanCircShift), im1Extended_32);

  IplImage * res = cvCreateImage (cvGetSize (im1), IPL_DEPTH_32F, 1);

  double minSum, maxI, maxJ, suma;
  maxI = maxJ = -1;
  minSum = std::numeric_limits<double>::max ();

  int a, r;
  vector<int>::iterator it;
  for (it = rotRange->begin (); it != rotRange->end (); it++)
  {
    a = (*it);
    cvSetImageROI (im1Extended_32, cvRect (0, a, im1Extended_32->width, im1->height));
    cvSetImageROI (im1ScaleExtended, cvRect (scaleRange, 0, im1->width, im1->height));
    cvCopy (im1Extended_32, im1ScaleExtended);

    for (r = -scaleRange; r <= scaleRange; r++)
    {
      cvSetImageROI (im1ScaleExtended, cvRect (scaleRange - r, 0, im1->width, im1->height));

      //Normal cross correlation
      cvAbsDiff (im1ScaleExtended, maskMinusAvgMask, res);
      suma = cvSum (res).val[0];
      if (suma < minSum)
      {
        maxI = a;
        minSum = suma;
        maxJ = r;
      }
    }
  }

  *maxIP = maxI;
  *maxJP = maxJ;
  return minSum;
}

inline double
crossCorrelationFFT (const cv::Mat & proj_i, const cv::Mat & proj_j, int * maxIP, int * maxJP,
                                            int scale_limit)
{
  int width = proj_i.cols;
  int height = proj_i.rows;
  size_t fft_size = width * height;

  /* allocate FFTW input and output arrays */
  fftw_complex *img1 = (fftw_complex*)fftw_malloc (sizeof(fftw_complex) * width * height);
  fftw_complex *img2 = (fftw_complex*)fftw_malloc (sizeof(fftw_complex) * width * height);
  fftw_complex *res = (fftw_complex*)fftw_malloc (sizeof(fftw_complex) * width * height);

  /* setup FFTW plans */

  fftw_plan fft_img1, fft_img2;
  fftw_plan ifft_res;

#pragma omp critical
  {
    fft_img1 = fftw_plan_dft_1d (width * height, img1, img1, FFTW_FORWARD, FFTW_ESTIMATE);
  }
#pragma omp critical
  {
    fft_img2 = fftw_plan_dft_1d (width * height, img2, img2, FFTW_FORWARD, FFTW_ESTIMATE);
  }

#pragma omp critical
  {
    ifft_res = fftw_plan_dft_1d (width * height, res, res, FFTW_BACKWARD, FFTW_ESTIMATE);
  }

  /* load images' data to FFTW input */
  size_t i, k, j;
  for (i = 0, k = 0; i < (size_t)height; i++)
  {
    for (j = 0; j < (size_t)width; j++, k++)
    {
      img1[k][0] = (double)proj_j.at<float> (i, j);
      img1[k][1] = 0.0;

      img2[k][0] = (double)proj_i.at<float> (i, j);
      img2[k][1] = 0.0;
    }
  }

  /* obtain the FFT of img1 */
  fftw_execute (fft_img1);

  /*for(i = 0; i < 20; i++) {
    std::cout << "FFTw img1: Element " << i << " : " << img1[i][0] << " " << img1[i][1] << std::endl;
  }*/

  /* obtain the FFT of img2 */

  fftw_execute (fft_img2);

  /*for(i = 0; i < 20; i++) {
    std::cout << "FFTw img2: Element " << i << " : " << img2[i][0] << " " << img2[i][1] << std::endl;
  }*/

  /* obtain the cross power spectrum */
  double tmp;
  int used = 0;
  double threshold = 15 * 15; //low-pass filter
  for (i = 0; (i < fft_size); i++)
  {

    int row = i / width;
    int col = i % width;
    if ((row * row + col * col) <= threshold)
    {

      res[i][0] = (img2[i][0] * img1[i][0]) - (img2[i][1] * (-img1[i][1]));
      res[i][1] = (img2[i][0] * (-img1[i][1])) + (img2[i][1] * img1[i][0]);
      tmp = sqrt (res[i][0] * res[i][0] + res[i][1] * res[i][1]);
      res[i][0] /= tmp;
      res[i][1] /= tmp;
      used++;
    }
    else
    {
      res[i][0] = 0;
      res[i][1] = 0;
    }
  }

  /*for(i = 0; i < 100; i++) {
     std::cout << "FFTw cross: Element " << i << " : " << res[i][0] << " " << res[i][1] << std::endl;
   }*/

  /* obtain the phase correlation array */
  fftw_execute (ifft_res);

  /*for(i = 0; i < 20; i++) {
     std::cout << "Inverse fft : Element " << i << " : " << res[i][0] << " " << res[i][1] << std::endl;
   }*/

  /* normalize and copy to result image */

  //std::cout << "used:" << used << std::endl;

  cv::Mat result = cv::Mat_<float>::zeros (width, height);
  for (i = 0; i < (size_t) (width * height); i++)
  {
    if ((i % width) <= (size_t)scale_limit || (i % width) >= (size_t) (width - scale_limit))
    {
      result.at<float> (i / width, i % width) = res[i][0] / (double)used;
    }
  }

  cv::Point p_max, p_min;
  double minVal, maxVal;
  cv::minMaxLoc (result, &minVal, &maxVal, &p_min, &p_max);

  if (p_max.x > (scale_limit + 1))
  {
    p_max.x = p_max.x - width;
  }

  *maxIP = p_max.y;
  *maxJP = p_max.x;

  /* deallocate FFTW arrays and plans */
  fftw_destroy_plan (fft_img1);
  fftw_destroy_plan (fft_img2);
  fftw_destroy_plan (ifft_res);
  fftw_free (img1);
  fftw_free (img2);
  fftw_free (res);

  //std::cout << "maxVal:" << maxVal <<  "used:" << used << std::endl;

  return maxVal;
}
