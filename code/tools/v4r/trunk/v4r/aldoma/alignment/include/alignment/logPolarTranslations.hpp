inline void
getWeightMatrix (cv::Mat & weight_matrix, int center_i, int center_j)
{
  for (size_t i = 0; i < (size_t)weight_matrix.cols; i++)
  {
    for (size_t j = 0; j < (size_t)weight_matrix.rows; j++)
    {
      weight_matrix.at<float> (i, j) = (i - center_i) * (i - center_i) + (j - center_j) * (j - center_j);
      //weight_matrix.at<float> (i, j) = 1;
    }
  }

  double minVal_w, maxVal_w;
  cv::minMaxLoc (weight_matrix, &minVal_w, &maxVal_w);
  weight_matrix /= maxVal_w;

  /*cv::namedWindow ("weight", CV_WINDOW_AUTOSIZE);
   cv::imshow ("weight", weight_matrix);
   cv::waitKey (0);*/
}

//im1 (i) is the partial view translated...
//im2 (j) is the mesh rotated and scaled...

void
StablePlanesAlignment::logPolarTranslationDiff (const cv::Mat & im1, const cv::Mat & im2, double * rotation,
                                                double * scale, int * maxII, int * maxJJ)
{
  //std::cout << "Log polar translation for:" << im1.rows << std::endl;

  int size = im1.cols;
  double MScale = getScaleLogPolar (size);

  cv::Mat im1_copy = im1.clone ();
  cv::Mat im2_copy = im2.clone ();

  //we dont want to translate the mesh, just the partial view (im1)
  //use a weighting function...

  cv::Mat weight_matrix = im1_copy.clone ();
  getWeightMatrix (weight_matrix, size / 2, size / 2);
  cv::multiply (im2_copy, weight_matrix, im2_copy);

  IplImage ipl_im2;
  ipl_im2 = im2_copy;

  IplImage * dst2 = cvCreateImage (cvGetSize (&ipl_im2), ipl_im2.depth, 1);
  cvLogPolar (&ipl_im2, dst2, cvPoint2D32f (size / 2, size / 2), MScale);
  cv::Mat proj_j_lp (dst2);

  int SCALE_RANGE = getScaleRange (size);
  int XRANGE, YRANGE;
  XRANGE = YRANGE = getXYRange (size);

  int ii, jj;
  double minSum = std::numeric_limits<double>::max ();
  int maxI, maxJ, maxIIL, maxJJL;
  maxI = maxJ = -1;
  maxIIL = maxJJL = -100;

  for (ii = -XRANGE; ii <= XRANGE; ii++)
  {
    for (jj = -YRANGE; jj <= YRANGE; jj++)
    {
      cv::Mat rotMatrix = cv::getRotationMatrix2D (cv::Point2f (size / 2, size / 2), 0, 1);
      rotMatrix.at<double> (0, 2) += ii;
      rotMatrix.at<double> (1, 2) += jj;

      cv::Mat mat1Translated;
      cv::warpAffine (im1_copy, mat1Translated, rotMatrix, cvGetSize (&ipl_im2));
      cv::multiply (mat1Translated, weight_matrix, mat1Translated);

      IplImage ipl_im1;
      ipl_im1 = mat1Translated;

      IplImage * dst1 = cvCreateImage (cvGetSize (&ipl_im1), ipl_im1.depth, 1);
      cvLogPolar (&ipl_im1, dst1, cvPoint2D32f (size / 2, size / 2), MScale);

      cv::Mat proj_i_lp (dst1); //projection translated

      //compute how much the mesh needs to be rotated, scaled to match partial view
      int newMaxI = 0, newMaxJ = 0;
      double suma;

      vector<int> range (dst1->height, 0);
      int i;
      for (i = 0; i < dst1->height; i++)
      {
        range[i] = i;
      }

      IplImage * dst1Expanded = cvCreateImage (cvSize (dst1->height, dst1->height * 2), IPL_DEPTH_32F, 1);
      cvZero (dst1Expanded);
      cvSetImageROI (dst1Expanded, cvRect (0, 0, dst1->height, dst1->height));
      cvCopy (dst1, dst1Expanded);
      cvResetImageROI (dst1Expanded);
      cvSetImageROI (dst1Expanded, cvRect (0, dst1->height, dst1->height, 2 * dst1->height));
      cvCopy (dst1, dst1Expanded);
      cvResetImageROI (dst1Expanded);

      suma = crossCorrelationAbsDifference (dst1, dst2, dst1Expanded, &range, SCALE_RANGE, &maxI, &maxJ);

      if (suma < minSum && suma >= 0 && suma <= 1.0)
      {
        minSum = suma;
        maxI = newMaxI;
        maxJ = newMaxJ;
        maxIIL = ii;
        maxJJL = jj;

        if (true || this->VISUALIZE_CROSS_)
        {
          double minVal, maxVal;

          cv::minMaxLoc (proj_i_lp, &minVal, &maxVal);
          cv::Mat_<float> B_i = proj_i_lp / maxVal;

          cv::minMaxLoc (proj_j_lp, &minVal, &maxVal);
          cv::Mat_<float> B_j = proj_j_lp / maxVal;

          //std::cout << "maxVal after logpolar:" << maxVal << std::endl;

          cv::namedWindow ("prova_i", CV_WINDOW_AUTOSIZE);
          cv::namedWindow ("prova_j", CV_WINDOW_AUTOSIZE);

          cv::imshow ("prova_i", B_i);
          cv::imshow ("prova_j", B_j);
          cv::waitKey (0);
        }

        //std::cout << "ii,jj" << ii << "," << jj << " suma: " << suma << " maxSum:" << maxSum << " maxI:" << maxI
        //    << " maxJ:" << maxJ << std::endl;
        //std::cout << "maxI:" << maxI << " suma:" << minSum << std::endl;
      }

      //release created images
      cvReleaseImage (&dst1);
    }
  }

  cvReleaseImage (&dst2);

  *rotation = maxI / (double)(size) * 360;
  *scale = exp (maxJ / MScale);
  *maxII = maxIIL;
  *maxJJ = maxJJL;

  //std::cout << "maxI:" << maxI << " suma:" << maxSum << " rotation:" << *rotation << "size:" << size << std::endl;
  //translation needs to be applied to partial_view
  //rotation,scale needs to mesh
  //cout << "Values log polar translation:" << *rotation << " " << *scale << " " << *maxII << " " << *maxJJ << endl;
}

//im1 (i) is the partial view translated...
//im2 (j) is the mesh rotated and scaled...

void
StablePlanesAlignment::logPolarTranslation (const cv::Mat & im1, const cv::Mat & im2, double * rotation,
                                            double * scale, int * maxII, int * maxJJ)
{
  //std::cout << "Log polar translation for:" << im1.rows << std::endl;

  int size = im1.cols;
  double MScale = getScaleLogPolar (size);

  cv::Mat im1_copy = im1.clone ();
  cv::Mat im2_copy = im2.clone ();

  //we dont want to translate the mesh, just the partial view (im1)
  //use a weighting function...

  cv::Mat weight_matrix = im1_copy.clone ();
  getWeightMatrix (weight_matrix, size / 2, size / 2);
  cv::multiply (im2_copy, weight_matrix, im2_copy);

  IplImage ipl_im2;
  ipl_im2 = im2_copy;

  IplImage * dst2 = cvCreateImage (cvGetSize (&ipl_im2), ipl_im2.depth, 1);
  cvLogPolar (&ipl_im2, dst2, cvPoint2D32f (size / 2, size / 2), MScale);
  cv::Mat proj_j_lp (dst2);

  cv::Mat to_display = proj_j_lp.clone();
  double minVal_w, maxVal_w;
  cv::minMaxLoc (to_display, &minVal_w, &maxVal_w);
  to_display /= maxVal_w;

  /*cv::namedWindow ("weight", CV_WINDOW_AUTOSIZE);
  cv::imshow ("weight", to_display);
  cv::waitKey (0);*/

  int SCALE_RANGE = getScaleRange (size);
  int XRANGE, YRANGE;
  XRANGE = YRANGE = getXYRange (size);

  int ii, jj;
  double maxSum = -1;
  int maxI, maxJ, maxIIL, maxJJL;
  maxI = maxJ = -1;
  maxIIL = maxJJL = -100;

  for (ii = -XRANGE; ii <= XRANGE; ii++)
  {
    for (jj = -YRANGE; jj <= YRANGE; jj++)
    {
      cv::Mat rotMatrix = cv::getRotationMatrix2D (cv::Point2f (size / 2, size / 2), 0, 1);
      rotMatrix.at<double> (0, 2) += ii;
      rotMatrix.at<double> (1, 2) += jj;

      cv::Mat mat1Translated;
      cv::warpAffine (im1_copy, mat1Translated, rotMatrix, cvGetSize (&ipl_im2));

      cv::Mat to_display = mat1Translated.clone();
      double minVal_w, maxVal_w;
      cv::minMaxLoc (to_display, &minVal_w, &maxVal_w);
      to_display /= maxVal_w;

      /*cv::namedWindow ("weight_2", CV_WINDOW_AUTOSIZE);
      cv::imshow ("weight_2", to_display);
      cv::waitKey (0);*/

      std::stringstream image_name;
      image_name << "/home/aitor/Desktop/images/image_" << ii << "_ " << jj << ".jpg";
      to_display.convertTo(to_display, CV_8UC1,255,0);
      cv::imwrite(image_name.str(), to_display);

      //getWeightMatrix (weight_matrix, size / 2 + jj, size / 2 + ii);
      getWeightMatrix (weight_matrix, size / 2, size / 2);
      cv::multiply (mat1Translated, weight_matrix, mat1Translated);

      /*cv::namedWindow ("im1 weighted", CV_WINDOW_AUTOSIZE);
      cv::imshow ("im1 weighted", mat1Translated);
      cv::namedWindow ("im2 weighted", CV_WINDOW_AUTOSIZE);
      cv::imshow ("im2 weighted", im2_copy);
      cv::waitKey (0);*/

      IplImage ipl_im1;
      ipl_im1 = mat1Translated;

      IplImage * dst1 = cvCreateImage (cvGetSize (&ipl_im1), ipl_im1.depth, 1);
      cvLogPolar (&ipl_im1, dst1, cvPoint2D32f (size / 2, size / 2), MScale);

      cv::Mat proj_i_lp (dst1); //projection translated

      /*to_display = proj_i_lp.clone();
      cv::minMaxLoc (to_display, &minVal_w, &maxVal_w);
      to_display /= maxVal_w;

      cv::namedWindow ("weighted2", CV_WINDOW_AUTOSIZE);
      cv::imshow ("weighted2", to_display);
      cv::waitKey (0);*/

      //compute how much the mesh needs to be rotated, scaled to match partial view
      int newMaxI = 0, newMaxJ = 0;
      double suma = 0;

      {
        struct timeval start, end;
        gettimeofday (&start, NULL);
        suma = crossCorrelationFFT (proj_i_lp, proj_j_lp, &newMaxI, &newMaxJ, SCALE_RANGE);

        gettimeofday (&end, NULL);
        //std::cout << "--------- cross correlation FFT:" << internal_pcl::timing (start, end) << " ms" << std::endl;
      }

      if (suma > maxSum && suma >= 0 && suma <= 1.0)
      {
        maxSum = suma;
        maxI = newMaxI;
        maxJ = newMaxJ;
        maxIIL = ii;
        maxJJL = jj;

        //std::cout << "ii,jj" << ii << "," << jj << " suma: " << suma << " maxSum:" << maxSum << " maxI:" << maxI
        //    << " maxJ:" << maxJ << std::endl;
      }

      //std::cout << "maxI:" << maxI << " suma:" << suma << std::endl;

      if (false || this->VISUALIZE_CROSS_)
      {
        double minVal, maxVal;

        cv::minMaxLoc (proj_i_lp, &minVal, &maxVal);
        cv::Mat_<float> B_i = proj_i_lp / maxVal;

        cv::minMaxLoc (proj_j_lp, &minVal, &maxVal);
        cv::Mat_<float> B_j = proj_j_lp / maxVal;

        //std::cout << "maxVal after logpolar:" << maxVal << std::endl;

        cv::namedWindow ("prova_i", CV_WINDOW_AUTOSIZE);
        cv::namedWindow ("prova_j", CV_WINDOW_AUTOSIZE);

        cv::imshow ("prova_i", B_i);
        cv::imshow ("prova_j", B_j);
        cv::waitKey (0);
      }

      //release created images
      cvReleaseImage (&dst1);
    }
  }

  cvReleaseImage (&dst2);

  *rotation = maxI / (double)(size) * 360;
  *scale = exp (maxJ / MScale);
  *maxII = maxIIL;
  *maxJJ = maxJJL;

  //translation needs to be applied to partial_view
  //rotation,scale needs to mesh
  //cout << "Values log polar translation:" << *rotation << " " << *scale << " " << *maxII << " " << *maxJJ << endl;
}

//GPU LOG POLAR

void
StablePlanesAlignment::logPolarTranslationGPU (const cv::Mat & im1, const cv::Mat & im2, double * rotation,
                                               double * scale, int * maxII, int * maxJJ)
{
  //std::cout << "Log polar translation for:" << im1.rows << std::endl;

  int size = im1.cols;
  double MScale = getScaleLogPolar (size);

  cv::Mat im1_copy = im1.clone ();
  cv::Mat im2_copy = im2.clone ();

  //we dont want to translate the mesh, just the partial view (im1)
  //use a weighting function...

  cv::Mat weight_matrix = im1_copy.clone ();
  getWeightMatrix (weight_matrix, size / 2, size / 2);
  cv::multiply (im2_copy, weight_matrix, im2_copy);

  IplImage ipl_im2;
  ipl_im2 = im2_copy;

  IplImage * dst2 = cvCreateImage (cvGetSize (&ipl_im2), ipl_im2.depth, 1);
  cvLogPolar (&ipl_im2, dst2, cvPoint2D32f (size / 2, size / 2), MScale);
  cv::Mat proj_j_lp (dst2);

  int SCALE_RANGE = getScaleRange (size);
  int XRANGE, YRANGE;
  XRANGE = YRANGE = getXYRange (size);

  int ii, jj;
  double maxSum = -1;
  int maxI, maxJ, maxIIL, maxJJL;
  maxI = maxJ = -1;
  maxIIL = maxJJL = -100;

  typedef float2 Complex;
  Complex *img_1_host;
  size_t one_img_size = size * size * sizeof(Complex);
  size_t size_array = (XRANGE * 2 + 1) * (YRANGE * 2 + 1) * one_img_size;

  img_1_host = (Complex *)malloc (size_array); // Allocate array on host
  int num_pixels = 0;

  cv::Mat mat1Big = cv::Mat_<float>::zeros (size + XRANGE * 2, size + YRANGE * 2);
  cv::Rect rect(XRANGE,YRANGE,size,size);
  cv::Mat roi(mat1Big, rect);
  im1_copy.copyTo(roi);

  std::map < int , std::pair<int,int> > img_to_ii_jj;
  size_t img_idx = 0;
  for (ii = -XRANGE; ii <= XRANGE; ii++)
  {
    for (jj = -YRANGE; jj <= YRANGE; jj++, img_idx++)
    {

      //instead of doing the whole wrapping stuff, just use image roi to shift the image...
      cv::Rect rect(XRANGE-ii,YRANGE-jj,size,size);
      cv::Mat mat1Trans(mat1Big, rect);
      cv::Mat mat1Translated = cv::Mat_<float>::zeros (size, size);
      cv::multiply (mat1Trans, weight_matrix, mat1Translated);

      IplImage ipl_im1;
      ipl_im1 = mat1Translated;

      IplImage * dst1 = cvCreateImage (cvGetSize (&ipl_im1), ipl_im1.depth, 1);
      cvLogPolar (&ipl_im1, dst1, cvPoint2D32f (size / 2, size / 2), MScale);

      cv::Mat proj_i_lp (dst1); //projection translated

      //allocate images in host memory to be copied afterwards all at once to device memory!
      size_t k, j;
      for (k = 0; k < (size_t)size; k++)
      {
        for (j = 0; j < (size_t)size; j++, num_pixels++)
        {
          (img_1_host + num_pixels)->x = (float)proj_i_lp.at<float> (k, j);
          (img_1_host + num_pixels)->y = 0.0;
        }
      }
      //release created images
      cvReleaseImage (&dst1);

      img_to_ii_jj.insert (std::make_pair (img_idx, std::make_pair (ii, jj)));
    }
  }


  int n_imgs = (int)size_array / one_img_size;
  int width = sqrt(one_img_size / sizeof(float));
  int n_elements_img1 = size_array / 4;
  int n_elements_img2 = width * width;

  //img_1_host contains the translated images and transformed to log polar
  //call wrapper function that will compute the cross-correlations for the chunks in the device

  Complex *img_2_host;
  num_pixels = 0;
  img_2_host = (Complex *)malloc (one_img_size); // Allocate array on host
  size_t k, j;
  for (k = 0; k < (size_t)size; k++)
  {
    for (j = 0; j < (size_t)size; j++, num_pixels++)
    {
      (img_2_host + num_pixels)->x = (float)proj_j_lp.at<float> (k, j);
      (img_2_host + num_pixels)->y = 0.0;
    }
  }

  //std::cout << "one_img_Size:" << one_img_size << std::endl;
  //critical section because otherwise we run out of memory in the GPU when allocating the plans
  //and the transforms...
  #pragma omp critical
  {
    struct timeval start, end;
    gettimeofday (&start, NULL);

    computeCrossCorrelations (img_1_host, (int)size_array, img_2_host, (int)one_img_size, (int)one_img_size, SCALE_RANGE);

    gettimeofday (&end, NULL);
    //std::cout << "--------- computeCrossCorrelations took:" << internal_pcl::timing (start, end) << " ms" << std::endl;
  }

  //img_1_host contain results for the cross-correlations, find max

  int max_idx = -1;
  for (size_t i = 0; i < (size_array / sizeof(Complex)); i++)
  {
    if (img_1_host[i].x > maxSum)
    {

      //std::cout << "img_1_host[i]" << img_1_host[i] << std::endl;
      maxSum = img_1_host[i].x;
      max_idx = i;
    }
  }

  //free img_1_host, img_2_host
  free(img_1_host);
  free(img_2_host);

  int max_idx_image = (max_idx % (size * size));
  maxI = max_idx_image / size;
  maxJ = max_idx_image % size;

  //std::cout << "maxI:" << maxI << " maxJ:" << maxJ << " size:" << size << std::endl;

  if (maxJ > (SCALE_RANGE + 1))
    maxJ = maxJ - size;

  //std::cout << max_idx_image << " " <<
  int image_idx = max_idx / (size * size);
  maxIIL = img_to_ii_jj[image_idx].first;
  maxJJL = img_to_ii_jj[image_idx].second;

  //free stuff...
  cvReleaseImage (&dst2);

  *rotation = maxI / (double)(size) * 360;
  *scale = exp (maxJ / MScale);
  *maxII = maxIIL;
  *maxJJ = maxJJL;

  //cout << "Values log polar translation GPU:" << *rotation << " " << *scale << " " << *maxII << " " << *maxJJ << endl;
}
