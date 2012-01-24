/*
 * @author:  Marko Mahnič
 * @created: jun 2010 
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef EXTRACTOR_6D61JTQX
#define EXTRACTOR_6D61JTQX

#include "Features.h"
#include <gl.h>
#include <SiftGPU.h>

#include <exception>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace cogx { namespace vision {

class SiftGpuNotSupported: public std::exception
{
public:
   virtual const char* what() const throw() {
      return "SiftGPU is not fully supported by the GPU hardware.";
   }
};

class CSiftExtractorGPU: public CSiftExtractor
{
   static SiftGPU* pSiftGpu;
   static long instcount;
   static SiftGPU* getSiftGpu();
public:
   CSiftExtractorGPU();
   virtual ~CSiftExtractorGPU();

   // Extract SIFT features from the image and add them to the list.
   // @return number of added SIFTs.
   virtual long extractSifts(IplImage* pImage, TSiftVector& sifts);
};

}} // namespace
#endif /* end of include guard: EXTRACTOR_6D61JTQX */
