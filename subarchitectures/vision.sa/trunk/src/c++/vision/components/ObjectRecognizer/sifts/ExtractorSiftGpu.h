
#ifndef EXTRACTOR_6D61JTQX
#define EXTRACTOR_6D61JTQX

#include "Features.h"
#include <gl.h>
#include <SiftGPU.h>

#include <exception>
#include <vector>

#include <opencv/cv.h>

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
