
#include "ExtractorSiftGpu.h"
#include "Exception.h"

namespace cogx { namespace vision {

SiftGPU* CSiftExtractorGPU::pSiftGpu = NULL;
long CSiftExtractorGPU::instcount = 0;

CSiftExtractorGPU::CSiftExtractorGPU()
{
}

CSiftExtractorGPU::~CSiftExtractorGPU()
{
   instcount--;
   if (pSiftGpu && instcount < 1) {
      delete pSiftGpu;
      pSiftGpu = NULL;
   }
}

SiftGPU* CSiftExtractorGPU::getSiftGpu()
{
   if (pSiftGpu) return pSiftGpu;

   // TODO: parameters
   // -d 3: best value for the number of scales per octave (lowe04ijcv)
   // -no 8: limit the number of octaves processed (default: unlimited)
   // -fo 0: first octave; -1 to upscale initial image
   // char * argv[] = {"-fo", "-1",  "-v", "1"};// "-cg"
   char * argv[] = {"-fo", "-1",  "-v", "1",  "-d", "3", "-no", "8"}; //
   int argc = sizeof(argv)/sizeof(char*);
   pSiftGpu = new SiftGPU();
   pSiftGpu->ParseParam(argc, argv);

   if(pSiftGpu->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
      throw SiftGpuNotSupported();
   }

   return pSiftGpu;
}

long CSiftExtractorGPU::extractSifts(IplImage* pImage, TSiftVector& sifts)
{
   if (pImage->depth != IPL_DEPTH_8U && pImage->nChannels!=1)
      throw EXCEPTION("SiftGPU: Unsupported Image Format ("
            << pImage->nChannels << "chn * " << pImage->depth << "bits).");
   int rv;
   SiftGPU* pGpu = getSiftGpu();
   rv = pGpu->RunSIFT(pImage->width, pImage->height, (unsigned char*) pImage->imageData,
         GL_LUMINANCE, GL_UNSIGNED_BYTE);

   if (! rv) {
      throw Exception("SiftGPU: Error in RunSIFT");
   }

   int nf = pGpu->GetFeatureNum();
   if (nf < 1) return nf;
   SiftGPU::SiftKeypoint* pKeys = new SiftGPU::SiftKeypoint[nf];
   SiftGPU::SiftKeypoint* pk = pKeys;
   float* pDescrs = new float[nf * CSiftFeature::NDims];
   float* pd = pDescrs;
   pGpu->GetFeatureVector(pKeys, pDescrs);
   for (int i = 0; i < nf; i++) {
      CSiftFeature* pSift = new CSiftFeature();
      pSift->setKeypoint(pk->x, pk->y, pk->s, pk->o);
      pSift->setDescriptor(pd);
      sifts.push_back(pSift);
      pk++;
      pd += CSiftFeature::NDims;
   }
   delete pKeys;
   delete pDescrs;

   return nf;
}

}} // namespace
