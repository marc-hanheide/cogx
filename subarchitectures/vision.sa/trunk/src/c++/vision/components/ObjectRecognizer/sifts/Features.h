#ifndef FEATURES_77JEL51B
#define FEATURES_77JEL51B

#include <vector>
#include <opencv/cv.h>

namespace cogx { namespace vision {

struct CSiftFeature
{
   enum { NDims=128, DescUCharRange=512 };
   float x, y, scale, orientation;
   unsigned char descriptor[NDims];
   void setKeypoint(float kx, float ky, float scl, float ornt);
   void setDescriptor(float* pDescriptor);
   void getDescriptor(float* pDescriptor);
};
typedef std::vector<CSiftFeature*> TSiftVector;

class CSiftExtractor
{
public:
   // Extract SIFT features from the image and add them to the list.
   // @return number of added SIFTs.
   virtual long extractSifts(IplImage* pImage, TSiftVector& sifts) { return 0; }
};

struct CFeatureMatch
{
   long indexA;
   long indexB;
   float distance;
};
typedef std::vector<CFeatureMatch> TFeatureMatchVector;

class CSiftMatcher
{
public:
   virtual void matchSiftDescriptors(TSiftVector& a, TSiftVector& b,
         TFeatureMatchVector& matches) {}
   virtual void matchSiftDescriptors(TSiftVector& a, std::vector<TSiftVector*>& bb,
         std::vector<TFeatureMatchVector*>& matches) {}
};


inline
void CSiftFeature::setKeypoint(float kx, float ky, float scl, float ornt)
{
   x = ky; y = ky; scale = scl; orientation = ornt;
}

inline
void CSiftFeature::setDescriptor(float* pDescriptor)
{
   unsigned char *pd = &descriptor[0];
   for (int i = 0; i < NDims; i++) {
      *pd = ((int)(*pDescriptor * DescUCharRange)) % 256;
      pd++; pDescriptor++;
   }
}

inline
void CSiftFeature::getDescriptor(float* pDescriptor)
{
   unsigned char *pd = &descriptor[0];
   for (int i = 0; i < NDims; i++) {
      *pDescriptor = *pd * 1.0f / DescUCharRange;
      pd++; pDescriptor++;
   }
}

}} // namespace

#endif /* end of include guard: FEATURES_77JEL51B */
