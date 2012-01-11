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
#ifndef FEATURES_77JEL51B
#define FEATURES_77JEL51B

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace cogx { namespace vision {

struct CSiftFeature
{
   enum { NDims=128, DescUCharRange=512 };
   long long id;
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
   //float score;       // Best score in range 0.6-1.0;  cudaSift min = 0.85; (from correlations)
   //float ambiguity;   // score2 / score1; 0.x-1.0; cudaSift max = 0.95
   float distance;      // min distance in range 0-sqrt(NDims)
   float ambiguity;     // distance0 / distance1; 0.0-1.0; lowe max = 0.8
   static int cmpDistance(CFeatureMatch* pa, CFeatureMatch* pb);
   static bool isLowerDistance(CFeatureMatch a, CFeatureMatch b);
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

class CDistanceCalculator
{
public:
   // descriptor dims are uchar[0..DescUCharRange], but distnace is float[0 .. sqrt(NDims)]
   double distance(CSiftFeature& a, CSiftFeature& b);
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

inline
double CDistanceCalculator::distance(CSiftFeature& a, CSiftFeature& b)
{
   unsigned char *pa = &a.descriptor[0];
   unsigned char *pb = &b.descriptor[0];
   long s = 0;
   for (int i = 0; i < CSiftFeature::NDims; i++) {
      long d = (long) *pa - (long) *pb;
      s += d*d;
      pa++; pb++;
   }
   return sqrt(s) / CSiftFeature::DescUCharRange;
}

inline
void sortmatches(TFeatureMatchVector& matches)
{
   std::sort(matches.begin(), matches.end(), CFeatureMatch::isLowerDistance);
}

inline
void sortmatches(std::vector<TFeatureMatchVector*>& matches)
{
   typeof(matches.begin()) it;
   for(it = matches.begin(); it != matches.end(); it++) {
      sortmatches(*(*it));
   }
}

}} // namespace

#endif /* end of include guard: FEATURES_77JEL51B */
// vim:sw=3:ts=8:et
