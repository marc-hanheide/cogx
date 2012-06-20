/*
 * @author:  Marko Mahnič
 * @created: jul 2010 
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

#include "MatcherCudaSift.h"

#include "CudaSift/cudaSift.h"
#include "CudaSift/featureUploadH.h"

#include <cmath>
#include <iostream>

namespace cogx { namespace vision {

// TODO: cast-global initialization of cuda. What about multiple threads?
static
void checkInit()
{
   static int initialized = 0;
   if (!initialized) {
      InitCuda();
      initialized = 1;
   }
}

static
float distance(float *data1, float* data2, int len)
{
   double diff, sum;
   sum = 0;
   while (len > 0) {
      diff = *data1 - *data2;
      sum += diff*diff;
      data1++; data2++; len--;
   }
   return sqrt(sum);
}


static
void copyVectorToSiftData(TSiftVector& a, SiftData &data)
{
   long nPoints = a.size();
#define ROUND(n) ((n + 7) / 8) * 8
   InitSiftData(&data, ROUND(nPoints), true, true); 
#undef ROUND

   int lendata = 128 * sizeof(float);
   memset((void*)&data.h_data[0], 0, lendata*nPoints);
   for (long i = 0; i < nPoints; i++) {
      a[i]->getDescriptor((float*)&data.h_data[i].data[0]);
   }
   
   data.numPts = nPoints;
}

static
long appendMatches(SiftData& a, SiftData& b, TFeatureMatchVector& matches, TSiftVector& sa, TSiftVector& sb)
{  
   static CDistanceCalculator dist;
   long maxIndex2 = b.numPts;
   long i, count = 0;
   for (i = 0; i < a.numPts; i++) {
      if (a.h_data[i].match >= 0 && a.h_data[i].match < maxIndex2) {
         count++;
         CFeatureMatch m;
         m.indexA = i;
         m.indexB = a.h_data[i].match;
         //m.score = a.h_data[i].score;
         // m.ambiguity = a.h_data[i].ambiguity;
         m.distance = dist.distance(*sa[m.indexA], *sb[m.indexB]);
         m.ambiguity = a.h_data[i].ambiguity * 0.8 / 0.95; // approx conversion from cudasift to lowe
         matches.push_back(m);
      }
   }

   return count;
}

void CSiftMatcherCudaSift::matchSiftDescriptors(TSiftVector& a, TSiftVector& b,
      TFeatureMatchVector& matches)
{
   matches.clear();
   if (a.size() < 1 || b.size() < 1) return;

   checkInit();
   SiftData siftData1, siftData2;
   copyVectorToSiftData(a, siftData1);
   copyVectorToSiftData(b, siftData2);

   // move data from host to device; we need a new .cu module
   UploadSiftData(&siftData1);
   UploadSiftData(&siftData2);
  
   MatchSiftData(&siftData1, &siftData2); // result in siftData1
   appendMatches(siftData1, siftData2, matches, a, b);

   FreeSiftData(&siftData1);
   FreeSiftData(&siftData2);
}

void CSiftMatcherCudaSift::matchSiftDescriptors(TSiftVector& a, std::vector<TSiftVector*>& bb,
      std::vector<TFeatureMatchVector*>& matches)
{
   assert(matches.size() == 0); // clearing matches would be dangerous (memory leaks)
   if (a.size() < 1 || bb.size() < 1) return;

   checkInit();
   SiftData siftData1, siftData2;
   copyVectorToSiftData(a, siftData1);

   UploadSiftData(&siftData1);
   
   typeof(bb.begin()) it;
   for (it = bb.begin(); it != bb.end(); it++) {
      TSiftVector* pVect = *it;
      copyVectorToSiftData(*pVect, siftData2);
      UploadSiftData(&siftData2);
     
      TFeatureMatchVector* pmatches = new TFeatureMatchVector();
      matches.push_back(pmatches);
      MatchSiftData(&siftData1, &siftData2); // result in siftData1
      appendMatches(siftData1, siftData2, *pmatches, a, *(*it));

      FreeSiftData(&siftData2);
   }

   FreeSiftData(&siftData1);
}

}} // namespace
// vim:sw=3:ts=8:et
