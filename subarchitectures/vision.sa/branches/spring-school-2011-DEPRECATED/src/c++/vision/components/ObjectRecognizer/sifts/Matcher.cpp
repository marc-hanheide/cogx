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

#include "Matcher.h"

#include <cmath>
#include <iostream>

namespace cogx { namespace vision {

void CSiftMatcherCpp::matchSiftDescriptors(TSiftVector& a, TSiftVector& b,
      TFeatureMatchVector& matches)
{
   CDistanceCalculator dist;
   matches.clear();
   if (a.size() < 1 || b.size() < 1) return;
   typeof(a.begin()) ita, itb;
   int i = 0;
   for (ita = a.begin(); ita != a.end(); ita++) {
      double dbest[2] = {1e99, 1e99};
      int j = 0, jbest=0;
      for (itb = b.begin(); itb != b.end(); itb++) {
         double d = dist.distance(*(*ita), *(*itb));
         if (d < dbest[0]) {
            jbest = j;
            dbest[1] = dbest[0];
            dbest[0] = d;
         }
         else if (d < dbest[1]) {
            dbest[1] = d;
         }
         j++;
      }
      CFeatureMatch m;
      m.indexA = i;
      m.indexB = jbest;
      m.distance = dbest[1];
      m.ambiguity = (dbest[1] > 0) ? dbest[0] / dbest[1] : 0;
      matches.push_back(m);
      i++;
   }
}

void CSiftMatcherCpp::matchSiftDescriptors(TSiftVector& a, std::vector<TSiftVector*>& bb,
      std::vector<TFeatureMatchVector*>& matches)
{
}

}} // namespace
// vim:sw=3:ts=8:et
