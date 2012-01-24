/* vim:set fileencoding=utf-8 sw=3 ts=8 et:vim */
/** 
 * @author Marko Mahnič
 */
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <cv.h>
#include <highgui.h>
#include <VisionData.hpp>
#include <PointCloud.hpp>
#include "../../../c++/vision/VisionUtils.h"

#include "MatlabHelper.h"
#include "Conversion.h"

using namespace std;
using namespace VisionData;
using namespace PointCloud;

namespace matlab {

void protoObjectToMwArray(const ProtoObject &Object, mwArray &image, mwArray &mask, mwArray &points3d)
{
   // Convert image patch
   image = CMatlabHelper::iplImage2array(&(Object.image.data[0]), 
         Object.image.width, Object.image.height, 3, 1); // WISH: number of channels in image
   // printf("Object Image: %dx%d\n", Object.image.width, Object.image.height);

   // Convert image mask
   mask = CMatlabHelper::iplImage2array( &(Object.mask.data[0]), 
         Object.mask.width, Object.mask.height, 1, 1);
   // printf("Object Mask: %dx%d\n", Object.mask.width, Object.mask.height);

   // Convert 3D points
   int npts = Object.points.size();
   const int ncol = 6;
   if (npts < 1) {
      printf("**** not enough points\n");
      points3d = mwArray();
   }
   else {
      printf("**** we have %d 3D points\n", npts);
      // x, y, z, r, g, b
      mwSize dimensions[2] = {npts, ncol};
      points3d = mwArray(2, dimensions, mxDOUBLE_CLASS, mxREAL);
      double *data = new double[npts * ncol];
      for (int i = 0; i < npts; i++) {
         const SurfacePoint &p = Object.points[i];
         data[i + npts*0] = p.p.x;
         data[i + npts*1] = p.p.y;
         data[i + npts*2] = p.p.z;
         data[i + npts*3] = p.c.r;
         data[i + npts*4] = p.c.g;
         data[i + npts*5] = p.c.b;
      }
      points3d.SetData(data, npts*ncol);
      delete data;
   }
}

void protoObjectToMwArray_Patches(const ProtoObject &Object, mwArray &patches)
{
   int nPatch = Object.surfacePatches.size();
   int nRows = nPatch + 1;
   const int nCol = 6;

   typeof(Object.surfacePatches.begin()) itPatch;
   for(itPatch = Object.surfacePatches.begin(); itPatch != Object.surfacePatches.end(); itPatch++) {
      nRows += itPatch->points.size();
   } 
   mwSize dimensions[2] = {nRows, nCol};
   patches = mwArray(2, dimensions, mxDOUBLE_CLASS, mxREAL);
   double *data = new double[nRows * nCol];
   memset(data, 0, sizeof(double)*nRows*nCol);

#define writedata(row, x, y, z, r, g, b) { \
         data[row + nRows*0] = (double)x; data[row + nRows*1] = (double)y; data[row + nRows*2] = (double)z; \
         data[row + nRows*3] = (double)r; data[row + nRows*4] = (double)g; data[row + nRows*5] = (double)b; \
         }

#define writeinfo(row, len, offs)  writedata(row, len, len, len, offs+1, offs+1, offs+1)

   int iHeadRow = 0;
   int iRow = nPatch + 1;

   writeinfo(iHeadRow, nPatch, 1);
   iHeadRow++;

   for(itPatch = Object.surfacePatches.begin(); itPatch != Object.surfacePatches.end(); itPatch++) {
      int npts = itPatch->points.size();
      writeinfo(iHeadRow, npts, iRow);
      iHeadRow++;

      for (int i = 0; i < npts; i++) {
         const SurfacePoint &p = itPatch->points[i];
         writedata(iRow, p.p.x, p.p.y, p.p.z, p.c.r, p.c.g, p.c.b);
         iRow++;
      }
   } 
   patches.SetData(data, nRows * nCol);
   delete data;
#undef writedata
#undef writeinfo
}

} // namespace

