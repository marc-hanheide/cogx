
#include "GraphCutSegmenter.h"

#include <Math.hpp>
#include <../../VisionUtils.h>
#include <list>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace PointCloud;
using namespace VisionData;
using namespace Video;
using namespace cogx::Math;

#define DILATE_FACTOR 1.3

// Segmentation costants

#define SMOOTH_COST 30
#define HSL_K_RATIO 40 // Number of nearest neighbours taken when calculating the cost for hls

#define OBJ_HSL_TOLERANCE 23
#define BG_HSL_TOLERANCE 24

#define OBJ_DIST_TOLERANCE 60
#define BG_DIST_TOLERANCE 9999

#define LABEL_FIX_COST 24

#define MAX_HSL_VAL 180
#define MIN_HSL_VAL 0

#define EXPAND_ITERS 3
#define SWAP_ITERS 2

#define MAX_PATCH_SIZE 36000

#define MAX_COLOR_SAMPLE 90
#define COLOR_FILTERING_THRESHOLD 7

#define COLOR_SAMPLE_IMG_WIDTH 5
#define COLOR_SAMPLE_IMG_HEIGHT 15

namespace cast
{

//struct gCutData {
//	int numLab;
//	int k;
//	std::list<int>
//	const IplImage *hlsImg;
//	};

static long smoothFn(int p1, int p2, int l1, int l2)
{
  if ( l1 != l2 ) return(SMOOTH_COST);
  //	else if (l1==0) return(0);
  else return 0;
}

GraphCutSegmenter::GraphCutSegmenter()
{
  objHueTolerance = OBJ_HSL_TOLERANCE;
  objDistTolerance = OBJ_DIST_TOLERANCE;
  bgHueTolerance = BG_HSL_TOLERANCE;
  bgDistTolerance = BG_DIST_TOLERANCE;
  lblFixCost = LABEL_FIX_COST;
  smoothCost = SMOOTH_COST;
}

void GraphCutSegmenter::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  objHueTolerance = OBJ_HSL_TOLERANCE;
  objDistTolerance = OBJ_DIST_TOLERANCE;
  bgHueTolerance = BG_HSL_TOLERANCE;
  bgDistTolerance = BG_DIST_TOLERANCE;
  lblFixCost = LABEL_FIX_COST;
  smoothCost = SMOOTH_COST;

  if((it = _config.find("--display")) != _config.end())
  {
    doDisplay = true;
  }

  if((it = _config.find("--objht")) != _config.end())
  {
    istringstream str(it->second);
    str >> objHueTolerance;
  }

  if((it = _config.find("--objdt")) != _config.end())
  {
    istringstream str(it->second);
    str >> objDistTolerance;
  }

  if((it = _config.find("--bght")) != _config.end())
  {
    istringstream str(it->second);
    str >> bgHueTolerance;
  }

  if((it = _config.find("--bgdt")) != _config.end())
  {
    istringstream str(it->second);
    str >> bgDistTolerance;
  }

  if((it = _config.find("--fixc")) != _config.end())
  {
    istringstream str(it->second);
    str >> lblFixCost;
  }  

  if((it = _config.find("--smoc")) != _config.end())
  {
    istringstream str(it->second);
    str >> smoothCost;
  }
}

void GraphCutSegmenter::verifySetRoi(IplImage* pImg, CvRect r, long line)
{
  if (r.width < 1 || r.height < 1) {
    error("%d: cvSetImageROI, Invalid Image Width or Height", line);
  }
  if (r.x < 0 || r.y < 0) {
    error("%d: cvSetImageROI, Invalid Image Top-Left Coordinate", line);
  }
  if (r.x + r.width > pImg->width || r.y + r.height > pImg->height) {
    error("%d: cvSetImageROI, Invalid Image Width or Height", line);
  }
  cvSetImageROI(pImg, r);
}

void GraphCutSegmenter::verifySetRoi(cv::Ptr<IplImage> imagePtr, CvRect rect, long line)
{
  verifySetRoi((IplImage*)imagePtr, rect, line);
}

void GraphCutSegmenter::project3DPoints(const vector<SurfacePoint> surfPoints, const ROI &roi, const float ratio,
    const Video::CameraParameters &cam, vector<CvPoint> &projPoints, vector<int> &hull)
{
  size_t n = surfPoints.size();

  // calculate projected points
  for(size_t i = 0; i < n; i++)
  {
    cogx::Math::Vector2 p = projectPoint(cam, surfPoints[i].p);

    /// HACK: artificially shrink cloud point
    //int x = (p.x - roi.rect.pos.x)*ratio*0.8 + roi.rect.width/2;
    //int y = (p.y - roi.rect.pos.y)*ratio*0.8 + roi.rect.height/2;
    int x = (p.x - roi.rect.pos.x) + roi.rect.width/2;
    int y = (p.y - roi.rect.pos.y) + roi.rect.height/2;

    projPoints.push_back(cvPoint(x, y));
  }

  // calculate convex hull
  //CvMat pointMat = cvMat( 1, n, CV_32SC2, &projPoints[0] );
  //CvMat hullMat = cvMat( 1, n, CV_32SC1, &hull[0] );
  //cvConvexHull2(&pointMat, &hullMat, CV_CLOCKWISE, 0);

}


vector<SurfacePoint>  GraphCutSegmenter::filter3DPoints(const vector<SurfacePoint> surfPoints, vector<CvPoint> &projPoints, vector<CvPoint> &errProjPoints, const SegmentMask segMask)
{
  size_t n = surfPoints.size();
  vector<SurfacePoint> filtSurfPoints;

  // 
  for(size_t i = 0; i < n; i++)
  {
    int x=projPoints[i].x;
    int y=projPoints[i].y;
    if (x >= 0 && y >= 0 && x < segMask.width && y < segMask.height
        && segMask.data[y*segMask.width + x] == 1)
    {
        filtSurfPoints.push_back(surfPoints[i]);
    }
    else
      errProjPoints.push_back(projPoints[i]);

  }

  return filtSurfPoints;

}


void GraphCutSegmenter::drawProjectedSOIPoints(IplImage *img, const vector<CvPoint> projPoints,
    const vector<CvPoint> bgProjPoints,  const vector<CvPoint> errProjPoints, const vector<int> hull)
{
  // draw foreground points
  for(size_t i = 0; i < projPoints.size(); i++)
  {
    cvCircle(img, cvPoint(projPoints[i].x, projPoints[i].y), 0, CV_RGB(0,255,0));
  }

  // draw convex hull
  //  int nhull = hull.size();
  //  CvPoint pt0 = projPoints[hull[nhull - 1]];
  //  for(int i = 0; i < nhull; i++)
  //  {
  //    CvPoint pt = projPoints[hull[i]];
  //    cvLine(img, pt0, pt, CV_RGB(255, 0, 0));
  //    pt0 = pt;
  //  }

  // draw background points inside SOI
  for(size_t i = 0; i < bgProjPoints.size(); i++)
  {
    cvCircle(img, cvPoint(bgProjPoints[i].x, bgProjPoints[i].y), 0, CV_RGB(255,0,0));
  }

  // draw misprojected points inside SOI
  for(size_t i = 0; i < errProjPoints.size(); i++)
  {
    cvCircle(img, cvPoint(errProjPoints[i].x, errProjPoints[i].y), 0, CV_RGB(127,127,0));
  }
}


void GraphCutSegmenter::drawPoints(IplImage *img, const vector<CvPoint> projPoints)
{
  for(size_t i = 0; i < projPoints.size(); i++)
  {
    cvCircle(img, cvPoint(projPoints[i].x, projPoints[i].y), 3, CV_RGB(0,255,0));
  }
}


void GraphCutSegmenter::drawHull(IplImage *img, const vector<CvPoint> projPoints, const vector<int> hull)
{
  int n = hull.size();
  CvPoint pt0 = projPoints[hull[n - 1]];
  for(int i = 0; i < n; i++)
  {
    CvPoint pt = projPoints[hull[i]];
    cvLine(img, pt0, pt, CV_RGB(255, 0, 0));
    pt0 = pt;
  }
}


static int hlsAbsDiff(CvScalar i, CvScalar j, float hueOverflow, float norm)
{

  float hueDiff = (float) abs((i.val[0] - j.val[0]));

  if (hueDiff > hueOverflow)
    hueDiff = hueOverflow*2 - hueDiff;

  if (hueDiff > norm)
    hueDiff = norm;

  hueDiff/=norm;

  float satDiff = (float) abs(i.val[2] - j.val[2])/255;
  float lumDiff = (float) abs(i.val[1] - j.val[1])/255;

  float satAvg = (float) (i.val[2] + j.val[2])/510;

  float hlDiff = satAvg*hueDiff + (1-satAvg)*lumDiff;
  float hlsDiff =  satDiff*satDiff + (1 - satDiff)*hlDiff;

  return (int) (hlsDiff * norm * 3); 	
}




vector<CvScalar> GraphCutSegmenter::getSortedHlsList(vector<SurfacePoint> surfPoints)
{
  vector<CvScalar> hlsList;

  vector<SurfacePoint>::iterator it;
  int size = surfPoints.size();

  if(size == 0)
  { 
    // log("WARNING: Surface point list empty");
    return hlsList;
  }

  // debug("0, size=%d", size);
  cv::Ptr<IplImage> src = cvCreateImage(cvSize(size, 1), IPL_DEPTH_8U, 3);
  cv::Ptr<IplImage> dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
  cv::Ptr<IplImage> srcL = cvCreateImage(
      cvSize(size*COLOR_SAMPLE_IMG_WIDTH , COLOR_SAMPLE_IMG_HEIGHT ), IPL_DEPTH_8U, 3);
  // cv::Ptr<IplImage> dstL = cvCreateImage(cvGetSize(srcL), IPL_DEPTH_8U, 3);

  for(int i=0; i< size; i++)
  {
    CvScalar v;
    //log("red: %i green: %i blue: %i", surfPoints[i].c.r, surfPoints[i].c.g, surfPoints[i].c.b);	  
    v.val[0] = surfPoints[i].c.r;
    v.val[1] = surfPoints[i].c.g;
    v.val[2] = surfPoints[i].c.b;

    cvSet2D(src, 0, i, v);
  }

  cvCvtColor(src, dst, CV_RGB2HLS);

  for(int i=0; i< size; i++)
  {	
    hlsList.push_back(cvGet2D(dst, 0, i));
    //	  log("h: %i l: %i s: %i", (unsigned) cvGet2D(dst, 0, i).val[0], (unsigned) cvGet2D(dst, 0, i).val[1], (unsigned) cvGet2D(dst, 0, i).val[2]);
  } 

  //safety check --- points inside calculated ROI might be outside the actual ROI (outside image patch)
  //	  if(it->x < hlsImg->width && it->y < hlsImg->height && it->x >= 0 && it->y >= 0) 
  //	    hlsList.push_back(cvGet2D(hlsImg, it->y, it->x).val[0]);

  //	sort(hlsList.begin(), hlsList.end(), hslCompare);

  //	list<int>::iterator itr;	
  //	printf("HueList: ");	
  //	for( itr=hlsList.begin(); itr!=hlsList.end(); itr++)
  //		printf("%i ", *itr);
  //		printf("\n");
  cvResize(src, srcL, CV_INTER_NN);
  //cvResize(dst, dstL, CV_INTER_NN);

  int pos;
  if (filterFlag)
    pos = 0;
  else
    pos =COLOR_SAMPLE_IMG_HEIGHT ;

  verifySetRoi(colorFiltering, cvRect( 0, pos, srcL->width, srcL->height), __LINE__ );
  cvCopyImage(srcL, colorFiltering);
  cvResetImageROI(colorFiltering);	

#ifdef FEAT_VISUALIZATION
  if (pDisplay)
  {
    //pDisplay->setImage("soif.Color Filtering 0", srcL);
    //pDisplay->setImage("soif.Obj HLS Colors", dstL);
  }
#else
  //cvShowImage("Color Filtering", srcL);
  //cvShowImage("Obj HLS Colors", dstL);
#endif

  return hlsList;
}


int GraphCutSegmenter::getHlsDiff(vector<CvScalar> hlsList, CvScalar hls, int k)
{
  int domainwdt = MAX_HSL_VAL - MIN_HSL_VAL;
  int overflow = domainwdt/2;
  int maxdiff = domainwdt/6;	

  list<int> costs;

  costs.assign(k, maxdiff*3);

  list<int>::iterator maxmin = costs.begin();	

  for(vector<CvScalar>::iterator it=hlsList.begin(); it != hlsList.end(); it++)
  {	
    int cost =	hlsAbsDiff(hls, *it, overflow, maxdiff);

    if(cost < *maxmin)
    {
      //			costs.erase(maxmin);
      //			costs.push_back(cost);
      *maxmin = cost;
      maxmin = max_element(costs.begin(), costs.end());
    }
  }

  long totalCost = 0;

  for (list<int>::iterator it=costs.begin(); it != costs.end(); it++)
    totalCost += *it;

  return totalCost/k;
}

/*
   int GraphCutSegmenter::getHlsDiff(vector<CvScalar> hlsList, CvScalar hls, int k)
   {
   int domainwdt = MAX_HSL_VAL - MIN_HSL_VAL;
   int overflow = domainwdt/2;
   int maxdiff = domainwdt/6;	

   long cost, mincost;
   vector<CvScalar> ordsamp = hlsList; 
   vector<CvScalar>::iterator ith, itl;

   for(ith=ordsamp.begin(); hslCompare(*ith, hls)  && ith != ordsamp.end(); ith++) // ith->val[0] < hls.val[0]
   {}

   cost = 0;
   itl = ith;
   int r=-1;

   if (ith != ordsamp.begin())
   {
   for(int i = 0; i < k; i++)
   if (itl == ordsamp.begin() && r>=-1) {

   cost+=maxdiff;
   r++;
   } 
   else {

   itl--;
   int diff = hlsAbsDiff(hls, *itl, overflow, maxdiff, 0.4, 0.4); //hls.val[0] - itl->val[0];
   cost+= diff;	  	

   if(itl == ordsamp.begin())
   r++;
   }
   } 
   else {

   cost = k*maxdiff;
   r=k;
   }  

   mincost = cost;
// printf("hls %i cost %i \n ", hls, cost);	  
while (cost <= mincost && ith!=ordsamp.end()) {
mincost = cost;

if (r>0) {

cost-=maxdiff;
r--;
}
else {

int diff = hlsAbsDiff(hls, *itl, overflow, maxdiff, 0.4, 0.4); //abs(itl->val[0] - hls.val[0]);  		
itl++;	
cost-= diff;		
}	 
int diff = hlsAbsDiff(hls, *itl, overflow, maxdiff, 0.4, 0.4); //abs(ith->val[0] - hls.val[0]); 
cost+= diff;
ith++;

}	  
mincost/=k;

return mincost;
}

*/


vector<CvScalar> GraphCutSegmenter::colorFilter( vector<CvScalar> colors, vector<CvScalar> filterColors, int k, int tolerance)
{

  vector<CvScalar> filteredList;

  for(vector<CvScalar>::iterator it= colors.begin(); it != colors.end(); it++)
  {
    int cost = getHlsDiff(filterColors, *it, k);

    if( cost >= tolerance)
      filteredList.push_back(*it);
  }

  return filteredList;

}

vector<SurfacePoint> GraphCutSegmenter::sample3DPoints(vector<SurfacePoint> points, int newSize)
{
  vector<SurfacePoint> fltPoints;
  srand (time(NULL));

  int fsize = points.size()-newSize;

  if( fsize >= newSize)
  {
    for(int i=points.size()-1; i>=fsize; i--)
    {
      int r = rand()%i;
      fltPoints.push_back(points[r]);
      points.erase(points.begin() + r);
    }

    return fltPoints;
  }
  else
  {
    for(int i=points.size()-1; i>=newSize; i--)
    {
      int r = rand()%i;
      fltPoints.push_back(points[r]);
      points.erase(points.begin() + r);
    }

    return points;

  }	  
}


IplImage* GraphCutSegmenter::getCostImage(IplImage *iplPatchHLS, vector<CvPoint> projPoints, vector<SurfacePoint> allSurfPoints, float hlsSigma, float distSigma, bool distcost)
{
  IplImage *hlsPatch; // = cvCreateImage(cvGetSize(iplPatchHLS), IPL_DEPTH_8U, 3);
  CvSize size = cvGetSize(iplPatchHLS);

  IplImage *samplePatch = cvCreateImage(size, IPL_DEPTH_8U, 1);
  IplImage *distPatch = cvCreateImage(size, IPL_DEPTH_32F, 1);
  IplImage *distScaledPatch = cvCreateImage(size, IPL_DEPTH_8U, 1);
  IplImage *costImg = cvCreateImage(size, IPL_DEPTH_8U, 1);
  cvSet(costImg, cvScalar(255));

  //cvSetImageCOI(iplPatchHLS, 1);
  hlsPatch = cvCloneImage(iplPatchHLS);

  vector<SurfacePoint> surfPoints =  allSurfPoints;

  if(distcost)
  {
    cvSet(samplePatch,cvScalar(1));
    vector<CvPoint>::iterator itr;

    for(itr = projPoints.begin(); itr != projPoints.end(); itr++)
    { 
      //safety check --- points inside calculated ROI might be outside the actual ROI (outside image patch)
      if(itr->x < samplePatch->width && itr->y < samplePatch->height && itr->x >= 0 && itr->y >= 0)
        cvSet2D(samplePatch, itr->y, itr->x, cvScalar(0));
      // TODO> readd
      //else
      //     log("WARNING: point <%i, %i> outside ROI <%i, %i>)", itr->y, itr->x, samplePatch->height, samplePatch->width);

    }

    cvDistTransform(samplePatch, distPatch, CV_DIST_C);
  }
  else
    cvSet(distPatch, cvScalar(0));

  cvConvertScale(distPatch, distScaledPatch, 1, 0);	

  if(filterFlag) //HACK
  {
    if (surfPoints.size() > MAX_COLOR_SAMPLE)
      surfPoints = sample3DPoints(surfPoints, MAX_COLOR_SAMPLE);
  }
  else
    if (surfPoints.size() > MAX_COLOR_SAMPLE*3/4)
      surfPoints = sample3DPoints(surfPoints, MAX_COLOR_SAMPLE*3/4);

  int colorKval = min(surfPoints.size(), (size_t) MAX_COLOR_SAMPLE)/HSL_K_RATIO + 1;

  vector<CvScalar> sortHlsList = getSortedHlsList(surfPoints); 

  if (!filterFlag)			//HACK
    filterList = sortHlsList;
  else
  {	
    int size = sortHlsList.size();
    int k = filterList.size()/HSL_K_RATIO + 1;
    sortHlsList = colorFilter(sortHlsList, filterList, k, COLOR_FILTERING_THRESHOLD);

    colorKval = sortHlsList.size()/HSL_K_RATIO + 1;

    // log("Filtered out %i color samples", size - sortHlsList.size());

    size = sortHlsList.size();
    if (size)
    {
      IplImage* src = cvCreateImage(cvSize(size, 1), IPL_DEPTH_8U, 3);
      IplImage* dst = cvCreateImage(cvSize(size, 1), IPL_DEPTH_8U, 3);
      IplImage* dstL = cvCreateImage(cvSize(size*COLOR_SAMPLE_IMG_WIDTH , COLOR_SAMPLE_IMG_HEIGHT), IPL_DEPTH_8U, 3);

      for(int i=0; i< size; i++)
      {
        CvScalar v = sortHlsList[i];

        cvSet2D(src, 0, i, v);
      }
      cvCvtColor(src, dst, CV_HLS2RGB);
      cvResize(dst, dstL, CV_INTER_NN);
      verifySetRoi(colorFiltering, cvRect( 0, COLOR_SAMPLE_IMG_HEIGHT*2, dstL->width, dstL->height) );
      cvCopyImage(dstL, colorFiltering);
      cvResetImageROI(colorFiltering);	

      cvReleaseImage(&dst);
      cvReleaseImage(&dstL);
      cvReleaseImage(&src);
    }
  }

  // log("%i color neighbours", colorKval);

  float hlsSigma2 = 2*sqr(hlsSigma);
  float distSigma2 = 2*sqr(distSigma);
  float norm = 100;

  for(int i=0; i < costImg->height; i++)
    for(int j=0; j < costImg->width; j++) {

      int hlsDiff = getHlsDiff(sortHlsList, cvGet2D(hlsPatch, i, j), colorKval);
      float hlsP = exp(-sqr((float) hlsDiff)/hlsSigma2);
      int distDiff = cvGet2D(distScaledPatch, i, j).val[0];
      float distP = exp(-sqr((float) distDiff)/distSigma2);
      int cost = (int) ((1 - hlsP*distP)*norm);

      cvSet2D(costImg, i, j, cvScalar(cost));
    }

  cvReleaseImage(&hlsPatch);      
  cvReleaseImage(&samplePatch);
  cvReleaseImage(&distPatch);      
  cvReleaseImage(&distScaledPatch);

  return costImg;
}



vector<unsigned char> GraphCutSegmenter::graphCut(int width, int height, int num_labels, IplImage* costImg, IplImage* bgCostImg)
{
  int num_pixels = width*height;
  vector<unsigned char> result(num_pixels);   // stores result of optimization

  // log("Segment image %ix%i", width, height);

  try{
    GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(width, height, num_labels);

    // set up the needed data to pass to function for the data costs

    //gCutData toFn;
    //toFn.hslList= hslList;
    //toFn.numLab = num_labels;
    //toFn.hslImg = hslImg;
    //toFn.k = k;

    //gc->setDataCost(&dataFn, &toFn);
    long *data = new long[num_labels*num_pixels];

    // TODO: possible error: lines of IplImage are aligned to 4 bytes, while gc->data has no alignment!
    //for(int i=0; i<num_pixels; i++) {
    //  int idx = num_labels *i;
    //  data[idx] = lblFixCost;
    //  data[idx + 1] = costImg->imageData[i];
    //  data[idx + 2] = bgCostImg->imageData[i];
    //}
    int idx = 0;
    for (int i = 0; i < costImg->height; i++) {
      int lineoffs = i * costImg->widthStep;
      for (int j = 0; j < costImg->width; j++) {
        data[idx++] = lblFixCost;
        data[idx++] = costImg->imageData[lineoffs+j];
        data[idx++] = bgCostImg->imageData[lineoffs+j];
      }
    }

    gc->setDataCost(data);

    // smoothness comes from function pointer
    gc->setSmoothCost(&smoothFn);


    // log("Before optimization energy is %d",gc->compute_energy());

    int e1 = gc->compute_energy();
    gc->expansion(3);
    int e2 = gc->compute_energy();

    int diff = abs(e1 - e2);
    int iter = 3;
    /*		
          while(diff > 15)
          {
            e1 = gc->compute_energy();
            gc->expansion(1); // run expansion
            e2 = gc->compute_energy();
            iter++;
            diff = abs(e1 - e2);
          }
    */		
    // log("After %i expand iterations the energy is %d", iter, gc->compute_energy());

    //		gc->swap(SWAP_ITERS);		 // run swap
    //		log("\nAfter %i swap iterations the energy is %d\n", SWAP_ITERS, gc->compute_energy());

    for ( int  i = 0; i < num_pixels; i++ )
      result[i] = (unsigned char) gc->whatLabel(i);

    delete gc;
    delete data;
  }
  catch (GCException e){
    e.Report();
  }

  return result;
}

// (review2010): Added pProto to fill the imageOrigin and imageSourceSize
bool GraphCutSegmenter::segmentObject(const SOIPtr soiPtr, Video::Image &imgPatch, SegmentMask &segMask, vector<SurfacePoint> &segPoints, ProtoObjectPtr& pProto)
{
  log("GraphCutSegmenter::segmentObject");

  Video::Image image, imageLarge;
  // The large image for the ProtoObject should be as large as possible
  pPcClient->getRectImage(LEFT, 1280, imageLarge);
  // Segmentation is optimized for 320; this could be scaled down from imageLarge
  pPcClient->getRectImage(LEFT, 320, image);

  // The scale is used to scale the patch and mask calculated on image
  // to the size of imageLarge.
  double inputScale = 1.0 * imageLarge.width / image.width;
  bool protoObj = false; // return value

#if defined(FEAT_VISUALIZATION)
  ostringstream ss;
#endif
#if 1 && defined(FEAT_VISUALIZATION)
  ss << "segmentObject<br>";
  ss << "Image for segmentation: " << image.width << "x" << image.height << "<br>";
  ss << "Image for ProtoObject:" << imageLarge.width << "x" << imageLarge.height << "<br>";
#endif

  try {
    soiPtr->boundingSphere.rad *= DILATE_FACTOR;

    ROIPtr roiPtr = projectSOI(image.camPars, *soiPtr);

    IplImage *iplImg = convertImageToIpl(image);
    //	IplImage *iplImg = cvCreateImage(cvGetSize(iplImgBGR),
    //                          iplImgBGR->depth,
    //                          iplImgBGR->nChannels);

    //	cvCvtColor(iplImgBGR, iplImg, CV_BGR2RGB);	

    CvRect rect;

    rect.width = roiPtr->rect.width;
    rect.height = roiPtr->rect.height;
    rect.x = roiPtr->rect.pos.x - rect.width / 2;
    rect.y = roiPtr->rect.pos.y - rect.height / 2;	

    debug("Calculated ROI x=%i, y=%i, width=%i, height=%i",
       rect.x, rect.y, rect.width, rect.height);
#if 1 && defined(FEAT_VISUALIZATION)
    ss << "Calculated ROI x=" << rect.x << " y=" << rect.y
      << " w=" << rect.width << " h=" << rect.height << "<br>";
#endif

    verifySetRoi(iplImg, rect, __LINE__);

    double patchScale = 1.0; // Strange: = 1.0 * cvGetSize(iplImg).width / cvGetSize(iplImg).height;
    //if ( patchScale > MAX_PATCH_SIZE) patchScale = sqrt(MAX_PATCH_SIZE / patchScale);
    //else patchScale = 1.0;

    CvSize sz = cvGetSize(iplImg); // actually this is the size of ROI(=rect), not of the whole image
    sz.width  *= patchScale;
    sz.height *= patchScale;

    debug("Adjusted ROI size(*patchScale)=%dx%d", sz.width, sz.height);
#if 1 && defined(FEAT_VISUALIZATION)
    ss << "Adjusted ROI: " << " w=" << sz.width << " h=" << sz.height << "<br>";
#endif

    // iplPatch: The patch obtained from the SOI
    IplImage *iplPatch = cvCreateImage(sz, iplImg->depth, iplImg->nChannels);;
    cvResize(iplImg, iplPatch, CV_INTER_LINEAR );

    IplImage *iplPatchHLS = cvCreateImage(sz, IPL_DEPTH_8U, iplPatch->nChannels);
    cvCvtColor(iplPatch, iplPatchHLS, CV_RGB2HLS);

    vector<CvPoint> projPoints, bgProjPoints, errProjPoints;
    vector<int>  hullPoints;    

    //projectSOIPoints(*soiPtr, *roiPtr, projPoints, bgProjPoints, hullPoints, ratio, image.camPars);
    project3DPoints(soiPtr->points, *roiPtr, patchScale, image.camPars, projPoints, hullPoints);
    project3DPoints(soiPtr->BGpoints, *roiPtr, patchScale, image.camPars, bgProjPoints, hullPoints);

    // log("window size %i vs %i vs %i", soiPtr->BGpoints.size(), soiPtr->points.size(), MAX_COLOR_SAMPLE);
    CvSize colSize = cvSize(
        min(
          (int) max(soiPtr->BGpoints.size(), soiPtr->points.size()),
          MAX_COLOR_SAMPLE)
        *COLOR_SAMPLE_IMG_WIDTH,
        COLOR_SAMPLE_IMG_HEIGHT*3);

    colorFiltering = cvCreateImage(colSize,  IPL_DEPTH_8U, 3);
    cvSetZero(colorFiltering);

    //  colorDest = bgColors; //HACK
    filterFlag = false;
    IplImage *bgCostPatch = getCostImage(iplPatchHLS, bgProjPoints, soiPtr->BGpoints, bgHueTolerance, bgDistTolerance, false); 

    //  colorDest = objColors; //HACK
    filterFlag = true;
    IplImage *costPatch = getCostImage(iplPatchHLS, projPoints, soiPtr->points, objHueTolerance, objDistTolerance, true);

    SegmentMask smallSegMask;
    smallSegMask.width  = iplPatch->width;   // == rect.width  * patchScale
    smallSegMask.height = iplPatch->height;  // == rect.height * patchScale

    smallSegMask.data = graphCut(smallSegMask.width, smallSegMask.height, 3, costPatch, bgCostPatch);

    segPoints = filter3DPoints(soiPtr->points, projPoints, errProjPoints, smallSegMask);

    segMask.width  = smallSegMask.width  * (inputScale / patchScale);
    segMask.height = smallSegMask.height * (inputScale / patchScale);

    IplImage *smallIplMask = convertBytesToIpl(smallSegMask.data, smallSegMask.width, smallSegMask.height, 1);
    IplImage *largeIplMask = cvCreateImage( cvSize(segMask.width, segMask.height), IPL_DEPTH_8U, 1);
    cvResize(smallIplMask, largeIplMask, CV_INTER_NN);

    convertIplToBytes(largeIplMask, segMask.data);

#if 1 && defined(FEAT_VISUALIZATION)
    ss << "--- Scale mask ---<br>";
    ss << "Small Patch Size w=" << smallSegMask.width << " h=" << smallSegMask.height << "<br>";
    ss << "Full Patch Size w=" << segMask.width << " h=" << segMask.height << "<br>";
#endif

    //Make the small and large segmentation masks displayable
    for(int y = 0; y < smallIplMask->height; y++) {
      int iRow = y*smallIplMask->widthStep;
      int i = iRow;
      for(int x = 0; x < smallIplMask->width; x++) {
        if(smallIplMask->imageData[i] == 1) protoObj = true;
        smallIplMask->imageData[i] *= 120;
        i++;
      }
    }

    for(int y = 0; y < largeIplMask->height; y++) {
      int iRow = y*largeIplMask->widthStep;
      int i = iRow;
      for(int x = 0; x < largeIplMask->width; x++) {
        largeIplMask->imageData[i] *= 120;
        i++;
      }
    }

    {
      // XXX We assume that inputScale >= 1.0; otherwise the scaling will loose data
      IplImage *iplFull = convertImageToIpl(imageLarge);
      // Calculate rectLarge (in imageLarge) from rect (in image/small)
      CvRect rectLarge;
      rectLarge.x = rect.x * inputScale;
      rectLarge.y = rect.y * inputScale;
      rectLarge.width  = segMask.width;  // == rect.width  * inputScale
      rectLarge.height = segMask.height; // == rect.height * inputScale

      if (pProto.get()) { // (review2010)
        pProto->sourceImageSize.x = imageLarge.width;
        pProto->sourceImageSize.y = imageLarge.height;
        pProto->imageOrigin.x = rectLarge.x;
        pProto->imageOrigin.y = rectLarge.y;
      }

      CvSize sz = cvGetSize(iplFull);
#if 1 && defined(FEAT_VISUALIZATION)
      ss << "--- Scale patch ---<br>";
      ss << "Full Image getSize w=" << sz.width << " h=" << sz.height << "<br>";
      ss << "Full Image ROI x=" << rectLarge.x << " y=" << rectLarge.y
        << " w=" << rectLarge.width << " h=" << rectLarge.height << "<br>";
#endif

      verifySetRoi(iplFull, rectLarge, __LINE__);

      sz = cvGetSize(iplFull);
#if 1 && defined(FEAT_VISUALIZATION)
      ss << "Full Patch Size w=" << sz.width << " h=" << sz.height << "<br>";
#endif
      IplImage *iplPatchFull = cvCreateImage(sz, iplFull->depth, iplFull->nChannels);
      cvResize(iplFull, iplPatchFull, CV_INTER_LINEAR );
      convertImageFromIpl(iplPatchFull, imgPatch);
      cvReleaseImage(&iplFull);
      cvReleaseImage(&iplPatchFull);
    }

    if (doDisplay)
    {
      drawProjectedSOIPoints(iplPatch, projPoints, bgProjPoints, errProjPoints, hullPoints);
      cvResetImageROI(iplImg);
      cvRectangle(iplImg, cvPoint(roiPtr->rect.pos.x-1, roiPtr->rect.pos.y-1),
          cvPoint(roiPtr->rect.pos.x+1, roiPtr->rect.pos.y+1),
          CV_RGB(0,255,0));
      cvRectangle(iplImg, cvPoint(rect.x, rect.y),
          cvPoint(rect.x + rect.width, rect.y + rect.height),
          CV_RGB(0,255,0));

#ifdef FEAT_VISUALIZATION
      if (pDisplay)
      {
        pDisplay->setImage("soif.Full image", iplImg);
        pDisplay->setImage("soif.Color Filtering", colorFiltering);
      }
#else
      cvShowImage("Full image", iplImg);
      cvShowImage("Color Filtering", colorFiltering);
#endif

      CvSize size = cvGetSize(iplPatch);

      cv::Ptr<IplImage> tetraPatch = cvCreateImage(cvSize(size.width*2, size.height*2), IPL_DEPTH_8U, 3);

      verifySetRoi(tetraPatch, cvRect( 0, 0, size.width, size.height), __LINE__ );
      cvCopyImage(iplPatch, tetraPatch);
      verifySetRoi(tetraPatch, cvRect( size.width, 0, size.width, size.height), __LINE__ );
      cvCvtColor(smallIplMask, tetraPatch, CV_GRAY2RGB);
      verifySetRoi(tetraPatch, cvRect( 0, size.height, size.width, size.height), __LINE__ );
      cvCvtColor(costPatch, tetraPatch, CV_GRAY2RGB);
      verifySetRoi(tetraPatch, cvRect( size.width, size.height, size.width, size.height), __LINE__ );
      cvCvtColor(bgCostPatch, tetraPatch, CV_GRAY2RGB);

      cvResetImageROI(tetraPatch);

#ifdef FEAT_VISUALIZATION
      if (pDisplay)
      {
        pDisplay->setImage(ID_OBJ_LAST_SEGMENTATION, tetraPatch);
        pDisplay->setImage("soif.Large segmentation mask", largeIplMask);
      }
#else
      cvShowImage("Last ROI Segmentation", tetraPatch);
#endif

#if 0 // Barry Code
      // temp HACK OUTPUT
      string id =  newDataID();
      string patchName = string("xdata/patch") + id + string(".bmp");
      string segName = string("xdata/segmentation") + id + string(".bmp");
      string listName = string("xdata/points") + id + string(".txt");

      cvSaveImage(patchName.c_str(), iplPatch);
      cvSaveImage(segName.c_str(), segPatch);

      filebuf points;
      points.open(listName.c_str(), ios::out);
      ostream os(&points);

      for(size_t i = 0; i < soiPtr->points.size(); i++)
        os << soiPtr->points[i].p.x << " " << soiPtr->points[i].p.y << " " << soiPtr->points[i].p.z << "\n";

      points.close();
#endif

    }

    // TODO These should be outside of try/catch, or declared as cv::Ptr<IplImage>
    cvReleaseImage(&iplPatch);
    cvReleaseImage(&iplImg);
    cvReleaseImage(&iplPatchHLS);
    cvReleaseImage(&smallIplMask);
    cvReleaseImage(&largeIplMask);
    cvReleaseImage(&costPatch);
    cvReleaseImage(&bgCostPatch);
    cvReleaseImage(&colorFiltering);
  }
  catch (exception& e) {
    error(" *** ERROR in GraphCutSegmenter::segmentObject ***");
    error(" *** exception.what():\n%s\n", e.what());
    error(" ***");
#if defined(FEAT_VISUALIZATION)
    ss << "<br> *** ERROR : <br>" << string(e.what()) << "<br> ***<br>";
#endif
  }

#if 1 && defined(FEAT_VISUALIZATION)
    if (pDisplay)
    {
      pDisplay->setHtml("soif.@debug", "segmentObject.input", ss.str());
    }
#endif

  // XXX: moved to updatedProtoObject
  //if (m_bAutoSnapshot && ! (protoObj == NULL)) {
  //  // XXX: (quick and dirty) assumption: we filled m_LastProtoObject that was passed from runComponent();
  //  saveSnapshot();
  //}

  return protoObj;
}

} // namespace
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */
