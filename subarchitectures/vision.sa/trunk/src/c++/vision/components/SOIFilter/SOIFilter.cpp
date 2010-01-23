/**
 * @author Alen Vrecko
 * @date July 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "SOIFilter.h"

#include <fstream>

#define TIME_THR_DEFAULT 500
#define UPD_THR_DEFAULT 5
#define CAM_ID_DEFAULT 0
#define DILATE_FACTOR 1.3

// Segmentation costants

#define SMOOTH_COST 30
#define HUE_K_RATIO 40 // Number of nearest neighbours taken when calculating the cost for hue

#define OBJ_HUE_TOLERANCE 23
#define BG_HUE_TOLERANCE 24

#define OBJ_DIST_TOLERANCE 60
#define BG_DIST_TOLERANCE 9999

#define LABEL_FIX_COST 24

#define MAX_HUE_VAL 180
#define MIN_HUE_VAL 0

#define EXPAND_ITERS 3
#define SWAP_ITERS 2

#define MAX_PATCH_SIZE 36000

#define MAX_COLOR_SAMPLE 150
#define COLOR_FILTERING_THRESHOLD 10

#define COLOR_SAMPLE_IMG_WIDTH 5
#define COLOR_SAMPLE_IMG_HEIGHT 15

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::SOIFilter();
  }
}

namespace cast
{

using namespace std;
using namespace cdl;
using namespace VisionData;
using namespace Video;

using namespace boost::interprocess;
using namespace boost::posix_time;

//struct gCutData {
//	int numLab;
//	int k;
//	std::list<int>
//	const IplImage *hueImg;
//	};

static long smoothFn(int p1, int p2, int l1, int l2)
{
  if ( l1 != l2 ) return(SMOOTH_COST);
  //	else if (l1==0) return(0);
  else return 0;
}

void SOIFilter::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  // first let the base classes configure themselves
  configureStereoCommunication(_config);

  updateThr = UPD_THR_DEFAULT;
  timeThr = TIME_THR_DEFAULT;
  camId = CAM_ID_DEFAULT;
  doDisplay = false;
  objHueTolerance = OBJ_HUE_TOLERANCE;
  objDistTolerance = OBJ_DIST_TOLERANCE;
  bgHueTolerance = BG_HUE_TOLERANCE;
  bgDistTolerance = BG_DIST_TOLERANCE;
  lblFixCost = LABEL_FIX_COST;
  smoothCost = SMOOTH_COST;

  if((it = _config.find("--upd")) != _config.end())
  {
	istringstream str(it->second);
	str >> updateThr;
  }

  if((it = _config.find("--time")) != _config.end())
  {
	istringstream str(it->second);
	str >> timeThr;
  }
  timeThr*= 1000;

  if((it = _config.find("--videoname")) != _config.end())
  {
	videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
	istringstream str(it->second);
	str >> camId;
  }

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

void SOIFilter::start()
{
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  startStereoCommunication(*this);

  char *name = "filterSemaphore";
  named_semaphore(open_or_create, name, 0);
  queuesNotEmpty = new named_semaphore(open_only, name);

  if (doDisplay)
  {
	cvNamedWindow("Full image", 1);
	cvNamedWindow("Last ROI Segmentation", 1);
	cvNamedWindow("Color Filtering", 1);
  }

  // we want to receive detected SOIs
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::ADD),
	  new MemberFunctionChangeReceiver<SOIFilter>(this,
		&SOIFilter::newSOI));
  // .., when they are updated
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<SOIFilter>(this,
		&SOIFilter::updatedSOI));
  // .. and when they are deleted
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::DELETE),
	  new MemberFunctionChangeReceiver<SOIFilter>(this,
		&SOIFilter::deletedSOI));

}

void SOIFilter::runComponent()
{
  while(isRunning())
  {
	ptime t(second_clock::universal_time() + seconds(2));

	if (queuesNotEmpty->timed_wait(t))
	{
	  log("Got something in my queues");

	  if(!objToAdd.empty())
	  { 

		log("An add proto-object instruction");
		SOIData &soi = SOIMap[objToAdd.front()];

		log("SOI retrieved");

		if(soi.status == STABLE)
		{
		  try
		  {
			SOIPtr soiPtr = getMemoryEntry<VisionData::SOI>(soi.addr);

			ProtoObjectPtr pobj = new ProtoObject;
			pobj->time = getCASTTime();
			pobj->SOIList.push_back(soi.addr.id);
			segmentObject(soiPtr, pobj->image, pobj->mask);
			pobj->points = soiPtr->points;

			string objId = newDataID();
			addToWorkingMemory(objId, pobj);

			soi.objectTime = getCASTTime();
			soi.status = OBJECT;
			soi.objId = objId;

			log("A proto-object added ID %s count %u at %u ",
				soi.addr.id.c_str(), soi.updCount,
				soi.stableTime.s, soi.stableTime.us);
		  }
		  catch (DoesNotExistOnWMException e)
		  {
			log("SOI ID: %s was removed before it could be processed", soi.addr.id);
		  }
		}
		else if(soi.status == DELETED)
		   log("SOI was removed before it could be processed");

		objToAdd.pop();
	  }
	  else if(!objToDelete.empty())
	  {
		log("A delete proto-object instruction");
	    SOIData &soi = SOIMap[objToDelete.front()];

		if(soi.status == OBJECT)
		{
		try
		{
		  deleteFromWorkingMemory(soi.objId);
		
		  log("A proto-object deleted ID: %s ", soi.objId.c_str());
		
		  SOIMap.erase(objToDelete.front());
		  soi.status = DELETED;	
		}
		catch (DoesNotExistOnWMException e)
		{
		log("WARNING: Proto-object ID %s not in WM", soi.objId.c_str());
		}
		}
		else if(soi.status == STABLE)
		{
		  log("Have to wait until the object is processed");
		  objToDelete.push(soi.objId);
		  queuesNotEmpty->post();
		}
		
		objToDelete.pop(); 
	  }
	}
	//    else
	//		log("Timeout");   
  }

  log("Removing semaphore ...");
  queuesNotEmpty->remove("filterSemaphore");
  delete queuesNotEmpty;

  if (doDisplay)
  {
	log("Destroying OpenCV windows..");
	cvDestroyWindow("Full image");
	cvDestroyWindow("Last ROI Segmentation");
	cvDestroyWindow("Color Filtering");
  }
}

void SOIFilter::newSOI(const cdl::WorkingMemoryChange & _wmc)
{
  SOIPtr obj =
	getMemoryEntry<VisionData::SOI>(_wmc.address);

  SOIData data;

  data.addr = _wmc.address;
  data.addTime = obj->time;
  data.updCount = 0;
  data.status = CANDIDATE;

  SOIMap.insert(make_pair(data.addr.id, data));

  debug("A new SOI ID %s ", data.addr.id.c_str());


}

void SOIFilter::updatedSOI(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::SOIPtr obj =
	getMemoryEntry<VisionData::SOI>(_wmc.address);

  SOIData &soi = SOIMap[_wmc.address.id];
  soi.updCount++;

  CASTTime time=getCASTTime();

  if(soi.status == CANDIDATE)
	if(soi.updCount >= updateThr && time.s > soi.addTime.s) // a very rudimental check for now
	{  	  
	  soi.status= STABLE;
	  soi.stableTime = time;
	  objToAdd.push(soi.addr.id);

	  log("An object candidate ID %s count %u at %u ",
		  soi.addr.id.c_str(), soi.updCount, soi.stableTime.s, soi.stableTime.us);

	  queuesNotEmpty->post();
	}
}

void SOIFilter::deletedSOI(const cdl::WorkingMemoryChange & _wmc)
{

  SOIData &soi = SOIMap[_wmc.address.id];

  CASTTime time=getCASTTime();
  soi.deleteTime = time;
  
  if(soi.status == OBJECT || soi.status == STABLE)
  {
  	objToDelete.push(soi.addr.id);
  	queuesNotEmpty->post();
  }
  else
  	soi.status= DELETED;


  debug("Detected SOI deletion ID %s count %u at %u:%u",
  	soi.addr.id.c_str(), soi.updCount, time.s, time.us);		 
}

void SOIFilter::project3DPoints(const vector<SurfacePoint> surfPoints, const ROI &roi, const float ratio,
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


void SOIFilter::drawProjectedSOIPoints(IplImage *img, const vector<CvPoint> projPoints,
	const vector<CvPoint> bgProjPoints, const vector<int> hull)
{
  // draw foreground points
  for(size_t i = 0; i < projPoints.size(); i++)
  {
	cvCircle(img, cvPoint(projPoints[i].x, projPoints[i].y), 3, CV_RGB(0,255,0));
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
	cvCircle(img, cvPoint(bgProjPoints[i].x, bgProjPoints[i].y), 3, CV_RGB(255,0,0));
  }
}


void SOIFilter::drawPoints(IplImage *img, const vector<CvPoint> projPoints)
{
  for(size_t i = 0; i < projPoints.size(); i++)
  {
	cvCircle(img, cvPoint(projPoints[i].x, projPoints[i].y), 3, CV_RGB(0,255,0));
  }
}


void SOIFilter::drawHull(IplImage *img, const vector<CvPoint> projPoints, const vector<int> hull)
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



static bool hslCompare(CvScalar i, CvScalar j)
{
  return (i.val[0] + i.val[2]*0 + i.val[1]*0  <  j.val[0] + j.val[2]*0 + j.val[1]*0);
}


/*
   static int hslAbsDiff(CvScalar i, CvScalar j, float hueOverflow, float norm)
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

   float hlDiff = satAvg*hueDiff*hueDiff + (1-satAvg)*lumDiff*lumDiff;
   float hlsDiff =  pow(satDiff, 3) + (1 - satDiff)*hlDiff;

   if (i.val[1] >200 && j.val[1] > 200)
   {
   printf("huei %f huej %f lumi %f lumj %f sati %f satj %f \n", i.val[0], j.val[0], i.val[1], j.val[1], i.val[2], j.val[2]);
   printf("hueDiff %f lumDiff %f satDiff %f \n", hueDiff, lumDiff,  satDiff);
   printf("satAvg %f hlDiff %f hlsDiff %f \n", satAvg, hlDiff, hlsDiff);
   }

   return (int) (sqrt(hlsDiff) * norm * 3);
   }
   */


static int hslAbsDiff(CvScalar i, CvScalar j, float hueOverflow, float norm)
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




vector<CvScalar> SOIFilter::getSortedHlsList(vector<SurfacePoint> surfPoints)
{
  vector<CvScalar> hlsList;

  vector<SurfacePoint>::iterator it;
  int size = surfPoints.size();

  if(size == 0)
  { 
	log("WARNING: Surface point list empty");
	return hlsList;
  }

  debug("0, size=%d", size);
  IplImage* src = cvCreateImage(cvSize(size, 1), IPL_DEPTH_8U, 3);
  IplImage* dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
  IplImage* srcL = cvCreateImage(cvSize(size*COLOR_SAMPLE_IMG_WIDTH , COLOR_SAMPLE_IMG_HEIGHT ), IPL_DEPTH_8U, 3);
  // IplImage* dstL = cvCreateImage(cvGetSize(srcL), IPL_DEPTH_8U, 3);
  debug("0end");

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
  //	  if(it->x < hueImg->width && it->y < hueImg->height && it->x >= 0 && it->y >= 0) 
  //	    hueList.push_back(cvGet2D(hueImg, it->y, it->x).val[0]);

  //	sort(hlsList.begin(), hlsList.end(), hslCompare);

  //	list<int>::iterator itr;	
  //	printf("HueList: ");	
  //	for( itr=hueList.begin(); itr!=hueList.end(); itr++)
  //		printf("%i ", *itr);
  //		printf("\n");
  cvResize(src, srcL, CV_INTER_NN);
  //cvResize(dst, dstL, CV_INTER_NN);
  
  int pos;
  if (filterFlag)
	pos = 0;
  else
	pos =COLOR_SAMPLE_IMG_HEIGHT ;
	
  cvSetImageROI(colorFiltering, cvRect( 0, pos, srcL->width, srcL->height) );
  cvCopyImage(srcL, colorFiltering);
  cvResetImageROI(colorFiltering);	
  //cvShowImage("Color Filtering", srcL);
  //cvShowImage("Obj HLS Colors", dstL);

  cvReleaseImage(&srcL);
  //cvReleaseImage(&dstL);
  cvReleaseImage(&src);
  cvReleaseImage(&dst);

  return hlsList;
}

/*
   int SOIFilter::getHlsDiff(vector<CvScalar> hlsList, CvScalar hls, int k)
   {
   int domainwdt = MAX_HUE_VAL - MIN_HUE_VAL;
   int overflow = domainwdt/2;
   int maxdiff = domainwdt/6;	

   list<int> costs;

   for(vector<CvScalar>::iterator it=hlsList.begin(); it != hlsList.end(); it++)
   {	
   costs.push_back(hslAbsDiff(hls, *it, overflow, maxdiff));
   }	

   long totalCost = 0;

   for (int i=0; i < k && i < costs.size(); i++)
   {
   list<int>::iterator min = min_element(costs.begin(), costs.end());
   totalCost += *min;
   costs.erase(min);
   }

   return totalCost/k;
   }
   */

int SOIFilter::getHlsDiff(vector<CvScalar> hlsList, CvScalar hls, int k)
{
  int domainwdt = MAX_HUE_VAL - MIN_HUE_VAL;
  int overflow = domainwdt/2;
  int maxdiff = domainwdt/6;	

  list<int> costs;

  costs.assign(k, maxdiff*3);

  list<int>::iterator maxmin = costs.begin();	

  for(vector<CvScalar>::iterator it=hlsList.begin(); it != hlsList.end(); it++)
  {	
	int cost =	hslAbsDiff(hls, *it, overflow, maxdiff);

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
   int SOIFilter::getHlsDiff(vector<CvScalar> hlsList, CvScalar hls, int k)
   {
   int domainwdt = MAX_HUE_VAL - MIN_HUE_VAL;
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
   int diff = hslAbsDiff(hls, *itl, overflow, maxdiff, 0.4, 0.4); //hls.val[0] - itl->val[0];
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
// printf("hue %i cost %i \n ", hue, cost);	  
while (cost <= mincost && ith!=ordsamp.end()) {
mincost = cost;

if (r>0) {

cost-=maxdiff;
r--;
}
else {

int diff = hslAbsDiff(hls, *itl, overflow, maxdiff, 0.4, 0.4); //abs(itl->val[0] - hls.val[0]);  		
itl++;	
cost-= diff;		
}	 
int diff = hslAbsDiff(hls, *itl, overflow, maxdiff, 0.4, 0.4); //abs(ith->val[0] - hls.val[0]); 
cost+= diff;
ith++;

}	  
mincost/=k;

return mincost;
}
/*


vector<int> SOIFilter::getHueDiffList(vector<CvScalar> hlsList, int k)
{
vector<int> hueCostList;

int domainwdt = MAX_HUE_VAL - MIN_HUE_VAL;
int overflow = domainwdt/2;
int maxdiff = domainwdt/6;

for(int hue=MIN_HUE_VAL; hue < MAX_HUE_VAL; hue++)
hueCostList.push_back(getHlsDiff(hlsList, cvScalar(hue, 0, 0), k));

//	    vector<int>::iterator itr;
//	    int h = 0;
//	    printf("HueCostList: \n");	
//	    for( int i=MIN_HUE_VAL; i < MAX_HUE_VAL; i++)
//	   			printf("h:%i c:%i\n", i, hueCostList[i]);
//	    printf("\n");

return hueCostList;
}


/*
IplImage* SOIFilter::getCostImage(IplImage *iplPatchHLS, vector<CvPoint> projPoints, vector<SurfacePoint> surfPoints, float hueSigma, float distSigma, bool distcost)
{
IplImage *huePatch = cvCreateImage(cvGetSize(iplPatchHLS), IPL_DEPTH_8U, 1);  
IplImage *samplePatch = cvCreateImage(cvGetSize(iplPatchHLS), IPL_DEPTH_8U, 1);
IplImage *distPatch = cvCreateImage(cvGetSize(iplPatchHLS), IPL_DEPTH_32F, 1);
IplImage *distScaledPatch = cvCreateImage(cvGetSize(iplPatchHLS), IPL_DEPTH_8U, 1);
IplImage *costImg = cvCreateImage(cvGetSize(iplPatchHLS), IPL_DEPTH_8U, 1);

cvSetImageCOI(iplPatchHLS, 1);
cvCopy(iplPatchHLS, huePatch);

int hueKval = surfPoints.size()/HUE_K_RATIO;
log("%i hue neighbours", hueKval);

if(distcost)
{
cvSet(samplePatch,cvScalar(1));
vector<CvPoint>::iterator itr;

for(itr = projPoints.begin(); itr != projPoints.end(); itr++)
{ 
//safety check --- points inside calculated ROI might be outside the actual ROI (outside image patch)
if(itr->x < samplePatch->width && itr->y < samplePatch->height && itr->x >= 0 && itr->y >= 0)
cvSet2D(samplePatch, itr->y, itr->x, cvScalar(0));
else log("WARNING: point <%i, %i> outside ROI <%i, %i>)", itr->y, itr->x, samplePatch->height, samplePatch->width);

}

cvDistTransform(samplePatch, distPatch, CV_DIST_C);
}
else
cvSet(distScaledPatch, cvScalar(0));

cvConvertScale(distPatch, distScaledPatch, 1, 0);	

vector<int> hueCostList = getHueDiffList(getSortedHlsList(surfPoints), hueKval);  

float hueSigma2 = 2*sqr(hueSigma);
float distSigma2 = 2*sqr(distSigma);
float norm = 100;

for(int i=0; i < costImg->height; i++)
for(int j=0; j < costImg->width; j++) {

int hueDiff = hueCostList[cvGet2D(huePatch, i, j).val[0]];
float hueP = exp(-sqr((float) hueDiff)/hueSigma2);
int distDiff = cvGet2D(distScaledPatch, i, j).val[0];
float distP = exp(-sqr((float) distDiff)/distSigma2);
int cost = (int) ((1 - hueP*distP)*norm);

cvSet2D(costImg, i, j, cvScalar(cost));
}

cvReleaseImage(&huePatch);      
cvReleaseImage(&samplePatch);
cvReleaseImage(&distPatch);      
cvReleaseImage(&distScaledPatch);

return costImg;
}*/


vector<CvScalar> SOIFilter::colorFilter( vector<CvScalar> colors, vector<CvScalar> filterColors, int k)
{

  vector<CvScalar> filteredList;
  int tolerance = COLOR_FILTERING_THRESHOLD;
  for(vector<CvScalar>::iterator it= colors.begin(); it != colors.end(); it++)
  {
	int cost = getHlsDiff(filterColors, *it, k);

	if( cost >= tolerance)
	  filteredList.push_back(*it);
  }

  return filteredList;

}


IplImage* SOIFilter::getCostImage(IplImage *iplPatchHLS, vector<CvPoint> projPoints, vector<SurfacePoint> allSurfPoints, float hueSigma, float distSigma, bool distcost)
{
  IplImage *hlsPatch; // = cvCreateImage(cvGetSize(iplPatchHLS), IPL_DEPTH_8U, 3);
  CvSize size = cvGetSize(iplPatchHLS);
  debug("1, size=%dx%d", size.width, size.height);
  IplImage *samplePatch = cvCreateImage(size, IPL_DEPTH_8U, 1);
  IplImage *distPatch = cvCreateImage(size, IPL_DEPTH_32F, 1);
  IplImage *distScaledPatch = cvCreateImage(size, IPL_DEPTH_8U, 1);
  IplImage *costImg = cvCreateImage(size, IPL_DEPTH_8U, 1);
  cvSet(costImg, cvScalar(255));

  //cvSetImageCOI(iplPatchHLS, 1);
  hlsPatch = cvCloneImage(iplPatchHLS);

  vector<SurfacePoint> surfPoints =  allSurfPoints;

  if(filterFlag) //HACK
  {
	if (surfPoints.size() > MAX_COLOR_SAMPLE)
	  surfPoints.resize(MAX_COLOR_SAMPLE);
  }
  else
	if (surfPoints.size() > MAX_COLOR_SAMPLE*2/3)
	  surfPoints.resize(MAX_COLOR_SAMPLE*2/3);


  int colorKval = surfPoints.size()/HUE_K_RATIO + 1;

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


  vector<CvScalar> sortHlsList = getSortedHlsList(surfPoints); 

  if (!filterFlag)			//HACK
	filterList = sortHlsList;
  else
  {	
	int size = sortHlsList.size();
	int k = filterList.size()/HUE_K_RATIO;
	if (k < 1) return costImg;
	sortHlsList = colorFilter(sortHlsList, filterList, k);

	colorKval = sortHlsList.size()/HUE_K_RATIO + 1;

	log("Filtered out %i color samples", size - sortHlsList.size());

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
	  cvSetImageROI(colorFiltering, cvRect( 0, COLOR_SAMPLE_IMG_HEIGHT*2, dstL->width, dstL->height) );
	  cvCopyImage(dstL, colorFiltering);
	  cvResetImageROI(colorFiltering);	

	  cvReleaseImage(&dst);
	  cvReleaseImage(&dstL);
	  cvReleaseImage(&src);
	}
  }

  log("%i color neighbours", colorKval);

  float hueSigma2 = 2*sqr(hueSigma);
  float distSigma2 = 2*sqr(distSigma);
  float norm = 100;

  for(int i=0; i < costImg->height; i++)
	for(int j=0; j < costImg->width; j++) {

	  int hueDiff = getHlsDiff(sortHlsList, cvGet2D(hlsPatch, i, j), colorKval);
	  float hueP = exp(-sqr((float) hueDiff)/hueSigma2);
	  int distDiff = cvGet2D(distScaledPatch, i, j).val[0];
	  float distP = exp(-sqr((float) distDiff)/distSigma2);
	  int cost = (int) ((1 - hueP*distP)*norm);

	  cvSet2D(costImg, i, j, cvScalar(cost));
	}

  cvReleaseImage(&hlsPatch);      
  cvReleaseImage(&samplePatch);
  cvReleaseImage(&distPatch);      
  cvReleaseImage(&distScaledPatch);

  return costImg;
}



vector<unsigned char> SOIFilter::graphCut(int width, int height, int num_labels, IplImage* costImg, IplImage* bgCostImg)
{
  int num_pixels = width*height;
  vector<unsigned char> result(num_pixels);   // stores result of optimization

  log("Segment image %ix%i \n", width, height);

  try{
	GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(width, height, num_labels);

	// set up the needed data to pass to function for the data costs

	//gCutData toFn;
	//toFn.hueList= hueList;
	//toFn.numLab = num_labels;
	//toFn.hueImg = hueImg;
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


	log("Before optimization energy is %d\n",gc->compute_energy());

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
	log("After %i expand iterations the energy is %d\n", iter, gc->compute_energy());

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

void SOIFilter::segmentObject(const SOIPtr soiPtr, Video::Image &imgPatch, SegmentMask &segMask)
{
  Video::Image image;
  getRectImage(LEFT, image);

  soiPtr->boundingSphere.rad*=DILATE_FACTOR;

  ROIPtr roiPtr = projectSOI(image.camPars, *soiPtr);

  IplImage *iplImg = convertImageToIpl(image);
  //	IplImage *iplImg = cvCreateImage(cvGetSize(iplImgBGR),
  //                          iplImgBGR->depth,
  //                          iplImgBGR->nChannels);

  //	cvCvtColor(iplImgBGR, iplImg, CV_BGR2RGB);	

  CvRect rect;

  rect.width = roiPtr->rect.width;
  rect.height = roiPtr->rect.height;
  rect.x = roiPtr->rect.pos.x - rect.width/2;
  rect.y = roiPtr->rect.pos.y - rect.height/2;	

  log("Calculated ROI x=%i, y=%i, width=%i, height=%i",
	  rect.x, rect.y, rect.width, rect.height);

  cvSetImageROI(iplImg, rect);

  float ratio = cvGetSize(iplImg).width * cvGetSize(iplImg).height;

  //if ( ratio > MAX_PATCH_SIZE)
  //  ratio = sqrt(MAX_PATCH_SIZE / ratio);
  //else
	ratio = 1;

  CvSize sz = cvGetSize(iplImg);
  sz.width *= ratio;
  sz.height *= ratio;
  debug("size(*ratio)=%dx%d", sz.width, sz.height);
  IplImage *iplPatch = cvCreateImage(sz, iplImg->depth, iplImg->nChannels);;

  cvResize(iplImg, iplPatch, CV_INTER_LINEAR );

  IplImage *iplPatchHLS = cvCreateImage(sz, IPL_DEPTH_8U, iplPatch->nChannels);

  cvCvtColor(iplPatch, iplPatchHLS, CV_RGB2HLS);

  log("Actual ROI width=%i, height=%i", iplPatchHLS->width, iplPatchHLS->height);

  vector<CvPoint> projPoints, bgProjPoints;
  vector<int>  hullPoints;    

  //projectSOIPoints(*soiPtr, *roiPtr, projPoints, bgProjPoints, hullPoints, ratio, image.camPars);
  project3DPoints(soiPtr->points, *roiPtr, ratio, image.camPars, projPoints, hullPoints);
  project3DPoints(soiPtr->BGpoints, *roiPtr, ratio, image.camPars, bgProjPoints, hullPoints);
  
  log("window size %i vs %i vs %i", soiPtr->BGpoints.size(), soiPtr->points.size(), MAX_COLOR_SAMPLE);
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

  segMask.width = iplPatchHLS->width;
  segMask.height = iplPatchHLS->height;

  segMask.data =  graphCut(segMask.width, segMask.height, 3, costPatch, bgCostPatch);
  convertImageFromIpl(iplPatch, imgPatch);

  sz = cvGetSize(iplPatchHLS);
  IplImage *segPatch = cvCreateImage(sz, IPL_DEPTH_8U, 1);

  // TODO: possible error: lines of IplImage are aligned to 4 bytes, while Image has no alignment!
  for(int i=0; i < segMask.data.size(); i++)
  {
	segPatch->imageData[i] = segMask.data[i]*120;
  }
  
  int idx = 0;
  for (int i = 0; i < segMask.height; i++) {
	int lineoffs = i * segPatch->widthStep;
	for (int j = 0; j < segMask.width; j++) {
	  segPatch->imageData[lineoffs + j] = segMask.data[idx++]*120;
	}
  }
  

  if (doDisplay)
  {
	drawProjectedSOIPoints(iplPatch, projPoints, bgProjPoints, hullPoints);
	cvResetImageROI(iplImg);
	cvRectangle(iplImg, cvPoint(roiPtr->rect.pos.x-1, roiPtr->rect.pos.y-1),
		cvPoint(roiPtr->rect.pos.x+1, roiPtr->rect.pos.y+1),
		CV_RGB(0,255,0));
	cvRectangle(iplImg, cvPoint(rect.x, rect.y),
		cvPoint(rect.x + rect.width, rect.y + rect.height),
		CV_RGB(0,255,0));
	cvShowImage("Full image", iplImg);
	
	cvShowImage("Color Filtering", colorFiltering);
	
	CvSize size = cvGetSize(iplPatch);
	
	IplImage *tetraPatch = cvCreateImage(cvSize(size.width*2, size.height*2), IPL_DEPTH_8U, 3);
	
	cvSetImageROI(tetraPatch, cvRect( 0, 0, size.width, size.height) );
	cvCopyImage(iplPatch, tetraPatch);
	cvSetImageROI(tetraPatch, cvRect( size.width, 0, size.width, size.height) );
	cvCvtColor(segPatch, tetraPatch, CV_GRAY2RGB);
	cvSetImageROI(tetraPatch, cvRect( 0, size.height, size.width, size.height) );
	cvCvtColor(costPatch, tetraPatch, CV_GRAY2RGB);
	cvSetImageROI(tetraPatch, cvRect( size.width, size.height, size.width, size.height) );
	cvCvtColor(bgCostPatch, tetraPatch, CV_GRAY2RGB);
	
	cvResetImageROI(tetraPatch);
	
	cvShowImage("Last ROI Segmentation", tetraPatch);
	
	cvReleaseImage(&tetraPatch);


#if 0 // Barry Code
	// temp HACK OUTPUT
	string id =  newDataID();
	string patchName = string("xdata/patch") + id + string(".bmp");
	string segName = string("xdata/segmentation") + id + string(".bmp");
	string listName = string("xdata/points") + id + string(".txt");

	cvSaveImage(patchName.c_str(), iplPatch)
	cvSaveImage(segName.c_str(), segPatch);

	filebuf points;
	points.open(listName.c_str(), ios::out);
	ostream os(&points);

	for(size_t i = 0; i < soiPtr->points.size(); i++)
	  os << soiPtr->points[i].p.x << " " << soiPtr->points[i].p.y << " " << soiPtr->points[i].p.z << "\n";

	points.close();
#endif

  }

  cvReleaseImage(&iplPatch);
  cvReleaseImage(&iplImg);
  cvReleaseImage(&iplPatchHLS);
  //	cvReleaseImage(&iplImgBGR);
  //	cvReleaseImage(&segPatch);
  cvReleaseImage(&costPatch);
  cvReleaseImage(&bgCostPatch);
  cvReleaseImage(&colorFiltering);
}

}
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim */

