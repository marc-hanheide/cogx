/**
 * @author Alen Vrecko
 * @date July 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "SOIFilter.h"

#define TIME_THR_DEFAULT 500
#define UPD_THR_DEFAULT 5
#define CAM_ID_DEFAULT 0
#define DILATE_FACTOR 1.3

// Segmentation costants

#define SMOOTH_COST 10
#define HUE_K_RATIO 25 // Number of nearest neighbours taken when calculating the cost for hue

#define OBJ_HUE_TOLERANCE 15
#define BG_HUE_TOLERANCE 15

#define OBJ_DIST_TOLERANCE 9999
#define BG_DIST_TOLERANCE 9999

#define LABEL_FIX_COST 10

#define MAX_HUE_VAL 180
#define MIN_HUE_VAL 0

#define EXPAND_ITERS 3
#define SWAP_ITERS 2

#define MAX_PATCH_SIZE 90000

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

struct gCutData {
	int numLab;
	int k;
	std::list<int> hueList;
	const IplImage *hueImg;
	};

static long smoothFn(int p1, int p2, int l1, int l2)
{
	if ( l1 != l2 ) return(SMOOTH_COST);
	else if (l1==0) return(0);
	else return 0;
}

vector<int> SOIFilter::graphCut(int width, int height, int num_labels, IplImage* costImg, IplImage* bgCostImg)
{
	int num_pixels = width*height;
	vector<int> result(num_pixels);   // stores result of optimization
	
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
	
		for(int i=0; i<num_pixels; i++) {
			int idx = num_labels *i;
			data[idx] = lblFixCost;
			data[idx + 1] = costImg->imageData[i];
			data[idx + 2] = bgCostImg->imageData[i];
		}
	
		gc->setDataCost(data);

		// smoothness comes from function pointer
		gc->setSmoothCost(&smoothFn);

		log("\nBefore optimization energy is %d\n",gc->compute_energy());
		gc->expansion(EXPAND_ITERS); // run expansion		
		log("\nAfter %i expand iterations the energy is %d\n", EXPAND_ITERS, gc->compute_energy());
		gc->swap(SWAP_ITERS);		 // run swap
		log("\nAfter %i swap iterations the energy is %d\n", SWAP_ITERS, gc->compute_energy());

		for ( int  i = 0; i < num_pixels; i++ )
			result[i] = gc->whatLabel(i);

		delete gc;
	}
	catch (GCException e){
		e.Report();
	}

//	delete [] result;
//	delete [] data;
	
	return result;

}

list<int> SOIFilter::getSortedHueList(vector<SurfacePoint> surfPoints)
{
	list<int> hueList;
	
	vector<SurfacePoint>::iterator it;
	int size = surfPoints.size();
	
	if(size == 0)
	{ 
		log("WARNING: Surface point list empty");
		return hueList;
	}
	
	IplImage* src = cvCreateImage(cvSize(size, 1), IPL_DEPTH_8U, 3);
	IplImage* dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
	IplImage* srcL = cvCreateImage(cvSize(size*10, 10), IPL_DEPTH_8U, 3);
	// IplImage* dstL = cvCreateImage(cvGetSize(srcL), IPL_DEPTH_8U, 3);
	
	for(int i=0; i< size; i++)
	{
	  CvScalar v;
	  //log("red: %i green: %i blue: %i", surfPoints[i].c.r, surfPoints[i].c.g, surfPoints[i].c.b);	  
	  v.val[0] = surfPoints[i].c.b;
	  v.val[1] = surfPoints[i].c.g;
	  v.val[2] = surfPoints[i].c.r;
	  
	  cvSet2D(src, 0, i, v);
	}
	
	cvCvtColor(src, dst, CV_RGB2HLS);
	
	for(int i=0; i< size; i++)
	{
	  hueList.push_back(cvGet2D(dst, 0, i).val[0]);
	  //log("hue: %i", (unsigned) cvGet2D(dst, 0, i).val[0]);
	} 
			
	  //safety check --- points inside calculated ROI might be outside the actual ROI (outside image patch)
//	  if(it->x < hueImg->width && it->y < hueImg->height && it->x >= 0 && it->y >= 0) 
//	    hueList.push_back(cvGet2D(hueImg, it->y, it->x).val[0]);
	
	hueList.sort();
	
//	list<int>::iterator itr;	
//	printf("HueList: ");	
//	for( itr=hueList.begin(); itr!=hueList.end(); itr++)
//		printf("%i ", *itr);
//		printf("\n");
	cvResize(src, srcL, CV_INTER_NN);
	//cvResize(dst, dstL, CV_INTER_NN);
	
	const char *winame = colorImg;
	cvShowImage(winame, srcL);
    //cvShowImage("Obj HLS Colors", dstL);

	cvReleaseImage(&srcL);
	//cvReleaseImage(&dstL);
	cvReleaseImage(&src);
	cvReleaseImage(&dst);
	
	return hueList;
}

vector<int> SOIFilter::getHueCostList(list<int> hueList, int k)
{
	vector<int> hueCostList;
	
	int domainwdt = MAX_HUE_VAL - MIN_HUE_VAL;
	int overflow = domainwdt/2;
	int maxdiff = domainwdt/6;
	
	for(int hue=MIN_HUE_VAL; hue < MAX_HUE_VAL; hue++)
	{	
		long cost, mincost;
		list<int> ordsamp = hueList; 
		list<int>::iterator ith, itl;
		  
		for(ith=ordsamp.begin(); hue > *ith && ith != ordsamp.end(); ith++)
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
			} else {
			    itl--;
				int diff = hue - *itl;
			  	if (diff > overflow)
			  		diff = domainwdt - diff;
									
				if (diff > maxdiff)
					diff = maxdiff;
					 		// printf("diff %i \n", diff);
			  	cost+= diff;	  	
			  	
			  	if(itl == ordsamp.begin())
			  		r++;
		  	}
		} else {
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
		  } else {	    
		  	int diff = abs(*itl-hue);
		  	
		  	if (diff > overflow)
			  	diff = domainwdt - diff;
							
			if (diff > maxdiff)
				diff = maxdiff;
		  		
		  	itl++;	
			cost-= diff;		
		  }	 
		  int diff = abs(*ith-hue);  
		  
		  if (diff > overflow)
			  diff = domainwdt - diff;
							
		  if (diff > maxdiff)
			  diff = maxdiff; //printf("hue: %i diff: %i ", hue, diff);
		  
		  cost+= diff;
		  ith++;    
		}	  
		mincost/=k;
		hueCostList.push_back(mincost);
	}

//	    vector<int>::iterator itr;
//	    int h = 0;
//	    printf("HueCostList: \n");	
//	    for( int i=MIN_HUE_VAL; i < MAX_HUE_VAL; i++)
//	   			printf("h:%i c:%i\n", i, hueCostList[i]);
//	    printf("\n");
	
	return hueCostList;
}


void SOIFilter::configure(const map<string,string> & _config)
{
  configureVideoCommunication(_config);

  map<string,string>::const_iterator it;
  
  updateThr = UPD_THR_DEFAULT;
  timeThr = TIME_THR_DEFAULT;
  camId = CAM_ID_DEFAULT;
  doDisplay = false;
  objHueTolerance = OBJ_HUE_TOLERANCE;
  objDistTolerance = OBJ_DIST_TOLERANCE;
  bgHueTolerance = BG_HUE_TOLERANCE;
  bgDistTolerance = BG_DIST_TOLERANCE;
  lblFixCost = LABEL_FIX_COST;
  
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
}

void SOIFilter::start()
{
  startVideoCommunication(*this);

  char *name = "queueSemaphore";
  named_semaphore(open_or_create, name, 0);
  queuesNotEmpty = new named_semaphore(open_only, name);
  
  if (doDisplay)
  {
  	cvNamedWindow("Last ROI", 1);
  	cvNamedWindow("Full image", 1);
  	cvNamedWindow("Last segmentation", 1);
  	cvNamedWindow("Last object cost image", 1);
  	cvNamedWindow("Last surface cost image", 1);
  	cvNamedWindow("Object Colors", 1);
  	cvNamedWindow("Surface Colors", 1);
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
	      SOIData &soi = SOIMap[objToAdd.front()];
	      
	      if(soi.status == STABLE)
	      {
		    try
		    {
		        SOIPtr soiPtr = getMemoryEntry<VisionData::SOI>(soi.addr);
		        
		        ProtoObjectPtr pobj = new ProtoObject;
		        pobj->time = getCASTTime();pobj->SOIList.push_back(soi.addr.id);
		        segmentObject(soiPtr, pobj->image, pobj->mask);
		
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
	            log("SOI <%s> was removed before it could be processed", soi.addr.id);
		    }
		  }
	   	   
	      objToAdd.pop();
	   }
     }
//     else
//	log("Timeout");   
  }

  log("Removing semaphor..");
  queuesNotEmpty->remove("queueSemaphore");
  delete queuesNotEmpty;
  
  if (doDisplay)
  {
    log("Destroying OpenCV windows..");
  	cvDestroyWindow("Last ROI");
  	cvDestroyWindow("Last segmentation");
  	cvDestroyWindow("Full image");
  	cvDestroyWindow("Last object cost image");
  	cvDestroyWindow("Last surface cost image");
  	cvDestroyWindow("Object Colors");
  	cvDestroyWindow("Surface Colors");
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

	  queuesNotEmpty->post();
  	  
  	  log("An object candidate ID %s count %u at %u ",
   		soi.addr.id.c_str(), soi.updCount,
   		soi.stableTime.s, soi.stableTime.us
   		);
  	}
}

void SOIFilter::deletedSOI(const cdl::WorkingMemoryChange & _wmc)
{
  
  SOIData &soi = SOIMap[_wmc.address.id];
  
  CASTTime time=getCASTTime();
  soi.status= DELETED;
  soi.deleteTime = time;
  objToDelete.push(soi.addr.id);
  
   debug("deleted SOI ID %s count %u at %u:%u",
   		soi.addr.id.c_str(), soi.updCount,
   		time.s, time.us
   		);
   		 
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
    int x = (p.x - roi.rect.pos.x)*ratio + roi.rect.width/2;
    int y = (p.y - roi.rect.pos.y)*ratio + roi.rect.height/2;
	
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
	log("%i hue neighbuours", hueKval);
     
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
	
	vector<int> hueCostList = getHueCostList(getSortedHueList(surfPoints), hueKval);  
 //   list<int> hueList = getSortedHueList(projPoints, huePatch);
 //   vector<int> hueCostList = getHueCostList(hueList, 3);

    float hueSigma2 = 2*sqr(hueSigma);
    float distSigma2 = 2*sqr(distSigma);
    //float factor = 1/sqrt(2*acos(-1.0))/sigma;
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
}


void SOIFilter::segmentObject(const SOIPtr soiPtr, Video::Image &imgPatch, SegmentMaskPtr &segMask)
{
	Video::Image image;
	getImage(camId,image);
    		
   	soiPtr->boundingSphere.rad*=DILATE_FACTOR;
	
	ROIPtr roiPtr = projectSOI(image.camPars, *soiPtr);
	
	IplImage *iplImgBGR = convertImageToIpl(image);
	IplImage *iplImg = cvCreateImage(cvGetSize(iplImgBGR),
                          iplImgBGR->depth,
                          iplImgBGR->nChannels);
    
	cvCvtColor(iplImgBGR, iplImg, CV_BGR2RGB);	

	CvRect rect;
	
	rect.width = roiPtr->rect.width;
	rect.height = roiPtr->rect.height;
	rect.x = roiPtr->rect.pos.x - rect.width/2;
	rect.y = roiPtr->rect.pos.y - rect.height/2;	

	log("Calculated ROI x=%i, y=%i, width=%i, height=%i",
		rect.x, rect.y, roiPtr->rect.width, rect.height);
	
	cvSetImageROI(iplImg, rect);
	
	float ratio = cvGetSize(iplImg).width * cvGetSize(iplImg).height;
	
	if ( ratio > MAX_PATCH_SIZE)
		ratio = sqrt(MAX_PATCH_SIZE / ratio);
	else
		ratio = 1;
	
	IplImage *iplPatch = cvCreateImage(cvSize(cvGetSize(iplImg).width*ratio, 
											  cvGetSize(iplImg).height*ratio),
                          iplImgBGR->depth,
                          iplImgBGR->nChannels);;
	
	cvResize(iplImg, iplPatch, CV_INTER_LINEAR );
	
	IplImage *iplPatchHLS = cvCreateImage(cvGetSize(iplPatch),
                          IPL_DEPTH_8U,
                          iplPatch->nChannels);
  
    cvCvtColor(iplPatch, iplPatchHLS, CV_RGB2HLS);
    
    log("Actual ROI width=%i, height=%i", iplPatchHLS->width, iplPatchHLS->height);
     
    vector<CvPoint> projPoints, bgProjPoints;
    vector<int>  hullPoints;    
   
    //projectSOIPoints(*soiPtr, *roiPtr, projPoints, bgProjPoints, hullPoints, ratio, image.camPars);
    project3DPoints(soiPtr->points, *roiPtr, ratio, image.camPars, projPoints, hullPoints);
    project3DPoints(soiPtr->BGpoints, *roiPtr, ratio, image.camPars, bgProjPoints, hullPoints);

	colorImg = "Object Colors"; //HACK
    IplImage *costPatch = getCostImage(iplPatchHLS, projPoints, soiPtr->points, objHueTolerance, objDistTolerance, true);
    colorImg = "Surface Colors"; //HACK
    IplImage *bgCostPatch = getCostImage(iplPatchHLS, bgProjPoints, soiPtr->BGpoints, bgHueTolerance, bgDistTolerance, false);  	

	segMask = new SegmentMask;
    
    segMask->width = iplPatchHLS->width;
    segMask->height = iplPatchHLS->height;

    vector<int> labels = graphCut(segMask->width, segMask->height, 3, costPatch, bgCostPatch);
    segMask->data = labels;
    convertImageFromIpl(iplPatch, imgPatch);
    
    IplImage *segPatch = cvCreateImage(cvGetSize(iplPatchHLS), IPL_DEPTH_8U, 1);

	for(int i=0; i < labels.size(); i++)
	{
		segPatch->imageData[i] = labels[i]*120;
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
    	
    	cvShowImage("Last ROI", iplPatch);
    	cvShowImage("Last segmentation",segPatch);
    	cvShowImage("Last object cost image", costPatch);
    	cvShowImage("Last surface cost image", bgCostPatch);
    }
    
    cvReleaseImage(&iplPatch);
    cvReleaseImage(&iplImg);
    cvReleaseImage(&iplPatchHLS);
	cvReleaseImage(&iplImgBGR);
	cvReleaseImage(&segPatch);
	cvReleaseImage(&costPatch);
}

}

