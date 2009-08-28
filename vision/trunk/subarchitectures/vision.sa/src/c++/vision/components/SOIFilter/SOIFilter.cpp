/**
 * @author Alen Vrecko
 * @date July 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "SOIFilter.h"

#define TIME_THR_DEFAULT 500
#define UPD_THR_DEFAULT 4
#define CAM_ID_DEFAULT 0
#define DILATE_FACTOR 1.0  // HACK: Michael Zillich, was 1.5

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

struct gCutData {
	int numLab;
	int k;
	std::list<int> hueList;
	const IplImage *hueImg;
	};

static long smoothFn(int p1, int p2, int l1, int l2)
{
	if ( l1 == 0 && l2==0 ) return(10);
	else return(5);
}

static long dataFn(int p, int l, void *vdata)
{
	long cost;
	gCutData *data = (gCutData *) vdata;
//	int numLab = data->numLab;
	
	if (l==1)
	{
	  int k = data->k;
	  int hue = data->hueImg->imageData[p];
	  list<int> ordsamp = data->hueList;
	  
	  list<int>::iterator ith, itl;
	  
	  for(ith=ordsamp.begin(); hue > *ith && ith != ordsamp.end(); ith++) 
	  {}
	  
	  cost = *ith - hue;
	  itl = ith;
	  int r=0;

	  for(int i = 0; i < k; i++)
	  {
	    if (itl == ordsamp.begin())
	    { 	
	    	cost+=90;
	    	r++;
	    }
	    else
	    {
		  itl--;

		  int diff = hue - *itl;
		  if (diff > 90)
		  	diff = 180 - diff;
//	printf("diff %i \n", diff);	  		
		  cost+= diff;
	  	}
	  }
	  
	  int min_cost = cost;
//printf("label 1 cost %i \n", cost);	  
	  while (cost <= min_cost && ith!=ordsamp.end())
	  {
	  	int diff;
	  	min_cost = cost;
	  	
	  	if (r>0)
	  	{
	  	  cost-=90;
	  	  r--;
	  	}
	  	else
	  	{
	  	  diff = abs(*itl-hue);
	  	  if (diff > 90)
	  		  diff = 180 - diff;
	  		
	      cost-= diff;
	      itl++;
	     }
	    
	    diff = abs(*ith-hue);
	  	if (diff > 90)
	  		diff = 180 - diff;
	  		
	    cost+= diff;
	    
	    ith++;    
	  }
	  
	  cost/= k;
	}
	else
	  cost=10;
	
//	if (l!=0)
//		printf("pixel %i label %i cost: %i \n", p, l, cost);
		
	return(cost); //  ( myData->data[p*numLab+l] );
}


vector<int> SOIFilter::graphCut(int width, int height, int num_labels, list<int> &hueList, const IplImage* hueImg, int k)
{
	int num_pixels = width*height;
	vector<int> result(num_pixels);   // stores result of optimization
	
	log("segment image %ix%i \n", width, height);
	
	list<int>::iterator it;
		printf("HueList: ");	
	for( it=hueList.begin(); it!=hueList.end(); it++)
		printf("%i ", *it);
		
		printf("\n");
	try{
		GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(width, height, num_labels);

		// set up the needed data to pass to function for the data costs
		gCutData toFn;
		toFn.hueList= hueList;
		toFn.numLab = num_labels;
		toFn.hueImg = hueImg;
		toFn.k = k;
log("graphcut check 1 \n");
		gc->setDataCost(&dataFn, &toFn);
log("graphcut check 2 \n");
		// smoothness comes from function pointer
		gc->setSmoothCost(&smoothFn);
log("graphcut check 3 \n");
		printf("\nBefore optimization energy is %d\n",gc->compute_energy());
		gc->expansion(2);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
		printf("\nAfter optimization energy is %d\n",gc->compute_energy());

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

list<int> SOIFilter::getSortedHueList(vector<CvPoint> projPoints, const IplImage* hueImg)
{
	list<int> hueList;

	vector<CvPoint>::iterator it;
	//printf("size %ix%i ", hueImg->width, hueImg->height);
	for(it=projPoints.begin(); it!=projPoints.end(); it++)
	{//printf("point %ix%i ", it->x, it->y); printf("hue %i \n", (int) cvGet2D(hueImg, it->x, it->y).val[0]);
	  if(it->x < hueImg->width && it->y < hueImg->height) //safety check --- imperfect projection might put points outside ROI
	    hueList.push_back(cvGet2D(hueImg, it->x, it->y).val[0]);
	}
	
	hueList.sort();
	
	return hueList;
}


void SOIFilter::configure(const map<string,string> & _config)
{
  configureVideoCommunication(_config);
  configureStereoCommunication(_config);

  map<string,string>::const_iterator it;
  
  updateThr = UPD_THR_DEFAULT;
  timeThr = TIME_THR_DEFAULT;
  camId = CAM_ID_DEFAULT;
  doDisplay = false;
  
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
}

void SOIFilter::start()
{
  startVideoCommunication(*this);
  startStereoCommunication(*this);
  
  if (doDisplay)
  {
  	cvNamedWindow("Last ROI", 1);
  	cvNamedWindow("Full image", 1);
  	cvNamedWindow("Last segmentation", 1);
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
    if(!objToAdd.empty())
    {
      SOIData &soi = SOIMap[objToAdd.front()];
      
      if(soi.status == STABLE)
      { 
        ProtoObjectPtr pobj = new ProtoObject;
        pobj->time = getCASTTime();
        pobj->SOIList.push_back(soi.addr.id);
        segmentObject(soi.addr, pobj->image, pobj->mask);
        
        //pobj->points = getPointCloud();
        // getPoints(pobj->points);
        //log("Object has a cloud of %i points", pobj->points.size());
        
        string objId = newDataID();
        addToWorkingMemory(objId, pobj);
        
        soi.objectTime = getCASTTime();
        soi.status = OBJECT;
        soi.objId = objId;
        
        log("A proto-object added ID %s count %u at %u ",
   			soi.addr.id.c_str(), soi.updCount,
   			soi.stableTime.s, soi.stableTime.us);
   	   }
   	   
   	   objToAdd.pop();
     }
      
  }
  
  if (doDisplay)
  {
  	cvDestroyWindow("Last ROI");
  	cvDestroyWindow("Last segmentation");
  	cvDestroyWindow("Full image");
  }
}

void SOIFilter::newSOI(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::SOIPtr obj =
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
   		soi.addr.id.c_str(), soi.updCount,
   		soi.stableTime.s, soi.stableTime.us
   		);
  	}
  
/*  log("#%u: changed SOI ID %s at %u:%u",
   		soi.updCount,
   		soi.id.c_str(),
   		time.s, time.us);
*/

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

void SOIFilter::projectSOIPoints(const SOI &soi, const ROI &roi, vector<CvPoint> &projPoints,
					vector<CvPoint> &bgProjPoints, vector<int> &hull, const Video::CameraParameters &cam)
{
  size_t n = soi.points.size();
  size_t nbg = soi.BGpoints.size();
  
  int dx = roi.rect.pos.x - roi.rect.width/2;
  int dy = roi.rect.pos.y - roi.rect.height/2;
  
  // calculate foreground points
  for(size_t i = 0; i < n; i++)
  {
    cogx::Math::Vector2 p = projectPoint(cam, soi.points[i]);

    projPoints.push_back(cvPoint(p.x - dx, p.y - dy));
  }
  
   // calculate hull points
//  CvMat pointMat = cvMat( 1, n, CV_32SC2, &projPoints[0] );
//  CvMat hullMat = cvMat( 1, n, CV_32SC1, &hull[0] );
//  cvConvexHull2(&pointMat, &hullMat, CV_CLOCKWISE, 0);

  // calculate background points inside SOI
  for(size_t i = 0; i < nbg; i++)
  {
    cogx::Math::Vector2 p = projectPoint(cam, soi.BGpoints[i]);
    
    bgProjPoints.push_back(cvPoint(p.x - dx, p.y - dy));
  }  
}

// HACK: Michael Zillich
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
// END HACK: Michael Zillich

void SOIFilter::segmentObject(const WorkingMemoryAddress soiAddr, Video::Image &imgPatch, SegmentMaskPtr &segMask)
{
	Video::Image image;
	getImage(camId,image);

	VisionData::SOIPtr soiPtr =
    		getMemoryEntry<VisionData::SOI>(soiAddr);
    		
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
		rect.x, rect.y, rect.width, rect.height);
	
	cvSetImageROI(iplImg, rect);
	
	IplImage *iplPatch = cvCloneImage(iplImg);
	
	IplImage *iplPatchHLS = cvCreateImage(cvGetSize(iplPatch),
                          IPL_DEPTH_8U,
                          iplPatch->nChannels);
    cvCvtColor(iplPatch, iplPatchHLS, CV_RGB2HLS);
   
    cvSetImageCOI(iplPatchHLS, 1);
    IplImage *huePatch = cvCreateImage(cvGetSize(iplPatchHLS), IPL_DEPTH_8U, 1);
    cvCopy(iplPatchHLS, huePatch);
    
    // HACK: Michael Zillic
    
    vector<CvPoint> projPoints, bgProjPoints;
    vector<int>  hullPoints;    
    
    projectSOIPoints(*soiPtr, *roiPtr, projPoints, bgProjPoints, hullPoints, image.camPars);



    list<int> hueList = getSortedHueList(projPoints, huePatch);

    vector<int> labels;
    labels = graphCut(rect.width, rect.height, 2, hueList, huePatch, 3); 
    
//    vector<int>::iterator it;
//		printf("Labels: ");	
//	for( it=labPoints.begin(); it!=labPoints.end(); it++)
//		printf("%i", *it);
//	printf("\n");
    
	segMask = new SegmentMask;
    
    segMask->width = rect.width;
    segMask->height = rect.height;
    segMask->data = labels;

    convertImageFromIpl(iplPatch, imgPatch);
    
    IplImage *segPatch = cvCreateImage(cvGetSize(iplPatchHLS), IPL_DEPTH_8U, 1);

	for(int i=0; i < labels.size(); i++)
	{
		segPatch->imageData[i] = labels[i]*200;
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
    }
    
    cvReleaseImage(&iplPatch);
    cvReleaseImage(&iplImg);
    cvReleaseImage(&iplPatchHLS);
	cvReleaseImage(&iplImgBGR);
	cvReleaseImage(&huePatch);
	cvReleaseImage(&segPatch);
}


}

