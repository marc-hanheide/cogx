#include "VisionTester.hpp"

#include <opencv/highgui.h>
#include <balt/interface/BALTTimer.hpp>

#include <vector>
#include <cast/architecture/ChangeFilterFactory.hpp>

using namespace std;
using namespace boost;
using namespace cast::cdl;

using namespace Vision;

using namespace FrameworkBasics;

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new VisionTester(_id);
  }
}


/** @brief Verifies the content of camera parameters retrieved
 *   from working memory.
 *
 *  Expecting ideal camera parameters, this function 
 *  searches for Vision::Camera in VWM and verifies 
 *  that its parameters match the expected values.
 *  CameraServer must be initialized with ideal.pose 
 *  and ideal.cal, as a left camera.
 */ 
void VisionTester::CameraIdealReadTest::startTest()  // called by run()
{    
    try {
	cout << "*** CameraIdealRead start test ***\n";

	// wait for CameraServer to be ready
	sleep(2);	
	
	vector<shared_ptr<const CASTData<Vision::Camera> > > cams;
	getWorkingMemoryEntries(cams);
	if (cams.size() == 1) {
	  shared_ptr<const Vision::Camera> spCam = cams[0]->getData();
	  if (spCam->m_num != Vision::CAM_LEFT) {
	    testComplete(false);
	  }
	  else {
	    if ((spCam->m_width != 640) ||
		(spCam->m_height != 480) ||
		(spCam->m_fx != 600) ||
		(spCam->m_fy != 600) || 
		(spCam->m_cx != 320) ||
		(spCam->m_cy != 240))
	      testComplete(false);
	    else
	      testComplete(true);
	  }
	  cams.clear();
	}
	else {
	  cams.clear();
	  testComplete(false);
	}
	
	
    } catch (CASTException &e) {
	cerr<<e.what()<<endl;
	testComplete(false); // m_passed = false
    }
}


/** @brief Verifies the ordering of images pulled from VideoServer.
 *
 *  Pull 50 image frames from VideoServer and make sure
 *  the order of images are correct (e.g., by checking timestamp).
 */
void VisionTester::VideoReadTest::startTest() 
{    
    try {
	cout << "*** VideoRead start test ***\n";
	sleep(2); // 2 secs

	BALTTime prev_time;

	for (unsigned i=0; i<10; i++) {
	    Vision::ImageFrame* imgFrame = m_tester.GetImage(Vision::CAM_LEFT);

	    cout << "img:(" << imgFrame->m_time.m_s  
		 << "," << imgFrame->m_time.m_us << ") at ";

	    BALTTime time = BALTTimer::getBALTTime();
	    cout << "t:(" << time.m_s  
		 << "," << time.m_us << ")\n";
	    
	    if (i>0) {
		if (prev_time.m_s > time.m_s)
		    testComplete(false);
		else if (prev_time.m_s == time.m_s) {		    
		    if (prev_time.m_us > time.m_us)
			testComplete(false);
		}
	    }
	    prev_time = time;

	    delete(imgFrame);

	    usleep(1000); // one milli sec
	}
	testComplete(true);

    } catch (CASTException &e) {
	cerr<<e.what()<<endl;     
	testComplete(false);
    }
}

void VisionTester::TrackByRoiTest::HandleSceneObjectEvent(const cdl::WorkingMemoryChange &_wmc) 
{
    static int i=0;
    shared_ptr<const CASTTypedData<SceneObject> > spCastSO =
	getWorkingMemoryEntry<SceneObject>(_wmc.m_address);    
    shared_ptr<const SceneObject> read_scObj = spCastSO->getData();      
    cout << "iteration: " << i 
	 << " , reading SceneObj: (" << read_scObj->m_bbox.m_centroid.m_x
	 << "," << read_scObj->m_bbox.m_centroid.m_y
	 << "," << read_scObj->m_bbox.m_centroid.m_z
	 << ")" << endl;
    i++;
}

/** @brief Tests the tracking initiated by a ROI in working memory.
 *
 *  A ROI is inserted manually into working memory. This tester 
 *  verifies that the roi is tracked correctly.
 */ 
void VisionTester::TrackByRoiTest::startTest() 
{
    try {
	cout << "*** TrackByRoiTest start test ***\n";
	
	addChangeFilter(createLocalTypeFilter<SceneObject>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<TrackByRoiTest>(this, &TrackByRoiTest::HandleSceneObjectEvent));
	
	// wait for CameraServer and VideoServer to be ready
	sleep(2);

	string objid(newDataID());
	Vision::SceneObject scObj;
	scObj.m_time = BALTTimer::getBALTTime();
	scObj.m_bbox.m_centroid.m_x = 0;
	scObj.m_bbox.m_centroid.m_y = 0;
	scObj.m_bbox.m_centroid.m_z = 0;
	scObj.m_bbox.m_size.m_x = 0;
	scObj.m_bbox.m_size.m_y = 0;
	scObj.m_bbox.m_size.m_x = 0;
	scObj.m_bbox.m_size.m_z = 0;
	addToWorkingMemory<SceneObject>(objid, new SceneObject(scObj), 
			   cdl::BLOCKING);

	println("wrote SceneObject to wm");

	string roiid(newDataID());
	Vision::ROI roi; // 147,126 : 170,141
	roi.m_camNum = Vision::CAM_LEFT;
	roi.m_time = BALTTimer::getBALTTime();
	roi.m_objId = objid.c_str();
	roi.m_bbox.m_center.m_x = 158; 
	roi.m_bbox.m_center.m_y = 133;
	roi.m_bbox.m_size.m_x = 23;
	roi.m_bbox.m_size.m_y = 15;
	addToWorkingMemory<ROI>(roiid, new ROI(roi), cdl::BLOCKING);
	
	println("wrote ROI to wm");

	for (unsigned i=0; i<5; i++) {
	    sleep(1);
	}

	testComplete(true);

    } catch (CASTException &e) {
	cerr<<e.what()<<endl;     
	testComplete(false);
    }
}


/** @brief Tests the behaviour of running all basic components together. 
 *  
 *  Tracks and verifies the life-span of objects in the scene.
 */
void VisionTester::AllBasicComponentsTest::startTest()
{
    try {
	cout << "*** AllBasicComponentsTest start test ***\n";
        _sceneObjOVR = 0;
	
	addChangeFilter(createLocalTypeFilter<SceneObject>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<AllBasicComponentsTest>(this, &AllBasicComponentsTest::HandleSceneObjectEvent));
	
	sleep(1);
        
	for (unsigned i=0; i<30 && _sceneObjOVR < 50; i++) {
	    sleep(1);
	}
	
	map<string,LifeSpan>::iterator rit;
	for (rit=objectSet.begin(); rit!=objectSet.end(); rit++) {
	    LifeSpan life = rit->second;
	    cout << "obj: " << rit->first.c_str()
		 << ", s: " << life.start.m_s << ":" << life.start.m_us
		 << ", e: " << life.end.m_s << ":" << life.end.m_us << endl;
	}

	// at least two hand blobs and one object should be tracked.
	// but if video streaming is jumpy, you might get one more.
	if ((objectSet.size()>1) && (objectSet.size()<5))
	    testComplete(true);
	else 
	    testComplete(false);

    } catch (CASTException &e) {
	cerr<<e.what()<<endl;     
	testComplete(false);
    }
        
}

void VisionTester::AllBasicComponentsTest::HandleSceneObjectEvent(const cdl::WorkingMemoryChange &_wmc) 
{
    shared_ptr<const CASTTypedData<SceneObject> > spCastSO =
	getWorkingMemoryEntry<SceneObject>(_wmc.m_address);    
    shared_ptr<const SceneObject> read_scObj = spCastSO->getData();
    unsigned total_rois = read_scObj->m_ROIsMemoryIDs.length();
    
    if (total_rois != 1)
	println("Number of ROIs is not equal 1");

    for (unsigned id=0; id<total_rois; id++) {
	string roi_address(read_scObj->m_ROIsMemoryIDs[id]);	
	map<string,LifeSpan>::iterator rit = objectSet.find(roi_address);
	if (rit == objectSet.end()) {
	    LifeSpan life;
	    life.start = read_scObj->m_time;
	    life.end = read_scObj->m_time;
	    objectSet[roi_address] = life;
	}
	else 
	    objectSet[roi_address].end = read_scObj->m_time;
    }
    
    _sceneObjOVR++;

    /*
    vector<shared_ptr<const CASTData<Vision::ROI> > > allRois;
    getWorkingMemoryEntries(allRois);
    m_numROIs = allRois.size();
    */
}



VisionTester::VisionTester(const string &_id)
    : WorkingMemoryAttachedComponent(_id),
      AbstractVisionTester(_id)  {
}

VisionTester::~VisionTester()
{

} 
 
void VisionTester::configure(std::map<std::string,std::string> & _config) 
{        
  shared_ptr<CameraIdealReadTest> CameraIdealRead_test(new CameraIdealReadTest(*this));
  registerTest("CameraIdealRead_test", CameraIdealRead_test);
  
  shared_ptr<VideoReadTest> VideoRead_test(new VideoReadTest(*this));
  registerTest("VideoRead_test", VideoRead_test);

  shared_ptr<TrackByRoiTest> TrackRoi_test(new TrackByRoiTest(*this));
  registerTest("TrackRoi_test", TrackRoi_test);
  
  shared_ptr<AllBasicComponentsTest> allBasics_test(new AllBasicComponentsTest(*this));
  registerTest("allBasics_test", allBasics_test);

  AbstractVisionTester::configure(_config);    
}

