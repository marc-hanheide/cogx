/** @file armcalib.cpp
 * 
 * Move arm to a clearly visible position, with calibration pattern attached to
 * the gripper. Then take image of calibration pattern and calculate pose of
 * gripper w.r.t. camera, using arm kinematics (known TCP pose) calculate pose
 * of arm base w.r.t. camera. And finally with given camera offset to world
 * origin calcuate arm base pose w.r.t. world. 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 * @author	Markus Bader (Vienna Universityl of Technology)
 * @author	Michael Zillich (Vienna Universityl of Technology)
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Golem/Ctrl/Arm.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Phys/Application.h>
#include <Golem/Phys/Data.h>
#include <Golem/PhysCtrl/PhysReacPlanner.h>
#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Demo/Common/Tools.h>

using namespace golem;

//------------------------------------------------------------------------------

void setupObjects(Scene* scene) {
	// Creator
	Creator creator(*scene);
	Actor::Desc *pActorDesc;
	
	// Create ground plane.
	pActorDesc = creator.createGroundPlaneDesc();
	scene->createObject(*pActorDesc);
	
	// "Base"
	pActorDesc = creator.createBoxDesc(Real(0.20), Real(0.10), Real(0.60));
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(0.00), NxReal(-0.22), NxReal(0.60));
	scene->createObject(*pActorDesc);
}

// create gripper, in open position
void createGripper(std::vector<Bounds::Desc::Ptr> &boundsDescSeq, Mat34 &referencePose, const Mat34 &pose) {
	// finger characteristic dimensions
	const Real tcpOffset = Real(0.11);

	// reference pose at the end (on Y-axis)
	referencePose.setId();
	referencePose.p.v2 += tcpOffset;
	referencePose.multiply(pose, referencePose);
	
	boundsDescSeq.clear();
	
	// model opened gripper as a simple bounding box
	BoundingBox::Desc gripper;
	gripper.dimensions = Vec3(Real(0.08), Real(0.07), Real(0.02));
	gripper.pose.p.v2 += (0.07 - 0.11);
	gripper.pose.multiply(referencePose, gripper.pose);

	boundsDescSeq.push_back(gripper.clone());
}

//------------------------------------------------------------------------------

// Add new bounds to the Joint Actor.
void addBounds(Actor* actor, Bounds::ConstSeq &boundsSeq, const Bounds::Desc::Seq &boundsDescSeq) {
	ASSERT(actor)
	boundsSeq.clear();
	for (Bounds::Desc::Seq::const_iterator i = boundsDescSeq.begin(); i != boundsDescSeq.end(); i++)
		boundsSeq.push_back(actor->createBounds(*i));
}

// Remove bounds of the Joint Actor.
void removeBounds(Actor* actor, Bounds::ConstSeq &boundsSeq) {
	ASSERT(actor)
	for (Bounds::ConstSeq::const_iterator i = boundsSeq.begin(); i != boundsSeq.end(); i++)
		actor->releaseBounds(**i);
}

//------------------------------------------------------------------------------

void setupPlanner(PhysReacPlanner::Desc &desc, XMLContext* xmlcontext, golem::Context* context) {
	// some planner parameter tuning
	desc.pPlannerDesc->pHeuristicDesc->distJointcoordMax[4] = Real(1.0)*REAL_PI;// last joint
	// Disable signal synchronization
	desc.pReacPlannerDesc->signalSync = false;
}

void openCvPoseToGolemPose(CvMat *tvec, CvMat *rvec, Mat34 &pose)
{
	Vec3 r(cvGet1D(rvec, 0), cvGet1D(rvec, 1), cvGet1D(rvec, 2));
	Real a = r.normalise();
	pose.R.fromAngleAxis(a, r);
  pose.p = Vec3(tvec, 0), cvGet1D(tvec, 1), cvGet1D(tvec, 2));
}

//------------------------------------------------------------------------------
Mat34 findPattern(IplImage *pImg, const string &camCalibFile, const string &camPoseFile)
{
	IplImage *pImgGray = cvCreateImage ( cvGetSize ( pImg ), IPL_DEPTH_8U, 1 );
	cvReleaseImage ( &pImg );

	int pattern_was_found = 0;
	int corner_count = 0;
	std::vector <CvPoint3D64f> object_points;
	std::vector <CvPoint2D64f> image_points;
	std::vector <int> count_points;
	int width = pImg->width;
	int height = pImg->height;

	cvCvtColor ( pImg, pImgGray, CV_BGR2GRAY );
	if ( ( pImgGray->width != pImg->width ) || ( pImgGray->height != pImg->height ) ) {
		  std::cout << " images has a different size\n";
		  exit ( 1 );
	}
	CvPoint2D32f *pImageCorners = ( CvPoint2D32f* ) malloc ( sizeof ( CvPoint2D32f ) * iNrOfCorners );
	pattern_was_found = cvFindChessboardCorners ( pImg, pattern_size, pImageCorners, &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
	cvDrawChessboardCorners ( pImg, pattern_size, pImageCorners, corner_count, pattern_was_found );
	if ( pattern_was_found ) {
		  std::cout << " pattern OK\n";
		  cvFindCornerSubPix ( pImgGray, pImageCorners, corner_count, cvSize ( 11,11 ), cvSize ( -1,-1 ), cvTermCriteria ( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ) );
		  for ( int j = 0; j < corner_count; j++ ) {
		      CvPoint2D64f imageCorner = cvPoint2D64f(pImageCorners[j].x,pImageCorners[j].y);
		      image_points.push_back ( imageCorner );
		      CvPoint3D64f worldPoint;
		      worldPoint.x = ( j % pattern_size.width ) * fBoardBoxHeight;
		      worldPoint.y = ( j / pattern_size.width ) * fBoardBoxWidth;
		      worldPoint.z = 0;
		      object_points.push_back ( worldPoint );
		  }
		  count_points.push_back ( iNrOfCorners );

		  cvDrawChessboardCorners ( pImg, pattern_size, pImageCorners, corner_count, pattern_was_found );
	}

	cvWaitKey ( 0 );
	free ( pImageCorners );

	CvMat *pImagePoints = cvCreateMat ( image_points.size(),2,CV_64FC1 );
	CvMat *pObjectPoints = cvCreateMat ( object_points.size(),3,CV_64FC1 );
	CvMat *pTraVecPattern2Cam = cvCreateMat( 3, 1, CV_64FC1 );
	CvMat *pRotVecPattern2Cam = cvCreateMat( 3, 1, CV_64FC1 );
	CvMat *pTraVecCam2Pattern = cvCreateMat( 3, 1, CV_64FC1 );
	CvMat *pRotVecCam2Pattern = cvCreateMat( 3, 1, CV_64FC1 );
	CvMat *pIntrinsic = 0;
	CvMat *pDistortions = 0;
	CvMat *pTraVecCam2World = 0;
	CvMat *pRotVecCam2World = 0;

	for ( unsigned int i = 0; i < image_points.size(); i++ ) {
		  CV_MAT_ELEM ( *pImagePoints, double, i, 0 ) = image_points[i].x;
		  CV_MAT_ELEM ( *pImagePoints, double, i, 1 ) = image_points[i].y;
		  CV_MAT_ELEM ( *pObjectPoints, double, i, 0 ) = object_points[i].x;
		  CV_MAT_ELEM ( *pObjectPoints, double, i, 1 ) = object_points[i].y;
		  CV_MAT_ELEM ( *pObjectPoints, double, i, 2 ) = object_points[i].z;
	}

	cv::FileStorage calibFile( camCalibFile.c_str(), cv::FileStorage::READ );
	pIntrinsic = (CvMat*)calibFile["intrinsic"].readObj();
	pDistortions = (CvMat*)calibFile["distortion"].readObj();

	cv::FileStorage calibFile( camPoseFile.c_str(), cv::FileStorage::READ );
	pTraVecCam2World = (CvMat*)calibFile["tvec"].readObj();
	pRotVecCam2World = (CvMat*)calibFile["rvec"].readObj();

	printf("estimating pose, please wait ...\n");
	cvFindExtrinsicCameraParams2( pObjectPoints, pImagePoints, pIntrinsic, pDistortions,
		  pRotVecPattern2Cam, pTraVecPattern2Cam );

	Mat34 gripper2Pattern, pattern2Cam, cam2World, pattern2World;
  gripper2Pattern.setId();  0.047
  gripper2Pattern.p.set(-0.126, 0.158, 0.);
	gripper2Pattern.R.fromAngleAxis(REAL_PI, Vec3(1., 0., 0.);
  openCvPoseToGolemPose(pTraVecPattern2Cam, pRotVecPattern2Cam, pattern2Cam);
  openCvPoseToGolemPose(pTraVecCam2World, pRotVecCam2World, cam2World);
	

  // invert pose
	invertPose(pTraVecPattern2Cam, pRotVecPattern2Cam, pTraVecCam2Pattern, pRotVecCam2Pattern);
	multPose(pTraVecPattern2World, pRotVecPattern2World, pTraVecCam2Pattern, pRotVecCam2Pattern,
		  pTraVecCam2World, pRotVecCam2World);

	poseFile.writeObj( "tvec", pTraVecCam2World );
	poseFile.writeObj( "rvec", pRotVecCam2World );
	printf("\nparameters saved to file: %s\n", "campose.xml");

	/*double o[6] = {0., 0., 0., 0., 0., 0.}, oi[4];
	CvMat omat = cvMat(1, 6, CV_64FC1, o);
	CvMat oimat = cvMat(1, 4, CV_64FC1, oi);
	cvProjectPoints2(&omat, pRotVecCam2World, pTraVecCam2World, pIntrinsic, pDistortions, &oimat);
	cvCircle(pImg, cvPoint(oi[0], oi[1]), 2, cvScalar(0, 255, 0));
	cvShowImage ( "Camera", pImg );
	cvWaitKey ( 0 );*/

	// HACK
	/*{
	double R[9] = {0, -1,  0,
		            -1,  0,  0,
		             0,  0,  -1};
	double r[3] = {0, 0, 0};
	CvMat Rmat = cvMat(3, 3, CV_64FC1, R);
	CvMat rvec = cvMat(3, 1, CV_64FC1, r);
	cvRodrigues2(&Rmat, &rvec);
	cvPrint ( &rvec, "rvec" );
	}*/

	cvReleaseMat ( &pImagePoints );
	cvReleaseMat ( &pObjectPoints );
	cvReleaseMat ( &pIntrinsic );
	cvReleaseMat ( &pDistortions );
	cvReleaseMat ( &pTraVecPattern2World );
	cvReleaseMat ( &pRotVecPattern2World );
	cvReleaseMat ( &pTraVecPattern2Cam );
	cvReleaseMat ( &pRotVecPattern2Cam );
	cvReleaseMat ( &pTraVecCam2Pattern );
	cvReleaseMat ( &pRotVecCam2Pattern );
	cvReleaseMat ( &pTraVecCam2World );
	cvReleaseMat ( &pRotVecCam2World );
	cvReleaseImage ( &pImg );
	cvReleaseImage ( &pImgGray );
}

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
		printf("Use the arrow keys to move the camera.\n");
		printf("Use the mouse to rotate the camera.\n");
		printf("Press p to pause simulations.\n");
		printf("Press pgup/pgdn/space to switch between simulations.\n");
		printf("Press v to show Actors reference frames.\n");
		printf("Use z, x, c to change randering mode.\n");
		printf("Use F1-F12 to display program specific debug information:\n");
		printf("\tF1 to display/hide the current destination pose.\n");
		printf("\tF2 to display/hide the current trajectory.\n");
		printf("\tF3 to display/hide the goal/desired pose.\n");
		printf("\tF4 to display/hide the global waypoint graph nodes.\n");
		printf("\tF5 to display/hide the global waypoint path.\n");
		printf("\tF6 to display/hide the local waypoint graph nodes.\n");
		printf("\tF7 to display/hide the local waypoint path.\n");
		printf("\tF8 to display/hide the optimised waypoint path.\n");
		printf("Press esc to exit.\n");
		
		// Random number generator seed
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Random number generator seed %d", context()->getRandSeed()._U32[0]);

		// Get arm driver name
		std::string driver;
		XMLData("driver", driver, xmlcontext()->getContextFirst("arm"));
		// Load driver and setup planner
		PhysReacPlanner::Desc physReacPlannerDesc;
		physReacPlannerDesc.pArmDesc = Arm::Desc::load(*context(), driver);
		setupPlanner(physReacPlannerDesc, xmlcontext(), context());

		// Create PhysReacPlanner
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Initialising reactive planner...");
		PhysReacPlanner *pPhysReacPlanner = dynamic_cast<PhysReacPlanner*>(scene()->createObject(physReacPlannerDesc));
		if (pPhysReacPlanner == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create ReacPlanner");
		
		// some useful pointers
		ReacPlanner &reacPlanner = pPhysReacPlanner->getReacPlanner();
		Planner &planner = pPhysReacPlanner->getPlanner();
		Arm &arm = pPhysReacPlanner->getArm();
		
		// Display arm information
		armInfo(arm);

		// Scene objects setup
		setupObjects(scene());

		// Create bounds to be attached to the end-effector (the last joint) 
		Bounds::Desc::Seq boundsDescSeq;
		const Mat34 initReferencePose = arm.getReferencePose();
		Mat34 referencePose;
		createGripper(boundsDescSeq, referencePose, initReferencePose);
		// set new arm reference pose
		arm.setReferencePose(referencePose);
		// Modify shape of the joint by adding new bounds to the Actor representing the end-effector.
		Bounds::ConstSeq boundsSeq;
		addBounds(pPhysReacPlanner->getJointActors().back(), boundsSeq, boundsDescSeq);

		// (Synchronous) Time Delta - a minimum elapsed time between two consecutive waypoints sent to the controller
		SecTmReal timeDelta = reacPlanner.getTimeDelta();
		// Asynchronous Time Delta - a minimum elapsed time between the current time and the first waypoint sent to the controller
		// (no matter what has been sent before)
		SecTmReal timeDeltaAsync = reacPlanner.getTimeDeltaAsync();
		
		// Define the initial pose in the Cartesian workspace
		Vec3 position(Real(0.0), Real(0.43), Real(0.73));
		Vec3 orientation(REAL_PI/Real(4.0), Real(0.0), Real(0.0));
		// and set target waypoint
		golem::GenWorkspaceState target;
		fromCartesianPose(target.pos, position, orientation);
		target.vel.setZero(); // it doesn't move
		target.t = context()->getTimer().elapsed() + timeDeltaAsync + SecTmReal(5.0); // i.e. the movement will last at least 5 sec
		
		// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
		reacPlanner.send(target, ReacPlanner::ACTION_GLOBAL);
		// wait for completion of the action (until the arm moves to the initial pose)
		reacPlanner.waitForEnd();

		context()->getMessageStream()->write(Message::LEVEL_INFO, "Now please attach calibpration pattern and press a key ...");
    getchar();

		CvCapture* capture = 0;
		int device_class = IEEE1394; //CV_CAP_ANY, V4L2, IEEE1394
		int cam = 0;    // camera (device) number
		IplImage* frame = 0;

		capture = cvCreateCameraCapture(device_class + cam);
		if(!capture)
		{
			fprintf(stderr,"Could not initialize capturing...\n");
			exit(1);
		}

		cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
		cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
		printf("image size %.0f x %.0f\n",
				cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH),
				cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT));
		printf("framerate %.2f Hz\n", cvGetCaptureProperty(capture, CV_CAP_PROP_FPS));

		cvNamedWindow("armcalib", 0);
		frame = cvQueryFrame( capture );
		printf("frame size: %d x %d\n", frame->width, frame->height);
		cvResizeWindow("armcalib", frame->width, frame->height);

	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
