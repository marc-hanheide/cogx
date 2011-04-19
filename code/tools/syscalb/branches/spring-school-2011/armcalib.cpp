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

#include <strstream>
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
#include <Golem/Tiny/Tiny.h>

#define PROGRAM_NAME "armcalib"

#define CHESSBOARD_WIDTH 8
#define CHESSBOARD_HEIGHT 6
#define CHESSBOARD_BOXES_WIDTH 0.028
#define CHESSBOARD_BOXES_HEIGHT 0.028

using namespace golem;
using namespace golem::tiny;

static string golemPoseToString(const Mat34 &pose) {
	ostringstream s;
	Vec3 r;
	Real a;
	pose.R.toAngleAxis(a, r);
	r *= a;
	s.precision(3);
	s << "t: " << pose.p.v1 << " " << pose.p.v2 << " " << pose.p.v3 << "\n";
	s << "r: " << r.v1 << " " << r.v2 << " " << r.v3 << "\n";
	s << "R: " << pose.R.m11 << " " << pose.R.m12 << " " << pose.R.m13 << "\n";
	s << "   " << pose.R.m21 << " " << pose.R.m22 << " " << pose.R.m23 << "\n";
	s << "   " << pose.R.m31 << " " << pose.R.m32 << " " << pose.R.m33 << "\n";
	return s.str();
}

static void openCvPoseToGolemPose(const CvMat *tvec, const CvMat *rvec, Mat34 &pose) {
	Vec3 r(Real(cvmGet(rvec, 0, 0)), Real(cvmGet(rvec, 1, 0)), Real(cvmGet(rvec, 2, 0)));
	Real a = r.normalise();
	pose.R.fromAngleAxis(a, r);
  pose.p = Vec3(Real(cvmGet(tvec, 0, 0)), Real(cvmGet(tvec, 1, 0)), Real(cvmGet(tvec, 2, 0)));
}

static void golemPoseToOpenCvPose(const Mat34 &pose, CvMat *tvec, CvMat *rvec) {
	Vec3 r;
	Real a;
	pose.R.toAngleAxis(a, r);
	r *= a;
	cvSet1D(rvec, 0, cvScalar(r.v1));
	cvSet1D(rvec, 1, cvScalar(r.v2));
	cvSet1D(rvec, 2, cvScalar(r.v3));
	cvSet1D(tvec, 0, cvScalar(pose.p.v1));
	cvSet1D(tvec, 1, cvScalar(pose.p.v2));
	cvSet1D(tvec, 2, cvScalar(pose.p.v3));	
}

static void storeOpenCvPose(cv::FileStorage &file, const CvMat *tvec, const CvMat *rvec) {
	CvMat *R = cvCreateMat( 3, 3, CV_64FC1 );
	cvRodrigues2(rvec, R);
	file.writeObj( "tvec", tvec );
	file.writeObj( "rvec", rvec );
	file.writeObj( "rmat", R );
	cvReleaseMat(&R);
}

/**
 * @param pImg opencv image with calibration pattern as it is held by the arm
 * @param camCalibFile intrinsic camera parameter calibration file
 * @param camPoseFile camera pose calbration file containing camera pose w.r.t. world
 *
 * @return pose of gripper w.r.t. world
 */
Mat34 findGripperInWorld(Tiny &tiny, IplImage *pImg, cv::FileStorage &camCalibFile, cv::FileStorage &camPoseFile) {

	CvSize pattern_size = cvSize ( CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT );
	int iNrOfCorners = pattern_size.width * pattern_size.height;
	float fBoardBoxWidth = CHESSBOARD_BOXES_WIDTH;
	float fBoardBoxHeight = CHESSBOARD_BOXES_HEIGHT;	IplImage *pImgGray = cvCreateImage ( cvGetSize ( pImg ), IPL_DEPTH_8U, 1 );

	int pattern_was_found = 0;
	int corner_count = 0;
	std::vector <CvPoint3D64f> object_points;
	std::vector <CvPoint2D64f> image_points;
	std::vector <int> count_points;
	int width = pImg->width;
	int height = pImg->height;
	Mat34 gripper2World;
	gripper2World.setId();

	cvCvtColor ( pImg, pImgGray, CV_BGR2GRAY );

	CvPoint2D32f *pImageCorners = new CvPoint2D32f[iNrOfCorners];

	pattern_was_found = cvFindChessboardCorners ( pImg, pattern_size, pImageCorners, &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

	if ( pattern_was_found ) {
	  tiny.print("calibration pattern OK");
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
	else {
		tiny.print("* Failed to find calibration pattern.");
	}
	delete[] pImageCorners;

	if(pattern_was_found) {
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

		pIntrinsic = (CvMat*)camCalibFile["intrinsic"].readObj();
		pDistortions = (CvMat*)camCalibFile["distortion"].readObj();

		pTraVecCam2World = (CvMat*)camPoseFile["tvec"].readObj();
		pRotVecCam2World = (CvMat*)camPoseFile["rvec"].readObj();

		tiny.print("estimating pose, please wait ...\n");
		cvFindExtrinsicCameraParams2( pObjectPoints, pImagePoints, pIntrinsic, pDistortions,
				pRotVecPattern2Cam, pTraVecPattern2Cam );

		Mat34 pattern2Gripper, gripper2Pattern, pattern2Cam, cam2Pattern, cam2World, pattern2World, gripper2Cam;
		// defined by how the gripper holds the pattern
		pattern2Gripper.setId();
		pattern2Gripper.p.set(-0.126, 0.158, 0.018);
		pattern2Gripper.R.fromAngleAxis(REAL_PI, Vec3(1., 0., 0.));
  	gripper2Pattern.setInverseRT(pattern2Gripper);
		tiny.print("pattern to gripper:\n%s\n", golemPoseToString(pattern2Gripper).c_str());
		tiny.print("gripper to pattern:\n%s\n", golemPoseToString(gripper2Pattern).c_str());

		// this what we get from checkerboard detection
		openCvPoseToGolemPose(pTraVecPattern2Cam, pRotVecPattern2Cam, pattern2Cam);
		// how the camera was caibrated w.r.t. world
		openCvPoseToGolemPose(pTraVecCam2World, pRotVecCam2World, cam2World);

  	cam2Pattern.setInverseRT(pattern2Cam);
		tiny.print("camera to pattern:\n%s\n", golemPoseToString(cam2Pattern).c_str());

		gripper2Cam.multiply(pattern2Cam, gripper2Pattern);
		gripper2World.multiply(cam2World, gripper2Cam);

		tiny.print("pattern to camera:\n%s\n", golemPoseToString(pattern2Cam).c_str());
		tiny.print("camera to world:\n%s\n", golemPoseToString(cam2World).c_str());
		tiny.print("gripper to camera:\n%s\n", golemPoseToString(gripper2Cam).c_str());
		tiny.print("gripper to world:\n%s\n", golemPoseToString(gripper2World).c_str());

		cvReleaseMat ( &pImagePoints );
		cvReleaseMat ( &pObjectPoints );
		cvReleaseMat ( &pIntrinsic );
		cvReleaseMat ( &pDistortions );
		cvReleaseMat ( &pTraVecPattern2Cam );
		cvReleaseMat ( &pRotVecPattern2Cam );
		cvReleaseMat ( &pTraVecCam2Pattern );
		cvReleaseMat ( &pRotVecCam2Pattern );
		cvReleaseImage ( &pImgGray );
	}

	return gripper2World;
}

static golem::tiny::Arm* createArm(Tiny &tiny, const string &armType) {
	//KatanaArmDesc* pArmDesc = new KatanaArmDesc; // specialised Katana 300/450 description
	ArmDesc* pArmDesc = new ArmDesc; // generic description
	pArmDesc->path = armType; // specify driver path
	return dynamic_cast<golem::tiny::Arm*>(tiny.createActor(ActorDescPtr(pArmDesc)));
}

static void attachGripper(golem::tiny::Arm *pArm) {
	// attach a joint to the end-effector (the last joint)
	golem::tiny::Joint* pEffector = pArm->getJoints().back();
	// get the end-effector reference pose
	Mat34 referencePose = pArm->getReferencePose();

	// construct a simple bounding box approximation of the opened gripper,
	// in local end-effector coordinates (arm at reference configuration stretched along Y-axis)
	BoxShapeDesc* pGripperShapeDesc = new BoxShapeDesc;
	pGripperShapeDesc->dimensions.set(Real(0.08), Real(0.07), Real(0.02));
	pGripperShapeDesc->localPose = referencePose;
	pGripperShapeDesc->localPose.p.v2 += Real(0.07);
	Shape* pGripperShape = pEffector->createShape(ShapeDescPtr(pGripperShapeDesc));

	// and change reference pose, so the end-effector pose (TCP) will between the fingers
	const Real tcpOffset = Real(0.11);
	referencePose.p.v2 += tcpOffset;
	pArm->setReferencePose(referencePose);
}

static void setupObstacles(Tiny &tiny) {
	// create ground plane using plane shape
	RigidBodyDesc* pGroundPlaneDesc = new RigidBodyDesc;
	PlaneShapeDesc* pGroundPlaneShapeDesc = new PlaneShapeDesc;
	pGroundPlaneDesc->shapes.push_back(ShapeDescPtr(pGroundPlaneShapeDesc));
	tiny.createActor(ActorDescPtr(pGroundPlaneDesc));

	RigidBodyDesc* pSuperStructureDesc = new RigidBodyDesc;
	BoxShapeDesc* pSuperStructureShapeDesc = new BoxShapeDesc;
	pSuperStructureShapeDesc->dimensions.set(Real(0.20), Real(0.10), Real(0.60));
	pSuperStructureShapeDesc->localPose.p.set(Real(0.00), Real(-0.22), Real(0.60));
	pSuperStructureDesc->shapes.push_back(ShapeDescPtr(pSuperStructureShapeDesc));
	RigidBody* pSuperStructure = dynamic_cast<RigidBody*>(tiny.createActor(ActorDescPtr(pSuperStructureDesc)));
}

static void moveJoints(Tiny &tiny, golem::tiny::Arm *pArm, golem::tiny::GenConfigspaceState &pos, Real duration = 3.0) {
	if(duration <= 0.) {
		duration = 3.0;
	}
	// trajectory
	GenConfigspaceStateSeq trajectory;
	// movement will last no shorter than "duraton" sec
	pos.t = tiny.getTime() + pArm->getTimeDeltaAsync() + duration;
	// compute movement end/target in joint configuration space
	golem::tiny::GenConfigspaceState cbegin = pArm->recvGenConfigspaceState(tiny.getTime());
	golem::tiny::GenConfigspaceState cend = pos;
	// compute trajectory using path planning with collision detection
	trajectory = pArm->findTrajectory(cbegin, cend);
	// move the arm and wait until it stops
	pArm->send(trajectory, numeric_const<double>::INF);
}

static void moveTCP(Tiny &tiny, golem::tiny::Arm *pArm, Mat34 &tcp, Real duration = 3.0) {
	if(duration <= 0.) {
		duration = 3.0;
	}
	// trajectory
	GenConfigspaceStateSeq trajectory;
	// setup target end-effector pose (the joint configuration is not known)
	golem::tiny::GenWorkspaceState target;
	target.pos = tcp;
	// movement will last no shorter than "duraton" sec
	target.t = tiny.getTime() + pArm->getTimeDeltaAsync() + duration;
	// compute movement end/target in joint configuration space
	golem::tiny::GenConfigspaceState cbegin = pArm->recvGenConfigspaceState(tiny.getTime());
	golem::tiny::GenConfigspaceState cend = pArm->findTarget(cbegin, target);
	// compute trajectory using path planning with collision detection
	trajectory = pArm->findTrajectory(cbegin, cend);
	// move the arm and wait until it stops
	pArm->send(trajectory, numeric_const<double>::INF);
}

Mat34 getTCP(Tiny &tiny, golem::tiny::Arm *pArm) {
	golem::tiny::GenWorkspaceState gwss = pArm->recvGenWorkspaceState(tiny.getTime());
	return gwss.pos;
}

CvCapture *setupCapture(Tiny &tiny, int deviceClass, int camId, cv::FileStorage &calib) {
	CvCapture *capture = cvCreateCameraCapture(deviceClass + camId);
	if(!capture) {
		tiny.print("* Failed to initialise capture.");
		return 0;
	}

	CvMat *imgSize = (CvMat*)calib["imgsize"].readObj();

	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, (int)cvGetReal1D(imgSize, 0));
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, (int)cvGetReal1D(imgSize, 1));

	if((int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH) != (int)cvGetReal1D(imgSize, 0) ||
	   (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT) != (int)cvGetReal1D(imgSize, 1)) {
		tiny.print("* Failed to set capture size to values specified in camera calibration.");
		return 0;
	}

	return capture;
}

int main(int argc, char *argv[]) {

	if(argc != 8) {
		printf("usage: %s <golem config> <arm type> <cam calibration> <cam pose> <cam type> <cam id> <out pose>\n", argv[0]);
		printf("  golem config .. golem config file\n"
		       "  arm type .. one of GolemDeviceKatana300Sim, GolemDeviceKatana300, GolemDeviceKatana450\n"
		       "  cam calibration .. camera intrinsic calibration file\n"
		       "  cam pose .. camera pose (extrinsic parameters), w.r.t. world\n"
		       "  cam type .. one of firewire, usb\n"
		       "  cam id .. which camera on the respective bus\n"
		       "  out pose .. output pose file, pose of gripper base w.r.t. world\n");
		printf("example: %s armcalib.xml GolemDeviceKatana300Sim camcalib.xml campose.xml firewire 0 armpose,xml\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	Tiny tiny(argc, argv);

	// note: argv[1] is the golem xml config file, e.g. "armcalib.xml"
	string armType(argv[2]);
	string camCalibFileName(argv[3]);
	string camPoseFileName(argv[4]);
	int camType = CV_CAP_ANY;
	int camId = 0;
	string imgFileName;
	string outPoseFileName(argv[7]);

	if(strcmp(argv[5], "firewire") == 0) {
		camType = CV_CAP_IEEE1394;
		camId = atoi(argv[6]);
	}
	else if(strcmp(argv[5], "usb") == 0) {
		camType = CV_CAP_V4L2;
		camId = atoi(argv[6]);
	}
	else {
		camType = CV_CAP_ANY;
		imgFileName = argv[6];
	}

	cvNamedWindow(PROGRAM_NAME, CV_WINDOW_AUTOSIZE);

	golem::tiny::Arm* pArm = createArm(tiny, armType);
	tiny.print("without gripper to base:\n%s\n", golemPoseToString(getTCP(tiny, pArm)).c_str());
	attachGripper(pArm);
	tiny.print("with gripper to base:\n%s\n", golemPoseToString(getTCP(tiny, pArm)).c_str());
	setupObstacles(tiny);

	cv::FileStorage camCalibFile( camCalibFileName.c_str(), cv::FileStorage::READ );
	cv::FileStorage camPoseFile( camPoseFileName.c_str(), cv::FileStorage::READ );
	CvCapture* capture = 0;

	if(camType != CV_CAP_ANY)
		capture = setupCapture(tiny, camType, camId, camCalibFile);

	// move to target position, show calibration pattern to camera
	Mat34 calibTarget;
	golem::tiny::GenConfigspaceState home;
	calibTarget.R.rotX(REAL_PI/Real(4.0));
	calibTarget.p.set(Real(0.0), Real(0.38), Real(0.68));
	// remember home (= initial) configuration (it is the current joint configuration)
	home = pArm->recvGenConfigspaceState(tiny.getTime());

	tiny.print("To move to calibration position, please press <return> ...");
	tiny.waitKey();

	tiny.print("Moving to calibration, wait for end of movement and press <return> ...");
	moveTCP(tiny, pArm, calibTarget);
	tiny.waitKey();

	Mat34 gripper2Base = getTCP(tiny, pArm);

	IplImage* frame = 0;
	if(capture != 0)
		frame = cvCloneImage(cvQueryFrame( capture ));
	else
		frame = cvLoadImage(imgFileName.c_str(), 1);
	if(frame == 0) {
		tiny.print("* Failed to load imgae '%s'\n", imgFileName.c_str());
		exit(EXIT_FAILURE);
	}

	Mat34 gripper2World = findGripperInWorld(tiny, frame, camCalibFile, camPoseFile);
	Mat34 base2Gripper;
  base2Gripper.setInverseRT(gripper2Base);
	Mat34 base2World;
	base2World.multiply(gripper2World, base2Gripper);

	tiny.print("gripper to world:\n%s\n", golemPoseToString(gripper2World).c_str());
	tiny.print("gripper to base:\n%s\n", golemPoseToString(gripper2Base).c_str());
	tiny.print("base to world:\n%s\n", golemPoseToString(base2World).c_str());
	
	CvMat *tvec = cvCreateMat(3, 1, CV_64FC1);
	CvMat *rvec = cvCreateMat(3, 1, CV_64FC1);
	golemPoseToOpenCvPose(base2World, tvec, rvec);
  cv::FileStorage outPoseFile( outPoseFileName, cv::FileStorage::WRITE );
	storeOpenCvPose(outPoseFile, tvec, rvec);
	outPoseFile.release();
  tiny.print("\nparameters saved to file: %s\n", outPoseFileName.c_str());

  cvShowImage(PROGRAM_NAME, frame);
	cvWaitKey(100);

	tiny.print("Press <return> to return arm to home position...");
	tiny.waitKey();

	tiny.print("Moving to home position, wait for end of movement and press <return> to quit...");
	moveJoints(tiny, pArm, home);
	tiny.waitKey();

	cvReleaseImage(&frame);
	if(capture != 0)
		cvReleaseCapture(&capture);

	exit(EXIT_SUCCESS);
}

