/**
 * @file StereoFlapDetector.cpp
 * @author Andreas Richtsfeld
 * @date September 2009
 * @version 0.1
 * @brief Detecting flaps with stereo for cogx implementation in cast.
 */


#include <cast/architecture/ChangeFilterFactory.hpp>

#include <opencv/highgui.h>
#include "StereoFlapDetector.h"


// using namespace Z;
using namespace std;
using namespace VisionData;

// using namespace cogx::Math;

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::StereoFlapDetector();
  }
}

namespace cast
{

void StereoFlapDetector::receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc)
{
	StereoFlapDetectionCommandPtr detect_cmd = getMemoryEntry<StereoFlapDetectionCommand>(_wmc.address);
	
	log("received detection command ...");
	switch(detect_cmd->cmd){
		case VisionData::SFSTART:
			if(cmd_detect){
				log("stereo flap detection is allready started.");
			}else{
				log("starting stereo flap detection.");
        // start receiving images pushed by the video server
        videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
				cmd_detect = true;
			}
			break;
		case VisionData::SFSTOP:
			if(cmd_detect){
				log("stopping stereo flap detection.");
				cmd_detect = false;
        videoServer->stopReceiveImages(getComponentID().c_str());
			}else{
				log("stereo flap detection is allready stopped.");
			}
			break;
		case VisionData::SFSINGLE:
			if(!cmd_single){
				log("single stereo flap detection command received.");
				cmd_single = true;
				videoServer->getImage(camIds[0], image_l);
				videoServer->getImage(camIds[1], image_r);
				processImage(image_l, image_r);
				cmd_single = false;
			}else{
				log("already running single stereo flap detection, too fast triggering.");
			}
			break;
		default:
			log("unknown detection command received, doing nothing");
			break;
	}	
}


void StereoFlapDetector::configure(const map<string,string> & _config)
{
	// create stereo core
	score = new Z::StereoCore("subarchitectures/vision.sa/config/tuw_stereo_new.ini");

// 	log("VisionSystem3::configure: Running configuration");
  map<string,string>::const_iterator it;
 
  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }

	showStereoImage = true;
	cmd_detect = false;
}


void StereoFlapDetector::start()
{
	log("StereoFlapDetector::start: Start Component");

  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

	// add change filter for commands
  addChangeFilter(createLocalTypeFilter<StereoFlapDetectionCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoFlapDetector>(this, &StereoFlapDetector::receiveDetectionCommand));

	// define openCV displays
  if(showStereoImage) 
	{
		cvNamedWindow("Stereo left", /*CV_WINDOW_AUTOSIZE*/ 1);
		cvNamedWindow("Stereo right", /*CV_WINDOW_AUTOSIZE*/ 1);
		cvMoveWindow("Stereo left", 1000, 100);
		cvMoveWindow("Stereo right", 1420, 100);
	}
}

void StereoFlapDetector::receiveImages(const std::vector<Video::Image>& images)
{
  if(images.size() <= 1)
    throw runtime_error(exceptionMessage(__HERE__, "image list too short: stereo image expected."));
  // process a single image
  if (cmd_detect) processImage(images[0], images[1]);
}

void StereoFlapDetector::runComponent()
{
}


void StereoFlapDetector::processImage(const Video::Image &image_l, const Video::Image &image_r)
{
	static unsigned frame_counter = 0;
	frame_counter++;
	static unsigned num_flaps = 0;

	log("process image");

	IplImage *iplImage_l = convertImageToIpl(image_l);
	IplImage *iplImage_r = convertImageToIpl(image_r);

	int runtime = 1600;
	score->SetIplImages(iplImage_l, iplImage_r);
	score->ProcessStereoImage(runtime/2);

	// get flaps
	if(score->NumFlaps() > 0)
	{
		// create trackers for all detected flaps
		for(int i = 0; i < score->NumFlaps(); i++)
		{
			// write flaps as visual object to working memory
			Z::Flap3D f = score->Flaps(i);

/// print vertices
// printf("\nVertex[0]: %4.3f / %4.3f / %4.3f\n", f.surf[0].vertices[0].p.x, f.surf[0].vertices[0].p.y, f.surf[0].vertices[0].p.z);
// printf("Vertex[0]: %4.3f / %4.3f / %4.3f\n", f.surf[0].vertices[1].p.x, f.surf[0].vertices[1].p.y, f.surf[0].vertices[1].p.z);
// printf("Vertex[0]: %4.3f / %4.3f / %4.3f\n", f.surf[0].vertices[2].p.x, f.surf[0].vertices[2].p.y, f.surf[0].vertices[2].p.z);
// printf("Vertex[0]: %4.3f / %4.3f / %4.3f\n\n", f.surf[0].vertices[3].p.x, f.surf[0].vertices[3].p.y, f.surf[0].vertices[3].p.z);
// 
// printf("Vertex[1]: %4.3f / %4.3f / %4.3f\n", f.surf[1].vertices[0].p.x, f.surf[1].vertices[0].p.y, f.surf[1].vertices[0].p.z);
// printf("Vertex[1]: %4.3f / %4.3f / %4.3f\n", f.surf[1].vertices[1].p.x, f.surf[1].vertices[1].p.y, f.surf[1].vertices[1].p.z);
// printf("Vertex[1]: %4.3f / %4.3f / %4.3f\n", f.surf[1].vertices[2].p.x, f.surf[1].vertices[2].p.y, f.surf[1].vertices[2].p.z);
// printf("Vertex[1]: %4.3f / %4.3f / %4.3f\n", f.surf[1].vertices[3].p.x, f.surf[1].vertices[3].p.y, f.surf[1].vertices[3].p.z);

			VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
			Flap2VisualObject(obj, f);

/// print object model details:
// printf("	Position: %4.3f / %4.3f / %4.3f\n", obj->pose.pos.x, obj->pose.pos.y, obj->pose.pos.z);
// printf("\nVertex[0]: %4.3f / %4.3f / %4.3f\n", obj->model->vertices[0].pos.x, obj->model->vertices[0].pos.y, obj->model->vertices[0].pos.z);
// printf("Vertex[1]: %4.3f / %4.3f / %4.3f\n", obj->model->vertices[1].pos.x, obj->model->vertices[1].pos.y, obj->model->vertices[1].pos.z);
// printf("Vertex[2]: %4.3f / %4.3f / %4.3f\n", obj->model->vertices[2].pos.x, obj->model->vertices[2].pos.y, obj->model->vertices[2].pos.z);
// printf("Vertex[3]: %4.3f / %4.3f / %4.3f\n\n", obj->model->vertices[3].pos.x, obj->model->vertices[3].pos.y, obj->model->vertices[3].pos.z);
// 
// printf("Vertex[4]: %4.3f / %4.3f / %4.3f\n", obj->model->vertices[4].pos.x, obj->model->vertices[4].pos.y, obj->model->vertices[4].pos.z);
// printf("Vertex[5]: %4.3f / %4.3f / %4.3f\n", obj->model->vertices[5].pos.x, obj->model->vertices[5].pos.y, obj->model->vertices[5].pos.z);
// printf("Vertex[6]: %4.3f / %4.3f / %4.3f\n", obj->model->vertices[6].pos.x, obj->model->vertices[6].pos.y, obj->model->vertices[6].pos.z);
// printf("Vertex[7]: %4.3f / %4.3f / %4.3f\n", obj->model->vertices[7].pos.x, obj->model->vertices[7].pos.y, obj->model->vertices[7].pos.z);

			// label object
			char obj_label[32];
			sprintf(obj_label, "Flap %d", num_flaps);
			obj->label = obj_label;
			num_flaps++;

			// add visual object to working memory
			addToWorkingMemory(newDataID(), obj);
			log("New flap at frame number %u: added visual object to working memory: %s", frame_counter, obj->label.c_str());
		}
	}

	/// clear results
	score->ClearResults();

// 	vs3Interface->ProcessSingleImage(iplImage);
// 
// 	// ----------------------------------------------------------------------------
// 	// Get objects after processing and create visual object for working memory
// 	// ----------------------------------------------------------------------------
// 
// 	// get all cubes and create visual object as working memory entry 
// 	if(getCubes)
// 	{
// 		Z::CubeDef cd;				// cube definition
// 
// 		while(vs3Interface->GetCube(number, cd, masked))
// 		{
// 			number++;
// 			if(!masked)
// 			{
// 				bool cubeExists = false;
// 	
// 				// read visual objects from working memory and assign already detected cubes
// 				std::vector <VisionData::VisualObjectPtr> results;
// 				getMemoryEntries<VisionData::VisualObject>(results);
// 	
// 				for(unsigned i=0; i<results.size(); i++)
// 				{
// 					// read vertices and calculate cube center points and radius
// 					vector<Vertex> vertices = results[i]->model->vertices;
// 	
// 					Vector3 newCubeCenter;
// 					newCubeCenter.x = cd.cubeCenter3D.x;
// 					newCubeCenter.y = cd.cubeCenter3D.y;
// 					newCubeCenter.z = cd.height/2.;
// 	
// 					double radius = length(vertices[0].pos);
// 	
// 					if(length(results[i]->pose.pos - newCubeCenter) < radius)
// 						cubeExists = true;
// 				}
// 	
// 				// Create visual object, if it does not already exists in the working memory
// 				if(!cubeExists)
// 				{
// 					num_cubes++;
// 					VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
// 					if(Cube2VisualObject(obj, cd))
// 					{
// 						char obj_label[32];
// 						sprintf(obj_label, "Cube %d", num_cubes);
// 						obj->label = obj_label;
// 	
// 						// Add VisualObject to working memory
// 						addToWorkingMemory(newDataID(), obj);
// 						log("new cube at frame number %u: added visual object to working memory: %s", frame_counter, obj->label.c_str());
// 					}
// 				}
// 			}
// 		}
// 	}
// 
// 
// 	// --------------------------------------------------------------------
// 	// Get input from the openCV image
// 	// -------------------------------------------------------------------
// 	int key = 0;
// 	key = cvWaitKey(10);
// 		if(key==1048603) return;									// return for escape
// 
// 	switch(key){
// 		case 1048619: detail++;										// Key '+'
// 			log("Detail switched to %i", detail);
// 			break;
// 		case 1048621: if(detail>0) detail--;			// Key '-'
// 			log("Detail switched to %i", detail);
// 			break;
// 		case 1048670: type = 29;									// OBJECT == ^
// 			log("Switched to OBJECT");
// 			break;
// 		case 1048625: type = 20;									// CUBE == 1
// 			log("Switched to CUBE");
// 			break;
// 		case 1048626: type = 12;									// CYLINDER == 2
// 			log("Switched to CYLINDER");
// 			break;
// 		case 1048627: type = 13;									// CONE == 3
// 			log("Switched to CONE");
// 			break;
// 		case 1048628: type = 6;										// BALL == 4
// 			log("Switched to BALL");
// 			break;
// 		case 1048629: type = 5;										// ELLIPSE == 5
// 			log("Switched to ELLIPSE");
// 			break;
// 		case 1048630: type = 0;										// SEGMENT == 6
// 			log("Switched to SEGMENT");
// 			break;
// 		case 1048631: type = 1;										// LINE == 7
// 			log("Switched to LINE");
// 			break;
// 		case 1048632: type = 17;									// RECTANGLE == 8
// 			log("Switched to RECTANGLE");
// 			break;
// 		case 1048633: type = 19;									// FLAP == 9
// 			log("Switched to FLAP");
// 			break;
// 		case 1048676: 														// Enable/Disable display == D
// 			if(showImage) showImage = false;
// 			else showImage = true;
// 			log("Enable/Disable object detector window");
// 			break;
// 		default: break;
// 	}

	// ----------------------------------------------------------------
	// Draw Gestalts to IplImage for the openCv window
	// ----------------------------------------------------------------
	if(showStereoImage) 
	{	
		// Convert from RGB to BGR
		cvConvertImage( iplImage_l, iplImage_l, CV_CVTIMG_SWAP_RB);
		cvConvertImage( iplImage_r, iplImage_r, CV_CVTIMG_SWAP_RB);

		cvShowImage("Stereo left", iplImage_l);
		cvShowImage("Stereo right", iplImage_r);

		cvWaitKey(10);

		cvReleaseImage(&iplImage_l);
		cvReleaseImage(&iplImage_r);
	}
}


/**
 * @brief Convert cube from object detector to working memory visual object
 * @param obj Visual Object
 * @param cd Cube
 * @return True for success
 */
bool StereoFlapDetector::Flap2VisualObject(VisionData::VisualObjectPtr &obj, Z::Flap3D &flap)
{
	obj->model = new VisionData::GeometryModel;

	// Recalculate pose of vertices (relative to the pose of the flap == cog)
	Pose3 pose;
	DefineFlapCoordsys(flap, pose);

	// add center point to the model
	cogx::Math::Pose3 cogxPose;
	cogxPose.pos.x = pose.pos.x;
	cogxPose.pos.y = pose.pos.y;
	cogxPose.pos.z = pose.pos.z;
	obj->pose = cogxPose;

	// create vertices (relative to the 3D center point)
	for(unsigned i=0; i<=1; i++)
	{
		for(unsigned j=0; j<=3; j++)
		{
			Vertex v;
			v.pos.x = flap.surf[i].vertices[j].p.x;
			v.pos.y = flap.surf[i].vertices[j].p.y;
			v.pos.z = flap.surf[i].vertices[j].p.z;
			obj->model->vertices.push_back(v);
		}
	}

// 	// add faces to the vision model
	Face f;
	f.vertices.push_back(0);									// first rectangle
	f.vertices.push_back(1);
	f.vertices.push_back(2);
	f.vertices.push_back(3);
	obj->model->faces.push_back(f);
	f.vertices.clear();

	f.vertices.push_back(4);									// second rectangle
	f.vertices.push_back(5);
	f.vertices.push_back(6);
	f.vertices.push_back(7);
	obj->model->faces.push_back(f);
	f.vertices.clear();

	obj->detectionConfidence = 1.0;						// detection confidence is always 1

	return true;
}



/**
 * @brief Extract camera parameters from video server.
 * @param image Image from the image server with the camera paramters.
 * @return True for success
 */
bool StereoFlapDetector::GetCameraParameter(const Video::Image & image)
{
	Video::CameraParameters camPars = image.camPars;

	double intrinsic[4];
	intrinsic[0] = camPars.fx;
	intrinsic[1] = camPars.fy;
	intrinsic[2] = camPars.cx;
	intrinsic[3] = camPars.cy;

	double dist[4];
	dist[0] = camPars.k1;
	dist[1] = camPars.k2;
	dist[2] = camPars.p1;
	dist[3] = camPars.p2;

	// get extrinsic parameters from camera image
	double extrinsic[16];
	getRow44(camPars.pose, extrinsic);

	// Recalculation of rotation matrix and transition vector
	// w ... world point ???
	// R ... rotation matrix/home/ari/projects/saa-cogx/tuw/subarchitectures/vision.sa/src/c++/vision/components/StereoFlapDetector/StereoFlapDetector.cpp:322: error: base operand of ‘->’ has non-pointer type ‘Z::Flap3D’

	// p ... image point ???
	// t ... translation vector
	// w = R.p + t
	// w-t = R.p
	// R^T.w - R^T.t = p		==> R(new) = -R^T.t

	// Invert rotation matrix (= transpose)
	double zw = extrinsic[1];
	extrinsic[1] = extrinsic[4];
	extrinsic[4] = zw;
	zw = extrinsic[2];
	extrinsic[2] = extrinsic[8];
	extrinsic[8] = zw;
	zw = extrinsic[6];
	extrinsic[6] = extrinsic[9];
	extrinsic[9] = zw;

	// Translation vector from m to mm
// 	extrinsic[3] *= 1000;
// 	extrinsic[7] *= 1000;
// 	extrinsic[11] *= 1000;

	// Recaclulate translation vector for inverted rotation matrix
	double tneu[3]; 
	tneu[0] = -(extrinsic[0]*extrinsic[3] + extrinsic[1]*extrinsic[7] + extrinsic[2]*extrinsic[11]);
	tneu[1] = -(extrinsic[4]*extrinsic[3] + extrinsic[5]*extrinsic[7] + extrinsic[6]*extrinsic[11]);
	tneu[2] = -(extrinsic[8]*extrinsic[3] + extrinsic[9]*extrinsic[7] + extrinsic[10]*extrinsic[11]);
	extrinsic[3] = tneu[0];
	extrinsic[7] = tneu[1];
	extrinsic[11] = tneu[2];

// printf("Translationsvektor neu: %6.5f   %6.5f   %6.5f\n\n", extrinsic[3], extrinsic[7], extrinsic[11]);

// printf("Camera parameters: extrinsic new:\n	%6.5f   %6.5f   %6.5f   %6.5f\n", extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]);
// printf("	%6.5f   %6.5f   %6.5f   %6.5f\n", extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]);
// printf("	%6.5f   %6.5f   %6.5f   %6.5f\n", extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]);
// printf("	%6.5f   %6.5f   %6.5f   %6.5f\n\n", extrinsic[12], extrinsic[13], extrinsic[14], extrinsic[15]);

//  	vs3Interface->SetCamParameters(intrinsic, dist, extrinsic);

	return true;
}

/**
 * Try to find a "natural" looking coordinate system for a flap.
 * The coordinate system really is arbitrary, there is no proper implicitly
 * defined coordinate system.
 * We take the (geometrical) center of gravity of the corner points as
 * as position and set orientation to identity.
 */
void StereoFlapDetector::DefineFlapCoordsys(Z::Flap3D &flap, Pose3 &pose)
{
  Vector3 c(0., 0., 0.);
  int cnt = 0;
  // find the center of gravity
  for(int i = 0; i <= 1; i++)
  {
    for(unsigned j = 0; j < flap.surf[i].vertices.Size(); j++)
    {
      c += flap.surf[i].vertices[j].p;
      cnt++;
    }
  }
  c /= (double)cnt;
  pose.pos.x = c.x;
  pose.pos.y = c.y;
  pose.pos.z = c.z;

	// set the orientation to identity, i.e. parallel to world coordinate system
  pose.rot.x = 0.;
  pose.rot.y = 0.;
  pose.rot.z = 0.;

  // invert to get pose of world w.r.t. flap
  Pose3 inv = pose.Inverse();

	// recalculate the vectors to the vertices from new center point
  for(int i = 0; i <= 1; i++)
  {
    for(unsigned j = 0; j < flap.surf[i].vertices.Size(); j++)
    {
      Vector3 p(flap.surf[i].vertices[j].p.x,
                flap.surf[i].vertices[j].p.y,
                flap.surf[i].vertices[j].p.z);
			flap.surf[i].vertices[j].p = inv.Transform(p);
    }
	}
}

}
