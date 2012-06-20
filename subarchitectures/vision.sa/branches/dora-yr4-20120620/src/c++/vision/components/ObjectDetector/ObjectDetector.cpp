/**
 * @file ObjectDetector.cpp
 * @author Andreas Richtsfeld
 * @date April 2009
 * @version 0.1
 * @brief Management component for running the basic object detector (vs3)
 */

#include <cast/architecture/ChangeFilterFactory.hpp>

#include <opencv/highgui.h>
#include <VideoUtils.h>
#include <../../VisionUtils.h>
#include <ObjectDetector.h>
#include <VisionData.hpp>
#include "BasicShapeGeometries.hpp"


/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectDetector();
  }
}


using namespace cogx;
using namespace Math;
using namespace std;
using namespace VisionData;

namespace cast
{

bool showImage = true;									///< show openCv image from image server.
bool getCubes = true;										///< get cubes from the object detector
bool getCylinders = true;								///< get cylinders from the object detector

void ObjectDetector::receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc)
{
	ObjectDetectionCommandPtr detect_cmd = getMemoryEntry<ObjectDetectionCommand>(_wmc.address);
	
	debug("received detection command ...");
	switch(detect_cmd->cmd){
		case VisionData::DSTART:
			if(cmd_detect){
				debug("already started object detection.");
			}else{
				log("starting object detection.");
				cmd_detect = true;
			}
			break;
		case VisionData::DSTOP:
			if(cmd_detect){
				log("stopping object detection");
				cmd_detect = false;
			}else{
				debug("already stopped object detection");
			}
			break;
		case VisionData::DSINGLE:
			log("single object detection");
			videoServer->getImage(camId, m_image);
			processImage(m_image);

			/* commented by TM
			if(!cmd_single){
				log("single detection");
				cmd_single = true;
			}else{
				log("got already single detection command: too fast triggering!");
			}
			*/
			break;
		default:
			debug("unknown detection command received, doing nothing");
			break;
	}	
}

void ObjectDetector::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
 
  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }

	vs3Interface = new Vs3Interface();

	detail = 0;
	type = 0;
	frame_counter = 0;

  cmd_detect = false;
	cmd_single = false;
}

void ObjectDetector::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  // start receiving images pushed by the video server
  vector<int> camIds;
  camIds.push_back(camId);
  videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);

	// add change filter for object detection commands
  addChangeFilter(createLocalTypeFilter<ObjectDetectionCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectDetector>(this,
        &ObjectDetector::receiveDetectionCommand));

  if(showImage) cvNamedWindow(getComponentID().c_str(), 1);
}

void ObjectDetector::receiveImages(const std::vector<Video::Image>& images)
{
  if(images.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "image list is empty"));

	if(cmd_detect)
	{
  	processImage(images[0]);
	}
	else if(cmd_single)
	{
		processImage(images[0]);
		cmd_single = false;
	}
}

void ObjectDetector::processImage(const Video::Image &image)
{
	debug("process new image");
	frame_counter++;
	int number = 0;
	bool masked = true;

	// Get camera parameters and convert image to openCV iplImage
	GetCameraParameter(image);
	IplImage *iplImage = convertImageToIpl(image);

	// read visual objects from working memory (check occurence)
	std::vector <VisionData::VisualObjectPtr> results;
	getMemoryEntries<VisionData::VisualObject>(results);
	
	// Process the image and find all Gestalts
	vs3Interface->ProcessSingleImage(iplImage);


	// ----------------------------------------------------------------------------
	// Get objects after processing and create visual object for working memory
	// ----------------------------------------------------------------------------
	if(getCylinders) GetCylinders();

	if(getCubes)
	{
		Z::CubeDef cd;				// cube definition

		while(vs3Interface->GetCube(number, cd, masked))
		{
			number++;
			if(!masked /*&& cd.height > 0.04*/)							/// HACK: Nasty threshold for cube height: >4 cm
			{
				bool cubeExists = false;
	
				// check, if the cube exists at same position in working memory
				for(unsigned i=0; i<results.size(); i++)
				{
					// read vertices and calculate cube center points and radius
					vector<Vertex> vertices = results[i]->model->vertices;
	
					Vector3 newCubeCenter;
					newCubeCenter.x = cd.cubeCenter3D.x;
					newCubeCenter.y = cd.cubeCenter3D.y;
					newCubeCenter.z = cd.height/2.;
	
					double radius = length(vertices[0].pos);
	
					if(length(results[i]->pose.pos - newCubeCenter) < radius)
						cubeExists = true;
				}

				// Create visual object, if it does not already exists in the working memory
				if(!cubeExists)
				{
					num_cubes++;
					VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
					if(Cube2VisualObject(obj, cd))
					{
						char obj_label[32];
						sprintf(obj_label, "Cube %d", num_cubes);
						obj->label = obj_label;
	
						// Add VisualObject to working memory
						addToWorkingMemory(newDataID(), obj);
						log("new cube at frame number %u: added visual object to working memory: %s", frame_counter, obj->label.c_str());
					}
				}
			}
		}
	}


	// --------------------------------------------------------------------
	// Get key input from openCV
	// -------------------------------------------------------------------
	int key = 0;
	key = cvWaitKey(10);
		if(key==1048603) return;									// return for escape

	switch(key){
		case 1048619: detail++;										// Key '+'
			log("Detail switched to %i", detail);
			break;
		case 1048621: if(detail>0) detail--;			// Key '-'
			log("Detail switched to %i", detail);
			break;
		case 1048670: type = 29;									// OBJECT == ^
			log("Switched to OBJECT");
			break;
		case 1048625: type = 20;									// CUBE == 1
			log("Switched to CUBE");
			break;
		case 1048626: type = 12;									// CYLINDER == 2
			log("Switched to CYLINDER");
			break;
		case 1048627: type = 13;									// CONE == 3
			log("Switched to CONE");
			break;
		case 1048628: type = 6;										// BALL == 4
			log("Switched to BALL");
			break;
		case 1048629: type = 5;										// ELLIPSE == 5
			log("Switched to ELLIPSE");
			break;
		case 1048630: type = 0;										// SEGMENT == 6
			log("Switched to SEGMENT");
			break;
		case 1048631: type = 1;										// LINE == 7
			log("Switched to LINE");
			break;
		case 1048632: type = 17;									// RECTANGLE == 8
			log("Switched to RECTANGLE");
			break;
		case 1048633: type = 19;									// FLAP == 9
			log("Switched to FLAP");
			break;
		case 1048676: 														// Enable/Disable display == D
			if(showImage) showImage = false;
			else showImage = true;
			log("Enable/Disable object detector window");
			break;
		default: break;
	}

	// ----------------------------------------------------------------
	// Draw Gestalts to IplImage for the openCv window
	// ----------------------------------------------------------------
	if(showImage) 
	{	
		vs3Interface->SetActiveDrawArea(iplImage);
		vs3Interface->DrawUnmaskedGestalts(type, detail);
	
		// Convert from RGB to BGR
// 		cvConvertImage( iplImage, iplImage, CV_CVTIMG_SWAP_RB);

		cvShowImage(getComponentID().c_str(), iplImage);
		cvWaitKey(10);
		cvReleaseImage(&iplImage);
	}
}


/**
 * @brief Convert cube from object detector to working memory visual object
 * @param obj Visual Object
 * @param cd Cube
 * @return True for success
 */
bool ObjectDetector::Cube2VisualObject(VisionData::VisualObjectPtr &obj, Z::CubeDef &cd)
{
	obj->model = new VisionData::GeometryModel;

	// add pose (cube center point) to the model
	cogx::Math::Pose3 p;
	p.pos.x = cd.cubeCenter3D.x;
	p.pos.y = cd.cubeCenter3D.y;
	p.pos.z = cd.height/2.;				// cube center is at height/2.
	obj->pose = p;
	setIdentity(obj->pose.rot);

	// create vertices (relative to the 3D cube center point)
	for(unsigned i=0; i<4; i++)
	{
		Vertex v0, v1; 
		v0.pos.x = cd.corner_points3D[i][0].x - cd.cubeCenter3D.x;
		v0.pos.y = cd.corner_points3D[i][0].y - cd.cubeCenter3D.y;
		v0.pos.z = cd.height/2.;
		v1.pos.x = cd.corner_points3D[i][1].x - cd.cubeCenter3D.x;
		v1.pos.y = cd.corner_points3D[i][1].y - cd.cubeCenter3D.y;
		v1.pos.z = -cd.height/2.;
		obj->model->vertices.push_back(v0);
		obj->model->vertices.push_back(v0);
		obj->model->vertices.push_back(v0);
		obj->model->vertices.push_back(v1);
		obj->model->vertices.push_back(v1);
		obj->model->vertices.push_back(v1);
	}

	// add faces to the vision model
	Face f;
	f.vertices.push_back(0);									// right
	f.vertices.push_back(6);
	f.vertices.push_back(9);
	f.vertices.push_back(3);
	obj->model->faces.push_back(f);
	f.vertices.clear();

	f.vertices.push_back(7);									// front
	f.vertices.push_back(12);
	f.vertices.push_back(15);
	f.vertices.push_back(10);
	obj->model->faces.push_back(f);
	f.vertices.clear();

	f.vertices.push_back(13);									// left
	f.vertices.push_back(18);
	f.vertices.push_back(21);
	f.vertices.push_back(16);
	obj->model->faces.push_back(f);
	f.vertices.clear();

	f.vertices.push_back(19);									// back
	f.vertices.push_back(1);
	f.vertices.push_back(4);
	f.vertices.push_back(22);
	obj->model->faces.push_back(f);
	f.vertices.clear();

	f.vertices.push_back(2);									// top
	f.vertices.push_back(20);
	f.vertices.push_back(14);
	f.vertices.push_back(8);
	obj->model->faces.push_back(f);
	f.vertices.clear();

	f.vertices.push_back(5);									// front
	f.vertices.push_back(11);
	f.vertices.push_back(17);
	f.vertices.push_back(23);
	obj->model->faces.push_back(f);
	f.vertices.clear();
	
	//* **** Compute Normals *****
	computeNormalsByFaces(obj->model);
	
	obj->detectionConfidence = 1.0;						// detection confidence is always 1

	return true;
}


/**
 * @brief Extract camera parameters from video server.
 * @param image Image from the image server with the camera paramters.
 * @return True for success
 */
bool ObjectDetector::GetCameraParameter(const Video::Image & image)
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
	// R ... rotation matrix
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

 	vs3Interface->SetCamParameters(intrinsic, dist, extrinsic);

	return true;
}


/**
 * @brief Get the cylinders as visual object from the object detector and write it to the working memory.
 * This method is only for cylinders, which are upright on the ground plane.
 */
void ObjectDetector::GetCylinders()
{
	Z::CylDef cd;									// cylinder properties from object detector
	int number = 0;								// number of fetched cylinders
	bool masked = false;					// cylinder is masked?
	static bool single = true;		/// HACK: detect only one single cylinder model, then stop immediately
	single = true;
	
	static vector<Z::CylDef> cylinders_old;		// old cylinders from the former image
	static vector<Z::CylDef> cylinders_new;		// new cylinders from this image

	cylinders_new.clear();

	while(vs3Interface->GetCylinder(number, cd, masked))
	{
		number++;
		if(!masked)
		{
			double radius, height;		// radius and height for the cylinder model

			// save new cylinder, if geometry constraints fulfilled
			if(cd.height > 0.05) cylinders_new.push_back(cd);					/// HACK: Cylinder must be higher than 5cm!!!

			// compare with former cylinders and try to find the same one: we compare the deviation between the center points
			bool matchFound = false;
			for(unsigned i=0; i < cylinders_old.size(); i++)
			{
				// calculate center deviation:
				double centerDeviation = (cylinders_old[i].cylinderCenter3D - cd.cylinderCenter3D).Norm();

				if(centerDeviation < 0.005)							/// HACK: Threshold: Only a deviation of 5mm between center points of cyl. from the former image is allowed
				{
					// calculate smoothed radius and height from both cylinders
					radius = (cylinders_old[i].topRadius3D + cd.topRadius3D)/2.;
					height = (cylinders_old[i].height + cd.height)/2.;

					matchFound = true;
				}
			}

			// Create visual object, when also found in the former image
			if(matchFound && single)
			{
				single = false;

				// print properties of the cylinder
// 				printf("	=> ground vertices: %4.4f / %4.4f - %4.4f / %4.4f\n", cd.vertex3D[0][0].x, cd.vertex3D[0][0].y, cd.vertex3D[0][1].x, cd.vertex3D[0][1].y);
// 				printf("	=>    top vertices: %4.4f / %4.4f - %4.4f / %4.4f\n", cd.vertex3D[1][0].x, cd.vertex3D[1][0].y, cd.vertex3D[1][1].x, cd.vertex3D[1][1].y);
// 				printf("	=> center point 3D: %4.4f / %4.4f / %4.4f\n", cd.cylinderCenter3D.x, cd.cylinderCenter3D.y, cd.height/2.);
// 				printf("	=> radius (ground/top): %4.4f - %4.4f\n", cd.radius3D, cd.topRadius3D);
// 				printf("	=> height: %4.4f\n", cd.height);

				// create model as visual object for working memory of cast
				VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
				obj->model = new VisionData::GeometryModel;
				genCylinder(obj->model, radius, height, 16, 1, true);
				obj->detectionConfidence = 1.0;

				// add pose (cylinder center point) to the model
				cogx::Math::Pose3 p;
				p.pos.x = cd.cylinderCenter3D.x;
				p.pos.y = cd.cylinderCenter3D.y;
				p.pos.z = cd.height/2.;
				obj->pose = p;

				// label object
				char obj_label[32];
				num_cylinders++;
				sprintf(obj_label, "Cylinder %d", num_cylinders);
				obj->label = obj_label;

				// Add VisualObject to working memory
				addToWorkingMemory(newDataID(), obj);
				log("new cylinder at frame number %u: added visual object to working memory: %s", frame_counter, obj->label.c_str());
			}
		}
	}

	// copy new cylinders to the old vector
	cylinders_old = cylinders_new;
}

}

