/**
 * @file ObjectDetector.cpp
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Management component for running simple object detector (vs3)
 */

#include <cast/architecture/ChangeFilterFactory.hpp>

#include <opencv/highgui.h>
#include <VideoUtils.h>
#include <ObjectDetector.h>
#include <VisionData.hpp>


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

namespace cast
{

bool showImage = true;			///< show openCv image from image server.

using namespace std;
using namespace VisionData;


void ObjectDetector::receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc){
	ObjectDetectionCommandPtr detect_cmd = getMemoryEntry<ObjectDetectionCommand>(_wmc.address);
	
	log("received detection command ...");
	switch(detect_cmd->cmd){
		case VisionData::DSTART:
			if(cmd_detect){
				log("allready started detection");
			}else{
				log("starting detection");
				cmd_detect = true;
			}
			break;
		case VisionData::DSTOP:
			if(cmd_detect){
				log("stopping detection");
				cmd_detect = false;
			}else{
				log("allready stopped detection");
			}
			break;
		default:
			log("unknown detection command received, doing nothing");
			break;
	}	
}


void ObjectDetector::configure(const map<string,string> & _config)
{
// 	log("VisionSystem3::configure: Running configuration");
  map<string,string>::const_iterator it;
 
  // first let the base classes configure themselves
  configureVideoCommunication(_config);

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }

	vs3Interface = new Vs3Interface();

	detail = 0;
	type = 0;
}

void ObjectDetector::start()
{
// 	log("VisionSystem3::start: Start Component");
  startVideoCommunication(*this);

  addChangeFilter(createLocalTypeFilter<ObjectDetectionCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectDetector>(this,
        &ObjectDetector::receiveDetectionCommand));

  if(showImage) cvNamedWindow(getComponentID().c_str(), 1);
}

void ObjectDetector::runComponent()
{
// 	log("VisionSystem3::runComponent");
  while(isRunning())
  {
		// process a single image
		if (cmd_detect) processImage();

    // wait a bit so we don't hog the CPU
    sleepComponent(10);
  }
}


void ObjectDetector::processImage()
{
	Video::Image image;
	getImage(camId, image);

	IplImage *iplImage = convertImageToIpl(image);
	vs3Interface->ProcessSingleImage(iplImage);

	// ----------------------------------------------------------------------------
	// Get objects after processing and create visual object for working memory
	// ----------------------------------------------------------------------------

	int number = 0;
	Z::CubeDef cd;
	bool masked = true;

	while(vs3Interface->GetCube(number, cd, masked))
	{
		log("Got new cube!!!");
		number ++;

		/// TODO TODO TODO TODO TODO Verarbeiten des Würfels: Anlegen des Visual objects

		// Generate VisualObject
		VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
		if(Cube2VisualObject(obj, cd))
		{
			log("Cube2VisualObject converted");

			// Add VisualObject to working memory
  		addToWorkingMemory(newDataID(), obj);
		}
	}


	// --------------------------------------------------------------------
	// Get input from the openCV image
	// -------------------------------------------------------------------
	int key = 0;
	key = cvWaitKey(10);
		if(key==1048603) return;									// return for escape
//  				printf("##############	Key: %i\n", key); 

	switch(key){
		case 1048619: detail++;										// Key '+'
			log("Detail switched to %i", detail);
			break;
		case 1048621: if(detail>0) detail--;			// Key '-'
			log("Detail switched to %i", detail);
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
			log("Enable/Disable vs3 window");
			break;
		default: break;
	}

	// ----------------------------------------------------------------
	// Draw Gestalts to IplImage for the openCv window
	// ----------------------------------------------------------------
	if(showImage) 
	{	
		vs3Interface->Draw(iplImage);
		vs3Interface->DrawGestalt(type, detail);


		cvShowImage(getComponentID().c_str(), iplImage);
// 		cvWaitKey(10);
		cvReleaseImage(&iplImage);
	}
}



/**
 * @brief Convert Cube from vs3 to Visual Object
 * @param obj Visual Object
 * @param cd Cube
 * @return True for success
 */
bool ObjectDetector::Cube2VisualObject(VisionData::VisualObjectPtr &obj, Z::CubeDef &cd)
{
	obj->model = new VisionData::GeometryModel;

	// create vertices
	for(unsigned i=0; i<4; i++)
	{
		Vertex v0, v1; 
		v0.pos.x = cd.corner_points[i][0].x;
		v0.pos.y = cd.corner_points[i][0].y;
		v0.pos.z = (cd.length_a + cd.length_b)/2.;	// currently not estimated ==> mean between side length
		v1.pos.x = cd.corner_points[i][1].x;
		v1.pos.y = cd.corner_points[i][1].y;
		v1.pos.z = 0;																// on ground plane!
		obj->model->vertices.push_back(v0);
		obj->model->vertices.push_back(v1);
	}

	// create faces
	Face f;
	f.vertices.push_back(0);									// right
	f.vertices.push_back(1);
	f.vertices.push_back(3);
	f.vertices.push_back(2);
	obj->model->faces.push_back(f);

	f.vertices.push_back(4);									// front
	f.vertices.push_back(2);
	f.vertices.push_back(3);
	f.vertices.push_back(5);
	obj->model->faces.push_back(f);

	f.vertices.push_back(4);									// left
	f.vertices.push_back(5);
	f.vertices.push_back(7);
	f.vertices.push_back(6);
	obj->model->faces.push_back(f);

	f.vertices.push_back(0);									// back
	f.vertices.push_back(1);
	f.vertices.push_back(7);
	f.vertices.push_back(6);
	obj->model->faces.push_back(f);

	f.vertices.push_back(0);									// top
	f.vertices.push_back(2);
	f.vertices.push_back(4);
	f.vertices.push_back(6);
	obj->model->faces.push_back(f);

	f.vertices.push_back(1);									// front
	f.vertices.push_back(3);
	f.vertices.push_back(5);
	f.vertices.push_back(7);
	obj->model->faces.push_back(f);

	obj->detectionConfidence = 1.0;						// detection confidence is always 1

	return true;
}

}

