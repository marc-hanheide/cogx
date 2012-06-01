/** @file TrackerScenario.h
 *
 * Tracker
 *
 * @author	Marek Kopicki (The University Of Birmingham)
 * @author      Sergio Roa (DFKI)
 *
 * @version 1.0
 *
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.
 */

#include <scenario/TrackerScenario.h>
#include <smltools/tracker_tools.h>

using namespace TomGine;
using namespace blortGLWindow;
using namespace Tracking;

namespace smlearning {

TrackerScenario::TrackerScenario (golem::Scene &scene) :
	Scenario(scene)
{
}

void TrackerScenario::init(boost::program_options::variables_map vm)
{
	Scenario::init (vm);
}

bool TrackerScenario::create(const TrackerScenario::Desc& desc) {
	if (!Scenario::create(desc)) // throws
		return false;
	this->desc = desc;

	this->desc.font = new TomGine::tgFont (desc.fontName.c_str());
	// Tracker
	const std::string& tracker_ini_file = desc.trackerConfig;
	const std::string& cam_ini_file = desc.cameraCalibrationFile;
	const std::string& pose_ini_file = desc.poseCalibrationFile;
	
	// PLY Model
	m_plypath = getModelPath(tracker_ini_file.c_str());
	std::string m_plyfile = getModelFile(tracker_ini_file.c_str());
	std::string m_plyname = m_plyfile;
	m_plyname.erase(m_plyname.begin()+m_plyname.find("."), m_plyname.end());
	m_plyfile.insert(0, m_plypath);
	
	int cam_width = getCamWidth(cam_ini_file.c_str());
	int cam_height = getCamHeight(cam_ini_file.c_str());	
	// Initialize camera capture using opencv
	g_Resources->InitCapture(float(cam_width), float(cam_height));
	_img = g_Resources->GetNewImage();

	
	glWindow.reset(new blortGLWindow::GLWindow(cam_width,cam_height,"Tracking"));
	
	// m_tracker.reset(new Tracking::TextureTracker());
	m_tracker = new Tracking::TextureTracker ();
	if(!m_tracker->init(tracker_ini_file.c_str(), cam_ini_file.c_str(), pose_ini_file.c_str()))
		throw golem::Message(golem::Message::LEVEL_CRIT, "TrackerPredOffline::create(): Failed to initialise tracker");
	
	m_initialPose = TomGine::tgPose();
	m_initialPose.t = vec3(-0.05f, 0.25f, 0.0f);
	m_initialPose.Rotate(0.0f, -1.57f, 0.0f);
	m_trackpred_id = m_tracker->addModelFromFile(m_plyfile.c_str(), m_initialPose, m_plyname.c_str());
	m_track_id = m_tracker->addModelFromFile(m_plyfile.c_str(), m_initialPose, m_plyname.c_str());
	m_ground_id = m_tracker->addModelFromFile(m_plyfile.c_str(), m_initialPose, m_plyname.c_str());
	
	Tracking::ModelLoader m_ply_loader;
	m_ply_loader.LoadPly(m_object, m_plyfile.c_str());

	_quit = false;
	
	return true;
}

void TrackerScenario::main ()
{
// Main Loop
	blortGLWindow::Event event;
	while( ! _quit ){
		printf("Entering loop...\n");
		// grab new image from camera
		_img = g_Resources->GetNewImage();
		
		// Image processing
		//m_tracker->image_processing_occluder((unsigned char*)img->imageData, model_occ, p_occ);
		m_tracker->image_processing((unsigned char*)_img->imageData);
		
		// Tracking (particle filtering)
		m_tracker->track(m_track_id);
		
		// store the updated position of the object
		m_tracker->getModelPose(m_track_id,m_track_pose);
		// _new_position = true;

		// Draw result
		//m_tracker->drawImage(0);
		m_tracker->drawResult(2);
		m_tracker->drawCoordinates();
		
		m_tracker->getModelMovementState(m_track_id, _movement);
		m_tracker->getModelQualityState(m_track_id, _quality);
		m_tracker->getModelConfidenceState(m_track_id, _confidence);
		
		
		glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
		if(_movement == ST_FAST)
			desc.font->Print("fast", 20, 10, 50);
		else if(_movement == ST_SLOW)
			desc.font->Print("slow", 20, 10, 50);
		else if(_movement == ST_STILL)
			desc.font->Print("still", 20, 10, 50);
			
		if(_confidence == ST_GOOD)
			desc.font->Print("good", 20, 10, 30);
		else if(_confidence == ST_FAIR)
			desc.font->Print("fair", 20, 10, 30);
		else if(_confidence == ST_BAD)
			desc.font->Print("bad", 20, 10, 30);
		
		if(_quality == ST_OK)
			desc.font->Print("ok", 20, 10, 10);
		else if(_quality == ST_OCCLUDED)
			desc.font->Print("occluded", 20, 10, 10);
		else if(_quality == ST_LOST)
			desc.font->Print("lost", 20, 10, 10, 1,0,0);
		else if(_quality == ST_LOCKED)
			desc.font->Print("locked", 20, 10, 10);
		
		glWindow->Update();
		
		while( glWindow->GetEvent(event) )
			_quit = !InputControl( m_tracker, event );
	}
}

void TrackerScenario::finish() {
	// m_tracker.release();
	delete m_tracker;
	glWindow.release();
	//capture->finish();
}



bool XMLData(TrackerScenario::Desc &val, XMLContext* xmlcontext, Context *context) {
	// XMLData ((Scenario::Desc&)val, xmlcontext, context);
	if (xmlcontext == NULL) {
		ASSERT(false)
		return false;
	}
    
	std::string driver;
	XMLData("driver", driver, xmlcontext->getContextFirst("arm")); // Get arm driver name
	val.descArmActor.armDesc.pArmDesc = Arm::Desc::load(*context, driver); // Load driver
	
	// finger setup
	val.descArmActor.fingerDesc.clear();
	golem::Real baseLength = 0.1848;
	XMLData(baseLength, xmlcontext->getContextFirst("effector base_length"));
	golem::Real fingerLength = 0.135;
	XMLData(fingerLength, xmlcontext->getContextFirst("effector finger_length"));
	golem::Real fingerDiam = 0.02;
	XMLData(fingerDiam, xmlcontext->getContextFirst("effector finger_diameter"));
	golem::Real tipRadius = 0.015;
	XMLData(tipRadius, xmlcontext->getContextFirst("effector tip_radius"));
	
	golem::BoundingBox::Desc* pFingerRodShapeDesc = new golem::BoundingBox::Desc;
	pFingerRodShapeDesc->dimensions.set(fingerDiam/2.0, fingerLength/2.0, fingerDiam/2.0);
	pFingerRodShapeDesc->pose.p.v2 += baseLength + fingerLength/2.0;
	pFingerRodShapeDesc->group = val.descArmActor.effectorGroup;
	val.descArmActor.fingerDesc.push_back(golem::Bounds::Desc::Ptr(pFingerRodShapeDesc));
	golem::BoundingSphere::Desc* pFingerTipShapeDesc = new golem::BoundingSphere::Desc;
	pFingerTipShapeDesc->radius = tipRadius;
	pFingerTipShapeDesc->pose.p.v2 += golem::Real(baseLength + fingerLength);
	pFingerTipShapeDesc->group = val.descArmActor.effectorGroup;
	val.descArmActor.fingerDesc.push_back(golem::Bounds::Desc::Ptr(pFingerTipShapeDesc));
	
	// end-effector reference pose
	val.descArmActor.referencePose.setId();
	val.descArmActor.referencePose.p.v2 += golem::Real(baseLength + fingerLength);



	
	//polyflap interaction settings

	//a number that slightly greater then the maximal reachable space of the arm
	//    - used for workspace position normalization and later as a position upper bound
	//      for random polyflap position
	//maximum X value for polyflap position
	XMLData(val.featLimits.maxX, xmlcontext->getContextFirst("ObjectInteraction maxX"));
	//maximum Y value for polyflap position
	XMLData(val.featLimits.maxY, xmlcontext->getContextFirst("ObjectInteraction maxY"));
	//maximum Z value for polyflap position
	XMLData(val.featLimits.maxZ, xmlcontext->getContextFirst("ObjectInteraction maxZ"));
	//minimum X value for polyflap position
	XMLData(val.featLimits.minX, xmlcontext->getContextFirst("ObjectInteraction minX"));
	//minimum Y value for polyflap position
	XMLData(val.featLimits.minY, xmlcontext->getContextFirst("ObjectInteraction minY"));
	//minimum Z value for polyflap position
	XMLData(val.featLimits.minZ, xmlcontext->getContextFirst("ObjectInteraction minZ"));
	//minimum duration value for pushing action
	XMLData(val.featLimits.minDuration, xmlcontext->getContextFirst("ObjectInteraction minPushDuration"));
	//maximum duration value for pushing action
	XMLData(val.featLimits.maxDuration, xmlcontext->getContextFirst("ObjectInteraction maxPushDuration"));
	//minimum value for a label
	XMLData(val.featLimits.minValLabel, xmlcontext->getContextFirst("ObjectInteraction minValLabel"));
	//maximum value for a label
	XMLData(val.featLimits.maxValLabel, xmlcontext->getContextFirst("ObjectInteraction maxValLabel"));

	//minimal duration of a movement (by normal speed)
	XMLData(val.descArmActor.minDuration, xmlcontext->getContextFirst("ObjectInteraction minDuration"));


	Real x;
	Real y;
	Real z;
	
	//Polyflap Position and orientation
	XMLData(x, xmlcontext->getContextFirst("ObjectInteraction startPosition x"));
	XMLData(y, xmlcontext->getContextFirst("ObjectInteraction startPosition y"));
	XMLData(z, xmlcontext->getContextFirst("ObjectInteraction startPosition z"));
	val.descActorObject.startPosition.set(x, y, z);

	XMLData(x, xmlcontext->getContextFirst("ObjectInteraction startRotation x"));
	XMLData(y, xmlcontext->getContextFirst("ObjectInteraction startRotation y"));
	XMLData(z, xmlcontext->getContextFirst("ObjectInteraction startRotation z"));
	val.descActorObject.startRotation.set(y, x, z);

	//Polyflap dimensions		
	XMLData(x, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions x"));
	XMLData(y, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions y"));
	XMLData(z, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions z"));
	val.descActorObject.dimensions.set(x, y, z);
	//Polyflap width
	XMLData(val.descActorObject.width, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions width"));
	

	//vertical distance from the ground
	//const Real over = 0.01;
	//vertical distance from the ground considering fingertip radius
	XMLData(val.descActorObject.over, xmlcontext->getContextFirst("ObjectInteraction over"));
	//distance from the front/back of the polyflap
	XMLData(val.descActorObject.dist, xmlcontext->getContextFirst("ObjectInteraction dist"));

	Real r;

	//distance from the side of the polyflap
	XMLData(r, xmlcontext->getContextFirst("ObjectInteraction side"));
	val.descActorObject.side = val.descActorObject.dimensions.v1*r;
	//center of the polyflap
	XMLData(r, xmlcontext->getContextFirst("ObjectInteraction center"));
	val.descActorObject.center = val.descActorObject.dimensions.v2*r;
	//distance from the top of the polyflap
	XMLData(r, xmlcontext->getContextFirst("ObjectInteraction top"));
	val.descActorObject.top = val.descActorObject.dimensions.v2 - r;
	//lenght of the movement		
	XMLData(val.distance, xmlcontext->getContextFirst("ObjectInteraction distance"));
	

	XMLData(val.startingPositionsConfig, xmlcontext->getContextFirst("loop startingPositions"));



	
	golem::XMLData("camera_file", val.cameraCalibrationFile, xmlcontext->getContextFirst("camera calibration"));
	golem::XMLData("pose_file", val.poseCalibrationFile, xmlcontext->getContextFirst("camera calibration"));
	golem::XMLData("config_file", val.trackerConfig, xmlcontext->getContextFirst("tracker"));
	golem::XMLData("font_name", val.fontName, xmlcontext->getContextFirst("tracker"));

	return true;

}

void TrackerScenarioApp::run(int argc, char *argv[]) {
	TrackerScenario::Desc desc;
	XMLData(desc, xmlcontext(), context());

	TrackerScenario *pTrackerScenario = dynamic_cast<TrackerScenario*>(scene()->createObject(desc)); // throws
	if (pTrackerScenario == NULL)
		throw Message(Message::LEVEL_CRIT, "TrackerScenarioApp::run(): unable to cast to TrackerScenario");

	// Random number generator seed
	context()->getMessageStream()->write(Message::LEVEL_INFO, "Random number generator seed %d", context()->getRandSeed()._U32[0]);
	
	try {
		// todo 
		pTrackerScenario->main();
	}
	catch (const Scenario::Interrupted&) {
		// todo
		pTrackerScenario->finish();
	}

	scene()->releaseObject(*pTrackerScenario);
}




} // namespace
