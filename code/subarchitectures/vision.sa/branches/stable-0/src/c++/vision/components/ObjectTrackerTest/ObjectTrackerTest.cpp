/* Dummy driver for the object tracker component
*
*  @author: Thomas Mörwald
*  @date: April 2009
*
*  This component is an example on how to control the
*  object tracker by loading a model from ply-file
*  to the working memory and calling several tracking commands.
*
*/

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "ObjectTrackerTest.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectTrackerTest();
  }
}

using namespace cast;

using namespace std;

void ObjectTrackerTest::configure(const map<string,string> & _config)
{
  pFile = fopen("subarchitectures/vision.sa/src/c++/vision/components/ObjectTrackerTest/img/trajectory.txt", "rb");
	
		if (pFile==NULL) {
			log("error loading trajectory file");
			exit (1);
		}

		// obtain file size:
		fseek (pFile , 0 , SEEK_SET);
		
		float t;
		float x, y, z;
		float m00, m01, m02,
					m10, m11, m12,
					m20, m21, m22;
		Pose3 p;
		int i;
		
		while(!feof(pFile))
		{
			fscanf(	pFile, "%f %f %f %f %f %f %f %f %f %f %f %f %f\n",
							&t,
							&x, &y, &z,
							&m00, &m01, &m02,
							&m10, &m11, &m12,
							&m20, &m21, &m22);
			
			p.pos.x=x; p.pos.y=y; p.pos.z=z;
			p.rot.m00=m00; p.rot.m01=m01; p.rot.m02=m02;
			p.rot.m10=m10; p.rot.m11=m11; p.rot.m12=m12;
			p.rot.m20=m10; p.rot.m21=m21; p.rot.m22=m22;
			
			trajectory.push_back(p);
			timestamps.push_back(t);
			i++;
			//printf("Trajectory loaded at: %f %f %f %f\n", t, p.pos.x, p.pos.y, p.pos.z);	
		}
		
		log("target trajectory loaded");
  fclose(pFile);
  
  timerstarted = false;
  m_error_pos = 0.0;
  m_error_rot = 0.0;
  
}

void ObjectTrackerTest::start()
{
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<ObjectTrackerTest>(this,
        &ObjectTrackerTest::receiveVisualObject));
}

void ObjectTrackerTest::runComponent()
{
  sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
                       
  // Load geometry from ply-file
  log("loading model 'box_red.ply'");
  m_model.load("subarchitectures/vision.sa/src/c++/vision/components/ObjectTracker/resources/model/red_box.ply");
    
  // Generate VisualObject
  VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
  obj->model = new VisionData::GeometryModel;
  if(!convertTrackerModel(&m_model, obj->model))
	log("no geometry model in Visual Object");
  obj->label = "red box";
  obj->detectionConfidence = 0.0;
  Particle p = Particle(0.0);
  convertParticle2Pose(p, obj->pose);  
  	
  // Add VisualObject to working memoryabs(trajectory[i].rot.m00 - obj->pose.rot.m00) +
  log("add model to working memory: '%s'", obj->label.c_str());
  addToWorkingMemory(newDataID(), obj);
  
  sleepProcess(1000);
  
  // Send start tracking command
  log("send tracking command: START");
  VisionData::TrackingCommandPtr track_cmd = new VisionData::TrackingCommand;
  track_cmd->cmd = VisionData::START;
  addToWorkingMemory(newDataID(), track_cmd);
  
  // Track for 10 seconds
  log("tracking 20 seconds (20 images @ 10Hz)");
  sleepComponent(20000);
  
  // Send stop tracking command
  log("send tracking command: STOP");
  track_cmd->cmd = VisionData::STOP;
  addToWorkingMemory(newDataID(), track_cmd);
  
  sleepComponent(1000);
  
  /*
  float m_error_pos_tol = 10.0;
  float m_error_rot_tol = 150.0;
  if(m_error_pos < m_error_pos_tol && m_error_rot < m_error_rot_tol){
  	log("Tracking test successfull");
  }else{
  	log("Tracking test failed");
  }
  log("  Position error: %.1f<%.1f ?, Rotation error: %.1f<%.1f ?)", m_error_pos/m_error_pos_tol, 1.0, m_error_rot/m_error_rot_tol,1.0);
  */
  //fclose(pFile);
}

void ObjectTrackerTest::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  trajectory.push_back(obj->pose);
  
  if(!timerstarted){
  	timerstarted = true;
  	m_timer.Reset();	
  }
  	
  m_timer.Update();
  
  // Get time since reset
  float t = m_timer.GetApplicationTime();
  /*
  fprintf(pFile, "%f %f %f %f %f %f %f %f %f %f %f %f %f\n",
  					t,
  					obj->pose.pos.x, obj->pose.pos.y, obj->pose.pos.z, 
  					obj->pose.rot.m00, obj->pose.rot.m01, obj->pose.rot.m02, 
  					obj->pose.rot.m10, obj->pose.rot.m11, obj->pose.rot.m12, 
  					obj->pose.rot.m20, obj->pose.rot.m21, obj->pose.rot.m22);
  
  */
  int i;
  
  for(i=0; i<timestamps.size() && timestamps[i]<t; i++)
  {;}
  
  m_error_pos +=	abs(trajectory[i].pos.x - obj->pose.pos.x) +
  								abs(trajectory[i].pos.y - obj->pose.pos.y) + 
  								abs(trajectory[i].pos.z - obj->pose.pos.z);
  								
  m_error_rot +=	abs(trajectory[i].rot.m00 - obj->pose.rot.m00) +
  								abs(trajectory[i].rot.m01 - obj->pose.rot.m01) +
  								abs(trajectory[i].rot.m02 - obj->pose.rot.m02) +
  								abs(trajectory[i].rot.m10 - obj->pose.rot.m10) +
  								abs(trajectory[i].rot.m11 - obj->pose.rot.m11) +
  								abs(trajectory[i].rot.m12 - obj->pose.rot.m12) +
  								abs(trajectory[i].rot.m20 - obj->pose.rot.m20) +
  								abs(trajectory[i].rot.m21 - obj->pose.rot.m21) +
  								abs(trajectory[i].rot.m22 - obj->pose.rot.m22);
  								
  //log("Pose error position: %f", m_error_pos);
  //log("Pose error rotation: %f", m_error_rot);
}



