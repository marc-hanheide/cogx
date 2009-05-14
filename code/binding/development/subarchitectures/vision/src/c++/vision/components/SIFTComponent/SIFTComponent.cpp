#include <iostream>
#include <sstream>
#include <cmath>
#include <math.h>

#include <opencv/highgui.h>

#include <vision/VisionOntologyFactory.hpp>
#include <vision/VisionGoals.h>
#include <vision/idl/Vision.hh>
#include <vision/utils/VisionUtils.h>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "SIFTComponent.h"


extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new SIFTComponent(_id);
  }
}


/**
 * Ze constructor, which starts it all...
 */
SIFTComponent::SIFTComponent(const string &_id) : 
  WorkingMemoryAttachedComponent(_id),
  VideoClientProcess(_id),
  m_curTaskID("") {
  cout << "creating SIFTComponent::SIFTComponent" << endl;
  
  // Initialize variables for the GUI-based display...
  dispLabel = string("UNKNOWN");
  dispPoint.m_x = 0.0;
  dispPoint.m_y = 0.0;
  currImage = NULL;

  // Set the required Ontologies...
//   CASTCompositeOntology *pComposite = new CASTCompositeOntology();
//   pComposite->addOntology( PlanningOntologyFactory::getOntology() );
//   pComposite->addOntology( VisionOntologyFactory::getOntology() );
//   setOntology( pComposite );

  setOntology(VisionOntologyFactory::getOntology());
  m_pSiftEngine = NULL;
}


/**
 * Ze destructor, where the component dies -- help, save me...:)
 */
SIFTComponent::~SIFTComponent() {
  cout << "SIFTComponent::~SIFTComponent()" << endl;

  m_taskActionMap.clear();
  delete m_pSiftEngine;
}



void SIFTComponent::start() {
  // VideoClientProcess::start();
  ManagedProcess::start();

  // Add a change filter that responds to changes in the WM, as result
  // of the planning process...
//   addChangeFilter(createLocalTypeFilter<Action>(cdl::ADD),
// 		  new MemberFunctionChangeReceiver<SIFTComponent>( this,
// 								   &SIFTComponent::handlePlanningActionRequest ) );
  
  // TEMPORARY -- FOR TESTING ALONE...
  addChangeFilter(createLocalTypeFilter<ROI>(cdl::ADD),
		  new MemberFunctionChangeReceiver<SIFTComponent>( this,
								   &SIFTComponent::proposeObjectDetectorGoal ) );
}



/**
 * Function that responds to a scene change, and if the change has
 * been processed (i.e. ROIS have been generated), it sets up a SIFT
 * process...
 */
void SIFTComponent::proposeObjectDetectorGoal( const cdl::WorkingMemoryChange& _wmChange ) {
  // Get the data that caused the changein WM...
  shared_ptr<const CASTData<Vision::ROI> > pROIData
    = getWorkingMemoryEntry<Vision::ROI>( _wmChange.m_address );
  
  if( pROIData ) {
    shared_ptr<const Vision::ROI> pROI = pROIData->getData();
    if( pROI ) {
      string label = doSIFT(pROI);
      string sceneObjectID( pROI->m_objId );
      if( sceneObjectID != "" ) {
	updateSceneObject( pROI, label );
      }
      else {
	println("SIFT:SceneObject for roi : " + label + " is missing");      
      }
      
      cout << "\n SIFT:Done with processing roi at time " 
	   << pROI->m_time.m_s << ":"  << pROI->m_time.m_us << endl;
    }
    else {
      println("failed to get ROI");
    }
  }
  
}




/**
 * Function that configures the sift process using the configuration
 * options available to the algorithm...
 */
void SIFTComponent::configure( map<string,string> & _config ) {
  // first let the base class configure itself
  ManagedProcess::configure(_config);
  // VideoClientProcess::configure(_config);

  if(_config["-f"] != "") {
    m_dbfilename = _config["-f"];
    log( "SIFT:dbname " + m_dbfilename );
  }
  else {
    m_dbfilename = "";
    log( "SIFT:dbname = noname" + m_dbfilename );
  }
}




/**
 * Function that adopts the task based on the list fo tasks proposed
 * before...
 */
void SIFTComponent::taskAdopted(const string &_taskID) {
  bool performedSift = false;
  string objLabel( "" );
  if( !existsOnWorkingMemory( _taskID ) ) {
    log( "ROI no longer exists on WM -- scene changed horribly?" );
  }
  // OK, now we can go ahead and run SIFT...
  else {
    log( "SIFT:taskAdopt -- ROI data in WM..." );
    shared_ptr<const CASTData<Vision::ROI> > pROIData
      = getWorkingMemoryEntry<Vision::ROI>(_taskID);
    
    if( pROIData ) {
      shared_ptr<const Vision::ROI> pROI = pROIData->getData();
      if( pROI ) {
	objLabel = doSIFT( pROI );
	string sceneObjectID( pROI->m_objId );
	if( sceneObjectID != "" ) {
	  updateSceneObject( pROI, objLabel );
	}
	else {
	  println("SIFT:SceneObject for roi : " + objLabel + " is missing");      
	}
	
	cout << "\n SIFT:Done with processing roi at time " 
	     << pROI->m_time.m_s << ":"  << pROI->m_time.m_us << endl;
	performedSift = true;
      }
      else {
	println("failed to get ROI");
      }
      
    }
  }
  
  // Time to tell the goal creator that the task is over successfully
  // -- we do live in an optimistic world don't we... :)
  taskComplete(_taskID,cdl::PROCESSING_COMPLETE_SUCCESS);
}



void SIFTComponent::taskRejected( const string& _taskID ) {
    m_curTaskID = "";
//     MemoryIDMap::iterator i = m_pProposedProcessing->find(_taskID);
//     if (i != m_pProposedProcessing->end()) 
// 	m_pProposedProcessing->erase(i);

}




/**
 * Function that actually sets up the sift engine to process the ROI
 * region of the current image...
 */
string SIFTComponent::doSIFT(shared_ptr<const Vision::ROI> _pROI) {
  struct timeval tInit, tFin;
  gettimeofday( &tInit, NULL );

  // Reset the object label to 'unknown'...
  string olabel="unknown";
  
  // Reset the object confidence value...
  m_objectConfidence = 0.0;
  
  if( ( _pROI->m_camNum == Vision::CAM_LEFT ) || 
      ( _pROI->m_camNum == Vision::CAM_RIGHT ) ) {
    // Get the image from a camera (at least one of them has an
    // image;)...
    ImageFrame pImg;
    getImage( _pROI->m_camNum, pImg );
    
    // Set up image for display...
    if( currImage != NULL ) {
      cvReleaseImage( &currImage );
    }
    currImage = buffer2image( &pImg );

    // Also do the BGR->RGB swap...
    int numChannels = currImage->nChannels;
    for( int i = 0; i < currImage->height * currImage->width; ++i ) {
      const char tempC = currImage->imageData[ i*numChannels + 0 ];
      currImage->imageData[ i*numChannels + 0 ] = currImage->imageData[ i*numChannels + 2 ];
      currImage->imageData[ i*numChannels + 2 ] = tempC;
    }
    
    // If a sift detector engine has not been created yet, create
    // one...
    if( m_pSiftEngine == NULL ) {
      m_pSiftEngine = new SiftDetectEngine( (int)(pImg.m_width), 
					    (int)(pImg.m_height),
					    m_dbfilename );
    }

    // Detect the object label, which also sets the probability
    // value if a suitable object is found...
    m_pSiftEngine->detectObject( &pImg, _pROI->m_bbox, 
				 olabel, m_objectConfidence );
    
    // Set up variables for display purposes...
    double bboxW = _pROI->m_bbox.m_size.m_x;
    double bboxH = _pROI->m_bbox.m_size.m_y;
    double bboxCx = _pROI->m_bbox.m_center.m_x;
    double bboxCy = _pROI->m_bbox.m_center.m_y;
    dispPoint.m_x = bboxCx - bboxW/4.0;
    dispPoint.m_y = bboxCy - bboxH/2.0;
    dispLabel = olabel;

    CvPoint topLeft = cvPoint( (int)(bboxCx - bboxW/2.0), 
			       (int)(bboxCy - bboxH/2.0) );
    CvPoint bottomRight = cvPoint( (int)(bboxCx + bboxW/2.0), 
				   (int)(bboxCy + bboxH/2.0) );
    cvRectangle( currImage, topLeft, bottomRight, CV_RGB(255,0,0), 2 );
    log( "SIFT: dosift: drawing current findings to GUI..." );
    redrawGraphicsNow();
  }

  gettimeofday( &tFin, NULL );
  double timeDiff1 = (tFin.tv_sec - tInit.tv_sec)*1.0 + (tFin.tv_usec - tInit.tv_usec)*1.0e-6;
  cout << "\n\n  SIFT processing time: " << timeDiff1 << endl;
  
  return olabel;
}



/**
 * Function updates the scene object corresponding to the ROI just
 * processed and sets properties (label, probability etc)...
 */
void SIFTComponent::updateSceneObject(shared_ptr<const Vision::ROI> _pROI, 
				      const string & _label) {
  // Get the ID of the scene object to update...
  string sceneObjectID(_pROI->m_objId);

  // Check if a corresponding entry exists in the WM...
  if( !existsOnWorkingMemory( sceneObjectID ) ) {
    log("SIFT:ROI exists but no scene object -- CRASH!!!!");
  }
  
  // Get the shared pointer to the corresponding sceneobject...
  shared_ptr<const CASTData<Vision::SceneObject> >pSceneObjectData 
      = getWorkingMemoryEntry<Vision::SceneObject>(sceneObjectID);

  if( pSceneObjectData ) {
    // Finally, we access the actual scene object -- phew!
    shared_ptr<const Vision::SceneObject> pSceneObject 
      = pSceneObjectData->getData();

    // Allocate memory for the updated scene object -- to be pushed
    // back into WM...
    Vision::SceneObject * pNewSceneObject = new Vision::SceneObject();

    // Set values that are currently known...
    *pNewSceneObject = *pSceneObject;
      
    // If the scene object is not null, well set the object label and
    // probability...
    if( pSceneObject ) {
      println("got SceneObject");
      string oldLabel(pSceneObject->m_label.m_string);
      println("old label: " + oldLabel);
      println("new label: " + _label);
      pNewSceneObject->m_label.m_string = CORBA::string_dup(_label.c_str());

      // Well, let us try and set a probability value that means
      // something for a change -- currently represents the % of
      // detected features that amtch the eventually assigned
      // object label...
      pNewSceneObject->m_label.m_confidence = m_objectConfidence;
      
      cout << "SIFTComponent:: Found " << _label.c_str() << ", at (" 
	   << pNewSceneObject->m_pose.m_position.m_x << "," 
	   << pNewSceneObject->m_pose.m_position.m_y << ","
	   << pNewSceneObject->m_pose.m_position.m_z << ") with prob: "
	   << pNewSceneObject->m_label.m_confidence << endl;
      
      overwriteWorkingMemory<SceneObject>( sceneObjectID, pNewSceneObject );
    }
    else {
      println("SIFTComponent:: failed to get SceneObject");
    }
      
    //delete pSceneObjectData;
  }

}



/**
 * Function inherited from Inspectable component for display
 * purposes...
 */
void SIFTComponent::redrawGraphics2D() {
  // Use previously stored data to set up the display variables...
  if( currImage == NULL ) {
    log( "SIFT: no image to display on screen..." );
    drawText2D( (int)dispPoint.m_x, (int)dispPoint.m_y, "Nothing to Display", 255, 0, 255, 0 );
    return;
  }

  // We would like the RGB image output on GUI screen...
  drawRGBImage( currImage->width, currImage->height, currImage->imageData, 0 );
  drawText2D( (int)dispPoint.m_x, (int)dispPoint.m_y, dispLabel,  255, 0, 0, 0 );

}



/**
 * Action registration happens here -- for now, the code is commented
 * out...
 */
void SIFTComponent::runComponent() {
//   ActionRegistration siftOp;
//   siftOp.m_component = CORBA::string_dup( getProcessIdentifier().c_str() );
//   // string archID( getSubarchitectureID() );
//   siftOp.m_subarchitecture = CORBA::string_dup( m_subarchitectureID.c_str() ); // archID.c_str() );
//   siftOp.m_action = CORBA::string_dup( string("sift-detector").c_str() );
//   addToWorkingMemory( newDataID(), PlanningOntology::ACTION_REGISTRATION_TYPE, new ActionRegistration( siftOp ) );
}

