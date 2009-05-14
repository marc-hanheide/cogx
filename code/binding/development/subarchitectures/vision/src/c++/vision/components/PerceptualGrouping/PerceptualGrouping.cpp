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
#include "PerceptualGrouping.h"

#include <vs2/VisionCore.hh>
#include <vs2/BufferVideo.hh>
#include <vs2/Rectangle.hh>
#include <vs2/Ellipse.hh>
#include <vs2/LJunction.hh>

using namespace Z;

// Pointers to be used to call the perceptual grouping functions...
VisionCore *vcore = 0;
BufferVideo *bufvid = 0;

// Pre-declaration of the helper function defined at the bottom of
// this file...
static bool RectangleSidesParallel( Rectangle* r );
static bool RectangleBigEnough( Rectangle* r );

// Yep -- I am now a valid component -- go ahead, add me to your
// ever-expanding list of utterly unrelated components -- one more
// cannot possibly hurt... ;)
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new PerceptualGrouping(_id);
  }
}


/**
 * The Constructor...
 */
PerceptualGrouping::PerceptualGrouping(const string &_id) : 
    //FrameworkProcess(_id),
    WorkingMemoryAttachedComponent(_id),
    VideoClientProcess(_id),
    m_curTaskID("") {
  cout << "creating PerceptualGrouping::PerceptualGrouping" << endl;

  // Initialize variables for the GUI-based display...
  dispLabel = string("UNKNOWN");
  dispPoint.m_x = 0.0;
  dispPoint.m_y = 0.0;
  currImage = NULL;
  roiImage = NULL;

  // Set the required Ontologies...
//   CASTCompositeOntology *pComposite = new CASTCompositeOntology();
//   pComposite->addOntology( PlanningOntologyFactory::getOntology() );
//   pComposite->addOntology( VisionOntologyFactory::getOntology() );
//   setOntology( pComposite );
  setOntology(VisionOntologyFactory::getOntology());

  // Set up the stuff for the perceptual grouping...
  bufvid = new BufferVideo();
  vcore = new VisionCore(bufvid);
  // Enable the desired gestalt principles, to be used for the
  // perceptual grouping -- could eventually be modified into a config
  // file...
  vcore->EnableGestaltPrinciple(GestaltPrinciple::FORM_SEGMENTS);
  vcore->EnableGestaltPrinciple(GestaltPrinciple::FORM_LINES);
  vcore->EnableGestaltPrinciple(GestaltPrinciple::FORM_JUNCTIONS);
  vcore->EnableGestaltPrinciple(GestaltPrinciple::FORM_CLOSURES);
}



/**
 * The Destructor, because what is created is eventually destroyed...
 */
PerceptualGrouping::~PerceptualGrouping() {
  cout << "PerceptualGrouping::~PerceptualGrouping()" << endl;
  
  // Clear up memory...
  m_taskActionMap.clear();

  if( currImage != NULL ) {
    cvReleaseImage( &currImage );
  }
  if( roiImage != NULL ) {
    cvReleaseImage( &roiImage );
  }

  // Also clear up the stuff related to the perceptual grouping...
  delete vcore; // deleting core also deletes buffvid...
}



/**
 * The function that starts it all...
 */
void PerceptualGrouping::start() {
  VideoClientProcess::start();
  // Add a change filter that responds to changes in the WM, as result
  // of the planning process...
//   addChangeFilter( createLocalTypeFilter<ACTION>(cdl::ADD),
//    		   new MemberFunctionChangeReceiver<PerceptualGrouping>( this,
// 									 &PerceptualGrouping::handlePlanningActionRequest ) );
  
  // TEMPORARY -- FOR TESTING ALONE...
  addChangeFilter(createLocalTypeFilter<ROI>(cdl::ADD),
		  new MemberFunctionChangeReceiver<PerceptualGrouping>( this,
									&PerceptualGrouping::handleROIPGDetection ) );
}




/**
 * Function that responds to a change in the vision SA's WM as a
 * result of a planning action being written into it by the CP-based
 * planning system...
 */
void PerceptualGrouping::handlePlanningActionRequest( const cdl::WorkingMemoryChange& _wmc ) {

}





/**
 * Function that adopts the task based on the list of tasks proposed
 * before...
 */
void PerceptualGrouping::taskAdopted( const string &_taskID ) {
  // And now we're finished, tell the goal manager that the task is
  // over successfully (assuming it is... naughty!)
  taskComplete(_taskID,cdl::PROCESSING_COMPLETE_SUCCESS);
}




/**
 * Why is this here? Ans: Because if tasks can be adopted, they can be
 * rejected too...:)
 */
void PerceptualGrouping::taskRejected( const string& _taskID ) {

}




/**
 * Function that responds to a ROI addition in vision's WM, resulting
 * in a task to find the perceptual groups in it...
 */
void PerceptualGrouping::handleROIPGDetection( const cdl::WorkingMemoryChange& _wmc ) {
  // Get the data that caused the changein WM...
  shared_ptr<const CASTData<Vision::ROI> > pROIData
    = getWorkingMemoryEntry<Vision::ROI>( _wmc.m_address );

  // Assume for now that the perceptual grouping will provide an
  // integer label depending on the 'type' of structures found in the
  // input ROI...
  int pgLabel = -1;
  
  // If the ROI data exists in the WM, get a shared pointer to it...
  if( pROIData ) {
    shared_ptr<const Vision::ROI> pROI = pROIData->getData();
    if( pROI ) {
      // Determine the perceptual grouping label by invoking Michael's
      // perceptual grouping code...
      pgLabel = doPerceptualGroups( pROI );

      // Update the corresponding scene object with the appropriate
      // perceptaul grouping label..
      string sceneObjectID( pROI->m_objId );
      if( sceneObjectID != "" ) {
	updateSceneObject( pROI, pgLabel );
      }
      else {
	log("PG:SceneObject for roi : " + enum2String(pgLabel) + " is missing");      
      }
      
      cout << "\n PG:Done with processing roi at time " 
	   << pROI->m_time.m_s << ":"  << pROI->m_time.m_us << endl;
    }
    else {
      println("PG:failed to get ROI");
    }
  }
}




/**
 * Function that actually computes the perceptual groups, and returns
 * a numerical index corresponding to the groups found...
 */
int PerceptualGrouping::doPerceptualGroups( shared_ptr<const Vision::ROI> _pROI ) {
  // Initialize the PG label...
  int pgLabel = -1;

  if ((_pROI->m_camNum == Vision::CAM_LEFT) || 
      (_pROI->m_camNum == Vision::CAM_RIGHT)) { 
    // Get the image (ah, at last!)...
    // ImageFrame *pImg = GetImage(_pROI->m_camNum);
    log( "PG: In detectpg, getting image data..." );
    ImageFrame pImg;
    getImage( _pROI->m_camNum, pImg );
    
    // Transform image into the IplImage format...
    if( currImage != NULL ) {
      cvReleaseImage( &currImage );
    }
    currImage = buffer2image( &pImg );
    
    // Iterate over the pixels in the IplImage struct and flip the R-B
    // pixel order, since the input image (from videoserver currently
    // uses BGR instead of RGB)...
    int numChannels = currImage->nChannels;
    for( int i = 0; i < currImage->height * currImage->width; ++i ) {
      const char tempC = currImage->imageData[ i*numChannels + 0 ];
      currImage->imageData[ i*numChannels + 0 ] = currImage->imageData[ i*numChannels + 2 ];
      currImage->imageData[ i*numChannels + 2 ] = tempC;
    }

    log( "PG: In detectpg, setting display variables..." );

    // Set up variables for display purposes...
    double bboxW = _pROI->m_bbox.m_size.m_x;
    double bboxH = _pROI->m_bbox.m_size.m_y;
    double bboxCx = _pROI->m_bbox.m_center.m_x;
    double bboxCy = _pROI->m_bbox.m_center.m_y;
    dispPoint.m_x = bboxCx - bboxW/4.0;
    dispPoint.m_y = bboxCy - bboxH/2.0;

    // Now to go on for the actual perceptual groups in the ROI --
    // MAGIC;)
    detectPerceptualGroups( _pROI->m_bbox, pgLabel );

    // Remember to clear up the pointer...
    // delete(pImg);
  }
  std::cout << "PG: pg shape detected: " 
	    << enum2String( pgLabel ) << endl;

  // We eventually will need to do some processing here for the
  // perceptual groups found in the ROI, to determine the 'perceptual
  // class' the ROI belongs to -- TODO **VERY** SOON...

  // TEMPORARY OUTPUT...
  return pgLabel;

}





/**
 * Function takes the input ROI and determines the perceptual groups
 * in the image region corresponding to this ROI, and sets an integer
 * variable to denote the perceptual 'class' the ROI is classified
 * into...
 */
void PerceptualGrouping::detectPerceptualGroups( const BBox2D& _bbox, int& pgLabel ) {
  // Just a baseline sanity check...
  if( currImage == NULL ) {
    log( "PG: no input image -- CRASH!!" );
  }

  // Bounding box of the ROI under consideration...
  BBox2D orig(_bbox);
  BBox2D extendedBox(_bbox); 

  // Artificially extend the bounding box by a little bit along each
  // axis, just in case the initial estimate (based on contours) fails
  // to include the entire area of interest...
  int extension = 19;
  extendedBox.m_size.m_x += (2 * extension);
  extendedBox.m_size.m_y += (2 * extension);

  // if this is over image extremes, reset
  if( ( extendedBox.m_center.m_x + ( extendedBox.m_size.m_x/2 ) ) > currImage->width ) {
    extendedBox = orig;
  }
  else if( ( extendedBox.m_center.m_y + ( extendedBox.m_size.m_y/2 ) ) > currImage->height ) {
    extendedBox = orig;
  }
  else {
    // cout<< "PG:extending bbox by: " << 2*extension << endl;
  }

  int topLeftX  = (int)( extendedBox.m_center.m_x - ( extendedBox.m_size.m_x/2.0 ) );
  int topLeftY  = (int)( extendedBox.m_center.m_y - ( extendedBox.m_size.m_y/2.0 ) );  

  cvSetImageROI( currImage, cvRect( topLeftX, topLeftY, (int)extendedBox.m_size.m_x, (int)extendedBox.m_size.m_y ) );
  if( roiImage != NULL ) {
    cvReleaseImage( &roiImage );
  }
  roiImage = cvCreateImage(cvSize((int)extendedBox.m_size.m_x,(int)extendedBox.m_size.m_y), 8, 3);
  cvCopy( currImage, roiImage );

  log( "PG: starting PG processing..." );
  // Push framework image into buffer video...
  // bufvid->Init( currImage->width, currImage->height, RGB24, currImage->imageData, false );
  bufvid->Init( roiImage->width, roiImage->height, RGB24, roiImage->imageData, false );
  vcore->NewImage();
  
  // Now, ah well, let us try the perceptual grouping code, as it is
  // done in the manipulator component...
  vcore->ProcessImage();
  
  // Filter the rectangles and prepare for display -- accept big
  // enough rectangles with roughly parallel sides...
  vector<Rectangle*> rects;
  for( size_t i = 0; i < NumRectangles(); ++i ) {
    Rectangle *r = Rectangles(RankedGestalts(Gestalt::RECTANGLE, i));
    if( RectangleSidesParallel(r) && 
	RectangleBigEnough(r) && 
	r->IsUnmasked() ) {
      rects.push_back(r);
    }
  }

  log( "PG: drawing rectangles..." );

  // For now, show all rectangles on display...
  for( size_t i = 0; i < rects.size(); ++i ) {
    cout << "PG: Rect: " << i << endl;
    
    vector<Vector2D> tmp_points(4);
    for( size_t j = 0; j < 4; ++j ) {
      tmp_points[j].m_x = LJunctions( rects[i]->jcts[j] )->isct.x;
      tmp_points[j].m_y = LJunctions( rects[i]->jcts[j] )->isct.y;
      cout << "Point: " << j << " (" << tmp_points[j].m_x << "," 
	   << tmp_points[j].m_y << "),  ";
    }
    cout << "\n";
    // drawPolygon( currImage, tmp_points, CV_RGB(0, 0, 255) );
    drawPolygon( roiImage, tmp_points, CV_RGB(0, 0, 255) );
  }

  // Copy over data back into original image for display purposes...
  cvCopy( roiImage, currImage );

  // For now assume that only rectangles need to be found -- i.e. iff
  // a valid rectangle is found in the ROI, the pgLabel is set to a
  // valid value...
  if( rects.size() >= 1 ) {
    pgLabel = PG_PROPERTIES_OFFSET + PG_RECTANGLE;
  } else {
    pgLabel = -1;
  }

  // Currently just display the number of rectangles found as the
  // label...
  stringstream oss;
  oss << rects.size();
  oss >> dispLabel;
  oss.clear();
}




/**
 * Function to update the scene object once the labels have been
 * found...
 */
void PerceptualGrouping::updateSceneObject( shared_ptr<const Vision::ROI> _pROI, 
					    const int& _label ) {
    // Access the scene object to update...
  string sceneObjectID( _pROI->m_objId );
  
  if( !existsOnWorkingMemory( sceneObjectID ) ) {
    log( "PG:ROI exists but no scene object -- CRASH!!!!" );
  }
  
  // Get the shared pointer to the actual scene object out of WM...
  shared_ptr<const CASTData<Vision::SceneObject> >pSceneObjectData 
    = getWorkingMemoryEntry<Vision::SceneObject>( sceneObjectID );

  if( pSceneObjectData ) {
    // Finally, we access the actual scene object -- phew!
    shared_ptr<const Vision::SceneObject> pSceneObject 
      = pSceneObjectData->getData();

    // Allocate memory for the updated scene object -- to be pushed
    // back into WM...
    Vision::SceneObject * pNewSceneObject = new Vision::SceneObject();

    // Set values currently known...
    *pNewSceneObject = *pSceneObject;
      
    // Set values to variables...
    if( pSceneObject ) {
      log( "Got SceneObject" );
      string oldLabel(  enum2String( pSceneObject->m_pgshape.m_int ) );
      log( "old pg: " + oldLabel );
      log( "new pg: " + enum2String(_label) );
      
      // Set the new sceneobject's color label...
      pNewSceneObject->m_pgshape.m_int = _label;

      // For now set the probability to be very high... TODO: modify
      // suitably to reflect uncertainty...
      pNewSceneObject->m_pgshape.m_confidence = 0.95;

      cout << "PG: Found " << enum2String(_label) << ", at new pose (" 
	   << pNewSceneObject->m_pose.m_position.m_x << "," 
	   << pNewSceneObject->m_pose.m_position.m_y << ","
	   << pNewSceneObject->m_pose.m_position.m_z << ") with prob: " 
	   << pNewSceneObject->m_pgshape.m_confidence << endl;

      // Write into WM again, with the newly-found properties...
      try {
	overwriteWorkingMemory<SceneObject>( sceneObjectID, pNewSceneObject );
      } catch( const ConsistencyException &c ) {
	updateSceneObject( _pROI, _label );
      }
    }
    else {
      log( "PG:failed to get SceneObject" );
    }
  }
}




/**
 * Function inherited from Inspectable component for display
 * purposes...
 */
void PerceptualGrouping::redrawGraphics2D() {
  // Use previously stored data to set up the display variables...
  if( currImage == NULL ) {
  // if( roiImage == NULL ) {
    drawText2D( (int)dispPoint.m_x, (int)dispPoint.m_y, "Nothing to Display", 255, 0, 0, 0 );
    return;
  }

  // We would like the RGB image output on GUI screen...
  drawRGBImage( currImage->width, currImage->height, currImage->imageData, 0 );
  drawText2D( (int)dispPoint.m_x, (int)dispPoint.m_y, dispLabel, 255, 0, 0, 0 );
  // drawRGBImage( roiImage->width, roiImage->height, roiImage->imageData, 0 );
}





/**
 * The process that is run each cycle -- currently sets up the action
 * mapping so that it can take action instructions from the planning
 * SA...
 */
void PerceptualGrouping::runComponent() {
//   ActionRegistration perceptGroupOp;
//   perceptGroupOp.m_component = CORBA::string_dup( getProcessIdentifier().c_str() );
//   perceptGroupOp.m_subarchitecture = CORBA::string_dup( m_subarchitectureID.c_str() ); // archID.c_str() );
//   perceptGroupOp.m_action = CORBA::string_dup( string("pg-detector").c_str() );
//   addToWorkingMemory( newDataID(), PlanningOntology::ACTION_REGISTRATION_TYPE, new ActionRegistration( perceptGroupOp ) );
}



/**
 * SOME HELPER FUNCTIONS TO HELP WITH THE PERCEPTUAL GROUPING --
 * CURRENTLY JUST COPIED OVER FROM THE MANIPULATOR CODE -- EVENTUALLY
 * WILL BE MOVED TO A MORE 'COMMON' LOCATION...
 */

static bool RectangleSidesParallel( Rectangle *r ) {
  static const double phi_max = 0.3;  // parallelity threshold
  Vector2 l[4];           // sides
  double phi0_2, phi1_3;  // angles between opposing sides
  int i, j;

  // normalise might throw an exception
  try {
    // l[i] is the line joining i and i+1
    for( i = 0; i < 4; ++i ) {
      j = (i < 3 ? i + 1 : 0);
      l[i] = LJunctions(r->jcts[j])->isct - LJunctions(r->jcts[i])->isct;
      l[i].Normalise();
    }
    phi0_2 = fabs(asin(Cross(l[0], l[2])));
    phi1_3 = fabs(asin(Cross(l[1], l[3])));
    return phi0_2 < phi_max && phi1_3 < phi_max;
  }
  catch(...) {
    return false;
  }
}


static bool RectangleBigEnough( Rectangle *r ) {
  static const double len_min = 40.;  // min length in pixels
  Vector2 l[4];   // sides
  double len = 0.;
  int i, j;

  // l[i] is the line joining i and i+1
  for( i = 0; i < 4; ++i ) {
    j = (i < 3 ? i + 1 : 0);
    l[i] = LJunctions(r->jcts[j])->isct - LJunctions(r->jcts[i])->isct;
    len = max(len, Length(l[i]));
  }
  return len > len_min;
}
