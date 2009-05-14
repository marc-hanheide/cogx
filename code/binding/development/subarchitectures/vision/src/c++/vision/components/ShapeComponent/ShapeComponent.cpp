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
#include "ShapeComponent.h"


extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new ShapeComponent(_id);
  }
}


/**
 * The Constructor...
 */
ShapeComponent::ShapeComponent(const string &_id) : 
    WorkingMemoryAttachedComponent(_id),
    VideoClientProcess(_id),
    m_curTaskID("") {
  cout << "creating ShapeComponent::ShapeComponent" << endl;

  // Initialize the array storing shape probabilities...
  for( int i = 0; i < NUM_SHAPES; ++i ) {
    m_shapeProbs[i] = 0.0;
  }
  dispLabel = string("UNKNOWN");
  dispPoint.m_x = 0.0;
  dispPoint.m_y = 0.0;
  currImage = NULL;

  // Initialize shape properties...
  m_numShapes = NUM_SHAPES;
  m_numSamples = NUM_SAMPLES;
  m_emptyClassProbThresh = EMPTY_CLASS_THRESH;
  m_unknownClassProbRatioThresh = UNKNOWN_CLASS_THRESH;

  // Set the required Ontologies...
//   CASTCompositeOntology *pComposite = new CASTCompositeOntology();
//   pComposite->addOntology( PlanningOntologyFactory::getOntology() );
//   pComposite->addOntology( VisionOntologyFactory::getOntology() );
//   setOntology( pComposite );

  setOntology(VisionOntologyFactory::getOntology());
}



/**
 * The Destructor, because what is created is eventually destroyed...
 */
ShapeComponent::~ShapeComponent() {
  cout << "ShapeComponent::~ShapeComponent()" << endl;
  
  // Clear up memory...
  m_taskActionMap.clear();

  if( currImage != NULL ) {
    cvReleaseImage( &currImage );
  }
}




/**
 * Function that sets some essential parameters for the class
 * variables...
 */
void ShapeComponent::configure( map<string,string>& _config ) {
  // Propagate paramaters to the higher-levels as well...
  VideoClientProcess::configure(_config);

  // Sets the stream for reading the configuration values...
  ostringstream outStream;

  // Setting number of shape classes being considered...
  if(_config["-s"] != "") {
    istringstream configStream(_config["-s"]);
    configStream >> m_numShapes;

    // We are in trouble if the allocated number exceeds the maximum
    // size of arrays set...
    if( m_numShapes > NUM_SHAPES ) {
      log( "SC: config, num shapes exceeds limit -- CRASH!!" );
    }

    outStream.str("");
    outStream << "setting number of shapes : " << m_numShapes;
    log( outStream.str() );
  }

  // Setting the number of samples per shape class...
  if(_config["-n"] != "") {
    istringstream configStream(_config["-n"]);
    configStream >> m_numSamples;

    // We are in trouble if the allocated number exceeds the maximum
    // size of arrays set...
    if( m_numSamples > NUM_SAMPLES ) {
      log( "SC: config, num samples per class exceeds limit -- CRASH!!" );
    }

    outStream.str("");
    outStream << "setting number of samples per class: "<< m_numSamples;
    log(outStream.str());
  }

  // Setting the threshold on the probabilities for deciding on the
  // 'extra' class label of 'empty'...
  if(_config["-e"] != "") {
    istringstream configStream(_config["-e"]);
    configStream >> m_emptyClassProbThresh;

    // The threshold for class label 'empty' cannot be greater than a
    // certain maximum value...
    if( m_emptyClassProbThresh > EMPTY_CLASS_THRESH ) {
      log( "SC: config, empty class label thresh too large -- resetting" );
      m_emptyClassProbThresh = 1./m_numShapes;
    }
    
    outStream.str("");
    outStream << "setting threshold for class label empty: "
	      << m_emptyClassProbThresh;
    log(outStream.str());
  }

  // Setting the threshold on the probabilities for deciding on the
  // 'extra' class label of 'unknown'...
  if(_config["-u"] != "") {
    istringstream configStream(_config["-u"]);
    configStream >> m_unknownClassProbRatioThresh;

    // The threshold for ratio of best to second-best match, which
    // decides on the class label 'unknown', cannot be greater than a
    // certain maximum value...
    if( m_unknownClassProbRatioThresh > UNKNOWN_CLASS_THRESH ) {
      log( "SC: config, unknown class label thresh too large -- resetting" );
      m_unknownClassProbRatioThresh = UNKNOWN_CLASS_THRESH;
    }
    
    outStream.str("");
    outStream << "setting threshold for class label unknown: "
	      << m_unknownClassProbRatioThresh;
    log(outStream.str());
  }

  // Set the training sample file to read from...
  if(_config["-f"] != "") {
    m_dbfilename = _config["-f"];
    log( "SC:dbname " + m_dbfilename );
  }
  else {
    m_dbfilename = "";
    log( "SC:dbname = noname" + m_dbfilename );
  }

  // Figure out the file to store the training samples into -- TO BE
  // USED ONLY DURING TRAINING...
  if(_config["-t"] != "") {
    m_trnfilename = _config["-t"];
    log( "SC:trnfilename " + m_trnfilename );
    m_writeVectors = true;
  }
  else {
    m_trnfilename = "";
    log( "SC:trfilename = noname" + m_trnfilename );
    m_writeVectors = false;
  }

  // Read the actual training samples from the file...
  loadDataBase(m_dbfilename);

}




/**
 * Function loads the shape hists database into the memory...
 */
void ShapeComponent::loadDataBase( string strDBFile ) {
  FILE* fp = fopen(strDBFile.c_str(), "r+");
  if( fp == NULL ) {
    log( "SC:cannot open file: " + strDBFile );
    return;
  }
  
  // TODO:: FIGURE OUT FILE FORMAT FOR THE STORED MOMENTS AND ADD
  // APPROPRIATE CODE...
  for( int i = 0; i < m_numShapes * m_numSamples; ++i ) {
    for( int j = 0; j < NUM_MOMENTS; ++j ) {
      fscanf( fp, "%lf", &shapeMoments[i][j] );
//       cout << "Value (" << i << "," << j << ": "
//        	   << shapeMoments[i][j] << "), ";
    }
    cout << "Read values of Moment vector " << i << "\n";
  }

  // Close the database file and exit...
  fclose(fp);
}




/**
 * The function that starts it all...
 */
void ShapeComponent::start() {
  VideoClientProcess::start();
  // Add a change filter that responds to changes in the WM, as result
  // of the planning process...
  //   addChangeFilter( createLocalTypeFilter<ACTION>(cdl::ADD),
  // 		   new MemberFunctionChangeReceiver<ShapeComponent>( this,
  // 								     &ShapeComponent::handlePlanningActionRequest ) );
  
  // TEMPORARY -- FOR TESTING ALONE...
  addChangeFilter(createLocalTypeFilter<ROI>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ShapeComponent>( this,
								    &ShapeComponent::handleROIShapeDetection ) );
}




/**
 * Function that responds to a ROI addition in vision's WM, resulting
 * in a shape detection task...
 */
void ShapeComponent::handleROIShapeDetection( const cdl::WorkingMemoryChange& _wmc ) {
  // Get the data that caused the changein WM...
  shared_ptr<const CASTData<Vision::ROI> > pROIData
    = getWorkingMemoryEntry<Vision::ROI>( _wmc.m_address );

  int shapeLabel = -1;
  
  // If the ROI data exists in the WM, get a shared pointer to it...
  if( pROIData ) {
    shared_ptr<const Vision::ROI> pROI = pROIData->getData();
    if( pROI ) {
      // Determine the shape label by comparing with stored moments...
      shapeLabel = doSHAPE( pROI );

      // Update the corresponding scene object with the appropriate
      // shape label..
      string sceneObjectID( pROI->m_objId );
      if( sceneObjectID != "" ) {
	updateSceneObject( pROI, shapeLabel );
      }
      else {
	log("SC:SceneObject for roi : " + enum2String(shapeLabel) + " is missing");      
      }
      
//       cout << "\n SC:Done with processing roi at time " 
// 	   << pROI->m_time.m_s << ":"  << pROI->m_time.m_us << endl;
    }
    else {
      println("SC:failed to get ROI");
    }
  }
  
}




/**
 * Function that adopts the task based on the list of tasks proposed
 * before...
 */
void ShapeComponent::taskAdopted( const string &_taskID ) {
  // Some variable initialization...
  bool performedShape = false;
  int shapeLabel = -1;

  if( !existsOnWorkingMemory( _taskID ) ) {
    log( "SC:ROI no longer exists on WM -- scene changed horribly?" );
  }
  // OK, now we can go ahead and run SHAPE...
  else {
    log( "SC:taskAdopt -- ROI data in WM..." );
    shared_ptr<const CASTData<Vision::ROI> > pROIData
      = getWorkingMemoryEntry<Vision::ROI>(_taskID);
    
    // If the pointer to the required ROI exists...
    if( pROIData ) {
      // Get a reference to the actual ROI data...
      shared_ptr<const Vision::ROI> pROI = pROIData->getData();
      if( pROI ) {
	// Go ahead and find the shape...
	shapeLabel = doSHAPE( pROI );
	string sceneObjectID( pROI->m_objId );
	// If the corresponding scene object exists...
	if( sceneObjectID != "" ) {
	  // Update the shape properties of the scene object...
	  updateSceneObject( pROI, shapeLabel );
	}
	else {
	  log("SC:SceneObject for roi : " + 
	      enum2String(shapeLabel) + " is missing");      
	}
	
// 	cout << "\n SC:Done with processing roi at time " 
// 	     << pROI->m_time.m_s << ":" << pROI->m_time.m_us << endl;

	performedShape = true;
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




/**
 * Why is this here? Ans: Because if tasks can be adopted, they can be
 * rejectd too...:)
 */
void ShapeComponent::taskRejected( const string& _taskID ) {

}




/**
 * Function that actually does the shape moments-based comparison, and
 * spews out a label...
 */
int ShapeComponent::doSHAPE( shared_ptr<const Vision::ROI> _pROI ) {
  struct timeval tInit, tFin;
  gettimeofday( &tInit, NULL );

  // Initialize the ROI shape label...
  int shapeLabel = -1;

  if ((_pROI->m_camNum == Vision::CAM_LEFT) || 
      (_pROI->m_camNum == Vision::CAM_RIGHT)) { 
    // Get the image (ah, at last!)...
    ImageFrame pImg;
    getImage( _pROI->m_camNum, pImg );

    // Set up variables for display purposes...
    double bboxW = _pROI->m_bbox.m_size.m_x;
    double bboxH = _pROI->m_bbox.m_size.m_y;
    double bboxCx = _pROI->m_bbox.m_center.m_x;
    double bboxCy = _pROI->m_bbox.m_center.m_y;
    dispPoint.m_x = bboxCx - bboxW/4.0;
    dispPoint.m_y = bboxCy - bboxH/2.0;

    // Now to go on for the actual shape of the ROI -- MAGIC;)
    detectShape( pImg, _pROI->m_contourPoints, shapeLabel );
  }

  std::cout << "SC: doSHAPE, shape label: " 
	    << enum2String( shapeLabel ) << endl;

  gettimeofday( &tFin, NULL );
  double timeDiff1 = (tFin.tv_sec - tInit.tv_sec)*1.0 + (tFin.tv_usec - tInit.tv_usec)*1.0e-6;
  cout << "\n\n  Shape processing time: " << timeDiff1 << endl;

  return shapeLabel;
}




/**
 * Function that gets the shape of the ROI, from the image sent in as
 * input...
 */
void ShapeComponent::detectShape( ImageFrame& _pImg, const vector2DSequence& contourPoints, int& sLabel ) {
  // log( "SC: In detect shape, getting image data..." );
  // Now get the image data within the input image...
  if( currImage != NULL ) {
    cvReleaseImage( &currImage );
  }
  currImage = buffer2image( &_pImg );

  // Iterate over the pixels in the IplImage struct and flip the RGB
  // pixel order, since the input image (from videoserver currently
  // uses BGR instead of RGB)...
  int numChannels = currImage->nChannels;
  for( int i = 0; i < currImage->height * currImage->width; ++i ) {
    const char tempC = currImage->imageData[ i*numChannels + 0 ];
    currImage->imageData[ i*numChannels + 0 ] = currImage->imageData[ i*numChannels + 2 ];
    currImage->imageData[ i*numChannels + 2 ] = tempC;
  }

  // log( "SC: In detect shape, getting contour points for ROI..." );
  // Prepare storage for contours.
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* contour = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage ); // CV_32SC2

  // HACK -- HEADERS+CONSTANTS THAT DEFINE A 'CONTOUR' -- OBTAINED
  // FROM OPENCV CONSTANTS FILE...
  contour->flags = 1117327884;
  contour->header_size = 88;
  contour->elem_size = 8;
  contour->delta_elems = 128;

//   cout << "SC: populating contour from " <<  contourPoints.length() 
//        << " stored points...\n";
  // Populate the contour...
  for( size_t i = 0; i < contourPoints.length(); ++i ) {
    CvPoint p;
    p.x = (int)contourPoints[i].m_x;
    p.y = (int)contourPoints[i].m_y;

    cvSeqPush( contour, &p );
    // cout << p.x << " " << p.y << endl;
  }
  // Force contours to be 'closed' by inserting first point as also
  // the last point in sequence...
  CvPoint p0;
  p0.x = (int)contourPoints[0].m_x;
  p0.y = (int)contourPoints[0].m_y;
  // cout << p0.x << " " << p0.y << endl;
  cvSeqPush( contour, &p0 );

  // Set up the variables to store the moments...
  CvMoments moments;
  CvHuMoments hu_moments;

  // log( "SC: In detect shape, getting shape moments..." );    
  // First compute all the moments upto the third order...
  cvContourMoments( contour, &moments );
  
  // Next find the invariant Hu-moments...
  cvGetHuMoments( &moments, &hu_moments );

  // log( "SC: found contour and moments, or so I hope..." );
  
  // Debugging -- only for display...
  CvScalar color = CV_RGB( 0, 255, 0 );
  /* replace CV_FILLED with 1 to see the outlines */
  cvDrawContours( currImage, contour, color, color, 2, 1, 8 );
//   static int j = 0;
//   char filename[256];
//   sprintf(filename, "test%04d.ppm", j++);
//   cvSaveImage(filename, image);

  // If the moments were found, store them in an array for later
  // comparison with stored values...
//   cout << "SC: Hu moments are: (" << hu_moments.hu1 << ", "
//        << hu_moments.hu2 << ", " << hu_moments.hu3 << ", "
//        << hu_moments.hu4 << ", " << hu_moments.hu5 << ", "
//        << hu_moments.hu6 << ", " << hu_moments.hu7 << ")\n";
  testMoments[0] = hu_moments.hu1;    testMoments[1] = hu_moments.hu2;
  testMoments[2] = hu_moments.hu3;    testMoments[3] = hu_moments.hu4;
  testMoments[4] = hu_moments.hu5;    testMoments[5] = hu_moments.hu6;
  testMoments[6] = hu_moments.hu7;
  
  // Determine the shape class of the input ROI...
  log( "SC: time to determine the class label..." );
  determineShapeClass( sLabel );
    
  // Now for some display using the GUI-based functions..
  dispLabel = enum2String( sLabel );
  // log( "SC: drawing objects to GUI..." );
  redrawGraphicsNow();
  
  // Once done with all procesing, get rid of the allocated memory...
  cvReleaseMemStorage( &storage );

  // DEBUGGING -- write out the histogram to file...
  if( m_writeVectors ) {
    log( "SC: Writing Shape vector to file..." );
    writeVectors2File();
  }
}





/**
 * Function determines the shape class of the current ROI, based on
 * previously stored moments -- simple NNr based on a distance
 * measure...
 */
void ShapeComponent::determineShapeClass( int& sLabel ) {
  // Compare with each of the stored vectors, using a distance
  // measure...
  double minVal = 1.0e9;
  int minIdx = -1;
  double dist[ m_numShapes * m_numSamples ];

  // Variables for computing within class distances and hence weighted
  // distances...
  double sumDistWithinClass[ m_numShapes ];
  double avgDistWithinClass[ m_numShapes ];
  for( int j = 0; j < m_numShapes; ++j ) {
    sumDistWithinClass[j] = 0.0;
    avgDistWithinClass[j] = 0.0;
  }

  // Go forth and bravely find distace with each of the stored
  // samples, updating related variables for further analysis...
  for( int i = 0; i < m_numShapes*m_numSamples; ++i ){
    dist[i] = distMeasure(i, 3);
 //    cout << "SC: comparing with sample " << i << " value: "
// 	 << dist[i] << endl;

    // Maintain the within-class distance sum...
    int idx = i/m_numSamples;
    // Just a sanity check...
    idx = ( ( idx >= m_numShapes ) ? 
	    ( m_numShapes-1 ) : idx );
    
    sumDistWithinClass[idx] += dist[i];

    // Find the closest sample in training set, though it is not going
    // to be used to make the final decision...
    if( dist[i] < minVal ) {
      minVal = dist[i];
      minIdx = i;
    }
  }

  // Then a computation of weighted average distance within each class
  // and hence the relative probability of the match with each shape
  // class...
  for( int k = 0; k < m_numShapes*m_numSamples; ++k ) {
    int idx = k/m_numSamples;
    // Just a sanity check...
    idx = ( ( idx >= m_numShapes ) ? 
	    ( m_numShapes-1 ) : idx );

    // Update the weighted average distance within this shape class...
    avgDistWithinClass[idx] += ( ( dist[k] * dist[k] ) / sumDistWithinClass[idx] );
  }

  // Probability of match with each shape class is given by inverse of
  // the weighted distances...
  double sumInvDist = 0.0;
  for( int m = 0; m < m_numShapes; ++m ) {
    sumInvDist += ( ( avgDistWithinClass[m] > 0 ) ? 
		    ( 1/avgDistWithinClass[m] ) : 0 );

    // Also clean up the shape probabilities array...
    m_shapeProbs[m] = 0.0;
  }

  // Just a little check to prevent a divide-by-zero error...
  sumInvDist = ( ( sumInvDist == 0 ) ? ( sumInvDist + 1.0 ) 
		 : sumInvDist );

  // Individual shape's probabilities are inversely proportional to
  // the average distances...
  double maxProbVal = 0.0;
  int maxProbIdx = -1;
  for( int a = 0; a < m_numShapes; ++a ) {
    m_shapeProbs[a] = ( ( avgDistWithinClass[a] == 0 ) ? 1.0 : 
			( ( 1/avgDistWithinClass[a] )/sumInvDist ) );
    cout << "SC: probability of shape " << enum2String(a+SHAPE_PROPERTIES_OFFSET) 
	 << " is: " << m_shapeProbs[a] << endl;

    // Also compute the index corresponding to the maximum probability
    // value...
    if( m_shapeProbs[a] > maxProbVal ) {
      maxProbVal = m_shapeProbs[a];
      maxProbIdx = a;
    }
  }

  // Also determine the second-highest probability of match...
  double max2ProbVal = 0.0;
  int max2ProbIdx = -1;
  for( int b = 0; b < m_numShapes; ++b ) {
    if( b == maxProbIdx ) {
      continue;
    }
    if( m_shapeProbs[b] > max2ProbVal ) {
      max2ProbVal = m_shapeProbs[b];
      max2ProbIdx = b;
    }
  }

  // Determine the class label of the sample, including the labels of
  // 'unknown' and 'empty', based on the probability of match with
  // each of the color classes...
  if( m_shapeProbs[ maxProbIdx ] < m_emptyClassProbThresh ) {
    cout << "SC: setting class label empty, (id,prob): ("
	 << maxProbIdx << ", " << m_shapeProbs[ maxProbIdx ] << ")\n";
    maxProbIdx = -1; // Change the value depending on index of
		     // 'EMPTY'...
  }
  else if( ( m_shapeProbs[ max2ProbIdx ] / m_shapeProbs[ maxProbIdx ] ) >
	   m_unknownClassProbRatioThresh ) {
    cout << "SC: setting class label unknown, (id,prob): ("
	 << maxProbIdx << ", " << m_shapeProbs[ maxProbIdx ] << "), ("
	 << max2ProbIdx << ", " << m_shapeProbs[ max2ProbIdx ] << ")\n";
    maxProbIdx = -1;
  }
  
  // The actual string representing the class label needs to be
  // obtained by indexing into an enum with a suitable offset...
  int actualIdx = -1;
  if( maxProbIdx >= 0 ) {
    actualIdx = maxProbIdx + SHAPE_PROPERTIES_OFFSET;
    cout << "\n SC: (Value,ID) = (" << minVal << ", " << minIdx 
	 << "), avg (Value, ID) = (" << avgDistWithinClass[ maxProbIdx ] 
	 << ", " << maxProbIdx << "), actual shape: " 
	 << enum2String( actualIdx ) << endl;
  }
  else {
    cout << "\n SC: (Value,ID) = (" << minVal << ", " << minIdx 
	 << "), prob: " << maxProbIdx << ", actual shape: " 
	 << enum2String( actualIdx ) << endl;
  }

  // Time to send the shape label back for the rest of the world to
  // deal with...
  sLabel = actualIdx;
}





/**
 * Function computes distance between test vector moments and the
 * stored vector of moments whose index is provided...
 */
double ShapeComponent::distMeasure( int idX, int method ) {
  double resultVal = 0.0;
  double distVal = 1.0e9;

  // Declare (and define) variables to be used later...
  int signA, signB;
  double absA, absB, mA, mB;
  double verySmall = 1.0e-6; // Copied from OpenCV constants for shape
			     // matching...

  // Choose a method based on input argument...
  switch( method ) {
  case 1: {
    // cout << "SC: Using dist method 1...\n";
    for( int i = 0; i < NUM_MOMENTS; ++i ) {
      // Access the individual moment values...
      absA = fabs( testMoments[i] );
      double storedVal = shapeMoments[idX][i];
      absB = fabs( storedVal );

      // Get the 'sign' of both moments...
      signA = ( ( testMoments[i] > 0 ) ? 1 :
		( ( testMoments[i] < 0 ) ? -1 : 0 ) );

      signB = ( ( storedVal > 0 ) ? 1 :
		( ( storedVal < 0 ) ? -1 : 0 ) );
      
      // Compute the logarithmic measure-1, i.e: calculate the value
      // \sigma_i abs( 1/mA_i - 1/mB_i )
      if( absA > verySmall && absB > verySmall ) {
	mA = signA * log10( absA );
	mB = signB * log10( absB );
	resultVal += fabs( 1/mA - 1/mB );
      }
    }
    break;
  }
    
  case 2: {
    // cout << "SC: Using dist method 2...\n";
    for( int i = 0; i < NUM_MOMENTS; ++i ) {
      // Access the individual moment values...
      absA = fabs( testMoments[i] );
      double storedVal = shapeMoments[idX][i];
      absB = fabs( storedVal );

      // Get the 'sign' of both moments...
      signA = ( ( testMoments[i] > 0 ) ? 1 :
		( ( testMoments[i] < 0 ) ? -1 : 0 ) );

      signB = ( ( storedVal > 0 ) ? 1 :
		( ( storedVal < 0 ) ? -1 : 0 ) );
      
   
      // Compute the logarithmic measure-2 (DEFAULT), i.e: calculate
      // the value \sigma_i abs( mA_i - mB_i )
      if( absA > verySmall && absB > verySmall ) {
	mA = signA * log10( absA );
	mB = signB * log10( absB );
	resultVal += fabs( mA - mB );
      }
    }
    break;
  }
  
  case 3: {
    // cout << "SC: Using dist method 3...\n";
    for( int i = 0; i < NUM_MOMENTS; ++i ) {
      // Access the individual moment values...
      absA = fabs( testMoments[i] );
      double storedVal = shapeMoments[idX][i];
      absB = fabs( storedVal );

      // Get the 'sign' of both moments...
      signA = ( ( testMoments[i] > 0 ) ? 1 :
		( ( testMoments[i] < 0 ) ? -1 : 0 ) );

      signB = ( ( storedVal > 0 ) ? 1 :
		( ( storedVal < 0 ) ? -1 : 0 ) );
   
      // Compute the logarithmic measure-3, i.e: calculate the value
      // \sigma_i abs( mA_i - mB_i )/abs( mA_i )
      if( absA > verySmall && absB > verySmall ) {
	mA = signA * log10( absA );
	mB = signB * log10( absB );
	resultVal += ( fabs( mA - mB ) / fabs( mA ) );
      }
    }
    break;
  }
  
  default:
    // SHOULD NEVER EVER GET HERE!!!!
    cout << "SC: Shape vector matching -- unknown method: " 
	 << method << " used...\n";
    resultVal += 1.0e9;
  }

  if( resultVal != 0 ) {
    distVal = resultVal;
  }

  return distVal;
}




/**
 * Function that is used just to write out the histogram values to a
 * file, for later analysis...
 */
void ShapeComponent::writeVectors2File() {
  // Open a file to write to...
  FILE* fpW = fopen( m_trnfilename.c_str(), "a+" );
  // FILE* fpW = fopen( "/home/staff/mzs/cosy/code/Databases/shapeTrial.txt", "a+" );
  if( NULL == fpW ) {
    log("SC: file cannot be opened to write training hists..." );
    return;
  }
  
  // Write to file...
  for( int i = 0; i < NUM_MOMENTS; ++i ) {
    if( i < NUM_MOMENTS - 1 ) {
      fprintf( fpW, "%lf\t", testMoments[i] );
    }
    else {
      fprintf( fpW, "%lf\n", testMoments[i] );
    }
  }

  // Finally close the file and be gone...
  fclose( fpW );
}



/**
 * Function to update the scene object once the shape label has been
 * found...
 */
void ShapeComponent::updateSceneObject( shared_ptr<const Vision::ROI> _pROI, 
					const int& _label ) {
  // Access the scene object to update...
  string sceneObjectID( _pROI->m_objId );
  
  if( !existsOnWorkingMemory( sceneObjectID ) ) {
    log("SC:ROI exists but no scene object -- CRASH!!!!");
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
      string oldLabel(  enum2String( pSceneObject->m_shape.m_int ) );
      log( "old shape: " + oldLabel );
      log( "new shape: " + enum2String(_label) );
      
      // Set the new sceneobject's shape label...
      pNewSceneObject->m_shape.m_int = _label;

      // Also the set the probability measure using the computed
      // probabilities...
      pNewSceneObject->m_shape.m_confidence = 
	m_shapeProbs[ _label - SHAPE_PROPERTIES_OFFSET ];

      cout << "SC: Found " << enum2String(_label) << ", at new pose (" 
	   << pNewSceneObject->m_pose.m_position.m_x << "," 
	   << pNewSceneObject->m_pose.m_position.m_y << ","
	   << pNewSceneObject->m_pose.m_position.m_z << ") with prob: " 
	   << pNewSceneObject->m_shape.m_confidence << endl;

      // Write into WM again, with the newly-found properties...
      try {
	overwriteWorkingMemory<SceneObject>( sceneObjectID, pNewSceneObject );
      } catch( const ConsistencyException &c ) {
	updateSceneObject( _pROI, _label );
      }
    }
    else {
      log("SC:failed to get SceneObject");
    }
  }

}



/**
 * Function inherited from Inspectable component for display
 * purposes...
 */
void ShapeComponent::redrawGraphics2D() {
  // Use previously stored data to set up the display variables...
  if( currImage == NULL ) {
    drawText2D( (int)dispPoint.m_x, (int)dispPoint.m_y, "Nothing to Display", 255, 0, 0, 0 );
    return;
  }

  // We would like the RGB image output on GUI screen...
  drawRGBImage( currImage->width, currImage->height, currImage->imageData, 0 );
  drawText2D( (int)dispPoint.m_x, (int)dispPoint.m_y, dispLabel, 255, 0, 0, 0 );
}



/**
 * Action registration happens here -- for now, the code is commented
 * out...
 */
void ShapeComponent::runComponent() {
//   ActionRegistration shapeOp;
//   shapeOp.m_component = CORBA::string_dup( getProcessIdentifier().c_str() );
//   shapeOp.m_subarchitecture = CORBA::string_dup( m_subarchitectureID.c_str() ); // archID.c_str() );
//   shapeOp.m_action = CORBA::string_dup( string("shape-detector").c_str() );
//   addToWorkingMemory( newDataID(), PlanningOntology::ACTION_REGISTRATION_TYPE, new ActionRegistration( shapeOp ) );
}


