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
#include "ColorComponent.h"


extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new ColorComponent(_id);
  }
}


/**
 * The Constructor...
 */
ColorComponent::ColorComponent(const string &_id) : 
    //FrameworkProcess(_id),
    WorkingMemoryAttachedComponent(_id),
    VideoClientProcess(_id),
    m_curTaskID(""),
    roiImg(NULL),
    currImage(NULL) {
  cout << "creating ColorComponent::ColorComponent" << endl;

  // Initialize the array storing color probabilities...
  for( int i = 0; i < NUM_COLORS; ++i ) {
    m_colorProbs[i] = 0.0;
  }
  dispLabel = string("UNKNOWN");
  dispPoint.m_x = 0.0;
  dispPoint.m_y = 0.0;
  currImage = NULL;

  // Initialize properties for the colors...
  m_numColors = NUM_COLORS;
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
ColorComponent::~ColorComponent() {
  cout << "ColorComponent::~ColorComponent()" << endl;
  
  // Clear up memory...
  m_taskActionMap.clear();
  
  if( roiImg != NULL ) {
    cvReleaseImage( &roiImg );
  }
  if( currImage != NULL ) {
    cvReleaseImage( &currImage );
  }
}




/**
 * Function that sets some essential parameters for the class
 * variables...
 */
void ColorComponent::configure( map<string,string>& _config ) {
  // Propagate paramaters to the higher-levels as well...
  VideoClientProcess::configure(_config);

  // Sets the stream for reading the configuration values...
  ostringstream outStream;

  // Setting number of color classes being considered...
  if(_config["-c"] != "") {
    istringstream configStream(_config["-c"]);
    configStream >> m_numColors;

    // We are in trouble if the allocated number exceeds the maximum
    // size of arrays set...
    if( m_numColors > NUM_COLORS ) {
      log( "CC: config, num colors exceeds limit -- CRASH!!" );
    }

    outStream.str("");
    outStream << "setting number of colors : " << m_numColors;
    log( outStream.str() );
  }
  // Setting the number of samples for each color class...
  if(_config["-n"] != "") {
    istringstream configStream(_config["-n"]);
    configStream >> m_numSamples;

    // We are in trouble if the allocated number exceeds the maximum
    // size of arrays set...
    if( m_numSamples > NUM_SAMPLES ) {
      log( "CC: config, num samples per class exceeds limit -- CRASH!!" );
    }

    outStream.str("");
    outStream << "setting number of samples per class: "<< m_numSamples;
    log(outStream.str());
  }

  // Setting the threshold on the probabilities for deciding on the
  // 'extra' class labels of 'empty'...
  if(_config["-e"] != "") {
    istringstream configStream(_config["-e"]);
    configStream >> m_emptyClassProbThresh;

    // The threshold for class label 'empty' cannot be greater than a
    // certain maximum value...
    if( m_emptyClassProbThresh > EMPTY_CLASS_THRESH ) {
      log( "CC: config, empty class label thresh too large -- resetting" );
      m_emptyClassProbThresh = 1./m_numColors;
    }
    
    outStream.str("");
    outStream << "CC: setting threshold for class label empty: "
	      << m_emptyClassProbThresh;
    log(outStream.str());
  }

  // Setting the threshold on the probabilities for deciding on the
  // 'extra' class labels of 'empty'...
  if(_config["-u"] != "") {
    istringstream configStream(_config["-u"]);
    configStream >> m_unknownClassProbRatioThresh;

    // The threshold for ratio of best to second-best match, which
    // decides on the class label 'unknown', cannot be greater than a
    // certain maximum value...
    if( m_unknownClassProbRatioThresh > UNKNOWN_CLASS_THRESH ) {
      log( "CC: config, unknown class label thresh too large -- resetting" );
      m_unknownClassProbRatioThresh = UNKNOWN_CLASS_THRESH;
    }
    
    outStream.str("");
    outStream << "CC: setting threshold for class label unknown: "
	      << m_unknownClassProbRatioThresh;
    log(outStream.str());
  }

  // Figure out the database to load -- previously trained samples...
  if(_config["-f"] != "") {
    m_dbfilename = _config["-f"];
    log( "CC:dbname " + m_dbfilename );
  }
  else {
    m_dbfilename = "";
    log( "CC:dbname = noname" + m_dbfilename );
  }

  // Figure out the file to store the training samples into -- TO BE
  // USED ONLY DURING TRAINING...
  if(_config["-t"] != "") {
    m_trnfilename = _config["-t"];
    log( "CC:trnfilename " + m_trnfilename );
    m_writeHists = true;
  }
  else {
    m_trnfilename = "";
    log( "CC:trfilename = noname" + m_trnfilename );
    m_writeHists = false;
  }

  // Load the training set...
  loadDataBase(m_dbfilename);
}



/**
 * Function loads the color hists database into the memory...
 */
void ColorComponent::loadDataBase( string strDBFile ) {
  FILE* fp = fopen(strDBFile.c_str(), "r+");
  if( fp == NULL ) {
    log( "CC:cannot open file: " + strDBFile );
    return;
  }
  
  for( int i = 0; i < m_numColors*m_numSamples; ++i ) {
    for( int j = 0; j < NUM_BINS; ++j ) {
      fscanf( fp, "%lf", &colorHists[i][j] );
//       cout << "Value (" << i << "," << j << ": "
// 	   << colorHists[i][j] << "), ";
    }
    cout << "Read values of Hist vector " << i << "\n";
  }
  
  fclose(fp);
}




/**
 * The function that starts it all...
 */
void ColorComponent::start() {
  VideoClientProcess::start();
  // Add a change filter that responds to changes in the WM, as result
  // of the planning process...
  //   addChangeFilter( createLocalTypeFilter<ACTION>(cdl::ADD),
  // 		   new MemberFunctionChangeReceiver<ColorComponent>( this,
  // 								     &ColorComponent::handlePlanningActionRequest ) );
  
  // TEMPORARY -- FOR TESTING ALONE...
  addChangeFilter(createLocalTypeFilter<ROI>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ColorComponent>( this,
								    &ColorComponent::handleROIColorDetection ) );
}




/**
 * Function that responds to a ROI addition in vision's WM, resulting
 * in a color detection task...
 */
void ColorComponent::handleROIColorDetection( const cdl::WorkingMemoryChange& _wmc ) {
  log( "CC:New ROI in visual WM... need to detect color..." );

  // Get the data that caused the changein WM...
  shared_ptr<const CASTData<Vision::ROI> > pROIData
    = getWorkingMemoryEntry<Vision::ROI>( _wmc.m_address );

  int colorLabel = -1;
  
  // If the ROI data exists in the WM, get a shared pointer to it...
  if( pROIData ) {
    shared_ptr<const Vision::ROI> pROI = pROIData->getData();
    if( pROI ) {
      // Determine the color label by comparing with stored histograms...
      colorLabel = doCOLOR( pROI );

      // Update the corresponding scene object with the appropriate
      // color label..
      string sceneObjectID( pROI->m_objId );
      if( sceneObjectID != "" ) {
	updateSceneObject( pROI, colorLabel );
      }
      else {
	log( "CC:SceneObject for roi : " + enum2String(colorLabel) + " is missing" );
      }
      
//       cout << "\n CC:Done with processing roi at time " 
// 	   << pROI->m_time.m_s << ":"  << pROI->m_time.m_us << endl;
    }
    else {
      println("CC:failed to get ROI");
    }
  }
}





/**
 * Function that adopts the task based on the list fo tasks proposed
 * before...
 */
void ColorComponent::taskAdopted(const string &_taskID) {
  bool performedColor = false;
  int colorLabel = -1;

  if( !existsOnWorkingMemory( _taskID ) ) {
    log( "ROI no longer exists on WM -- scene changed horribly?" );
  }
  // OK, now we can go ahead and run COLOR...
  else {
    log( "CC:taskAdopt -- ROI data in WM..." );
    shared_ptr<const CASTData<Vision::ROI> > pROIData
      = getWorkingMemoryEntry<Vision::ROI>(_taskID);
    
    // If the pointer to the required ROI exists...
    if( pROIData ) {
      // Get a reference to the actual ROI data...
      shared_ptr<const Vision::ROI> pROI = pROIData->getData();
      if( pROI ) {
	// Go ahead and find the color...
	colorLabel = doCOLOR( pROI );
	string sceneObjectID( pROI->m_objId );
	// If the corresponding scene object exists...
	if( sceneObjectID != "" ) {
	  // Update the color properties of the scene object...
	  updateSceneObject( pROI, colorLabel );
	}
	else {
	  log( "CC:SceneObject for roi : " + enum2String(colorLabel) + " is missing" );
	}
	
// 	cout << "\n CC:Done with processing roi at time " 
// 	     << pROI->m_time.m_s << ":" << pROI->m_time.m_us << endl;

	performedColor = true;
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
void ColorComponent::taskRejected( const string& _taskID ) {

}



/**
 * Function that actually does the color histogram-based comparison,
 * and spews out a label...
 */
int ColorComponent::doCOLOR( shared_ptr<const Vision::ROI> _pROI ) {
  struct timeval tInit, tFin;
  gettimeofday( &tInit, NULL );

  // Initialize the ROI color label...
  int colorLabel = -1;

  log( "CC:Getting image to detect color..." );
  if ((_pROI->m_camNum == Vision::CAM_LEFT) || 
      (_pROI->m_camNum == Vision::CAM_RIGHT)) { 
    // Get the image (ah, at last!)...
    ImageFrame pImg;
    getImage( _pROI->m_camNum, pImg );
    // ImageFrame* pImg = GetImage( _pROI->m_camNum );
    // Now to go on for the actual color of the ROI -- MAGIC;)
    detectColor( &pImg, _pROI->m_bbox, colorLabel );

    // delete(pImg);
  }
  std::cout << "CC: doCOLOR, color label: " 
	    << enum2String( colorLabel ) << endl;

  gettimeofday( &tFin, NULL );
  double timeDiff1 = (tFin.tv_sec - tInit.tv_sec)*1.0 + (tFin.tv_usec - tInit.tv_usec)*1.0e-6;
  cout << "CC: processing time: " << timeDiff1 << endl;

  return colorLabel;
}




/**
 * Function that gets the color of the ROI, from the image sent in as
 * input...
 */
void ColorComponent::detectColor( ImageFrame* _pimg, const BBox2D& _bbox, int& cLabel ) {
  // Bounding box of the ROI under consideration...
  BBox2D orig(_bbox);

  log( "CC: In detect color, getting image data..." );
  // Now get the image data within the input image...
  if( currImage != NULL ) {
    log( "CC: detectColor: prev image exists... trying to release..." );
    cvReleaseImage( &currImage );
    log( "CC: detectColor: released prior image data..." );
  }
  currImage = buffer2image( _pimg );

  // cout << "CC: Got image from buffer...\n";

  // Iterate over the pixels in the IplImage struct and flip the RGB
  // pixel order, since the input image (from videoserver currently
  // uses BGR instead of RGB)...
  int numChannels = currImage->nChannels;
  for( int i = 0; i < currImage->height * currImage->width; ++i ) {
    const char tempC = currImage->imageData[ i*numChannels + 0 ];
    currImage->imageData[ i*numChannels + 0 ] = currImage->imageData[ i*numChannels + 2 ];
    currImage->imageData[ i*numChannels + 2 ] = tempC;
  }

  // Set the box limits of the ROI...
  int topLeft_x  = (int)( orig.m_center.m_x - ( orig.m_size.m_x/2.0 ) );
  int topLeft_y  = (int)( orig.m_center.m_y - ( orig.m_size.m_y/2.0 ) );  

  // Now get the data within the bounding box...
  cvSetImageROI( currImage, cvRect( topLeft_x, topLeft_y, 
				    (int)orig.m_size.m_x, 
				    (int)orig.m_size.m_y ) );

  // Copy the data from original image corresponding to the ROI, onto
  // the separate roi-image...
  if( roiImg != NULL ) {
    log( "CC: detectColor: prev roiImg exists... trying to release..." );
    cvReleaseImage( &roiImg );
    log( "CC: detectColor: released prior roiImg data..." );
  }
  roiImg = cvCreateImage( cvSize( (int)orig.m_size.m_x, 
				  (int)orig.m_size.m_y ), 8, 3 );
  cvCopy( currImage, roiImg );

  // cout << "CC: got roi image...\n";

  // log( "CC: In detect color, getting pixel values..." );
  // Now the data is in roiImg->imageData => get it out into a static
  // (a HACK!) array of RGB vectors...
  int numSamps = 0;
  char* pRgb = roiImg->imageData;
  for( int nRow = 0;  nRow < roiImg->width; ++nRow ) {
    for( int nCol = 0;  nCol < roiImg->height; ++nCol, pRgb += 3 ) {
      rgbPixels[numSamps][0] = *(pRgb);
      rgbPixels[numSamps][1] = *(pRgb+1);
      rgbPixels[numSamps][2] = *(pRgb+2);
      // ((0.299 * (*(pRgb+2)) + 0.587 * (*(pRgb+1)) + 0.114 * (*pRgb))) / 255.0;
      numSamps += 1;
      numSamps = ( numSamps >= NUM_HIST_SAMPS_MAX ? 0 : numSamps );
    }
  }

  // cout << "CC: got pixels from image...\n";
  // Some sanity checks -- do not consider image ROIs that are too
  // small or too big...
  if( numSamps <= NUM_HIST_SAMPS_MIN || numSamps >= NUM_HIST_SAMPS_MAX ) {
    log( "CC:Too few pixel samples in ROI..." );
    cLabel = -1;
    return;
  }

  // Well, next generate the normalized histogram corresponding to
  // this ROI data -- assumes that the rgbPixels have already been
  // populated...
  // log( "CC: Generating histogram..." );
  generateHist( numSamps );
  
  // Compare with stored hists to determine color class -- assumes
  // that standard hists are stored in memory, and that current hist
  // has been generated already...
  log( "CC: Determining color class label..." );
  determineColorClass( cLabel );

  // Now for some display using the GUI-based functions..
  // log( "CC: drawing objects to GUI..." );
  CvScalar dispColor = CV_RGB( 255, 0, 0 );
  double bboxW = _bbox.m_size.m_x;
  double bboxH = _bbox.m_size.m_y;
  double bboxCx = _bbox.m_center.m_x;
  double bboxCy = _bbox.m_center.m_y;

  // Draw a rectangle -- as four lines, just for testing out the line
  // drawing...
  CvPoint topLeft = cvPoint( topLeft_x, topLeft_y );
  CvPoint bottomRight = cvPoint( (int)(bboxCx + bboxW/2.0), 
				 (int)(bboxCy + bboxH/2.0) );
  CvPoint topRight = cvPoint( bottomRight.x, topLeft_y );
  CvPoint bottomLeft = cvPoint( topLeft_x, bottomRight.y );

//   cout << "CC: connecting:(" << topLeft_x << "," << topLeft_y << "),("
//        << topRight.x << "," << topRight.y << "),(" <<  bottomRight.x 
//        << "," << bottomRight.y << "),(" << bottomLeft.x << "," << bottomLeft.y
//        << ")\n";

  cvLine( currImage, topLeft, topRight, dispColor, 2 );
  cvLine( currImage, topRight, bottomRight, dispColor, 2 );
  cvLine( currImage, bottomRight, bottomLeft, dispColor, 2 );
  cvLine( currImage, bottomLeft, topLeft, dispColor, 2 );
  
  dispLabel = enum2String( cLabel );
  dispPoint.m_x = bboxCx - bboxW/4.0;
  dispPoint.m_y = topLeft.y;
  // log( "CC: done drawing objects to GUI..." );
  redrawGraphicsNow();

  // DEBUGGING -- write out the histogram to file...
  if( m_writeHists ) {
    log( "CC: Writing Color Histogram to file..." );
    writeHist2File();
  }
}





/**
 * Function to determine the histogram, assuming that a certain number
 * of samples have already been stored in an array (a statis member
 * variable)...
 */
void ColorComponent::generateHist( int numVals ) {
  // First clear out the histogram array...
  for( int j = 0; j < NUM_BINS; ++j ) {
    testHist[j] = 0.0;
  }

  // Then, populate the array based on current input...
  double sumHistVals = 0.0;
  for( int i = 0; i < numVals; ++i ) {
    int R = rgbPixels[i][0];
    int G = rgbPixels[i][1];
    int B = rgbPixels[i][2];

    int rId = (int)floor( (double)R / SS_RATIO ); 
    int gId = (int)floor( (double)G / SS_RATIO ); 
    int bId = (int)floor( (double)B / SS_RATIO ); 

    // Sanity check on limits of indices...
    rId = ( ( rId > NUM_BINS_DIM-1 ) ? (NUM_BINS_DIM-1) : ( ( rId < 0 ) ? 0 : rId ) );
    gId = ( ( gId > NUM_BINS_DIM-1 ) ? (NUM_BINS_DIM-1) : ( ( gId < 0 ) ? 0 : gId ) );
    bId = ( ( bId > NUM_BINS_DIM-1 ) ? (NUM_BINS_DIM-1) : ( ( bId < 0 ) ? 0 : bId ) );

    int binID = ( bId + ( gId * NUM_BINS_DIM ) + 
		  ( rId * NUM_BINS_DIM * NUM_BINS_DIM ) );

    // Sanity check on bin indices again...
    binID = ( binID >= NUM_BINS ? NUM_BINS-1 : binID );

    // Increment the appropriate bin in the histogram...
    testHist[binID] += 1.0;
    sumHistVals += 1.0;
  }

  // Normalize current histogram to get a pdf...
  // cout << "CC: Normalizing histogram...\n";
  for( int k = 0; k < NUM_BINS; ++k ) {
    testHist[k] = ( testHist[k] / sumHistVals );
    // Print it out for fun...
    // cout << "(Value " << k << " is " << testHist[k] << ")\t";
  }
  
}




/**
 * Function does a simple NNr operation to determine the best-suited
 * color class label (integer values: 1,2,3) using JS as the distance
 * measure...
 */
void ColorComponent::determineColorClass( int& cLabel ) {
  // Compare with each of the stored histograms, using Jensen-Shannon
  // distance measure...
  double minVal = 1.0e9;
  int minIdx = -1;
  double dist[m_numColors*m_numSamples];

  // Variable for computing average within class distances and hence
  // final probabilities of match...
  double avgDistWithinClass[ m_numColors ];
  for( int j = 0; j < m_numColors; ++j ) {
    avgDistWithinClass[j] = 0.0;
  }

  // Compute distance with each training sample + some additional
  // statistics...
  for( int i = 0; i < m_numColors*m_numSamples; ++i ){
    dist[i] = jsDistance(i);
    
    // Maintain the within-class distance average...
    int idx = i/m_numSamples;
    // Just a sanity check...
    idx = ( ( idx >= m_numColors ) ? 
	    ( m_numColors-1 ) : idx );

    // Update the average distance within this color class...
    avgDistWithinClass[idx] += dist[i]/m_numSamples;

    // Find the closest sample in the training set, though it is not
    // going to be used to make the final decision...
    if( dist[i] < minVal ) {
      minVal = dist[i];
      minIdx = i;
    }
  }

  // Compute the sum of the inverse of the individual distances --
  // used for match probability computation below...
  double sumInvDist = 0.0;
  for( int k = 0; k < m_numColors; ++k ) {
    sumInvDist += ( ( avgDistWithinClass[k] > 0 ) ? 
		    1/avgDistWithinClass[k] : 0 );

    // Also clean up the color probabilities array...
    m_colorProbs[k] = 0.0;
  }

  // Just a little check to prevent a divide-by-zero error...
  sumInvDist = ( ( sumInvDist == 0 ) ? ( sumInvDist + 1.0 ) 
		 : sumInvDist );

  // Individual color's probabilities are inversely proportional to
  // the average distances...
  double maxProbVal = 0.0;
  int maxProbIdx = -1;
  for( int a = 0; a < m_numColors; ++a ) {
    m_colorProbs[a] = ( ( avgDistWithinClass[a] == 0 ) ? (1.0/sumInvDist) : 
		      ( ( 1/avgDistWithinClass[a] )/sumInvDist ) );
    cout << "CC: probability of color " << enum2String(a+COLOR_PROPERTIES_OFFSET) 
	 << " is: " << m_colorProbs[a] << endl;
    
    // Also compute the class with highest probability...
    if( m_colorProbs[a] > maxProbVal ) {
      maxProbVal = m_colorProbs[a];
      maxProbIdx = a;
    }
  }

  // Also determine the second-highest probability of match...
  double max2ProbVal = 0.0;
  int max2ProbIdx = -1;
  for( int b = 0; b < m_numColors; ++b ) {
    if( b == maxProbIdx ) {
      continue;
    }
    if( m_colorProbs[b] > max2ProbVal ) {
      max2ProbVal = m_colorProbs[b];
      max2ProbIdx = b;
    }
  }

  // Determine the class label of the sample, including the labels of
  // 'unknown' and 'empty', based on the probability of match with
  // each of the color classes...
  if( m_colorProbs[ maxProbIdx ] < m_emptyClassProbThresh ) {
    cout << "CC: setting label empty, (id,prob): ("
	 << maxProbIdx << ", " << m_colorProbs[ maxProbIdx ] << ")\n";
    maxProbIdx = -1; // Change the value depending on index of
		    // 'EMPTY'...
  }
  else if( ( m_colorProbs[ max2ProbIdx ] / m_colorProbs[ maxProbIdx ] ) > 
	   m_unknownClassProbRatioThresh ) {
    cout << "CC: setting label unknown, (id,prob): ("
	 << maxProbIdx << ", " << m_colorProbs[ maxProbIdx ] << "), ("
	 << max2ProbIdx << ", " << m_colorProbs[ max2ProbIdx ] << ")\n";
    maxProbIdx = -1;
  }
  
  // The actual string representing the class label needs to be
  // obtained by indexing into an enum with a suitable offset...
  //   int actIdx = ( ( minIdx >= 0 ) ? ( minIdx/m_numSamples + 
  // 				     COLOR_PROPERTIES_OFFSET ) : (-1) );
  int actualIdx = -1;
  if( maxProbIdx >= 0 ) {
    actualIdx = maxProbIdx + COLOR_PROPERTIES_OFFSET;
    cout << "\n CC: (Value,ID) = (" << minVal << ", " << minIdx 
	 << "), avg (Value, ID) = (" << avgDistWithinClass[maxProbIdx] 
	 << ", " << maxProbIdx << "), actual color: " 
	 << enum2String( actualIdx ) << endl;
  }
  else {
    cout << "\n CC: (Value,ID) = (" << minVal << ", " << minIdx 
	 << ",) prob: " << maxProbIdx << ", actual color: " 
	 << enum2String( actualIdx ) << endl;
  }

  // Time to send the color label back for the rest of the world to
  // deal with...
  cLabel = actualIdx;
}




/**
 * Function computes jsDistance between test histogram and the staored
 * histogram whose index is provided...
 */
double ColorComponent::jsDistance( int idX ) {
  double jsVal = 0.0;
  double klAM = 0.0;
  double klBM = 0.0;

  // A really small number addded to all quantities to get rid of
  // divide-by-zero errors...
  double minAdd = 0.0001;

  double m[512];

  for( int i = 0; i < NUM_BINS; ++i ) {
    double a = ( testHist[i] + minAdd );
    double b = ( colorHists[idX][i] + minAdd );
    
    m[i] = ( ( a + b ) / 2.0 );
    
    klAM += a * std::log( a / m[i] );
    klBM += b * std::log( b / m[i] );
    
    jsVal += ( klAM + klBM )/2.0;
  }
  return jsVal;

}




/**
 * Function that is used just to write out the histogram values to a
 * file, for later analysis...
 */
void ColorComponent::writeHist2File() {
  // Open a file to write to...
  FILE* fpW = fopen( m_trnfilename.c_str(), "a+" );
  // FILE* fpW = fopen( "/home/staff/mzs/cosy/code/Databases/shapeTrial.txt", "a+" );
  if( NULL == fpW ) {
    log("SC: file cannot be opened to write training hists..." );
    return;
  }

  // Write to file...
  for( int i = 0; i < NUM_BINS; ++i ) {
    if( i < NUM_BINS - 1 ) {
      fprintf( fpW, "%lf\t", testHist[i] );
    }
    else {
      fprintf( fpW, "%lf\n", testHist[i] );
    }
  }

  // Finally close the file and be gone...
  fclose( fpW );
}






/**
 * Function to update the scene object once the color label has been
 * found...
 */
void ColorComponent::updateSceneObject( shared_ptr<const Vision::ROI> _pROI, 
					const int& _label ) {
  // Access the scene object to update...
  string sceneObjectID( _pROI->m_objId );
  
  if( !existsOnWorkingMemory( sceneObjectID ) ) {
    log("CC:ROI exists but no scene object -- CRASH!!!!");
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
      string oldLabel(  enum2String( pSceneObject->m_color.m_int ) );
      log( "old color: " + oldLabel );
      log( "new color: " + enum2String(_label) );
      
      // Set the new sceneobject's color label...
      pNewSceneObject->m_color.m_int = _label;

      // Also the set the probability measure using the computed
      // probabilities...
      pNewSceneObject->m_color.m_confidence = m_colorProbs[ _label - 1 ];

      cout << "CC: Found " << enum2String(_label) << ", at new pose (" 
	   << pNewSceneObject->m_pose.m_position.m_x << "," 
	   << pNewSceneObject->m_pose.m_position.m_y << ","
	   << pNewSceneObject->m_pose.m_position.m_z << ") with prob: " 
	   << pNewSceneObject->m_color.m_confidence << endl;

      // Write into WM again, with the newly-found properties...
      try {
	overwriteWorkingMemory<SceneObject>( sceneObjectID, pNewSceneObject );
      } catch( const ConsistencyException &c ) {
	updateSceneObject( _pROI, _label );
      }
    }
    else {
      log("CC:failed to get SceneObject");
    }
  }
}




/**
 * Function inherited from Inspectable component for display
 * purposes...
 */
void ColorComponent::redrawGraphics2D() {
  // Use previously stored data to set up the display variables...
  if( currImage == NULL ) {
    log( "CC: no image to display on screen..." );
    drawText2D( (int)dispPoint.m_x, (int)dispPoint.m_y, "Nothing to Display", 255, 255, 255, 0 );
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
void ColorComponent::runComponent() {
//   ActionRegistration colorOp;
//   colorOp.m_component = CORBA::string_dup( getProcessIdentifier().c_str() );
//   colorOp.m_subarchitecture = CORBA::string_dup( m_subarchitectureID.c_str() ); // archID.c_str() );
//   colorOp.m_action = CORBA::string_dup( string("color-detector").c_str() );
//   addToWorkingMemory( newDataID(), PlanningOntology::ACTION_REGISTRATION_TYPE, new ActionRegistration( colorOp ) );
}


