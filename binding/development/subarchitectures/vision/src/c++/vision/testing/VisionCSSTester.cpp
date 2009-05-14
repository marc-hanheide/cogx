#include "VisionCSSTester.hpp"

#include <opencv/highgui.h>
#include <balt/interface/BALTTimer.hpp>

#include <vector>
#include <string>
#include <cast/architecture/ChangeFilterFactory.hpp>

using namespace std;
using namespace boost;
using namespace cast::cdl;

using namespace Vision;

using namespace FrameworkBasics;


extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new VisionCSSTester(_id);
  }
}


/**
 *  This tester verifies that the input ROI's color is determined
 *  properly...
 */ 
void VisionCSSTester::ColorTest::HandleSceneObjectEvent(const cdl::WorkingMemoryChange &_wmc) {
  shared_ptr<const CASTTypedData<SceneObject> > spCastSO =
    getWorkingMemoryEntry<SceneObject>(_wmc.m_address);    
  shared_ptr<const SceneObject> read_scObj = spCastSO->getData(); 

  m_numAnalyzed += 1;
  cout << "CT: Object: " << m_numAnalyzed << ", reading color: " 
       << read_scObj->m_color.m_int << endl;
  
  if( read_scObj->m_color.m_int == TARGET_COLOR ) {
    cout << "Found color: " << read_scObj->m_color.m_int << endl;
    m_numCorrect += 1;
  }
  else {
    cout << "No valid color: " << read_scObj->m_color.m_int << endl;
  }
}


void VisionCSSTester::ColorTest::startTest() {
  try {
    cout << "*** ColorTest start test ***\n";
    
    addChangeFilter(createLocalTypeFilter<SceneObject>(cdl::OVERWRITE),
		    new MemberFunctionChangeReceiver<ColorTest>(this, &ColorTest::HandleSceneObjectEvent));
    
    // Wait for the color operator to work on sequence...
    sleep(OPERATORS_TIME);

    cout << "CT stats: " << m_numCorrect << ", " << m_numAnalyzed << endl;
    // Then check is any valid objects were seen, and if yes, indicate
    // termination...
    if( m_numAnalyzed >= 1 && m_numCorrect >= 0.5 * m_numAnalyzed ) {
      cout << "CT: test complete..." << endl;
      testComplete(true);
      sleep(SPECIAL_SLEEP_TIME);
    } else {
      cout << "CT: test not complete..." << endl;
      testComplete(false);
      sleep(SPECIAL_SLEEP_TIME);
    }
    
  } catch (CASTException &e) {
    cerr<<e.what()<<endl;     
    testComplete(false);
    sleep(SPECIAL_SLEEP_TIME);
    sleep(2);
  }
}




/**
 *  This tester verifies that the input ROI's shape is determined
 *  properly...
 */ 
void VisionCSSTester::ShapeTest::HandleSceneObjectEvent(const cdl::WorkingMemoryChange &_wmc) {
  shared_ptr<const CASTTypedData<SceneObject> > spCastSO =
    getWorkingMemoryEntry<SceneObject>(_wmc.m_address);    
  shared_ptr<const SceneObject> read_scObj = spCastSO->getData(); 

  m_numAnalyzed += 1;
  cout << "ST: Object: " << m_numAnalyzed << ", reading shape: " 
       << read_scObj->m_shape.m_int << endl;
  
  if( read_scObj->m_shape.m_int == TARGET_SHAPE ) {
    cout << "Found shape: " << read_scObj->m_shape.m_int << endl;
    m_numCorrect += 1;
  }
  else {
    cout << "No valid shape: " << read_scObj->m_shape.m_int << endl;
  }
}


void VisionCSSTester::ShapeTest::startTest() {
  try {
    cout << "*** ShapeTest start test ***\n";
    
    addChangeFilter( createLocalTypeFilter<SceneObject>(cdl::OVERWRITE),
		     new MemberFunctionChangeReceiver<ShapeTest>( this, &ShapeTest::HandleSceneObjectEvent ) );
    
    // Wait for the shape operator to work on sequence...
    sleep(OPERATORS_TIME);

    cout << "ST stats: " << m_numCorrect << ", " << m_numAnalyzed << endl;
    // Then check is any valid objects were seen, and if yes, indicate
    // termination...
    if( m_numAnalyzed >= 1 && m_numCorrect >= 0.5 * m_numAnalyzed ) {
      cout << "ST: test complete..." << endl;
      testComplete(true);
      sleep(SPECIAL_SLEEP_TIME);
    } else {
      cout << "ST: test not complete..." << endl;
      testComplete(false);
      sleep(SPECIAL_SLEEP_TIME);
    }
    
  } catch (CASTException &e) {
    cerr<<e.what()<<endl;     
    testComplete(false);
    sleep(SPECIAL_SLEEP_TIME);
  }
}




/**
 *  This tester verifies that the input ROI's perceptual groups are
 *  determined properly...
 */ 
void VisionCSSTester::PGTest::HandleSceneObjectEvent(const cdl::WorkingMemoryChange &_wmc) {
  shared_ptr<const CASTTypedData<SceneObject> > spCastSO =
    getWorkingMemoryEntry<SceneObject>(_wmc.m_address);    
  shared_ptr<const SceneObject> read_scObj = spCastSO->getData(); 

  m_numAnalyzed += 1;
  cout << "PGT: Object: " << m_numAnalyzed << ", reading pg: " 
       << read_scObj->m_pgshape.m_int << endl;

  if( read_scObj->m_pgshape.m_int == TARGET_PG ) {
    cout << "Found pg: " << read_scObj->m_pgshape.m_int << endl;
    m_numCorrect += 1;
  }
  else {
    cout << "No valid pg: " << read_scObj->m_pgshape.m_int << endl;
  }
}


void VisionCSSTester::PGTest::startTest() {
  try {
    cout << "*** PGTest start test ***\n";
    
    addChangeFilter(createLocalTypeFilter<SceneObject>(cdl::OVERWRITE),
		    new MemberFunctionChangeReceiver<PGTest>(this, &PGTest::HandleSceneObjectEvent));
    
    // Wait for the color operator to work on sequence...
    sleep(OPERATORS_TIME);

    cout << "PGT stats: " << m_numCorrect << ", " << m_numAnalyzed << endl;
    // Then check is any valid objects were seen, and if yes, indicate
    // termination...
    if( m_numAnalyzed >= 1 && m_numCorrect >= 0.5 * m_numAnalyzed ) {
      cout << "PGT: test complete..." << endl;
      testComplete(true);
      sleep(SPECIAL_SLEEP_TIME);
    } else {
      cout << "PGT: test not complete..." << endl;
      testComplete(false);
      sleep(SPECIAL_SLEEP_TIME);
    }
    
  } catch (CASTException &e) {
    cerr<<e.what()<<endl;     
    testComplete(false);
    sleep(SPECIAL_SLEEP_TIME);
    sleep(2);
  }
}




/**
 * The 'baap' of all tests -- tests for color, shape and sift
 * properties of input sceneobjects and maintains statistics over the
 * image stream...
 */
void VisionCSSTester::AllCSSComponentsTest::HandleSceneObjectEvent(const cdl::WorkingMemoryChange &_wmc) {
  shared_ptr<const CASTTypedData<SceneObject> > spCastSO =
    getWorkingMemoryEntry<SceneObject>(_wmc.m_address);    
  shared_ptr<const SceneObject> read_scObj = spCastSO->getData(); 
  // unsigned total_rois = read_scObj->m_ROIsMemoryIDs.length();

  // bool roiSeen = false;
  map<string, vector<int> >::iterator iter = 
    roiMemoryOps.find( (string)read_scObj->m_ROIsMemoryIDs[0] );

  if( iter == roiMemoryOps.end() ) {
    vector<int> tempVec;    tempVec.clear();
    roiMemoryOps.insert( make_pair( (string)read_scObj->m_ROIsMemoryIDs[0], tempVec ) );
    m_cssAnalyzed += 1;
    cout << "CSS: reading new SceneObj " << m_cssAnalyzed
	 << ": (" << read_scObj->m_color.m_int << "," 
	 << read_scObj->m_shape.m_int << "," 
	 << read_scObj->m_label.m_string << ")" << endl;
  } else {
    cout << "CSS: reading old SceneObj " << m_cssAnalyzed
	 << ": (" << read_scObj->m_color.m_int << "," 
	 << read_scObj->m_shape.m_int << "," 
	 << read_scObj->m_label.m_string << ")" << endl;
  }

  vector<int> opsPerformed = roiMemoryOps[(string)read_scObj->m_ROIsMemoryIDs[0]];

  // Color check...
  if( read_scObj->m_color.m_int == TARGET_COLOR ) {
    cout << "Found color: " << read_scObj->m_color.m_int << endl;
    bool colorChecked = false;
    for( size_t i = 0; i < opsPerformed.size(); ++i ) {
      if( opsPerformed.at(i) == COLOR_OP ) {
	colorChecked = true;	break;
      }
    }
    m_colorCorrect = ( colorChecked ? m_colorCorrect : m_colorCorrect + 1 );
    if( !colorChecked ) {
      roiMemoryOps[(string)read_scObj->m_ROIsMemoryIDs[0]].push_back( COLOR_OP );
    }
  } else {
    cout << "No valid color: " << read_scObj->m_color.m_int << endl;
  }

  // Shape check...
  if( read_scObj->m_shape.m_int == TARGET_SHAPE ) {
    cout << "Found shape: " << read_scObj->m_shape.m_int << endl;
    bool shapeChecked = false;
    for( size_t i = 0; i < opsPerformed.size(); ++i ) {
      if( opsPerformed.at(i) == SHAPE_OP ) {
	shapeChecked = true;	break;
      }
    }

    m_shapeCorrect = ( shapeChecked ? m_shapeCorrect : m_shapeCorrect + 1 );
    if( !shapeChecked ) {
      roiMemoryOps[(string)read_scObj->m_ROIsMemoryIDs[0]].push_back( SHAPE_OP );
    }

  } else {
    cout << "No valid shape: " << read_scObj->m_shape.m_int << endl;
  }

  // SIFT check -- coming soon...


  // Perceptual grouping check...
  if( read_scObj->m_pgshape.m_int == TARGET_PG ) {
    cout << "Found pg: " << read_scObj->m_pgshape.m_int << endl;
    bool pgChecked = false;
    for( size_t i = 0; i < opsPerformed.size(); ++i ) {
      if( opsPerformed.at(i) == PG_OP ) {
	pgChecked = true;	break;
      }
    }

    m_pgCorrect = ( pgChecked ? m_pgCorrect : m_pgCorrect + 1 );
    if( !pgChecked ) {
      roiMemoryOps[(string)read_scObj->m_ROIsMemoryIDs[0]].push_back( PG_OP );
    }

  } else {
    cout << "No valid shape: " << read_scObj->m_pgshape.m_int << endl;
  }

  // Clean up...
  opsPerformed.clear();
}



void VisionCSSTester::AllCSSComponentsTest::startTest() {
  try {
    cout << "*** AllCSSComponentsTest start test ***\n";
    addChangeFilter(createLocalTypeFilter<SceneObject>(cdl::OVERWRITE),
		    new MemberFunctionChangeReceiver<AllCSSComponentsTest>(this, &AllCSSComponentsTest::HandleSceneObjectEvent));
    
    // Wait for the operators to go to work...
    sleep(OPERATORS_TIME_TOTAL);
    
    cout << "CSS stats: Color: " << m_colorCorrect << ", Shape: "
	 << m_shapeCorrect << ", sift: " << m_siftCorrect 
	 << ", pg: " << m_pgCorrect << ", Total: " 
	 << m_cssAnalyzed << endl;

    if( m_cssAnalyzed >= 1 && 
	m_colorCorrect >= 0.5 * m_cssAnalyzed &&
	m_shapeCorrect >= 0.5 * m_cssAnalyzed &&
	m_pgCorrect >= 0.5 * m_cssAnalyzed ) {
      cout << "CSS: test complete..." << endl;
      testComplete(true);
      sleep(SPECIAL_SLEEP_TIME);
    } else {
      cout << "CSS: test not complete..." << endl;
      testComplete(false);
      sleep(SPECIAL_SLEEP_TIME);
    }
    
  } catch( CASTException &e ) {
    cerr << e.what() << endl;
    testComplete(false);
    sleep(SPECIAL_SLEEP_TIME);
  }
}




// Constructor...
VisionCSSTester::VisionCSSTester(const string &_id)
  : WorkingMemoryAttachedComponent(_id),
      AbstractVisionTester(_id)  {
}


// Destructor...
VisionCSSTester::~VisionCSSTester(){
} 


/**
 * Configure and set up all the tests under consideration...
 */
void VisionCSSTester::configure(std::map<std::string,std::string> & _config) {        
  shared_ptr<ColorTest> color_test( new ColorTest(*this) );
  registerTest("color_test", color_test);
  
  shared_ptr<ShapeTest> shape_test( new ShapeTest(*this) );
  registerTest("shape_test", shape_test);

  shared_ptr<PGTest> pg_test( new PGTest(*this) );
  registerTest("pg_test", pg_test);

  shared_ptr<AllCSSComponentsTest> allCSS_test( new AllCSSComponentsTest(*this) );
  registerTest("allCSS_test", allCSS_test);

  AbstractVisionTester::configure(_config);    
}

