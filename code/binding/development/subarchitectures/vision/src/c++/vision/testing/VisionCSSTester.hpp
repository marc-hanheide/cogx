#ifndef VISION_CSS_TESTER_H
#define VISION_CSS_TESTER_H

#include <map>
#include <vector>
#include <string>
#include <vision/idl/Vision.hh>
#include <vision/testing/AbstractVisionTester.hpp>

#define OPERATORS_TIME 100
#define OPERATORS_TIME_TOTAL 140
#define SPECIAL_SLEEP_TIME 2

#define TARGET_COLOR RED
#define TARGET_SHAPE SQUARED
#define TARGET_PG RECTANGULAR

#define COLOR_OP 0
#define SHAPE_OP 1
#define SIFT_OP 2
#define PG_OP 3

using namespace std;
using namespace cast; using namespace boost;

class VisionCSSTester : public AbstractVisionTester {
protected:
  // The class for the color testing...
  class ColorTest : public AbstractVisionTest {  
  public:
    ColorTest(AbstractVisionTester & _tester) :
      AbstractVisionTest(_tester), 
      m_numCorrect(0), m_numAnalyzed(0) {};

  protected:
    virtual void startTest();
    void HandleSceneObjectEvent( const cdl::WorkingMemoryChange &_wmc );
    
  private:
    int m_numCorrect, m_numAnalyzed;
  };
  

  // The class for the shape testing...
  class ShapeTest : public AbstractVisionTest {  
  public:
    ShapeTest(AbstractVisionTester & _tester) :
      AbstractVisionTest(_tester), 
      m_numCorrect(0), m_numAnalyzed(0) {};

  protected:
    virtual void startTest();
    void HandleSceneObjectEvent( const cdl::WorkingMemoryChange &_wmc );
    
  private:
    int m_numCorrect, m_numAnalyzed;
  };


  // The class for the perceptual groups testing...
  class PGTest : public AbstractVisionTest {  
  public:
    PGTest(AbstractVisionTester & _tester) :
      AbstractVisionTest(_tester), 
      m_numCorrect(0), m_numAnalyzed(0) {};

  protected:
    virtual void startTest();
    void HandleSceneObjectEvent( const cdl::WorkingMemoryChange &_wmc );
    
  private:
    int m_numCorrect, m_numAnalyzed;
  };



  // The class for the overall testing...
  class AllCSSComponentsTest : public AbstractVisionTest {
  public:
    AllCSSComponentsTest( AbstractVisionTester & _tester ) :
      AbstractVisionTest(_tester),
      m_colorCorrect(0), m_shapeCorrect(0), 
      m_siftCorrect(0), m_pgCorrect(0),
      m_cssAnalyzed(0) {
      roiMemoryOps.clear();
    };
    
  protected:
    virtual void startTest();
    void HandleSceneObjectEvent(const cdl::WorkingMemoryChange &_wmc);
    // A map between seen sceneobject memory IDs and tests performed,
    // to prevent multiple counts of the same ROI...
    map< string, vector<int> > roiMemoryOps;

  private:
    int m_colorCorrect, m_shapeCorrect, m_siftCorrect, m_pgCorrect, m_cssAnalyzed;
  };
  
  friend class ColorTest;
  friend class ShapeTest;
  friend class PGTest;
  friend class AllCSSComponentsTest;
  
public:
  VisionCSSTester( const std::string &_id );
  virtual ~VisionCSSTester();
  virtual void configure( std::map<std::string,std::string> & _config );

};


#endif
