#ifndef VISION_TESTER_H
#define VISION_TESTER_H

#include <map>
#include <string>
#include <vision/idl/Vision.hh>
#include <vision/testing/AbstractVisionTester.hpp>

//#include <vision/VisionOntology.h>

using namespace cast; using namespace boost;

class VisionTester : public AbstractVisionTester
{
 protected:
    class CameraIdealReadTest : public AbstractVisionTest 
	{
	public:
	    CameraIdealReadTest(AbstractVisionTester & _tester) :
	      AbstractVisionTest(_tester) {};
	    
	protected:
	    virtual void startTest();
	};
    
    class VideoReadTest : public AbstractVisionTest 
	{
	public:
	    VideoReadTest(AbstractVisionTester & _tester) :
		AbstractVisionTest(_tester) {};
	    
	protected:
	    virtual void startTest();
	};
    

    class TrackByRoiTest : public AbstractVisionTest
    {
    public:
	TrackByRoiTest(AbstractVisionTester & _tester) :
	    AbstractVisionTest(_tester) {};
	
    protected:
	virtual void startTest();
	void HandleSceneObjectEvent(const cdl::WorkingMemoryChange &_wmc);
    };


    class AllBasicComponentsTest : public AbstractVisionTest
    {
	struct LifeSpan {
	    FrameworkBasics::BALTTime start;
	    FrameworkBasics::BALTTime end;
	};

    public:
	AllBasicComponentsTest(AbstractVisionTester & _tester) :
	    AbstractVisionTest(_tester),
	    m_numROIs(0) {};
		
    protected:
	unsigned m_numROIs;
	std::map<std::string, LifeSpan> objectSet;
	virtual void startTest();
	void HandleSceneObjectEvent(const cdl::WorkingMemoryChange &_wmc);
        
      private:
        unsigned _sceneObjOVR;
    };


    // Make the test class "friend"
    friend class CameraIdealReadTest;
    friend class VideoReadTest;
    friend class TrackByRoiTest;
    friend class AllBasicComponentsTest;

 public:
    VisionTester(const std::string &_id);
    virtual ~VisionTester();

    virtual void configure(std::map<std::string,std::string> & _config);
};


#endif
