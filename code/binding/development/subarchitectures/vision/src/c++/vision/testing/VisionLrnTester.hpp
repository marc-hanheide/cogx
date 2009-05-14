#ifndef VISION_TESTER_H
#define VISION_TESTER_H

#include <vision/idl/Vision.hh>
#include <binding/idl/BindingData.hh>
#include <vision/testing/AbstractVisionTester.hpp>
//#include <cast/architecture/WorkingMemoryChangeReceiver.hpp>


using namespace cast;
using namespace boost;

class VisionLrnTester : public AbstractVisionTester 
{
 protected:

    class ChangeDetectorTest : public AbstractVisionTest 
    {
      public:
          ChangeDetectorTest(AbstractVisionTester & _tester) :
              AbstractVisionTest(_tester) {};
          
      protected:
        virtual void startTest();
        bool verify_parameters();
        void HandleAddSceneChanged(const cdl::WorkingMemoryChange &change);
        void HandleSceneChanged(const cdl::WorkingMemoryChange &change);
      private:
        int m_phase;
    };
    
    
    class SegmentorTest : public AbstractVisionTest 
    {
      public:
          SegmentorTest(AbstractVisionTester & _tester) :
              AbstractVisionTest(_tester) {};
          
      protected:
          virtual void startTest();
          void HandleAddSceneChanged(const cdl::WorkingMemoryChange &change);
          void HandleSceneChanged(const cdl::WorkingMemoryChange &change);
      private:
        int m_phase;
    };
    
    
    class DrySegmentorTest : public AbstractVisionTest 
    {
      public:
          DrySegmentorTest(AbstractVisionTester & _tester) :
            AbstractVisionTest(_tester) {};
          
      protected:
        virtual void startTest();
        void HandleSceneChanged(const cdl::WorkingMemoryChange &change);
      private:
        int m_phase;
    };

    class HandPointerTest : public AbstractVisionTest 
    {
      public:
          HandPointerTest(AbstractVisionTester & _tester) :
              AbstractVisionTest(_tester) {};
          
      protected:
          virtual void startTest();
          void HandleSceneChanged(const cdl::WorkingMemoryChange &change);
      private:
        int m_phase;
    };
    
    
    class FeatureExtractorTest : public AbstractVisionTest 
    {
      public:
          FeatureExtractorTest(AbstractVisionTester & _tester) :
            AbstractVisionTest(_tester) {};
          
      protected:
        virtual void startTest();
        void HandleROIChanged(const cdl::WorkingMemoryChange &change);
      private:
        int m_phase;
    };
    
    
    class RecogniserTest : public AbstractVisionTest 
    {
      public:
          RecogniserTest(AbstractVisionTester & _tester) :
            AbstractVisionTest(_tester) {};
          
      protected:
        virtual void startTest();
        void HandleROIChanged(const cdl::WorkingMemoryChange &change);
        void HandleSceneObjectChanged(const cdl::WorkingMemoryChange &change);
      private:
        int m_phase;
    };

    
    class DryBindingMonitorTest : public AbstractVisionTest 
    {
      public:
          DryBindingMonitorTest(AbstractVisionTester & _tester) :
            AbstractVisionTest(_tester) {};
          
      protected:
        virtual void startTest();
        void HandleBindingProxyAdded(const cdl::WorkingMemoryChange &change);
        void HandleSceneObjectAdded(const cdl::WorkingMemoryChange &change);
      private:
        int m_phase;
    };

    
    friend class ChangeDetectorTest;
    friend class SegmentorTest;
    friend class DrySegmentorTest;
    friend class HandPointerTest;
    friend class FeatureExtractorTest;
    friend class RecogniserTest;
    friend class DryBindingMonitorTest;

 public:
    VisionLrnTester(const std::string &_id);
    virtual ~VisionLrnTester();

    virtual void configure(std::map<std::string,std::string> & _config);
};


#endif
