#include "VisionLrnTester.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <vector>

using namespace std;
using namespace boost;
using namespace cast::cdl;
using namespace cast::cdl::testing;

using namespace Vision;

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new VisionLrnTester(_id);
  }
}


/**
 */ 
void VisionLrnTester::ChangeDetectorTest::startTest()  // called by run()
{    
    try
    {
      m_phase = 0;

      cout << "*** ChangeDetectorTest start test ***\n";


      addChangeFilter(createLocalTypeFilter<SceneChanged>(cdl::ADD),
                      new MemberFunctionChangeReceiver<VisionLrnTester::ChangeDetectorTest>(this, 
                          &VisionLrnTester::ChangeDetectorTest::HandleAddSceneChanged));
 
      addChangeFilter(createLocalTypeFilter<SceneChanged>(cdl::OVERWRITE),
                      new MemberFunctionChangeReceiver<VisionLrnTester::ChangeDetectorTest>(this,
                          &VisionLrnTester::ChangeDetectorTest::HandleSceneChanged));

     sleep(15);
     
     if (m_phase != -1)
     {
        m_phase = -1;
	cout << "Test timeout." << endl;
        testComplete(false);
     }


    } catch (CASTException &e) {
	cerr<<e.what()<<endl;
	testComplete(false); // m_passed = false
    }
}

bool VisionLrnTester::ChangeDetectorTest::verify_parameters()
{
    return true;
}

void VisionLrnTester::ChangeDetectorTest::HandleAddSceneChanged(const cdl::WorkingMemoryChange &change) {
  
  cout << "SceneChanged WM item added.\n";
  
  shared_ptr<const CASTData<Vision::SceneChanged> > pSceneChangedData =
      getWorkingMemoryEntry<Vision::SceneChanged>(CORBA::string_dup(change.m_address.m_id));

  shared_ptr<const Vision::SceneChanged> sceneChanged = pSceneChangedData->getData();



  if(!sceneChanged->m_sceneChanged && !sceneChanged->m_sceneChanging) {
    m_phase=1;
    cout << "Please put an object on desktop.\n";
  }
  else {
    cout << "Wrong property values in SceneChanged structure (phase 0).\n";
    m_phase = -1;
    testComplete(false);
  }
}

void VisionLrnTester::ChangeDetectorTest::HandleSceneChanged(const cdl::WorkingMemoryChange &change) {
  
  shared_ptr<const CASTData<Vision::SceneChanged> > pSceneChangedData
      = getWorkingMemoryEntry<Vision::SceneChanged>(change.m_address);
  shared_ptr<const Vision::SceneChanged> sceneChanged = pSceneChangedData->getData();


  if(m_phase == 2) {
    if(sceneChanged->m_sceneChanged && !sceneChanged->m_sceneChanging) {
      cout << "Scene is stable.\n";
      m_phase = -1;
      testComplete(true);
    }
    else if(sceneChanged->m_sceneChanged && sceneChanged->m_sceneChanging
            || !sceneChanged->m_sceneChanged && !sceneChanged->m_sceneChanging)
    {
      cout << "Wrong property values in SceneChanged structure (phase 2).\n";
      m_phase = -1;
      testComplete(false);
    }
    
  }
  else if(m_phase == 1)
  {
    if(!sceneChanged->m_sceneChanged && sceneChanged->m_sceneChanging) {
      m_phase=2;
      cout << "Scene is changing.\n";
    }
    else
    {
      cout << "Wrong property values in SceneChanged structure (phase 1).\n";
      m_phase = -1;
      testComplete(false);
    }
  }
  else {
    m_phase = -1;
    testComplete(false);
  }
}


void VisionLrnTester::SegmentorTest::startTest() 
{    
    try {
        cout << "*** segmentor test start ***\n";
    
        m_phase = 0;
        addChangeFilter(createLocalTypeFilter<SceneChanged>(cdl::OVERWRITE),
                        new MemberFunctionChangeReceiver<VisionLrnTester::SegmentorTest>(this,
                            &VisionLrnTester::SegmentorTest::HandleSceneChanged));
        addChangeFilter(createLocalTypeFilter<SceneChanged>(cdl::ADD),
                        new MemberFunctionChangeReceiver<VisionLrnTester::SegmentorTest>(this,
                            &VisionLrnTester::SegmentorTest::HandleAddSceneChanged));
        
        cout << "Wait for segmentor to initialize.\n";
        sleep(10);
        cout << "Adding two objects to desktop.\n";
	
	sleep(20);
	
      	if (m_phase != -1)
     	{
          m_phase = -1;
	  cout << "Test timeout." << endl;
          testComplete(false);
	}
       

    } catch (CASTException &e) {
	cerr<<e.what()<<endl;     
	testComplete(false);
    }
}


void VisionLrnTester::SegmentorTest::HandleAddSceneChanged(const cdl::WorkingMemoryChange &change) {
  
  shared_ptr<const CASTData<Vision::SceneChanged> > pSceneChangedData
      = getWorkingMemoryEntry<Vision::SceneChanged>(change.m_address);
  shared_ptr<const Vision::SceneChanged> sceneChanged = pSceneChangedData->getData();


  if(m_phase == 0) {
    if(!sceneChanged->m_sceneChanged && !sceneChanged->m_sceneChanging)
    {
      m_phase = 1;
      cout << "SceneChanged struct added to WM.\n";
    }
    else {
      m_phase = -1;
      cout << "Wrong property values in SceneChanged structure (phase 0).\n";
      testComplete(false);
    }
  }
  else {
    m_phase = -1;
    testComplete(false);
  }
}


void VisionLrnTester::SegmentorTest::HandleSceneChanged(const cdl::WorkingMemoryChange &change) {
  
  shared_ptr<const CASTData<Vision::SceneChanged> > pSceneChangedData
      = getWorkingMemoryEntry<Vision::SceneChanged>(change.m_address);
  shared_ptr<const Vision::SceneChanged> sceneChanged = pSceneChangedData->getData();


  if(m_phase == 1) {
    cout << "Scene is changing.\n";
    
    if(!sceneChanged->m_sceneChanged && sceneChanged->m_sceneChanging)
    {
      m_phase = 2;
    }
    else {
      m_phase = -1;
      cout << "Wrong property values in SceneChanged structure (phase 1).\n";
      testComplete(false);
    }
  }
  else if(m_phase == 2)
  {
    if(sceneChanged->m_sceneChanged && !sceneChanged->m_sceneChanging)
    {
      m_phase = 3;
      cout << "Scene is stable.\n";
    }
    else if(!sceneChanged->m_sceneChanged && sceneChanged->m_sceneChanging)
    {
      m_phase = 2;
    }
    else {
      m_phase = -1;
      cout << "Wrong property values in SceneChanged structure (phase 1).\n";
      testComplete(false);
    }
  }
  else if(m_phase == 3)
  {
    if(!sceneChanged->m_sceneChanged && sceneChanged->m_sceneChanging)
    {
      cout << "Scene has been processed.\n";
      
      std::vector < boost::shared_ptr< const CASTData<Vision::ROI> > > roilist;
      getWorkingMemoryEntries(roilist);
      
      if(roilist.size() == 2)
      {
        m_phase = -1;
        testComplete(true);
      }
      else
      {
        m_phase = -1;
        cout << "Wrong number of objects segmented (phase 3).\n";
        testComplete(false);
      }
    }
    else
    {
      m_phase = -1;
      cout << "Wrong property values in SceneChanged structure (phase 3).\n";
      testComplete(false);
    }
  }
  else {
    m_phase = -1;
    testComplete(false);
  }
}


void VisionLrnTester::DrySegmentorTest::startTest() 
{    
  try {
    cout << "*** segmentor dry test start ***\n";
    
    m_phase = 0;
    addChangeFilter(createLocalTypeFilter<SceneChanged>(cdl::OVERWRITE),
                    new MemberFunctionChangeReceiver<VisionLrnTester::DrySegmentorTest>(this,
                        &VisionLrnTester::DrySegmentorTest::HandleSceneChanged));
    
    Vision::SceneChanged* sc = new Vision::SceneChanged();
    sc->m_sceneChanging = false;
    sc->m_sceneChanged = false;
    sc->m_sceneProcessed = false;
    sc->m_camNum = 0;
    std::string id = newDataID();
    
    addToWorkingMemory<SceneChanged>(id, sc);
    
    sleep(15);

    sc = new Vision::SceneChanged();
    sc->m_sceneChanging = false;
    sc->m_sceneChanged = true;
    sc->m_sceneProcessed = false;
    sc->m_camNum = 0;

    cout << "Simulated scene change.\n";
    overwriteWorkingMemory<SceneChanged>(id, sc);

    sleep(5);
    
    sc = new Vision::SceneChanged();
    sc->m_sceneChanging = false;
    sc->m_sceneChanged = false;
    sc->m_sceneProcessed = false;
    sc->m_camNum = 0;
    
    cout << "Simulated scene change.\n";
    overwriteWorkingMemory<SceneChanged>(id, sc);
    
    sleep(15);

    if(m_phase != -1)
    {
      m_phase = -1;
      cout << "Test timeout.\n";
      testComplete(false);
    }
	

  } catch (CASTException &e) {
    cerr<<e.what()<<endl;     
    testComplete(false);
  }
}

void VisionLrnTester::DrySegmentorTest::HandleSceneChanged(const cdl::WorkingMemoryChange &change) {
  
  shared_ptr<const CASTData<Vision::SceneChanged> > pSceneChangedData
      = getWorkingMemoryEntry<Vision::SceneChanged>(change.m_address);
  shared_ptr<const Vision::SceneChanged> sceneChanged = pSceneChangedData->getData();


  if(m_phase == 0) {
    if(sceneChanged->m_sceneChanged && !sceneChanged->m_sceneChanging)
    {
      m_phase = 1;
    }
    else {
      m_phase = -1;
      cout << "Wrong property values in SceneChanged structure (phase 0).\n";
      testComplete(false);
    }
  }
  else if(m_phase == 1)
  {
    if(!sceneChanged->m_sceneChanged && !sceneChanged->m_sceneChanging)
    {
      m_phase = -1;
      cout << "Scene has been processed.\n";
      testComplete(true);
    }
    else
    {
      m_phase = -1;
      cout << "Wrong property values in SceneChanged structure (phase 1).\n";
      testComplete(false);
    }
  }
  else {
    m_phase = -1;
    testComplete(false);
  }
}


void VisionLrnTester::HandPointerTest::startTest() 
{    
    try {
        cout << "*** handpointer test start ***\n";
    
        m_phase = 0;
        addChangeFilter(createLocalTypeFilter<SceneChanged>(cdl::OVERWRITE),
                        new MemberFunctionChangeReceiver<VisionLrnTester::HandPointerTest>(this,
                            &VisionLrnTester::HandPointerTest::HandleSceneChanged));
        
        cout << "Wait for handpointer to initialize.\n";
        sleep(30);
 	
      	if (m_phase != -1)
     	{
          m_phase = -1;
	  cout << "Test timeout." << endl;
          testComplete(false);
	}
       

    } catch (CASTException &e) {
	cerr<<e.what()<<endl;     
	testComplete(false);
    }
}


void VisionLrnTester::HandPointerTest::HandleSceneChanged(const cdl::WorkingMemoryChange &change) {

  shared_ptr<const CASTData<Vision::SceneChanged> > pSceneChangedData
       = getWorkingMemoryEntry<Vision::SceneChanged>(change.m_address);
  shared_ptr<const Vision::SceneChanged> sceneChanged = pSceneChangedData->getData();

  std::vector<shared_ptr<const CASTData<Vision::HandPointingResults> > > hpr;
  getWorkingMemoryEntries(hpr);
  vector<shared_ptr<const CASTData<HandPointingResults> > >::const_iterator zac = hpr.begin();

  bool handinscene = (*zac)->getData().get()->m_handInScene;
  bool pointerdetected = (*zac)->getData().get()->m_pointerDetected;

  if(sceneChanged->m_sceneChanged && !sceneChanged->m_sceneChanging && !handinscene && pointerdetected)
  {
    cout << "Wrong property values in HandpointingResults structure\n";
    testComplete(false);
  }
  else
  {
    testComplete(true);
  }

}


// Feature Extractor test
void VisionLrnTester::FeatureExtractorTest::startTest() 
{    
  try {
    cout << "*** feature extractor test start ***\n";
    
    m_phase = 0;
    addChangeFilter(createLocalTypeFilter<ROI>(cdl::OVERWRITE),
                    new MemberFunctionChangeReceiver<VisionLrnTester::FeatureExtractorTest>(this,
                        &VisionLrnTester::FeatureExtractorTest::HandleROIChanged));

    sleep(25);

    if(m_phase != -1)
    {
      m_phase = -1;
      cout << "Test timeout.\n";
      testComplete(false);
    }
    
cout << "TEST completed";

    testComplete(true);
    
cout << "OK";

  } catch (CASTException &e) {
    cerr<<e.what()<<endl;     
    testComplete(false);
  }
}

// Feature extractor ROI change test: Checks if ROI updates performed by feature extractor is correct
void VisionLrnTester::FeatureExtractorTest::HandleROIChanged(const cdl::WorkingMemoryChange &change) {
  
  shared_ptr<const CASTData<Vision::ROI> > pROIData
      = getWorkingMemoryEntry<Vision::ROI>(change.m_address);
  shared_ptr<const Vision::ROI> pROI = pROIData->getData();

  if(m_phase == 0) {
    if (pROI->m_features.data_.length() > 0) {
      m_phase = -1;
      cout << "Extracted features have been added to ROI.\n";
      testComplete(true);
    } else {
      m_phase = -1;
      cout << "Extracted features not in ROI (phase 1).\n";
      testComplete(false);
    }
  }
  else {
    m_phase = -1;
    testComplete(false);
  }
}
// Feature Extractor test


// Recogniser test
void VisionLrnTester::RecogniserTest::startTest() 
{    
  try {
    cout << "*** recogniser test start ***\n";
    
    m_phase = 0;
    addChangeFilter(createLocalTypeFilter<ROI>(cdl::OVERWRITE),
                    new MemberFunctionChangeReceiver<VisionLrnTester::RecogniserTest>(this,
                        &VisionLrnTester::RecogniserTest::HandleROIChanged));
    addChangeFilter(createLocalTypeFilter<SceneObject>(cdl::OVERWRITE),
                    new MemberFunctionChangeReceiver<VisionLrnTester::RecogniserTest>(this,
                        &VisionLrnTester::RecogniserTest::HandleSceneObjectChanged));

    sleep(30);

    if(m_phase != -1)
    {
      m_phase = -1;
      cout << "Test timeout.\n";
      testComplete(false);
    }
	

  } catch (CASTException &e) {
    cerr<<e.what()<<endl;     
    testComplete(false);
  }
}


void VisionLrnTester::RecogniserTest::HandleSceneObjectChanged(const cdl::WorkingMemoryChange &change) {
  
  shared_ptr<const CASTData<Vision::ROI> > pROIData
      = getWorkingMemoryEntry<Vision::ROI>(change.m_address);
  shared_ptr<const Vision::ROI> pROI = pROIData->getData();

  if(m_phase == 1) {
    m_phase = -1;
    cout << "SceneObject updated with recognised attribute values.\n";
    testComplete(true);
  }
  else
  {
      m_phase = -1;
      cout << "SceneObject should not be updated yet (phase 2).\n";
      testComplete(false);
  }
 
}

// Checks if ROI update performed by feature extractor is correct
void VisionLrnTester::RecogniserTest::HandleROIChanged(const cdl::WorkingMemoryChange &change) {
  
  shared_ptr<const CASTData<Vision::ROI> > pROIData
      = getWorkingMemoryEntry<Vision::ROI>(change.m_address);
  shared_ptr<const Vision::ROI> pROI = pROIData->getData();


  if(m_phase == 0) {
    if (pROI->m_features.data_.length() > 0) {
      m_phase = 1;
      cout << "Extracted features have been added to ROI.\n";
    }
    else {
      m_phase = -1;
      cout << "Extracted features not in ROI (phase 1).\n";
      testComplete(false);
    }
  }
  else {
    m_phase = -1;
    testComplete(false);
  }
}
// Recogniser test


// Dry BindingMonitor test
void VisionLrnTester::DryBindingMonitorTest::startTest() 
{    
  try {
    cout << "*** recogniser test start ***\n";
    
    m_phase = 0;
    addChangeFilter(createGlobalTypeFilter<BindingData::BindingProxy>(cdl::ADD),
                    new MemberFunctionChangeReceiver<VisionLrnTester::DryBindingMonitorTest>(this,
                        &VisionLrnTester::DryBindingMonitorTest::HandleBindingProxyAdded));
    addChangeFilter(createLocalTypeFilter<SceneObject>(cdl::ADD),
                    new MemberFunctionChangeReceiver<VisionLrnTester::DryBindingMonitorTest>(this,
                        &VisionLrnTester::DryBindingMonitorTest::HandleSceneObjectAdded));

    sleep(25);

    if(m_phase != -1)
    {
      m_phase = -1;
      cout << "Test timeout.\n";
      testComplete(false);
    }
	

  } catch (CASTException &e) {
    cerr<<e.what()<<endl;     
    testComplete(false);
  }
}


void VisionLrnTester::DryBindingMonitorTest::HandleSceneObjectAdded(const cdl::WorkingMemoryChange &change) {
  
//   shared_ptr<const CASTData<Vision::ROI> > pROIData
//       = getWorkingMemoryEntry<Vision::ROI>(change.m_address);
//   shared_ptr<const Vision::ROI> pROI = pROIData->getData();

  if(m_phase == 0) {
    m_phase = 1;
    cout << "SceneObject added.\n";
  }
//   else
//   {
//     m_phase = -1;
//     cout << "SceneObject should not be updated yet (phase 1).\n";
//     testComplete(false);
//   }
 
}

// Checks if ROI update performed by feature extractor is correct
void VisionLrnTester::DryBindingMonitorTest::HandleBindingProxyAdded(const cdl::WorkingMemoryChange &change) {
  
//   shared_ptr<const CASTData<Vision::ROI> > pROIData
//       = getWorkingMemoryEntry<Vision::ROI>(change.m_address);
//   shared_ptr<const Vision::ROI> pROI = pROIData->getData();


  if(m_phase == 1) {

    m_phase = -1;
    cout << "Binding proxy added.\n";
    testComplete(true);

  }
  else {
    m_phase = -1;
    testComplete(false);
  }
}
// Dry BindingMonitor test


VisionLrnTester::VisionLrnTester(const string &_id)
    : WorkingMemoryAttachedComponent(_id),
      AbstractVisionTester(_id)
{

}

VisionLrnTester::~VisionLrnTester()
{
} 
 
 
// configure Tester
void VisionLrnTester::configure(std::map<std::string,std::string> & _config) 
{
  shared_ptr<ChangeDetectorTest> chad_test(new ChangeDetectorTest(*this));
  registerTest("chad_test", chad_test);

  shared_ptr<SegmentorTest> seg_test(new SegmentorTest(*this));
  registerTest("seg_test", seg_test);
  
  shared_ptr<DrySegmentorTest> dryseg_test(new DrySegmentorTest(*this));
  registerTest("dryseg_test", dryseg_test);
  
  shared_ptr<FeatureExtractorTest> ext_test(new FeatureExtractorTest(*this));
  registerTest("ext_test", ext_test);
    
  shared_ptr<RecogniserTest> rec_test(new RecogniserTest(*this));
  registerTest("rec_test", rec_test);
    
  shared_ptr<DryBindingMonitorTest> drymon_test(new DryBindingMonitorTest(*this));
  registerTest("drymon_test", drymon_test);

  shared_ptr<HandPointerTest> hp_test(new HandPointerTest(*this));
  registerTest("hp_test", hp_test);

  AbstractVisionTester::configure(_config);
}

