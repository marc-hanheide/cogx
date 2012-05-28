#include "NavTester.hpp"
//#include "binding/ontology/BindingOntologyFactory.hpp"
//#include "binding/ontology/BindingLocalOntology.hpp"
//#include <cast/core/CASTCompositeOntology.hpp>
#include <boost/lexical_cast.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

#include <Navigation/NavGraphNode.hh>
#include <Navigation/NavGraphEdge.hh>
#include <Navigation/NavGraphGateway.hh>
#include <testing/idl/TestingData.hh>

namespace Nav {
using namespace boost;
using namespace std;
using namespace cast;
using namespace Binding;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
FrameworkProcess* newComponent(const string &_id) {
  return new NavTester(_id);
}
}

NavTester::NavTester(const string &_id) :
  WorkingMemoryAttachedComponent(_id), AbstractBinder(_id),
      NavGraphEventListener("NavTester"), m_test(-1), m_handler(*this),
      m_testFinished(false) {
  //   static CASTCompositeOntology ont;
  //   ont.addOntology(&BindingOntologyFactory::getOntology());
  //   ont.addOntology(&comsys::ComsysOntology::construct());

  //   setOntology(&ont);
  m_queueBehaviour = cdl::QUEUE;
}

void NavTester::start() {
  AbstractBinder::start();

  m_CurrPerson = -1;
  m_CurrPersonWMid = "";

  m_TopRobPosWMid = "";

  //addChangeFilter(BindingLocalOntology::BINDING_PROXY_TYPE, 
  //cdl::ADD, 
  //cdl::ALL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<NavTester>(this,
  //&NavTester::bindingProxyAdded));
  addChangeFilter(createChangeFilter<BindingData::BindingProxy> (cdl::ADD, "",
      "", m_bindingSubarchID, cdl::ALL_SA), new MemberFunctionChangeReceiver<
      NavTester> (this, &NavTester::bindingProxyUpdated));
  //addChangeFilter(BindingLocalOntology::BINDING_PROXY_TYPE, 
  //cdl::ADD, 
  //cdl::ALL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<NavTester>(this,
  //&NavTester::bindingProxyAdded));
  addChangeFilter(createChangeFilter<BindingData::BindingProxy> (cdl::DELETE,
      "", "", m_bindingSubarchID, cdl::ALL_SA),
      new MemberFunctionChangeReceiver<NavTester> (this,
          &NavTester::bindingProxyDeleted));
  //addChangeFilter(BindingLocalOntology::BINDING_PROXY_TYPE, 
  //cdl::OVERWRITE, 
  //cdl::ALL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<NavTester>(this,
  //&NavTester::bindingProxyUpdated));
  addChangeFilter(createChangeFilter<BindingData::BindingProxy> (
      cdl::OVERWRITE, "", "", m_bindingSubarchID, cdl::ALL_SA),
      new MemberFunctionChangeReceiver<NavTester> (this,
          &NavTester::bindingProxyUpdated));

  //addChangeFilter(BindingLocalOntology::BINDER_STATUS_TYPE,
  //cdl::ADD,
  //cdl::ALL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<NavTester>(this,
  //&NavTester::statusUpdated));
  addChangeFilter(createChangeFilter<BindingData::BinderStatus> (cdl::ADD, "",
      "", m_bindingSubarchID, cdl::ALL_SA), new MemberFunctionChangeReceiver<
      NavTester> (this, &NavTester::statusUpdated));
  //addChangeFilter(BindingLocalOntology::BINDER_STATUS_TYPE,
  //cdl::OVERWRITE,
  //cdl::ALL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<NavTester>(this,
  //&NavTester::statusUpdated));
  addChangeFilter(createChangeFilter<BindingData::BinderStatus> (
      cdl::OVERWRITE, "", "", m_bindingSubarchID, cdl::ALL_SA),
      new MemberFunctionChangeReceiver<NavTester> (this,
          &NavTester::statusUpdated));

  // Change filter to be added only if we're being controlled externally by MultiSATester
  if (m_test == -1) {
    addChangeFilter(createLocalTypeFilter<TestingData::TestingCommand> (
        cdl::ADD), new MemberFunctionChangeReceiver<NavTester> (this,
        &NavTester::testerCommandReceived));
    log("NavTester started");
  }
}

void NavTester::configure(map<string, string> & _config) {
  AbstractBinder::configure(_config);
  map<string, string>::const_iterator itr = _config.find("--test");
  if (itr == _config.end()) {
    m_test = -1; //Indicates test is to be managed by a MultiSATester component
    //    cerr << "Error in NavTester: --test N must be given";
    //   abort();
  } else
    m_test = lexical_cast<int> (itr->second);
  m_cureNavGraphChanged = false;
  m_pNavGraph = 0;
  m_NavGraphWMid = "";

  if (_config["--nsa"] != "") {
    setNavSubarchID(_config["--nsa"]);
  } else if (_config["-nsa"] != "") {
    setNavSubarchID(_config["-nsa"]);
  } else {
    log("Nav subarch ID (\"--nsa\") not set. Defaulting to \"nav.sa\"");
    setNavSubarchID("nav.sa");
  }
  /*  if (_config["--bsa"] != "") {
   setBindingSubarchID(_config["--bsa"]);
   }
   else {
   log("Binding subarch ID (\"--bsa\") not set. Defaulting to \"binding.sa\"");
   setBindingSubarchID("binding.sa");
   }*/
  if (_config["-f"] != "") {

    m_EventFile.open(_config["-f"].c_str(), std::ios::in);

    if (!m_EventFile.is_open()) {
      printf("NavTester::configure could not open event file\n");
      exit();
    }

    CreateNewNavGraphIDL();
    m_WriteFirstTopologicalPose = false;
  } else {
    CreateNewNavGraphIDL();
    m_WriteFirstTopologicalPose = true;
  }

  if (_config["--fake"] != "") {
    m_fakeTest = true;
  } else {
    m_fakeTest = false;
  }

  if (_config["-t"] != "") {
    m_Threshold = atof(_config["-t"].c_str());
  } else {
    m_Threshold = 0.05;
  }

  m_cureNavGraph.addEventListener(this);
}

void NavTester::bindingProxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.m_address.m_id);
  char buf[256];
  sprintf(buf, "NavTester::bindingProxyUpdated (%s)", id.c_str());
  debug(buf);
  m_addSignalledProxyIDs.insert(id);
  try {
    log( m_bindingSubarchID);
    m_status = *loadBindingDataFromWM<BindingData::BinderStatus> (m_statusID);
    testCompleteness();
  } catch (const DoesNotExistOnWMException& _e) {
    log("Caught this in NavTester::bindingProxyUpdated: " + string(_e.what()));
    abort();
  }
}

void NavTester::bindingProxyDeleted(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.m_address.m_id);
  std::set<std::string>::iterator it = m_addSignalledProxyIDs.find(id);
  if (it != m_addSignalledProxyIDs.end())
    m_addSignalledProxyIDs.erase(it);
}

void NavTester::statusUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  debug("NavTester::statusUpdated");
  m_status = *loadBindingDataFromWM<BindingData::BinderStatus> (_wmc);
  m_statusID = string(_wmc.m_address.m_id);
  try {
    testCompleteness();
  } catch (const DoesNotExistOnWMException& _e) {
    log("Caught this in NavTester::statusUpdated: " + string(_e.what()));
    abort();
  }
}

void NavTester::runComponent() {
  sleepProcess(1000); // sleep for a second to allow the rest to be properly started
  //m_sourceID = subarchitectureID();
  char buf[256];
  sprintf(buf, "Running test no.: %i", m_test);
  log(buf);

  m_testFinished = false;

  if (m_test != -1) {
    if (m_fakeTest) { // This is a fake WM event test
      //Initial state
      AddFreeNode(0, 0, 0, 0.0, 0, 0, "UNKNOWN", 0, 2, 1, "-");
      WriteNavGraphToWorkingMemory();
      setRobotPose(0, 0, 0);
      setRobotArea(0);
      switch (m_test) {
      case 0: // Test initial state only.
        sleepProcess(500);
        setRobotArea(0);
        sleepProcess(500);
        m_testFinished = true;
        break;
      case 1: // Add one object in same Area
        sleepProcess(500);
        AddObjectNode(1, 1, 0, 0, 0, "UNKNOWN", "rice", 0.3, 0);
        sleepProcess(500);
        m_testFinished = true;
        break;
      default:
        cerr << "incorrect test number: " << m_test << endl;
        failExit();
      }
    } else {
      sleepProcess(30000);
      m_testFinished = true;
      log("Ready to check result");
    }
    testCompleteness();
  }
}

struct AreaIDMatcher: public std::unary_function<BindingFeatures::AreaID, bool> {
  const int m_compareID;
  //   ConceptRegexMatcher(const boost::regex& _regex) : m_regex(_regex){}
  AreaIDMatcher(int id) :
    m_compareID(id) {
  }
  bool operator()(const BindingFeatures::AreaID& _f) const {
    return _f.m_id == m_compareID;
  }
};

struct RelationLabelMatcher: public std::unary_function<
    BindingFeatures::RelationLabel, bool> {
  const string m_compareRelationLabel;
  //   RelationLabelRegexMatcher(const boost::regex& _regex) : m_regex(_regex){}
  RelationLabelMatcher(string label) :
    m_compareRelationLabel(label) {
  }
  bool operator()(const BindingFeatures::RelationLabel& _f) const {
    return CORBA::string_dup(_f.m_label) == m_compareRelationLabel;
  }
};

struct ConceptMatcher: public std::unary_function<BindingFeatures::Concept,
    bool> {
  const string m_compareConcept;
  //   ConceptRegexMatcher(const boost::regex& _regex) : m_regex(_regex){}
  ConceptMatcher(string concept) :
    m_compareConcept(concept) {
  }
  bool operator()(const BindingFeatures::Concept& _f) const {
    return CORBA::string_dup(_f.m_concept) == m_compareConcept;
  }
};

void NavTester::testCompleteness() {
  if (m_test != -1) {
    log("testCompleteness");
    m_handler.purgeAllLoadedData();
    ProxySet proxies;
    bool consistency_checked = false;
    while (!consistency_checked) {
      proxies = m_handler.allProxiesFromWM();
      if (true_for_all(proxies, ConsistencyCheck<ProxyPtr> (*this))) {
        consistency_checked = true;
      } else {
        m_handler.reloadAllLoadedData();
      }
    }
    proxies = proxies | !ProxyUnionIDChecker("");
    //assert(true_for_all(proxies,ProxyStateChecker(BindingData::BOUND) && !ProxyUnionIDChecker("")));
    UnionSet unions = m_handler.allUnionssFromWM();//extractUnionsFromProxies(proxies);
    char buf[256];
    sprintf(buf, "Checking test no. %i for completeness", m_test);
    switch (m_test) {
    case 0:
      // Test of initial state. Should have: Robot proxy, Area proxy of ID 0, 
      // Position relation proxy combining the two, dummy object proxy.
      if (m_testFinished && allBound()) {
        log(buf);
        //    ProxySet proxies = m_handler.allProxiesFromWM();
        //UnionSet unions = m_handler.extractUnionsFromProxies(proxies);

        assert(proxies.size() == 3);
        assert(unions.size() == 3);
        assert(true_for_all(proxies, (hasFeature<ProxyPtr,
            BindingFeatures::Concept> () || hasFeature<ProxyPtr,
            BindingFeatures::AreaID> () || hasFeature<ProxyPtr,
            BindingFeatures::RelationLabel> ())));
        assert(true_for_some(proxies, hasFeature<ProxyPtr,
            BindingFeatures::AreaID> () && featureCheck<ProxyPtr,
            BindingFeatures::AreaID> (AreaIDMatcher(0))));
        assert(!true_for_some(proxies, (InportsCountCheck<ProxyPtr> (0))
            && (OutportsCountCheck<ProxyPtr> (0))));
        assert(true_for_some(proxies, (InportsCountCheck<ProxyPtr> (1))
            && (OutportsCountCheck<ProxyPtr> (0))));
        assert(true_for_some(proxies, (InportsCountCheck<ProxyPtr> (0))
            && (OutportsCountCheck<ProxyPtr> (2))));
        assert(!true_for_some(proxies, (InportsCountCheck<ProxyPtr> (0))
            && (OutportsCountCheck<ProxyPtr> (1))));
        assert(!true_for_some(proxies, (InportsCountCheck<ProxyPtr> (2))
            && (OutportsCountCheck<ProxyPtr> (0))));
        successExit();
      }
      break;
    case 1:
      // One object in same area. Should have: Robot proxy, 
      //Area proxy of ID 0, Object proxy, Position relation proxies combining each object and the area,
      //Dummy object proxy
      if (m_testFinished && allBound()) {
        log(buf);

        //ProxySet proxies = m_handler.allProxiesFromWM();
        //UnionSet unions = m_handler.extractUnionsFromProxies(proxies);

        assert(proxies.size() == 5);
        assert(unions.size() == 5);
        assert(true_for_all(proxies, (hasFeature<ProxyPtr,
            BindingFeatures::Concept> () || hasFeature<ProxyPtr,
            BindingFeatures::AreaID> () || hasFeature<ProxyPtr,
            BindingFeatures::RelationLabel> ())));
        assert(true_for_some(proxies, hasFeature<ProxyPtr,
            BindingFeatures::AreaID> () && featureCheck<ProxyPtr,
            BindingFeatures::AreaID> (AreaIDMatcher(0))));
        assert(true_for_some(proxies, //Area
            (InportsCountCheck<ProxyPtr> (2))
                && (OutportsCountCheck<ProxyPtr> (0))));
        assert(true_for_some(proxies, //Robot, object
            (InportsCountCheck<ProxyPtr> (1))
                && (OutportsCountCheck<ProxyPtr> (0))));
        assert(!true_for_some(proxies, (InportsCountCheck<ProxyPtr> (0))
            && (OutportsCountCheck<ProxyPtr> (1))));
        assert(true_for_some(proxies, //Relations
            (InportsCountCheck<ProxyPtr> (0))
                && (OutportsCountCheck<ProxyPtr> (2))));
        successExit();
      }
      break;
    case 2:
      // Test of initial state. Should have: Robot proxy, Area proxy of ID 0, 
      // Position relation proxy combining the two, dummy object proxy.
      if (m_testFinished && allBound()) {
        log(buf);
        //    ProxySet proxies = m_handler.allProxiesFromWM();
        //UnionSet unions = m_handler.extractUnionsFromProxies(proxies);

        assert(proxies.size() == 3);
        assert(unions.size() == 3);
        assert(true_for_all(proxies, (hasFeature<ProxyPtr,
            BindingFeatures::Concept> () || hasFeature<ProxyPtr,
            BindingFeatures::AreaID> () || hasFeature<ProxyPtr,
            BindingFeatures::RelationLabel> ())));
        assert(true_for_some(proxies, hasFeature<ProxyPtr,
            BindingFeatures::AreaID> () && featureCheck<ProxyPtr,
            BindingFeatures::AreaID> (AreaIDMatcher(2))));
        assert(!true_for_some(proxies, (InportsCountCheck<ProxyPtr> (0))
            && (OutportsCountCheck<ProxyPtr> (0))));
        assert(true_for_some(proxies, (InportsCountCheck<ProxyPtr> (1))
            && (OutportsCountCheck<ProxyPtr> (0))));
        assert(true_for_some(proxies, (InportsCountCheck<ProxyPtr> (0))
            && (OutportsCountCheck<ProxyPtr> (2))));
        assert(!true_for_some(proxies, (InportsCountCheck<ProxyPtr> (0))
            && (OutportsCountCheck<ProxyPtr> (1))));
        assert(!true_for_some(proxies, (InportsCountCheck<ProxyPtr> (2))
            && (OutportsCountCheck<ProxyPtr> (0))));
        successExit();
      }
      break;
    default:
      sprintf(buf, "incorrect test number %i", m_test);
      log(buf);
      failExit();
    }
    log("Exiting testCompleteness");
  }
}

bool NavTester::allBound() {
  debug("allBound");
  if (!m_addSignalledProxyIDs.empty())
    debug("allBound:!m_addSignalledProxyIDs.empty())");
  if (!m_addSignalledProxyIDs.empty() && m_status.m_stable) {
    debug("allBound: stage 1");
    for (std::set<std::string>::iterator it = m_addSignalledProxyIDs.begin(); it
        != m_addSignalledProxyIDs.end(); it++) {
      log(*it);
    }
    ProxySet proxies;
    proxies = m_handler.loadProxies(m_addSignalledProxyIDs);
    if (true_for_all(proxies, ProxyStateChecker(BindingData::BOUND)
        && !ProxyUnionIDChecker(""))) { // return true after some assertions
      // some consistency checks:
      debug("allBound: stage 2");
      assert(true_for_all(proxies, hasFeature<ProxyPtr,
          BindingFeatures::ThisProxyID> () && hasFeature<ProxyPtr,
          BindingFeatures::SourceID> () && hasFeature<ProxyPtr,
          BindingFeatures::CreationTime> () && (TypeCheck<ProxyPtr> (
          BindingData::GROUP)
          == hasFeature<ProxyPtr, BindingFeatures::Group> ())));
      // make sure all ThisProxyID points correctly (well... not a dead certain test, but well)
      ProxySet proxies2 = m_handler.extractProxiesFromProxies(proxies,
          ProxyFromThisProxyIDExtractor<ProxyPtr> ());
      assert(proxies == proxies2);
      // have a look at the singulars:
      proxies = proxies | hasFeature<ProxyPtr, BindingFeatures::Singular> ();
      // and make sure they're all part of groups
      proxies = m_handler.extractProxiesFromProxies(proxies,
          GroupProxyFromSingularExtractor<ProxyPtr> ());
      assert( true_for_all(proxies, TypeCheck<ProxyPtr> ( BindingData::GROUP)));
      return true;
    }
  }
  return false;
}

void NavTester::taskAdopted(const string &_taskID) {
}

void NavTester::taskRejected(const string &_taskID) {
}

void NavTester::changedArea(int aid) {
  checkAndAddNewArea(aid);

  WriteTopologicalPosToWorkingMemory(aid);

}

void NavTester::checkAndAddNewArea(int aid) {
  char buf[256];
  sprintf(buf, "checkAndAddNewArea(%d)", aid);
  log(buf);
  // First we check if this area already exists
  std::list<NavTester::Area>::iterator aIter;
  for (aIter = m_Areas.begin(); aIter != m_Areas.end(); aIter++) {
    if (aIter->m_data.m_id == aid) {
      break;
    }
  }

  if (aIter != m_Areas.end()) {
    // We found an exiting areas so we do not have to add anything
  } else {
    // Create a new area objects
    NavTester::Area newArea;
    newArea.m_WMid = newDataID();
    newArea.m_data.m_id = aid;
    sprintf(buf, "Adding new Area struct (%d)", aid);
    log(buf);
    m_Areas.push_back(newArea);
    addToWorkingMemory<NavData::Area> (newArea.m_WMid, getNavSubarchID(),
        new NavData::Area(newArea.m_data), cdl::BLOCKING);
  }
}

void NavTester::mergedAreaIds(int aid1, int aid2) {
  // Find the area that will be removed (the second one)
  std::list<NavTester::Area>::iterator aIter;
  for (aIter = m_Areas.begin(); aIter != m_Areas.end(); aIter++) {
    if (aIter->m_data.m_id == aid2) {
      break;
    }
  }

  if (aIter != m_Areas.end()) {
    // We found the area to remove and do so
    deleteFromWorkingMemory(aIter->m_WMid, getNavSubarchID(), cdl::BLOCKING);
    m_Areas.erase(aIter);
  }

}

void NavTester::AddFreeNode(long nodeID, double x, double y, double z,
    double theta, long areaID, const std::string &areaType, short gateway,
    double maxSpeed, double width, const std::string &name) {
  int numFNodes = m_pNavGraph->m_FNodes.length();
  m_pNavGraph->m_FNodes.length(numFNodes + 1);

  // Add the free node

  m_pNavGraph->m_FNodes[numFNodes].m_x = x;
  m_pNavGraph->m_FNodes[numFNodes].m_y = y;
  m_pNavGraph->m_FNodes[numFNodes].m_z = z;
  m_pNavGraph->m_FNodes[numFNodes].m_theta = theta;
  m_pNavGraph->m_FNodes[numFNodes].m_areaID = areaID;
  m_pNavGraph->m_FNodes[numFNodes].m_areaType = CORBA::string_dup(
      areaType.c_str());
  m_pNavGraph->m_FNodes[numFNodes].m_nodeID = nodeID;
  m_pNavGraph->m_FNodes[numFNodes].m_gateway = gateway;
  m_pNavGraph->m_FNodes[numFNodes].m_maxSpeed = maxSpeed;
  m_pNavGraph->m_FNodes[numFNodes].m_width = width;
  m_pNavGraph->m_FNodes[numFNodes].m_name = name.c_str();

  m_pNavGraph->m_numNodes = m_pNavGraph->m_numNodes + 1;
  checkAndAddNewArea(areaID);

  log("added free node");

  ///////////////////////
  addToWorkingMemory<NavData::FNode> (newDataID(), getNavSubarchID(),
      new NavData::FNode(m_pNavGraph->m_FNodes[numFNodes]), cdl::BLOCKING); // sync!

  log("added free node to working memory");
}

void NavTester::MovePerson(long personID, double x, double y, double theta,
    long areaID) {
  log("NavTester::MovePerson");
  vector<shared_ptr<const CASTData<NavData::Person> > > vec;
  getWorkingMemoryEntries<NavData::Person> (getNavSubarchID(), 0, vec);

  for (vector<shared_ptr<const CASTData<NavData::Person> > >::iterator it =
      vec.begin(); it != vec.end(); it++) {
    if ((*it)->getData()->m_id == personID) {
      NavData::Person person;
      person.m_id = personID;
      person.m_x = x;
      person.m_y = y;
      person.m_theta = theta;
      person.m_speed = (*it)->getData()->m_speed;
      person.m_visibility = (*it)->getData()->m_visibility;
      person.m_areaID = areaID;

      overwriteWorkingMemory<NavData::Person> ((*it)->getID(),
          getNavSubarchID(), new NavData::Person(person), cdl::BLOCKING);
      return;
    }
  }
  //Not found!
  log("Person not found on Nav.sa WM - ignoring!");
}

void NavTester::AddPerson(long personID, double x, double y, double theta,
    double speed, double visibility, long areaID) {
  log("added person");
  NavData::Person person;
  person.m_id = personID;
  person.m_x = x;
  person.m_y = y;
  person.m_theta = theta;
  person.m_speed = speed;
  person.m_visibility = visibility;
  person.m_areaID = areaID;

  addToWorkingMemory<NavData::Person> (newDataID(), getNavSubarchID(),
      new NavData::Person(person), cdl::BLOCKING);
}

void NavTester::RemovePerson(long personID) {
  log("removing person");
  vector<shared_ptr<const CASTData<NavData::Person> > > vec;
  getWorkingMemoryEntries<NavData::Person> (getNavSubarchID(), 0, vec);

  for (vector<shared_ptr<const CASTData<NavData::Person> > >::iterator it =
      vec.begin(); it != vec.end(); it++) {
    if ((*it)->getData()->m_id == personID) {
      deleteFromWorkingMemory((*it)->getID(), getNavSubarchID());
      return;
    }
  }
  //Not found!
  log("Person not found on Nav.sa WM - ignoring!");
}

void NavTester::AddObjectNode(long nodeID, double x, double y, double z,
    long areaID, const std::string &areaType, const std::string &category,
    double radius, long objectID) {
  int numONodes = m_pNavGraph->m_ONodes.length();
  m_pNavGraph->m_ONodes.length(numONodes + 1);

  // Add the object node

  m_pNavGraph->m_ONodes[numONodes].m_nodeID = nodeID;
  m_pNavGraph->m_ONodes[numONodes].m_x = x;
  m_pNavGraph->m_ONodes[numONodes].m_y = y;
  m_pNavGraph->m_ONodes[numONodes].m_z = z;
  m_pNavGraph->m_ONodes[numONodes].m_areaID = areaID;
  m_pNavGraph->m_ONodes[numONodes].m_areaType = CORBA::string_dup(
      areaType.c_str());

  m_pNavGraph->m_ONodes[numONodes].m_category = CORBA::string_dup(
      category.c_str());
  m_pNavGraph->m_ONodes[numONodes].m_radius = radius;
  m_pNavGraph->m_ONodes[numONodes].m_objectID = objectID;

  m_pNavGraph->m_numNodes = m_pNavGraph->m_numNodes + 1;

  // Add the object to object list

  int numObjects = m_pNavGraph->m_Objects.length();
  m_pNavGraph->m_Objects.length(numObjects + 1);

  m_pNavGraph->m_Objects[numObjects].m_category = CORBA::string_dup(
      category.c_str());
  m_pNavGraph->m_Objects[numObjects].m_radius = radius;
  m_pNavGraph->m_Objects[numObjects].m_objectID = objectID;

  log("added object node");

  ////////////////////////////
  addToWorkingMemory<NavData::ONode> (newDataID(), getNavSubarchID(),
      new NavData::ONode(m_pNavGraph->m_ONodes[numONodes]), cdl::BLOCKING); // sync!

  log("added object node to working memory");
}

void NavTester::AddAccessEdge(long startNodeID, long endNodeID, double weight) {
  int numAEdges = m_pNavGraph->m_AEdges.length();
  m_pNavGraph->m_AEdges.length(numAEdges + 1);

  m_pNavGraph->m_AEdges[numAEdges].m_startNodeID = startNodeID;
  m_pNavGraph->m_AEdges[numAEdges].m_endNodeID = endNodeID;
  m_pNavGraph->m_AEdges[numAEdges].m_w = weight;

  m_pNavGraph->m_numEdges = m_pNavGraph->m_numEdges + 1;

  log("added access edge");

  //////////////////////
  addToWorkingMemory<NavData::AEdge> (newDataID(), getNavSubarchID(),
      new NavData::AEdge(m_pNavGraph->m_AEdges[numAEdges]), cdl::BLOCKING); // sync!

  log("added access edge to working memory");
}

void NavTester::AddVisibilityEdge(long startNodeID, long endNodeID,
    double weight) {
  int numVEdges = m_pNavGraph->m_VEdges.length();
  m_pNavGraph->m_VEdges.length(numVEdges + 1);

  m_pNavGraph->m_VEdges[numVEdges].m_startNodeID = startNodeID;
  m_pNavGraph->m_VEdges[numVEdges].m_endNodeID = endNodeID;
  m_pNavGraph->m_VEdges[numVEdges].m_w = weight;

  m_pNavGraph->m_numEdges = m_pNavGraph->m_numEdges + 1;
  log("added visibility edge");

  //////////////////////
  addToWorkingMemory<NavData::VEdge> (newDataID(), getNavSubarchID(),
      new NavData::VEdge(m_pNavGraph->m_VEdges[numVEdges]), cdl::BLOCKING); // sync!

  log("added vis edge to working memory");

}

void NavTester::CreateNewNavGraphIDL() {
  // Create a new nav graph objects
  if (m_pNavGraph)
    delete m_pNavGraph;
  m_pNavGraph = new NavData::NavGraph();

  // initialize the new nav graph IDL

  m_pNavGraph->m_numNodes = 0;
  m_pNavGraph->m_numEdges = 0;

}

void NavTester::ConvertCureNavGraphToIDL() {

  CreateNewNavGraphIDL();

  list<Cure::NavGraphNode*>::iterator it1;

  for (it1 = m_cureNavGraph.m_Nodes.begin(); it1
      != m_cureNavGraph.m_Nodes.end(); it1++) {

    Cure::NavGraphNode* node = (*it1);

    long nodeID = node->getId();
    double x = node->getX();
    double y = node->getY();
    double z = 0.0;
    double theta = node->getTheta();
    long areaID = node->getAreaId();
    string areaType = node->getAreaName();
    double maxSpeed = node->getMaxSpeed();

    short gateway;

    if (node->getType() == 0)
      gateway = 0;
    else
      gateway = 1;

    double width;
    if (gateway) {

      Cure::NavGraphGateway *gnode = ((Cure::NavGraphGateway*) (*it1));
      width = gnode->getWidth();

    }

    char buf[256];
    sprintf(buf, "AddFreeNode with areaID = %li", areaID);
    log(buf);
    AddFreeNode(nodeID, x, y, z, theta, areaID, areaType, gateway, maxSpeed,
        width, node->getName());

  }

  list<Cure::NavGraphEdge>::iterator it2;

  for (it2 = m_cureNavGraph.m_Edges.begin(); it2
      != m_cureNavGraph.m_Edges.end(); it2++) {

    Cure::NavGraphEdge edge = *it2;

    long startNodeID = edge.getNodeId1();
    long endNodeID = edge.getNodeId2();

    double weight = edge.getCost();

    AddAccessEdge(startNodeID, endNodeID, weight);

  }
}

void NavTester::WriteNavGraphToWorkingMemory() // cp: done
{

  if (m_NavGraphWMid == "") {
    m_NavGraphWMid = newDataID();
    addToWorkingMemory<NavData::NavGraph> (m_NavGraphWMid, getNavSubarchID(),
        new NavData::NavGraph(*m_pNavGraph), cdl::BLOCKING); // sync!

    log("added navgraph to working memory");
  } else {
    // Consistency checking...
    //getWorkingMemoryEntry<NavData::NavGraph>(m_NavGraphWMid);
    overwriteWorkingMemory<NavData::NavGraph> (m_NavGraphWMid,
        getNavSubarchID(), new NavData::NavGraph(*m_pNavGraph), cdl::BLOCKING);

    log("changed navgraph on working memory");
  }
}

void NavTester::WriteTopologicalPosToWorkingMemory(int aid) {
  log("Calling WriteTopologicalPosToWorkingMemory");

  if (m_TopRobPosWMid == "") {
    m_TopRobPosWMid = newDataID();
    NavData::TopologicalRobotPos *newRobotPos =
        new NavData::TopologicalRobotPos(m_TopRobPos);
    newRobotPos->m_areaID = aid;
    addToWorkingMemory<NavData::TopologicalRobotPos> (m_TopRobPosWMid,
        getNavSubarchID(), newRobotPos, cdl::BLOCKING); // sync!

    char buf[256];
    sprintf(buf, "added TopologicalRobotPos to working memory: %i", aid);
    log(buf);
  } else {

    if (m_TopRobPos.m_areaID != aid) {
      m_TopRobPos.m_areaID = aid;
      overwriteWorkingMemory<NavData::TopologicalRobotPos> (m_TopRobPosWMid,
          getNavSubarchID(), new NavData::TopologicalRobotPos(m_TopRobPos),
          cdl::BLOCKING);
      char buf[256];
      sprintf(buf, "changed TopologicalRobotPos on working memory: %i", aid);
      log(buf);
    }
  }
  log("End WriteTopologicalPosToWorkingMemory");
}

void NavTester::updatePeopleInWM() {
  debug("Updating people in working memory");

  // Got through the list of people hypotheses and update the working memory
  for (std::vector<PersonHyp>::iterator pi = m_People.begin(); pi
      != m_People.end();) {

    if (pi->m_Status == PersonHyp::STATUS_DELETE) {
      // Delete the working memory entry for this person
      deleteFromWorkingMemory(pi->m_WMid, getNavSubarchID(), cdl::BLOCKING);
      char buf[256];
      sprintf(buf, "Removed person hyp with id %ld from working memory",
          pi->m_data.m_id);
      log(buf);
      pi = m_People.erase(pi);
      continue;
    }

    if (pi->m_Status == PersonHyp::STATUS_CREATE) {
      pi->m_WMid = newDataID();
      addToWorkingMemory<NavData::Person> (pi->m_WMid, getNavSubarchID(),
          new NavData::Person(pi->m_data), cdl::BLOCKING); // sync!
      char buf[256];
      sprintf(buf, "Added new Person with id %ld to working memory",
          pi->m_data.m_id);
      log(buf);
    } else {
      overwriteWorkingMemory<NavData::Person> (pi->m_WMid, getNavSubarchID(),
          new NavData::Person(pi->m_data), cdl::BLOCKING);
      char buf[256];
      sprintf(buf, "Updated person hyp with id %ld in in working memory",
          pi->m_data.m_id);
      debug(buf);
    }

    pi++;
  }
}

void NavTester::setRobotArea(long id) {
  checkAndAddNewArea(id);

  WriteTopologicalPosToWorkingMemory(id);
}

void NavTester::setRobotPose(double x, double y, double theta) {
  // Write estimated robot pose data into robot pose IDL structure
  NavData::RobotPose *pRP = new NavData::RobotPose();
  pRP->m_time.m_s = 0;
  pRP->m_time.m_us = 0;
  pRP->m_x = x;
  pRP->m_y = y;
  pRP->m_theta = theta;

  if (m_RobotPoseWMid == "") {
    m_RobotPoseWMid = newDataID();
    addToWorkingMemory<NavData::RobotPose> (m_RobotPoseWMid, getNavSubarchID(),
        pRP, cdl::BLOCKING); // sync!
    debug("Added RobotPose to WM");
  } else {
    overwriteWorkingMemory<NavData::RobotPose> (m_RobotPoseWMid,
        getNavSubarchID(), pRP, cdl::BLOCKING);

    debug("Overwriting RobotPose in WM x=%.3f y=%.3f theta=%.3f", x, y, theta);
  }
}

void NavTester::updateTrackedPersonInWM(long id) {
  NavData::PersonFollowed p;
  p.m_id = m_CurrPerson;

  if (m_CurrPersonWMid == "") {
    // Create the entry in the working memory for the 
    m_CurrPersonWMid = newDataID();
    addToWorkingMemory<NavData::PersonFollowed> (m_CurrPersonWMid,
        getNavSubarchID(), new NavData::PersonFollowed(p), cdl::BLOCKING); // sync!
    log("added id of person to follow");
  } else {
    if (id != m_CurrPerson) {
      debug("Updating id of person to follow");
      overwriteWorkingMemory<NavData::PersonFollowed> (m_CurrPersonWMid,
          getNavSubarchID(), new NavData::PersonFollowed(p), cdl::BLOCKING);
      debug("updated id of person to follow");
    }
  }

  m_CurrPerson = id;
}

void NavTester::proxyConsistencyCheck(string &check) {
  addToWorkingMemory(newDataID(), "binding.sa",
      new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
  istringstream str(check);
  string quantifier;
  str >> quantifier;

  m_handler.purgeAllLoadedData();
  ProxySet proxies;
  bool consistency_checked = false;
  while (!consistency_checked) {
    proxies = m_handler.allProxiesFromWM();
    if (true_for_all(proxies, ConsistencyCheck<ProxyPtr> (*this))) {
      consistency_checked = true;
    } else {
      m_handler.reloadAllLoadedData();
    }
  }
  proxies = proxies | !ProxyUnionIDChecker("");
  UnionSet unions = m_handler.allUnionssFromWM();

  while (!allBound()) {
    debug("All proxies not bound yet. Sleeping...");
    sleepProcess(250);
  }
  if (quantifier == "ALL") {
    shared_ptr<AbstractPredicate<ProxyPtr> > predicate = getPredicates(str);
    if (!true_for_all(proxies, *predicate)) {
      log(string("Proxy check failed: ") + check);
      failExit();
    }
  } else if (quantifier == "SOME") {
    shared_ptr<AbstractPredicate<ProxyPtr> > predicate = getPredicates(str);
    if (!true_for_some(proxies, *predicate)) {
      log(string("Proxy check failed: ") + check);
      failExit();
    }
  } else if (quantifier == "NONE") {
    shared_ptr<AbstractPredicate<ProxyPtr> > predicate = getPredicates(str);
    if (true_for_some(proxies, *predicate)) {
      log(string("Proxy check failed: ") + check);
      failExit();
    }
  } else if (quantifier == "PROXYCOUNT") {
    unsigned int shouldBe;
    str >> shouldBe;
    if (proxies.size() != shouldBe) {
      log(string("Proxy check failed: ") + check);
      failExit();
    }
  } else if (quantifier == "UNIONCOUNT") {
    unsigned int shouldBe;
    str >> shouldBe;
    if (unions.size() != shouldBe) {
      log(string("Proxy check failed: ") + check);
      failExit();
    }
  } else {
    log(string("Syntax error in proxy check: ") + check);
    failExit();
  }
}

shared_ptr<AbstractPredicate<ProxyPtr> > NavTester::getPredicates(istream &str) {
  shared_ptr<AbstractPredicate<ProxyPtr> > pred = getPredicate(str);
  while (!str.eof()) {
    string separator;
    str >> separator;
    if (separator == "AND") {
      pred = (*pred && *(getPredicate(str))).clone();
    } else if (separator == "OR") {
      pred = (*pred || *(getPredicate(str))).clone();
    } else {
      log(string("Syntax error in proxy check!"));
      failExit();
    }
  }
  return pred;
}

shared_ptr<AbstractPredicate<ProxyPtr> > NavTester::getPredicate(istream &str) {
  string name;
  str >> name;
  if (name == "INPORTCOUNT") {
    int expected;
    str >> expected;
    char buf[256];
    sprintf(buf, "%s %i", name.c_str(), expected);
    debug(buf);
    return (InportsCountCheck<ProxyPtr> (expected)).clone();
  } else if (name == "OUTPORTCOUNT") {
    int expected;
    str >> expected;
    char buf[256];
    sprintf(buf, "%s %i", name.c_str(), expected);
    debug(buf);
    return (OutportsCountCheck<ProxyPtr> (expected)).clone();
  } else if (name == "HASLABEL") {
    debug(name);
    return (hasFeature<ProxyPtr, BindingFeatures::RelationLabel> ()).clone();
  } else if (name == "ISLABEL") {
    string expected;
    str >> expected;
    char buf[256];
    sprintf(buf, "%s %s", name.c_str(), expected.c_str());
    debug(buf);
    return (featureCheck<ProxyPtr, BindingFeatures::RelationLabel> (
        RelationLabelMatcher(expected))).clone();
  } else if (name == "HASCONCEPT") {
    debug(name);
    return (hasFeature<ProxyPtr, BindingFeatures::Concept> ()).clone();
  } else if (name == "ISCONCEPT") {
    string expected;
    str >> expected;
    char buf[256];
    sprintf(buf, "%s %s", name.c_str(), expected.c_str());
    debug(buf);
    return (featureCheck<ProxyPtr, BindingFeatures::Concept> (ConceptMatcher(
        expected))).clone();
  } else if (name == "HASAREAID") {
    debug(name);
    return (hasFeature<ProxyPtr, BindingFeatures::AreaID> ()).clone();
  } else if (name == "ISAREAID") {
    int expected;
    str >> expected;
    char buf[256];
    sprintf(buf, "%s %i", name.c_str(), expected);
    debug(buf);
    return (featureCheck<ProxyPtr, BindingFeatures::AreaID> (AreaIDMatcher(
        expected))).clone();
  } else if (name == "HASRELATIONLABEL") {
    debug(name);
    return (hasFeature<ProxyPtr, BindingFeatures::RelationLabel> ()).clone();
  } else {
    log("Syntax error in proxy check!");
    failExit();
    return (TruePredicate<ProxyPtr> ()).clone();
  }
}

void NavTester::testerCommandReceived(const cdl::WorkingMemoryChange &_wmc) {
  shared_ptr<const CASTData<TestingData::TestingCommand> > command =
      getWorkingMemoryEntry<TestingData::TestingCommand> (_wmc.m_address);
  string cmd = CORBA::string_dup(command->getData()->m_command);
  log(cmd);
  istringstream str(cmd);

  string temp;
  str >> temp;

  if (temp == "SETROBOTAREA") {
    int area;
    str >> area;
    setRobotArea(area);
  } else if (temp == "SETROBOTPOSE") {
    double x, y, theta;
    str >> x >> y >> theta;
    setRobotPose(x, y, theta);
  } else if (temp == "ADDPERSON") {
    long personID, areaID;
    double x, y, theta, speed, visibility;
    str >> personID >> x >> y >> theta >> speed >> visibility >> areaID;
    AddPerson(personID, x, y, theta, speed, visibility, areaID);
  } else if (temp == "REMOVEPERSON") {
    long personID;
    str >> personID;
    RemovePerson(personID);
  } else if (temp == "MOVEPERSON") {
    long personID, areaID;
    double x, y, theta;
    str >> personID >> x >> y >> theta >> areaID;
    MovePerson(personID, x, y, theta, areaID);
  } else if (temp == "ADDOBJECTNODE") {
    long nodeID, areaID, objectID;
    string category, areaType;
    double x, y, z, radius;
    str >> nodeID >> x >> y >> z >> areaID >> areaType >> category >> radius
        >> objectID;
    AddObjectNode(nodeID, x, y, z, areaID, areaType, category, radius, objectID);
  } else if (temp == "ADDFREENODE") {
    long areaID, nodeID;
    string category, areaType, name;
    short gateway;
    double x, y, z, theta, maxSpeed, width;
    str >> nodeID >> x >> y >> z >> theta >> areaID >> areaType >> gateway
        >> maxSpeed >> width >> name;
    AddFreeNode(nodeID, x, y, z, theta, areaID, areaType, gateway, maxSpeed,
        width, name);
  } else if (temp == "REMOVEPERSON") {
    long personID;
    str >> personID;
    RemovePerson(personID);
  } else if (temp == "ADDACCESSEDGE") {
    long startNodeID, endNodeID;
    double weight;
    str >> startNodeID >> endNodeID >> weight;
    AddAccessEdge(startNodeID, endNodeID, weight);
  } else if (temp == "ADDVISIBILITYEDGE") {
    long startNodeID, endNodeID;
    double weight;
    str >> startNodeID >> endNodeID >> weight;
    AddVisibilityEdge(startNodeID, endNodeID, weight);
  } else if (temp == "PROXYTEST") {
    string test;
    getline(str, test, ';');
    proxyConsistencyCheck(test);
  } else {
    string err("Unknown command: ");
    err = err + temp;
    failExit();
  }
  assert(str.eof());
  TestingData::TestingCommand *owtCmd = new TestingData::TestingCommand;
  owtCmd->m_status = TestingData::COMPLETED;
  string logStr = string("Marking command ") + CORBA::string_dup(
      _wmc.m_address.m_id) + " as completed.";
  log(logStr);
  addToWorkingMemory(newDataID(), "binding.sa",
      new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
  overwriteWorkingMemory<TestingData::TestingCommand> (CORBA::string_dup(
      _wmc.m_address.m_id), owtCmd);
}

} // namespace Binding

