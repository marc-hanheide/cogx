//
// = FILENAME
//   SpatialTester.hpp
//
// = FUNCTION
//    Running consistency tests for Spatial in conjunction with Binder
//
// = AUTHOR(S)
//    Kristoffer Sjöö, based on ComsysTester.hpp
//
// = COPYRIGHT
//    Copyright (c) 2008 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#ifndef NAV_TESTER_HPP_
#define NAV_TESTER_HPP_

#include <map>
#include <set>
#include <string>
#include <binding/utils/GraphLoader.hpp>
#include <binding/utils/Predicates.hpp>
#include <binding/idl/BindingData.hh>
#include <binding/idl/BindingFeatures.hh>
#include <binding/BindingException.hpp>
#include <binding/feature-utils/AbstractFeature.hpp>
#include <binding/abstr/AbstractBinder.hpp>
#include <cast/core/CASTCore.hpp>
#include <nav/idl/NavData.hh>
#include <nav/idl/SpatialData.hh>
#include <laser/idl/Laser.hh>
#include <Navigation/NavGraph.hh>
#include <Transformation/Pose3D.hh>
#include <Utils/MutexWrapper.hh>

using namespace Binding;
using namespace cast;
namespace Spatial {

/// a component which writes nav WM entries, and checks the 
/// resulting Binder state

class SpatialTester: public AbstractBinder, public Cure::NavGraphEventListener {
  /// the test number
  int m_test;

  /// the latest registered status of the binder
  BindingData::BinderStatus m_status;
  /// the id of the status
  std::string m_statusID;

  std::string m_comsysID;
  std::set<std::string> m_addSignalledProxyIDs;

  BindingGraphHandler m_handler;

  bool m_testFinished;

  std::string m_navSubarchID;

public:
  SpatialTester(const std::string &_id);

  /// overridden start method used to set filters
  virtual void start();
  virtual void configure(std::map<std::string, std::string> & _config);

protected:
  void bindingProxyAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingProxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingProxyDeleted(const cast::cdl::WorkingMemoryChange & _wmc);
  void statusUpdated(const cast::cdl::WorkingMemoryChange & _wmc);
  void testerCommandReceived(const cast::cdl::WorkingMemoryChange & _wmc);

  void CreateNewNavGraphIDL();

  void taskAdopted(const std::string& _taskID);
  void taskRejected(const std::string& _taskID);
  void runComponent();
  void setNavSubarchID(const std::string &_saID) {
    m_navSubarchID = _saID;
  }
  std::string getNavSubarchID() const {
    return m_navSubarchID;
  }
  /// exits and signals success
  void successExit() const {
    const_cast<SpatialTester&> (*this).sleepProcess(2000); // sleep for a while to allow dotviewer to finish
    log("\nSUCCESS\n");
    ::exit(cast::cdl::testing::CAST_TEST_PASS);
  }
  /// exits and signals failure
  void failExit() const {
    log("\nFAILURE\n");
    //::exit(cast::cdl::testing::CAST_TEST_FAIL);
    abort();
  }

  /// tests if the binding is now complete and exits with a success
  /// signal if so.
  void testCompleteness();
  /// returns true if all added proxies are bound
  bool allBound();

  /// Performs a consistency check on Binding proxies
  void proxyConsistencyCheck(std::string &str);
private:
  /// Methods to write to Spatial WM
  void ConvertCureNavGraphToIDL();
  void WriteNavGraphToWorkingMemory();
  void WriteTopologicalPosToWorkingMemory(int aid);
  void AddFreeNode(long nodeID, double x, double y, double z, double theta,
      long areaID, const std::string &areaType, short gateway, double maxSpeed,
      double width, const std::string &name);

  void AddObjectNode(long nodeID, double x, double y, double z, long areaID,
      const std::string &areaType, const std::string &category, double radius,
      long objectID);

  void AddPerson(long personID, double x, double y, double theta, double speed,
      double visiblity, long areaID);
  void MovePerson(long personID, double x, double y, double theta, long areaID);
  void RemovePerson(long personID);

  void AddAccessEdge(long startNodeID, long endNodeID, double weight);
  void AddVisibilityEdge(long startNodeID, long endNodeID, double weight);
  void setRobotArea(long id);
  void updatePeopleInWM();
  void setRobotPose(double x, double y, double theta);
  void updateTrackedPersonInWM(long id);

  void changedArea(int aid);
  void checkAndAddNewArea(int aid);
  void mergedAreaIds(int aid1, int aid2);

  /// Methods for predicate handling

  /// Gets a conjunction or disjunction of predicates from string
  boost::shared_ptr<AbstractPredicate<ProxyPtr> > getPredicates(
      std::istream &str);
  /// Gets a single predicate from string
  boost::shared_ptr<AbstractPredicate<ProxyPtr> > getPredicate(
      std::istream &str);

  class PersonHyp {
  public:

    enum Status {
      STATUS_UNKNOWN = 0, STATUS_FOUND, STATUS_DELETE, STATUS_CREATE,
    };

    /// The status of this person hypothesis. This is used to tell
    /// what to do with the working memory image of this object (if it
    /// exist)
    int m_Status;

    /// The id of this person in the working memory
    std::string m_WMid;

    ///// The data about the person
    //SpatialData::Person m_data;
  };

  std::vector<PersonHyp> m_People;

  Cure::NavGraph m_cureNavGraph;
  Cure::MutexWrapper m_cureNavGraphLock;

  std::fstream m_EventFile;

  ///// This class holds the data for the areas and pointers to where
  ///// they can be found in working memory
  //class Area {
  //public:
  //std::string m_WMid;
  //NavData::Area m_data;
  //};

  class Place {
  public:
    std::string m_WMid;
    SpatialData::Place m_place;
  };

  // List of areas
  std::list<Place> m_places;

  //// Pointer to the struct in working memory that holds the
  //// topological position
  //NavData::TopologicalRobotPos m_TopRobPos;
  //std::string m_TopRobPosWMid;
  //bool m_WriteFirstTopologicalPose; // true when navgraph read from file

  // The string that identifies the robotpose in the working memory
  std::string m_RobotPoseWMid;

  // Whether the tester itself should create Spatial WM entries rather than wait for them
  bool m_fakeTest;
  double m_Threshold;

  NavData::NavGraph* m_pNavGraph;
  std::string m_NavGraphWMid;

  //int m_CurrPerson;
  //std::string m_CurrPersonWMid;

  bool m_cureNavGraphChanged;
}; // ComsysTester
} // namespace Binding

#endif // TESTER_MONITOR_H_
