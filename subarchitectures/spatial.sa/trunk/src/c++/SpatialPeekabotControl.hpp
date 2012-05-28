//
// = FILENAME
//   SpatialPeekabotControl.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2008 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef SpatialPeekabotControl_hpp
#define SpatialPeekabotControl_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialData.hpp>
#include <NavData.hpp>
#include <FrontierInterface.hpp>
#include <set>

#include <peekabot.hh>
#include <peekabot/Types.hh>

namespace spatial {

/**
 * This class allows you to control the robot using peekabot, by
 * dragging an icon on the screen to where you want the robot to go.
 *
 * @param -c a CURE config file to optionally read the
 * PEEKABOT_HOST and PEEKABOT_ROBOTXMLFILE. The first are to say where
 * the peekabot server is running and the second gives the name of the
 * xml file with the robot definition.
 * @param --retry-interval  How many secs to wait to try to connect again to the peekabot server (default 10s). Negative value mean no retries
 * @param --action what action to perform 
 * @param --do-path-query whether to make a PathProbabilityQuery before sending NavCommand
 *
 * @author Patric Jensfelt
 */
class SpatialPeekabotControl: public cast::ManagedComponent {
  class PendingQueryReceiver: public cast::WorkingMemoryChangeReceiver {
  public:
    PendingQueryReceiver(SpatialPeekabotControl *parent) :
      m_parent(parent), m_dependentCommand(0) {
    }
    void setDependentCommand(SpatialData::NavCommandPtr &depCmd) {
      m_dependentCommand = depCmd;
    }
    virtual void workingMemoryChanged(
        const cast::cdl::WorkingMemoryChange &_wmc);
  private:
    SpatialPeekabotControl *m_parent;
    SpatialData::NavCommandPtr m_dependentCommand;
  };

public:
  SpatialPeekabotControl();
  virtual ~SpatialPeekabotControl();

protected:

  virtual void configure(const std::map<std::string, std::string>& config);
  virtual void start();
  virtual void runComponent();

  void newPlace(const cast::cdl::WorkingMemoryChange &_wmc);
  void deletedPlace(const cast::cdl::WorkingMemoryChange &_wmc);

  void connectPeekabot();
  void updatePeekabotGadget();

  std::string m_PbHost;
  int m_PbPort;
  //std::set<int> m_Places;
  int m_maxPlaces;

  bool m_doPathQuery;
  PendingQueryReceiver m_pendingQueryReceiver;

  int m_RetryDelay; // Seconds to retry if cannot connect. -1 means dont retry

  int m_CtrlAction;
  SpatialData::PlaceIDSeq placeseq;
  peekabot::PeekabotClient m_PeekabotClient;
  peekabot::GroupProxy m_controlmodule;
  const double gadget_y;
  const double gadget_ystep;
  int gadgetLineLength;

  //Visualization of placeholders - Year 1 only!
  peekabot::GroupProxy m_placeholderModule;
  void newNodeHypothesis(const cast::cdl::WorkingMemoryChange &_wmc);
  void deletedNodeHypothesis(const cast::cdl::WorkingMemoryChange &_wmc);
  void updatePlaceholderVisualization();
  int m_maxPlaceholderID;
private:
  NavData::FNodePtr getCurrentNavNode();
  bool m_hideGadget;

  FrontierInterface::PlaceInterfacePrx m_placeInterface;

};

}
; // namespace spatial

#endif // SpatialPeekabotControl_hpp
