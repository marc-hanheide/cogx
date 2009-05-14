/**
 * DumbArse
 * Fuckedi-fuckedi-fuck-fuck-fuck.
 *
 * @author Michael Zillich
 * @date October 2006
 */

#ifndef CAST_DUMB_ARSE_H
#define CAST_DUMB_ARSE_H

#include <cast/architecture/ManagedProcess.hpp>
#include "vision/idl/Vision.hh"

using namespace cast;  using namespace cast::cdl; 
using namespace cast::cdl::guitypes; 
using namespace std; using namespace boost; 

class DumbArse : public ManagedProcess
{
private:
  string pursuedObjId;
  list<string> pickableObjects; // list of WM IDs
  list<Math::Pose3D> putdownPoses;
  list<string> hands;  // list of WM IDs
  string visionSA;

  void handleAddObject(const WorkingMemoryChange &changeId);
  void handleUpdateObject(const WorkingMemoryChange &changeId);
  void handleDeleteObject(const WorkingMemoryChange &changeId);
  void handlePNPFinished(const WorkingMemoryChange &changeId);
  void handleSceneChanged(const WorkingMemoryChange &changeId);
  bool isSceneStatic();
  void pickNextObject();
  void handEntered();
  void allHandsExited();
  bool performingPick() {return !pursuedObjId.empty();}

protected:
  virtual void runComponent();
  virtual void taskAdopted(const string &_taskID){};
  virtual void taskRejected(const string &_taskID){};
  virtual void redrawGraphics3D();
  virtual void redrawGraphicsText();

public:
  DumbArse(const string &_id);
  virtual ~DumbArse();
  virtual void start();
};

#endif

