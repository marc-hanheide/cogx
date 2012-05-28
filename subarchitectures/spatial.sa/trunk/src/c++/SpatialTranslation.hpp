//
// = Filename
//   SpatialTranslation.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Chandana Paul
//    Dorian Galvez
//
// = COPYRIGHT
//    Copyright (c) 2007 Chandana Paul
//    		    2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

//NOTE: This file "adapted" (nicked) from nav.sa version with NavCommand structs.
//SpatialData doesn't have everything that NavData does; the extraneous
//portions of this code have been commented out. /KS

#ifndef SpatialTranslation_hpp
#define SpatialTranslation_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialData.hpp>
#include <NavData.hpp>
#include <string>
#include <list>
#include <map>
#include <Utils/MutexWrapper.hh>
#include <FrontierInterface.hpp>

class Rendezvous;

namespace spatial {

/**
 * This class listens for NavData::NavCommand structs written to WM
 * and handles the execution of these.
 *
 * @author Dorian Galvez Lopez
 * @see
 */
class SpatialTranslation: public cast::ManagedComponent {

public:
  SpatialTranslation();
  virtual ~SpatialTranslation();
  virtual void start();
  virtual void configure(const std::map<std::string, std::string>& config);
  virtual void stop();
  virtual void runComponent();

protected:

  virtual void taskAdopted(const std::string &_taskID) {
  }
  virtual void taskRejected(const std::string &_taskID) {
  }

private:

  void newNavCommand(const cast::cdl::WorkingMemoryChange &objID);
  void newNavGraph(const cast::cdl::WorkingMemoryChange &objID);
  void owtNavGraph(const cast::cdl::WorkingMemoryChange &objID);

private:
  int m_StartupDelay;
  // The last NavGraph used by some operation
  // Attention! It may not be updated, use getNavGraph instead
  NavData::NavGraphPtr m_TmpNavGraph;
  Cure::MutexWrapper m_NavGraphMutex;
  // This says if have been there changes on the nav graph since then
  bool m_navGraphChanged;
  bool m_bNoNavGraph;

  // Set to true for GOTOPLACE that refer to Placeholders
  bool m_isExplorationAction;
  // Set to true in order for SpatialTranslation to send visual exploration
  // commands when an exploration movment finished
  bool m_issueVisualExplorationActions;

  bool m_generatePlaceholdersOnPlace;

  // NavCommand with its id on WM
  typedef std::pair<std::string, SpatialData::NavCommandPtr> tpNavCommandWithId;

  // Queue of tasks with mutex
  class tpQueue: public std::list<tpNavCommandWithId>,
      public Cure::MutexWrapper {
  public:
    /* When true, the process must abort all the NavCtrlCommands
     * and stop the work of the ongoing task.
     */
    bool m_Abort;

    /* Says if there is some task executing right now
     * (this task is not in the queue)
     */
    bool m_Executing;

    /* Saves the NavCmd WM Id of the executed task, if any
     */
    std::string m_ExecutingId;

    tpQueue() :
      m_Abort(false), m_Executing(false) {
    }
  };
  tpQueue m_Tasks;

  // Condition variable to signal when there are work to do
  // (or the component was stopped)
  pthread_cond_t m_MutexCond; // this is used along with m_Tasks.m_Mutex

private:

  /* Changes the completion and status of a NavCommand.
   * @param id: command id on WM
   * @param completion: completion field
   * @param status: status field
   */
  void changeNavCmdCompletion(const std::string &id,
      const SpatialData::Completion &completion,
      const SpatialData::StatusError &status);

  /* Sends a VisualExplorationCommand to SpatialControl
   * @param rv: Rendezvous object, on which to add a change filter
   * for VisualExplorationCommand overwrites
   */
  void issueVisualExplorationCommand(Rendezvous &rv);

  void issueEndPlaceTransitionCommand(Rendezvous &rv, bool failed,
      bool generate_placeholders);

  /* Executes the given command, marks its completion and status
   * and returns when it is over.
   * @param cmd: the NavCommand to execute
   * @note: invoked by this thread
   */
  void executeCommand(const tpNavCommandWithId &cmd);

  /* "Executes" a block command by blocking the queue until the command
   * removed or aborted.
   * @param cmd: the block command and its id id on WM
   * @note: invoked by this thread; the thread is blocked
   */
  void executeBlockCommand(const tpNavCommandWithId &cmd);

  /* Adds a task to the queue, taking into account its priority.
   * It also signals the m_MutexCond
   * @param id: id of the cmd on WM
   * @param cmd: the NavCommand
   * @note: invoked by other component's thread
   */
  void addTaskToQueue(const std::string &id,
      const SpatialData::NavCommandPtr &cmd);

  /* Cancels the ongoing task
   * @param stop_robot: iif true, sends a NavCtrlCommand to stop the robot
   * @[param navCtrlCmdId]: id of the current navCtrlCommand, to remove it
   * @note: invoked by this thread
   */
  void cancelCurrentTask(bool stop_robot, std::string navCtrlCmdId = "");

  /* Cancels all the enqueued tasks and removes them from the queue.
   * It also marks the ongoing task as aborted.
   * @attention: this function is called with m_Tasks locked
   * @note: invoked by other component's thread
   */
  void cancelQueueTasks();

  /* Translates a NavCommand into a (or its first) NavCtrlCommand
   * @param nav: the NavCommand to translate
   * @param ctrl (out): the resulting NavCtrlCommand
   * @param status (out): status of the ctrl command
   * @return: false iif some error occurred and nav cannot be executed
   */
  bool translateCommand(const SpatialData::NavCommandPtr &nav,
      NavData::InternalNavCommand &ctrl, SpatialData::StatusError &status);

  /* Looks up the name of the given object in the given navgrpah
   * to return the (x,y) position of it.
   * If there are more than one, it returns the first only.
   * @param objectName: object name
   * @param graph: nav graph
   * @param x (out): x-coord of the object
   * @param y (out): y-coord of the object
   * @return true iif the node is found
   */
  bool findObjectNode(const std::string &objectName,
      const NavData::NavGraphPtr &graph, double &x, double &y);

  /* Looks up the name of the given object in the given navgraph
   * to return a suitable node to see the object from.
   * Note that this function is different from findObjectNode, because
   * this one returns a free node where the robot can move, and the other, 
   * the position of an object node
   * @param objectName: object name
   * @param graph: nav graph
   * @param nodeID (out): id of the best node
   * @return true iif some node is found
   */
  bool findObjectBestNode(const std::string &objectName,
      const NavData::NavGraphPtr &graph, long &nodeID);

  /* Returns the position of the robot if available
   * @param pose (out): position of the robot
   * @return: true iif pose is valid
   */
  bool getRobotPose(NavData::RobotPose2dPtr &pose);

  /* Returns the nav graph updated
   * @return: updated nav graph
   */
  NavData::NavGraphPtr getNavGraph();

  FrontierInterface::PlaceInterfacePrx m_placeInterface;
};

}
; // namespace navsa

#endif // SpatialTranslation_hpp
