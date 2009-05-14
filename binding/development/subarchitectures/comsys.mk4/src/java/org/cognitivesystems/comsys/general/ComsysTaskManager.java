// =================================================================
// Copyright (C) 2006 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION
// =================================================================

package org.cognitivesystems.comsys.general;

// =================================================================
// IMPORTS
// =================================================================

// -----------------------------------------------------------------
// CAST IMPORTS
// -----------------------------------------------------------------

import java.io.IOException;
import java.util.*;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.RecogResult;

import org.cognitivesystems.interconnectivity.data.SyncState;
import org.cognitivesystems.interconnectivity.data.SyncStateModel;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.architecture.subarchitecture.SubarchitectureTaskManager;
import cast.cdl.*;
import cast.core.data.CASTData;

// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
 * The class <b>ComsysTaskManager</b> implements the task manager for
 * the communication subsystem. The task manager needs to be provided a
 * pointer to a synchronization configuration file (<tt>--file</tt>
 * command-line argument), from which it can build a state-based
 * synchronization model. This model describes the synchronization
 * constraints on building data structures in the subarchitecture, thus
 * influencing when particular processes are to be allowed to run.
 * <p>
 * Synchronization includes "wait" transitions, and "time-out"
 * transitions. Time-outs mean that no data arrives, but that a
 * transition to a next processing state is triggered nevertheless. To
 * be able to have the process(es) at that state running, the
 * task-manager enables processes to register tasks, and wait
 * (TaskManagementDecision.GOAL_WAIT). Then, once the processes are
 * runnable, the task manager approves their goals
 * (TaskManagementDecision.GOAL_ADOPTED).
 * 
 * @see cast.corba.autogen.CAST.TaskManagementDecision
 * @version 061020 (started 061017)
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
 * @author Nick A. Hawes (n.a.hawes@cs.bham.ac.uk)
 */

public class ComsysTaskManager extends SubarchitectureTaskManager

{

    // =================================================================
    // GLOBAL VARIABLES
    // =================================================================

    // Queue with processing goals, including the ones told to wait.
    protected Vector<InformationProcessingTask> goalQueue;

    // The state-based synchronized model
    SyncStateModel syncModel;
    // Identifier of the state in which the synchronization model is
    private int taskState;

    // The filename for the synchronization configuration file
    private String confFileName;

    long lastTime;

    // =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================

    /**
     * @param _id
     */

    public ComsysTaskManager(String _id) {
        super(_id);
        init();
        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
    } // end constructor

    private void init() {
        goalQueue = new Vector<InformationProcessingTask>();
        confFileName = "./interconnectivity.sync";
         syncModel = new SyncStateModel();
        syncModel.setLogging(false);
    } // end init

    // =================================================================
    // CORE CAST METHODS
    // =================================================================

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.SubarchitectureTaskManager#goalProposed(java.lang.String,
     *      cast.corba.autogen.CAST.InformationProcessingGoal)
     */
    @Override
    protected void taskProposed(String _src,
                                InformationProcessingTask _data) {

        // println("adding to queue " + _data.m_id + " ...");
        goalQueue.add(_data);
        // println("... addition done");
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.SubarchitectureTaskManager#taskRetracted(java.lang.String,
     *      java.lang.String)
     */
    @Override
    protected void taskRetracted(String _src, String _taskID) {
        // println("retracting task: " + _taskID);

        for (ListIterator<InformationProcessingTask> i = goalQueue
            .listIterator(); i.hasNext();) {
            InformationProcessingTask task = i.next();
            if (task.m_id.equals(_taskID)) {
                i.remove();
                // println("retracted");
                return;
            }
        }

        // println("not found");
    }

    /**
     * The method <i>processGoalQueue</i> gets the current runnable
     * processes from the synchronization model, and checks whether
     * these have any posted tasks. If they do, then the task manager
     * informs these processes that their tasks have been adopted.
     * Otherwise, the tasks are put on hold (GOAL_WAITING).
     */

    protected void processGoalQueue() {
        // println("checking goals.... queue size: " +
        // m_goalQueue.size());

        ListIterator<InformationProcessingTask> i = goalQueue
            .listIterator();
        while (i.hasNext()) {

            InformationProcessingTask ipg = i.next();

            // just accept goals at this point...
            // println("sending task decision...");
            // sendTaskDecision(ipg,
            // TaskManagementDecision.GOAL_ADOPTED);

            if (syncModel.isCurrentlySupported(ipg.m_taskName)) {
                log("Information processing task [" + ipg.m_taskName
                    + "] supported at current state");
                sendTaskDecision(ipg,
                    TaskManagementDecision.GOAL_ADOPTED);
            }
            else {
 
            	
                log("Information processing task [" + ipg.m_taskName
                    + "] NOT supported at current state: " +  syncModel.getCurrentlySupported());
                sleepProcess(500);
                sendTaskDecision(ipg,
                    TaskManagementDecision.GOAL_WAITING);
            } // end if..else check whether process can execute
            // println("... task decision sent");
            // ... and remove them from queue
            i.remove();
        }
        // println("... goal checking done");
    }

    boolean componentStarted = false;
    
    /**
     * The method <i>runComponent</i> first creates the state-based
     * synchronization model from the synchronization configuration
     * file. It then initializes the model, and loops through processing
     * the tasks on the goal queue.
     * 
     * @see cast.core.components.CASTComponent#runComponent()
     */
    @Override
    public void runComponent() {
        lastTime = (new Date()).getTime();
        log("Starting comsys task manager");
        // Initialize state model; this may throw an exception
        try {
            syncModel.construct(confFileName);
        }
        catch (IOException e) {
            log("Error while trying to construct synchronization model:\n "
                + e.getMessage() + "\nExiting ...");
            System.exit(0);
        } // end try.. catch
        // Start the synchronization model
        syncModel.start();
        componentStarted = true;
        // Update the goal queue
        while (m_status == ProcessStatus.RUN) {
            // try {
            // println("waiting to check queue...");
            lockProcess();
            processGoalQueue();
            unlockProcess();
            // sleepProcess(20);
            waitForProposals();
            // println("... queue done");
            // Thread.sleep(100);
            // } catch (InterruptedException e) {
            // e.printStackTrace();
            // } // end try..catch
        } // end while running
    } // end runComponent

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.SubarchitectureTaskManager#taskCompleted(java.lang.String,
     *      cast.corba.autogen.CAST.GoalProcessingResult)
     */
    @Override
    protected void taskCompleted(String _src, TaskResult _data) {}

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.SubarchitectureTaskManager#taskRegistered(java.lang.String,
     *      cast.corba.autogen.CAST.TaskDescription)
     */
    @Override
    protected void taskRegistered(String _src, TaskDescription[] _desc) {
        for (int i = 0; i < _desc.length; i++) {
            println("task registered: " + _desc[i].m_taskName);
        }

    }

    /**
     * The <i>configure</i> method overrides the method from
     * CASTProcessingComponent (though calls the super methods to ensure
     * any higher-level configuration is done), and looks for a
     * command-line argument <tt>--file</tt> specifying the filename
     * for the synchronization configuration file. If no such argument
     * is given, the filename defaults to <i>./interconnectivity.sync</i>.
     * 
     * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
     */
    @Override
    public void configure(Properties _config) {
//        _config.list(System.out);
        super.configure(_config);
        if (_config.containsKey("--file")) {
            confFileName = _config.getProperty("--file");
        }
        else {
            confFileName = "./interconnectivity.sync";
        } // end if..else check for command-line argument
    } // end configure

    /**
     * The method <i>workingMemoryChanged</i> is triggered whenever
     * working memory has been updated with new information. The type of
     * change is retrieved, and communicated to the state model (without
     * the prefix "Comsys:").
     * <p>
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged(cast.corba.autogen.SubArchitecture.WorkingMemoryChange[])
     */
    @Override
    public void start() {

        super.start();
        
        try {
            addChangeFilter(
            		ChangeFilterFactory.createOperationFilter(WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        newAddition(_wmc);
                    }
                });
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        }

    } // end start

    /**
     * @param _wmc
     * @param i
     */
    private void newAddition(WorkingMemoryChange _wmc) {
    	
    	while (!componentStarted) {
    		log("Waiting, component not started yet...");
    		sleepProcess(100);
    	}
        try {
            long time = (new Date()).getTime();
            long helperTime = time;
            time = time - lastTime;
            lastTime = helperTime;
            // log("[DELTA(ms):"+time+"] Memory update, check for data
            // type and transitions");
            String id = _wmc.m_address.m_id;
            String subarch_id = _wmc.m_address.m_subarchitecture;
            // Added by Henrik J to avoid conflicts (not fully
            // understood ones) when the updates are in CatSys WM
            if (subarch_id.startsWith("comsys")) {
                CASTData dataItem = getWorkingMemoryEntry(id);
                String dataType = dataItem.getType();
                log("Updated WM - local namespace datatype ["+dataType+"]");
                syncModel.updateOnData(dataItem);
                String supportedTasks = "[";
                Iterator currStatesIter = syncModel.getCurrentStates();
                while (currStatesIter.hasNext()) {
                    Integer stateId = (Integer) currStatesIter.next();
                    SyncState state = syncModel.getState(stateId);
                    supportedTasks = supportedTasks
                        + state.getProcessIds();
                    if (currStatesIter.hasNext()) {
                        supportedTasks = supportedTasks + ",";
                    }
                } // end while
                supportedTasks = supportedTasks + "]";
               log("Supported tasks: "+supportedTasks);
            } // end try
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        } // end try..catch
    }

    // =================================================================
    // MISC METHOD
    // =================================================================

} // end class
