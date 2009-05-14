// =================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
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

package org.cognitivesystems.comsys.components;

// =================================================================
// IMPORTS
// =================================================================

// -----------------------------------------------------------------
// CAST IMPORTS
// -----------------------------------------------------------------

import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.*;
import cast.core.data.CASTData;

// -----------------------------------------------------------------
// COMSYS IMPORTS
// -----------------------------------------------------------------

import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.data.ProcessingData;
import org.cognitivesystems.comsys.general.ComsysException;
import org.cognitivesystems.comsys.ontology.ComsysGoals;

// -----------------------------------------------------------------
// JAVA IMPORTS
// -----------------------------------------------------------------
import java.util.Hashtable;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Vector;

import cast.architecture.abstr.ChangeFilterFactory;

// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
	The abstract class <b>PackedLFProcessorComponent</b> implements the
	basic functionality for a component that listens to the working memory 
	for a PackedLF struct to appear, and process it. The class implements
	all the necessary CAST methods, and simple access methods for dealing
	with the PackedLF. 

*/ 




public abstract class PackedLFProcessorComponent 
	extends ManagedProcess
{

    // =================================================================
    // GLOBAL DATA STRUCTURES
    // =================================================================

    // Hashtable used to record the tasks we want to carry out
    private Hashtable<String, ProcessingData> m_proposedProcessing = new Hashtable<String, ProcessingData>();

    // Hashtable linking data IDs to goal IDs
    private Hashtable<String, String> m_dataToProcessingGoalMap = new Hashtable<String, String>();

    // Vector with objects to be processed
    private Vector<ProcessingData> m_dataObjects = new Vector<ProcessingData>();

    // Processing Data id counter
    private int pdIdCounter = 0;

    // =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================

	/** The unary constructor 
	
		@param _id The id of the process
	*/ 
	
	public PackedLFProcessorComponent (String _id) { 
		super(_id);
	} // end constructor
	
	/** The method <i>init</i> initializes the internal variables. This method is called in <i>start</i>, i.e. after 
	    the <i>configure</i> method has been called by the super class.
		<p>
		This method has been declared abstract, to enforce its implementation in the actual class that extends this one. 
	*/
	
	abstract void init (); 
	
	/**
     * The method <i>taskAdopted</i> processes a parsing task once the
     * task manager has informed the parser component it can run. 
     * This method does <b>not</b> inform the task manager, whether the
     * task has been succesfully completed. This notification happens in
     * the <i>runComponent</i> method, which executes the task (run on its 
	 * associated data). 
     * 
	 * @param _goalID The identifier of the task that has been adopted
	 * 
     * @see #runComponent
     * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
     */
	 
    @Override
    protected void taskAdopted(String _goalID) {
        // get the data we stored for this goal
        ProcessingData data = m_proposedProcessing.remove(_goalID);
        // if we have stored this goal earlier
        if (data != null) {
            // add the data item to the data objects queue
            m_dataObjects.addElement(data);
            // get the identifier of the CAST data type
            String dataID = data.getID();
            // link the data ID to the goal ID, for future reference
            // on task completion (done in runComponent)
            m_dataToProcessingGoalMap.put(dataID, _goalID);
        }
        else {
            log("ERROR: Goal without data: " + _goalID);
        } // end if..else
    } // end taskAdopted
	
    /**
     * The method <i>taskRejected</i> prints out a log message that the proposed task 
	 * has been rejected, and removes the goal from the queue of proposed tasks (by ID). 
     * 
     * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _goalID) {
        log("WARNING: The goal with ID [" + _goalID + "] has been rejected.");
        m_proposedProcessing.remove(_goalID);
    } // end taskRejected
	
    /**
	 * The method "start" calls the class's <i>init</i> method, and adds a working memory change filters on PackedLFs objects. 
	 * If an object has been added to, or overwritten in, working memory, the method <i>handleWorkingMemoryChange</i> is being called. <p>
     * 
	 * @see #handleWorkingMemoryChange(WorkingMemoryChange _wmc)
 	 * @see #handleActiveDataWorkingMemoryChange(WorkingMemoryChange _wmc)
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged(cast.corba.autogen.SubArchitecture.WorkingMemoryChange[])
     */
    @Override
    public void start() {
        super.start();
		// now call the init method. 
		init();

        try {
			// register change filters for PackedLFs, which triggers continuation of parsing				
            
           	addChangeFilter(
           			ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.ADD),
                    new WorkingMemoryChangeReceiver() {
                        public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                             handleWorkingMemoryChange(_wmc);
                        }
                    });


			// register change filters for PackedLFs, which triggers continuation or finalization of parsing				
            addChangeFilter(
           			ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.OVERWRITE),
                    new WorkingMemoryChangeReceiver() {
                        public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                             handleWorkingMemoryChange(_wmc);
                        }
                    });
            
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        } // end try..catch
    }// end start
	
	
    /**
	 * The method <i>handleWorkingMemoryChange</i> retrieves a PackedLF CASTData object from the working memory, 
	 * and stores it in the <tt>m_proposedProcessing</tt> table with a <tt>ComsysGoals.PACKEDLF_PROCESSING_TASK</tt>.
	 * 
     * @param _wmc The working memory change -- a recently added or overwritten CASTData object with a PackedLF
     */
    private void handleWorkingMemoryChange(WorkingMemoryChange _wmc) {
        try {
            // get the id of the working memory entry
            String id = _wmc.m_address.m_id;
            // get the data from working memory and store it with its id
            CASTData data = new CASTData(id,
			    getWorkingMemoryEntry(id));
            // get a new id for the task
            String taskID = newTaskID();
            // store the data we want to process for later
            ProcessingData pd = new ProcessingData(newProcessingDataId());
            pd.add(data);
            m_proposedProcessing.put(taskID, pd);
			// set the goal
            String taskGoal = ComsysGoals.PACKEDLF_PROCESSING_TASK;
			// store the goal with its information
            proposeInformationProcessingTask(taskID, taskGoal);
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        } // end try..catch
    } // end handleWorkingMemoryChange	
	
	/** Returns a new identifier for a ProcessingData object 
		@return String The new (unique) identifier
	*/

    private String newProcessingDataId() {
        String result = "pd" + pdIdCounter;
        pdIdCounter++;
        return result;
    } // end newProcessingDataId	
	
	
	// =================================================================
    // RUN METHODS
    // =================================================================	
	
    /**
     * The method <i>runComponent</i> cycles over the queues with data
     * items (for which tasks have been proposed), checking whether
     * tasks can be executed. If there is a data item on a queue, the
     * appropriate task is executed, and the task manager is informed of
     * task completion (including success / failure).
	 * <p>
	 * By default, a PACKEDLF_PROCESSING_TASK is proposed for a PackedLF. On this data item, 
	 * the method will call the method <i>executeProcessingTask</i>. This method has been 
	 * declared abstract here -- any instantiating class needs to implement this. 
     * 
     * @see cast.core.components.CASTComponent#runComponent()
     */
    @Override
    public void runComponent() {
        try {
            while (m_status == ProcessStatus.RUN) {
                // lock from external access
                lockProcess();
                // check (synchronised) data objects queue
                ListIterator<ProcessingData> i = m_dataObjects
                    .listIterator();
                // for the data on the queue, tasks have been approved, so
                // iterate over data objects, and execute the tasks.
                while (i.hasNext()) {
                    // get the data from the queue
                    ProcessingData data = i.next();
                    // check its ID, get the corresponding task ID
                    String dataID = data.getID();
                    String taskID = m_dataToProcessingGoalMap
                        .get(dataID);
                    // make sure we know what to do, and have something
                    // to do it with
                    if (data != null) {
                        // execution tasks throw comsys exceptions
                        try {
                            executeProcessingTask(data);
                            // inform the goal manager that the task has
                            // been completed
                            try {
                                taskComplete(
                                    taskID,
                                    TaskOutcome.PROCESSING_COMPLETE_SUCCESS);
                            }
                            catch (SubarchitectureProcessException e) {
                                e.printStackTrace();
                            } // end try..catch
                        }
                        catch (ComsysException e) {
                            log("Exception while executing a task: "
                                + e.getMessage());
                            // inform the goal manager that the task has
                            // been completed, but unsuccessfully
                            // we may want to make this more specific
                            try {
                                taskComplete(
                                    taskID,
                                    TaskOutcome.PROCESSING_COMPLETE_FAILURE);
                            }
                            catch (SubarchitectureProcessException ex) {
                                ex.printStackTrace();
                            } // end try..catch
                        } // end try..catch for processing exceptions
                    }
					
                    else {
                        log("Nothing to process: data null");
                    } // end
                    // Clean up!
                    // no matter what happened, remove the data item
                    // from the queue
                    i.remove();
                } // end while
                // Free the process
                unlockProcess();

                sleepProcess(20);
                
                // wait for new tasks!
                // waitForNotifications(m_dataObjects);
				waitForNotifications();
            } // end while running
        }
        catch (Exception e) {
            e.printStackTrace();
        } // end try..catch
    } // end runComponent
	
	// =================================================================
    // COMPUTATION METHODS
    // =================================================================	
		
	/**
		The method <i>executeProcessingTask</i> is the method called when a PackedLF processing task 
		has been accepted. 
		
		@param  pd The PackedLF data to be processed
		@throws ComsysException Thrown if something goes wrong 
	*/ 

	abstract void executeProcessingTask (ProcessingData pd) throws ComsysException; 
	


} // end abstract class
