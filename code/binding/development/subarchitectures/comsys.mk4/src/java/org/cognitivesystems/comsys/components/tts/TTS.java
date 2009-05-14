// =================================================================
// Copyright (C) 2005,2006 Maria Staudte (mary@staudte.de),
// Geert-Jan M. Kruijff (gj@acm.org)
//
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

package org.cognitivesystems.comsys.components.tts;

// =================================================================
// IMPORTS
// =================================================================

// -----------------------------------------------------------------
// CAST IMPORTS
// -----------------------------------------------------------------
import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.*;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.AudioData;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.RecogResult;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.SpokenOutputItem;
import org.cognitivesystems.comsys.data.ProcessingData;
import org.cognitivesystems.comsys.general.ComsysException;
import org.cognitivesystems.comsys.ontology.ComsysGoals;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.*;
import cast.core.data.CASTData;
import de.dfki.lt.mary.client.MaryClient;
import cast.core.CASTUtils;

// =================================================================
// JAVADOC CLASS DOCUMENTATION
// =================================================================

/**
 * This class is a wrapper for the Mary system. When its solavble is
 * requested the agent receives a string with it that it transforms into
 * spoken output by means of Mary-classes. The system is able to
 * synthesize speech both locally (using the local speakers) and
 * remotely by inserting AudioData objects into the working memory,
 * which will then be retrieved by the SIP process to send them over the
 * connection.
 */

public class TTS extends ManagedProcess {

    // =================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    // =================================================================

    // ----------------------------------------------------------------
    // INFORMATION PROCESSING DATA STRUCTURES
    // ----------------------------------------------------------------

    // Hashtable used to record the tasks we want to carry out. For each
    // taskID we store a Vector with the data it is to work on
    private Hashtable<String, ProcessingData> m_proposedProcessing;

    // Hashtable linking data IDs to goal IDs
    private Hashtable<String, String> m_dataToProcessingGoalMap;

    // Hashtable linking task IDs to task types
    private Hashtable<String, String> m_taskToTaskTypeMap;

    // Vector with objects to be processed,
    // can be ComSys:PhonString,...
    private Vector<ProcessingData> m_dataObjects;

    // Identifiers for ProcessData objects
    private int pdIdCounter;

    // The name of the voice to be used
    static String voiceName = "female";

    // mary server host
    static String serverHost =
            System.getProperty("server.host", "susan.dfki.uni-sb.de");

    // mary server port
    static int serverPort =
            Integer.getInteger("server.port", 59125).intValue();

    // completely ignore mary if this is set
    private boolean m_bNoMary = false;

    // mary client
    static MaryClient mary;

    // local and remote silence
    private static boolean m_bSilentModeLocal = false;
    private static boolean m_bSilentModeRemote = false;

    // local and remote TTS
    TTSLocal ttsLocal;
   // TTSRemote ttsRemote;

    // =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================

    public TTS(String _id) {
        super(_id);
        init();
    } // end constructor

    private void init() {
        log("Initializing text-to-speech synthesis component");

        // general information processing structures
        m_proposedProcessing = new Hashtable<String, ProcessingData>();
        m_dataToProcessingGoalMap = new Hashtable<String, String>();
        m_taskToTaskTypeMap = new Hashtable<String, String>();
        m_dataObjects = new Vector<ProcessingData>();
        pdIdCounter = 0;
        // synthesis

        // nah: making all the comsys queue changes... don't want to
        // miss a thing
        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

        // let the guy say something
        // nah: moved to configure so that silent mode flag can be set
        // first
        // speak("hello !");
    } // end init

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
     */
    @Override
    public void start() {
        super.start();
        try {
			/**
            addChangeFilter(ComsysOntology.SPOKENOUTPUTITEM_TYPE,
                WorkingMemoryOperation.ADD, true,
                new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        newStringToSynthesizeAdded(_wmc);
                    }
                });
			*/
            addChangeFilter(
            		ChangeFilterFactory.createLocalTypeFilter(SpokenOutputItem.class,  WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        newStringToSynthesizeAdded(_wmc);
                    }
                });			
			
			
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        }
    }

    // =================================================================
    // CAST TASK METHODS
    // =================================================================

    /**
     * The method <i>taskAdopted</i> processes a dialogue production
     * task once the task manager has informed the component it can run.
     * The method pushes the processing data for a given task onto the
     * m_dataObjects queue, so that the runComponent() method can spot
     * something needs to be done. The method does not distinguish
     * between different types of tasks.
     * <p>
     * This method does <b>not</b> inform the task manager, whether the
     * task has been succesfully completed. This notification happens in
     * the <i>runComponent</i> method.
     * 
     * @see #runComponent
     * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _goalID) {
        // get the data we stored for this goal
        ProcessingData pd = m_proposedProcessing.remove(_goalID);
        if (pd != null) {
            // add the data item to the data objects queue
            m_dataObjects.addElement(pd);
            // get the identifier of the processing data object
            String pdID = pd.getID();
            // link the data ID to the goal ID, for future reference
            // on task completion (done in runComponent)
            m_dataToProcessingGoalMap.put(pdID, _goalID);
        }
        else {
            log("ERROR: Goal without data: " + _goalID);
        } // end if..else
    } // end taskAdopted

    /**
     * The method <i>taskRejected</i> removes a rejected task from the
     * list of proposed processing tasks.
     * 
     * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _goalID) {

        log("WARNING: The goal with ID [" + _goalID
            + "] has been rejected.");
        m_proposedProcessing.remove(_goalID);
    } // end taskRejected

    // =================================================================
    // CAST WORKING MEMORY MONITORING
    // =================================================================

    /**
     * The method <i>workingMemoryChanged</i> is triggered whenever
     * working memory has been updated with new information. The
     * dialogue interpretation component should only act when it is its
     * turn, according to the synchronization model of the task manager.
     * <p>
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged(cast.corba.autogen.SubArchitecture.WorkingMemoryChange[])
     */

    protected void newStringToSynthesizeAdded(WorkingMemoryChange _wmc) {
        // if we have a logical form collection string,
        try {
            // get the id of the working memory entry
            String id = _wmc.m_address.m_id;
            // get the data from working memory and store it
            // with its id
            CASTData soiWM =
                    new CASTData(id, getWorkingMemoryEntry(id));
            SpokenOutputItem soi = (SpokenOutputItem) soiWM.getData();
            // Propose a new task
            String taskID = newTaskID();
            // store the data we want to process for later
            ProcessingData pd =
                    new ProcessingData(newProcessingDataId());
            pd.add(soiWM);
            m_proposedProcessing.put(taskID, pd);
            // set up the goal
            String taskGoal;
            taskGoal = ComsysGoals.SPEECHSYNTHESIS_TASK;
            // then ask for permission
            proposeInformationProcessingTask(taskID, taskGoal);
            // store the id with the task type
            m_taskToTaskTypeMap.put(taskID, taskGoal);

        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        } // end try..catch

    } // end workingMemoryChanged

    /** Returns a new identifier for a ProcessingData object */

    private String newProcessingDataId() {
        String result = "pd" + pdIdCounter;
        pdIdCounter++;
        return result;
    } // end newProcessingDataId

    // =================================================================
    // CAST RUN COMPONENT
    // =================================================================

    /**
     * The method <i>runComponent</i> cycles over the queue with
     * processing data objects (for which tasks have been proposed),
     * checking whether tasks can be executed. If there is a processing
     * data object on the queue, the appropriate task is executed, and
     * the task manager is informed of task completion (including
     * success / failure).
     * 
     * @see cast.core.components.CASTComponent#runComponent()
     */
    @Override
    public void runComponent() {
        try {
            log("Entering loop checking for data in speech synthesis component");
            while (m_status == ProcessStatus.RUN) {

                // lock from external access
                lockProcess();
                // check (synchronised) processing data objects queue
                ListIterator<ProcessingData> i =
                        m_dataObjects.listIterator();
                // for the data on the queue, tasks have been approved,
                // so
                // iterate over data objects, and execute the
                // appropriate tasks.
                // we retrieve the task to be executed based on the type
                // of the approved
                // task, linked thru taskID/dataID.
                while (i.hasNext()) {

                    // get the data from the queue
                    ProcessingData data = i.next();
                    // check its ID, get the corresponding task ID, and
                    // the task type
                    String dataID = data.getID();
                    String taskID =
                            m_dataToProcessingGoalMap.get(dataID);
                    String taskType =
                            (String) m_taskToTaskTypeMap.get(taskID);
                    // log("For data [" + dataID + "/" + data.getTypes()
                    // + "] do [" + taskID + "/" + taskType + "]");
                    // make sure we know what to do, and have something
                    // to do it with
                    if (taskType != null && data != null) {
                        // execution tasks throw comsys exceptions
                        try {
                            if (taskType
                                .equals(ComsysGoals.SPEECHSYNTHESIS_TASK)) {

                                executeSpeechSynthesisTask(data,
                                    taskType);
                            }
                            else {
                                log("Unknown task type to process in Comsys:speechSynth component");
                            } // end if..else check for task type
                            // inform the goal manager that the task has
                            // been completed, but unsuccessfully
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
                            log("Exception while executing a task in speech synthesis: "
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
                        log("Nothing to process: taskType / data null");
                    } // end
                    // Clean up!
                    // no matter what happened, remove the data item
                    // from the queue
                    i.remove();
                    m_taskToTaskTypeMap.remove(taskID);
                } // end while
                // Free the process
                unlockProcess();

                // nah... put in a tiny sleep so it doesn't use 100% of
                // the CPU!
                sleepProcess(20);

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
     * If the SPEECHSYNTHESIS_TASK has been approved by the task
     * manager, proceed to the synthesis of the string given in the
     * SpokenOutputItem object.
     * <p>
     * the string can be synthesized either locally, remotely, or both
     * (depending on the configuration parameters).
     */
    public void executeSpeechSynthesisTask(ProcessingData pd,
                                           String task)
            throws ComsysException {
        CASTData soiWM =
                pd.getByType(CASTUtils.typeName(SpokenOutputItem.class));
        if (soiWM != null) {
            SpokenOutputItem soi = (SpokenOutputItem) soiWM.getData();

            try {

                if (m_bNoMary) {
                    printOutputItem(soi);
                }
                else {
					log("Trying to say the following: ["+soi.phonString+"]");
                    // Synthesize speech locally
                    ttsLocal.speak(soi.phonString);

                    // Synthesize speech remotely
          /**          byte[] data = ttsRemote.speak(soi.phonString);
                    if (data != null) {
                        // .. and insert an AudioData object into the
                        // working memory
                        // addToWorkingMemory(newDataID(), ComsysOntology.AUDIODATA_TYPE, new AudioData(data));
						addToWorkingMemory(newDataID(), new AudioData(data)); // refactored
                    } */ 
                }
            }
            catch (Exception e) {
                log("test");
                e.printStackTrace();
            }
        }
        else {
            throw new ComsysException(
                "Error: speech synthesis task on illegal processing data type "
                    + pd.getTypes());
        }
    } // end executeSpeechSynthesisTask

    /**
     * Method to print out the text to be spoken if mary is not running.
     * 
     * @param _soi
     */
    private void printOutputItem(SpokenOutputItem _soi) {
        println("\n\n" + _soi.phonString + "\n\n");
    }

    // =================================================================
    // CAST CONFIGURATION METHODS
    // =================================================================

    /**
     * The <i>configure</i> method overrides the method from
     * CASTProcessingComponent (though calls the super methods to ensure
     * any higher-level configuration is done), and looks for a
     * command-line argument <tt>--voice</tt> specifying the name of
     * the voice to be used. Default is "kevin".
     * 
     * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
     */
    @Override
    public void configure(Properties _config) {
//         _config.list(System.out);
        super.configure(_config);

        // nah: adding switch to ignore Mary for ease of testing

        // local silence
        if (_config.containsKey("--startserver")) {
            log("starting the MARY server");
			try {
				String[] cmd = {  "/bin/sh", "-c", "tools/mary/bin/maryserver &" };
				Process proc = Runtime.getRuntime().exec(cmd);
				InputStream stderr = proc.getErrorStream();
	            InputStreamReader isrerr= new InputStreamReader(stderr);
	            BufferedReader br1 = new BufferedReader(isrerr);
	            String line1 = null;
	            if ( (line1 = br1.readLine()) != null)
	                System.out.println(line1);
	                    
	            int exitVal = proc.waitFor();
	            System.out.println("Process exitValue: " + exitVal);

				}
				catch(Exception e){
					//process exception
				}
        }
        
        
        if (_config.containsKey("--no-mary")) {
            println("Running with no Mary server, all output items will be printed.");
            m_bNoMary = true;
        }
        else {
            // complete silence (both locally and remotely)
            if (_config.containsKey("--silent")
                || _config.containsKey("-s")) {
                log("switching to silent mode");
                m_bSilentModeLocal = true;
                m_bSilentModeRemote = true;
            }

            // local silence
            if (_config.containsKey("--silentLocal")
                || _config.containsKey("-s")) {
                log("switching to locally silent mode");
                m_bSilentModeLocal = true;
            }

            // remote silence
            if (_config.containsKey("--silentRemote")
                || _config.containsKey("-s")) {
                log("switching to remotely silent mode");
                m_bSilentModeRemote = true;
            }

            // voice name
            if (_config.containsKey("--voice")) {
                voiceName = _config.getProperty("--voice");
            }

            // IP address of the Mary server
            if (_config.containsKey("--serverHost")) {
                serverHost = _config.getProperty("--serverHost");
            }

			
			
            // port of the mary server
            if (_config.containsKey("--serverPort")) {
                serverPort =
                        Integer.valueOf(_config
                            .getProperty("--serverPort"));
            }

			String startingUp = ""; 
			
            // Line to be said on startup
            if (_config.containsKey("--startingUp")) {
                startingUp = _config.getProperty("--startingUp");
            }						
			
            try {
                mary = new MaryClient(serverHost, serverPort);
                // speakLocal("hello !");
            }

            catch (Exception e) {
                log(e);
            }

            // create local and remote TTS
            ttsLocal =
                    new TTSLocal(mary, voiceName, m_bSilentModeLocal, "WAVE");
       //     ttsRemote =
       //             new TTSRemote(mary, voiceName, m_bSilentModeRemote);
			
			if (!startingUp.equals("")) { 
				ttsLocal.speak(startingUp);
			}
			
        }

    } // end configure
} // end TTSAgent-class
