// =================================================================
// Copyright (C) 2005-2010 DFKI GmbH Talking Robots
// Maria Staudte (mary@staudte.de),
// Geert-Jan M. Kruijff (gj@acm.org)
//
// Raveesg Meena (rame01@dfki.de) Modified in Oct 2010
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
package de.dfki.lt.tr.cast.dialogue;

// =================================================================
// IMPORTS

// Java
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.*;
import java.util.concurrent.CountDownLatch;
import java.io.File;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.synthesize.*;

// Dialogue API CAST extension 
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.cast.dialogue.DialogueGoals;

// Dialogue API 
import de.dfki.lt.tr.dialogue.util.DialogueException;
import de.dfki.lt.tr.dialogue.tts.MaryTTSLocal;
import de.dfki.lt.tr.dialogue.tts.ProsodicTextToRawMARYXml;
import de.dfki.lt.tr.dialogue.tts.SynthesisRAWMaryXMLInput;

// CAST
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.SubarchitectureComponentException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

// MARY
import marytts.client.MaryClient;
import marytts.client.http.Address;


// =================================================================
// JAVADOC CLASS DOCUMENTATION
// =================================================================

/**
 *
 * @author 	Geert-Jan M. Kruijff (gj@dfki.de)
 */

public class MaryTTS extends ManagedComponent {

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
	private int m_pdIdCounter;

	// The name of the voice to be used
	static String m_voiceName = "us2";

	// mary server host
	static String m_serverHost =
			System.getProperty("server.host", "susan.dfki.uni-sb.de");

	// mary server port
	static int m_serverPort =
			Integer.getInteger("server.port", 59125).intValue();

	// completely ignore mary if this is set
	private boolean m_bNoMary = false;

	// mary client
	static MaryClient m_mary;

	// local and remote silence
	private static boolean m_bSilentModeLocal = false;
	private static boolean m_bSilentModeRemote = false;

	//Raveesh change start
	private static String m_RAWMARYXMLHeader=null;
	private static String m_GenrtdXMLFileLoc=null;
	private static boolean m_SaveGenrtdXMLFile=true;
	private static boolean m_SaveAudio2wav=false;
	//Raveesh change end

	// local and remote TTS
	MaryTTSLocal m_ttsLocal;
	// TTSRemote ttsRemote;

	// =================================================================
	// CONSTRUCTOR METHODS
	// =================================================================

	public MaryTTS(String _id) {
		init();
	} // end constructor

	public MaryTTS() {
	}

	private void init() {
		log("Initializing text-to-speech synthesis component");

		// general information processing structures
		m_proposedProcessing = new Hashtable<String, ProcessingData>();
		m_dataToProcessingGoalMap = new Hashtable<String, String>();
		m_taskToTaskTypeMap = new Hashtable<String, String>();
		m_dataObjects = new Vector<ProcessingData>();
		m_pdIdCounter = 0;
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
		init();
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
		catch (Exception e) {
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
			String id = _wmc.address.id;
			// get the data from working memory and store it
			// with its id
			CASTData soiWM = getWorkingMemoryEntry(id);
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
			taskGoal = DialogueGoals.SPEECHSYNTHESIS_TASK;
			// then ask for permission
			proposeInformationProcessingTask(taskID, taskGoal);
			// store the id with the task type
			m_taskToTaskTypeMap.put(taskID, taskGoal);

		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} // end try..catch

	} // end workingMemoryChanged

	/** Returns a new identifier for a ProcessingData object */

	private String newProcessingDataId() {
		String result = "pd" + m_pdIdCounter;
		m_pdIdCounter++;
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
			while (this.isRunning()) {

				// lock from external access
				lockComponent();
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
							if (taskType.equals(DialogueGoals.SPEECHSYNTHESIS_TASK)) 
							{	
								executeSpeechSynthesisTask(data,taskType);
							}
							else {
								log("Unknown task type to process in Comsys:speechSynth component");
							} // end if..else check for task type
							// inform the goal manager that the task has
							// been completed, but unsuccessfully
							taskComplete(
									taskID,
									TaskOutcome.ProcessingCompleteSuccess);
						}
						catch (DialogueException e) {
							log("Exception while executing a task in speech synthesis: "
									+ e.getMessage());
							// inform the goal manager that the task has
							// been completed, but unsuccessfully
							// we may want to make this more specific
							taskComplete(
									taskID,
									TaskOutcome.ProcessingCompleteFailure);
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
				unlockComponent();

				// nah... put in a tiny sleep so it doesn't use 100% of
				// the CPU!
				sleepComponent(20);

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
					throws DialogueException {
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

					//Here starts the code for processing prosodic input.
					//Note: '%' and '@' are the token that are "present" in prosodic grammar output
					if(soi.phonString.contains("%") || soi.phonString.contains("@") ){
						// Synthesize speech locally
						ProsodicTextToRawMARYXml l_convert = new ProsodicTextToRawMARYXml(m_RAWMARYXMLHeader,m_GenrtdXMLFileLoc,"cast");

						//Make a user friendly filename for this RawMayXMLfile
						l_convert.g_xmlfilename=l_convert.XmlFileName(soi.phonString);
						m_ttsLocal.m_SaveAudio2Wav=m_SaveAudio2wav;
						m_ttsLocal.m_AudioFileName=m_GenrtdXMLFileLoc.concat(l_convert.g_xmlfilename);
						log("Trying to say the following: ["+soi.phonString+"]");
						//Process the "text" string for Prosodic markers
						//if(soi.phonString.contains("%") || soi.phonString.contains("@") ){
						//Do prosodic to RAWMaryXML
						String l_xmlfile = new String();

						//A function that takes the prosodic text as input, converts it into RawMaryXML and returns the filename
						l_xmlfile=l_convert.ConvertToRawMarxXml(soi.phonString);

						log("XML file written: ["+l_xmlfile+"]");

						//Now Synthesize this file
						try {
							//Set inputs for RAWMARYXML processing
							m_ttsLocal.m_inputType="RAWMARYXML";
							//Save Audio to wav options

							SynthesisRAWMaryXMLInput l_synthsis = new SynthesisRAWMaryXMLInput(m_ttsLocal);
							CountDownLatch finished = new CountDownLatch(1);
							l_synthsis.Utter(m_GenrtdXMLFileLoc.concat(l_convert.g_xmlfilename), finished);
							finished.await();

							//Delete the generated RAWMaryXML
							if(!m_SaveGenrtdXMLFile){
								File f = new File(m_GenrtdXMLFileLoc.concat(l_convert.g_xmlfilename));
								if(f.exists()){
									log("XML file deleted: ["+m_GenrtdXMLFileLoc.concat(l_convert.g_xmlfilename)+"]");
									boolean success = f.delete();
									if (!success)
										throw new IllegalArgumentException("Delete RamMaryXMl failed");
								}
							}
//							Thread.sleep(2500);
						} catch (Exception e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}else {
						m_ttsLocal.m_inputType="TEXT";
						CountDownLatch finished = new CountDownLatch(1);
						m_ttsLocal.speak(soi.phonString, finished);
						finished.await();
//						Thread.sleep(2500);
					}
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
			throw new DialogueException(
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
	 * the voice to be used. Default is "us2".
	 * 
	 * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
	 */
	@Override
	public void configure(Map<String, String> _config) {
		//         _config.list(System.out);

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
					log(line1);

				log("wait for mary server to start");

				while (!line1.contains("started in")) {
					log("still waiting...");
					line1 = br1.readLine();
					if (line1==null) {
						log("failure reading from stderr");
						break;
					}
					log(line1);
				}

				log("continuing mary clientstart up");

				int exitVal = proc.waitFor();
				System.out.println("Process exitValue: " + exitVal);

				sleepComponent(200);
			}
			catch(Exception e){
				//process exception
			}
		}


		if (_config.containsKey("--no-mary")) {
			log("running with no Mary server, all output items will be printed");
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
				m_voiceName = _config.get("--voice");
			}

			// IP address of the Mary server
			if (_config.containsKey("--serverHost")) {
				m_serverHost = _config.get("--serverHost");
			}



			// port of the mary server
			if (_config.containsKey("--serverPort")) {
				m_serverPort =
						Integer.valueOf(_config
								.get("--serverPort"));
			}

			String startingUp = ""; 

			// Line to be said on startup
			if (_config.containsKey("--startingUp")) {
				startingUp = _config.get("--startingUp");
			}

			try {
				m_mary = MaryClient.getMaryClient(new Address(m_serverHost, m_serverPort));
			}
			catch (IOException ex) {
				getLogger().error("connection to the Mary server failed, switching to no-Mary mode", ex);
				m_bNoMary = true;
			}

			// Location of RAWMARYXMLHeader
			if (_config.containsKey("--rawMaryXmlHeader")) {
				m_RAWMARYXMLHeader = _config.get("--rawMaryXmlHeader");
			}

			// Location of temporarily storing generated MaryXMLFiles
			if (_config.containsKey("--writeXmlTo")) {
				m_GenrtdXMLFileLoc = _config.get("--writeXmlTo");
			}

			// Location of temporarily storing generated MaryXMLFiles
			if (_config.containsKey("--saveXml2disk")) {
				String tmp = _config.get("--saveXml2disk");
				if(tmp.equals("true")) m_SaveGenrtdXMLFile =true;
				else m_SaveGenrtdXMLFile =false;
			}
			// Saving audio files to wav, location is same as that of m_GenrtdXMLFileLoc
			if (_config.containsKey("--saveAudio2wav")) {
				String tmp = _config.get("--saveAudio2wav");
				if(tmp.equals("true")) m_SaveAudio2wav =true;
				else m_SaveAudio2wav =false;
			}

			// create local and remote TTS
			m_ttsLocal =
					new MaryTTSLocal(m_mary, "TEXT", m_voiceName, m_bSilentModeLocal, "WAVE");
			//     ttsRemote =
			//             new TTSRemote(m_mary, m_voiceName, m_bSilentModeRemote);

			if (!startingUp.equals("")) { 
				m_ttsLocal.speak(startingUp, new CountDownLatch(0));
			}

		}

	} // end configure
} // end TTSAgent-class
