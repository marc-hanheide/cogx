//=================================================================
//Copyright (C) 2006-2010 DFKI GmbH
// Geert-Jan M. Kruijff (gj@dfki.de)
// Pierre Lison (pierre.lison@dfki.de)
//
//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.
//
//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//Lesser General Public License for more details.
//
//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION
package de.dfki.lt.tr.cast.dialogue;

//=================================================================
//IMPORTS

// Java
import java.util.*;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.asr.RecogResult;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;

// CAST-extensions to Dialogue API
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.cast.dialogue.DialogueGoals;

// Dialogue API 
import de.dfki.lt.tr.dialogue.asr.NuanceRecognitionResult;
import de.dfki.lt.tr.dialogue.util.DialogueMissingValueException;

// CAST 
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.SubarchitectureComponentException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;


import vcommerce.core.util.*;
import vcommerce.core.sc.*;

import nuance.core.sc.*;
import nuance.core.util.*;



//=================================================================
//CLASS DOCUMENTATION
//=================================================================

/**
 * The class <b>ASR</b> implements a goal-driven process for providing
 * the comsys with a string to parse. 2 options are available:
 * <ol>
 * <li> the process can either wait for a RecogResult to arrive in the
 * working memory (inserted by the SIP process, responsible for handling
 * the 2 SIP-based remote connections, one from the caller to the local
 * machine, and one from the local machine to the Nuance recognition
 * server),
 * <li> or start up a a dialogue window, in which the user can type a
 * string to be parsed.
 * </ol>
 * Upon getting a string, the ASR component creates a PhonString object
 * for working memory, including the recognized string.
 * <p>
 * Whether nuance is used, or a gui is provided, can be set through the
 * command-line argument "--src". [--src gui] specifies the GUI as the
 * source for getting a phonological string, [--src nuance] sets the
 * source to the nuance speech recognizer.
 * 
 * @version 090519 (started 060924; ported to CAST v0.2 on 090519)
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
 */
 
public class NuanceASR extends ManagedComponent {

	// =================================================================
	// GLOBAL VARIABLES
	// =================================================================

	// Constants for the mode -- i.e. the source for input
	final int ASR_SPEECHREC = 0;
	final int ASR_GUI = 1;
	final int ASR_CONFIG = 2;
	// The mode for speech recognition: GUI or using ASR
	int asr_mode;

	String timeout;
	String packageASR;
	String grammarweight;
	String WTW;
	String recPPR;
	String confidencethreshold;
	String port;
	String pruning;

	// Whether the result of the speech recognition are to be
	// synthesized
	// and retransmitted to the user
	boolean playback = false;

	static boolean logging = true;

	// The phonological string object
	PhonString phonString;

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
	protected Hashtable<String, String> m_dataToProcessingGoalMap;

	// Hashtable linking task IDs to task types
	protected Hashtable<String, String> m_taskToTaskTypeMap;

	// Vector with objects to be processed,
	// can be ComSys:PhonString,...
	protected Vector<ProcessingData> m_dataObjects;

	// Identifiers for ProcessData objects
	private int pdIdCounter;


	// text field
	JTextField textField;
	JButton ok;

	//utterance specified as rec result in config file
	private String m_utterance;
	//sleep before utterace is added
	private long m_sleep = 10000;

	// =================================================================
	// CONSTRUCTOR METHODS
	// =================================================================

	/**
	 * @param _id
	 */
/**	public cc_ASR (String _id) {
		init();
	} // end constructor
*/ 
	
	private void init() {
		// set the ontology for this method

//		asr_mode = ASR_SPEECHREC;
		m_bLogOutput = true;
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

	} // end init

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {
		init();

		//try {
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(RecogResult.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							newRecogResultReceived(_wmc);
						}
					});
		//}
		//catch (SubarchitectureComponentException e) {
		//	e.printStackTrace();
		//}
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

			log("Run ASR...");

			lockComponent();
			sleepComponent(100);

			
			// plison - give enough time for other
			// processes to start before sending
			// anything

			unlockComponent(); 

			while (this.isRunning()) {

				if (asr_mode == ASR_SPEECHREC) {
					executeASRTask();
				}

				// If we are using a dialogue window to retrieve the
				// string to be parsed
				else if (asr_mode == ASR_GUI) {
					executeGUITask();
				}
				else if (asr_mode == ASR_CONFIG) {
					log("sleeping for "+ m_sleep + "ms before adding: \"" + m_utterance + "\"");

					sleepComponent(m_sleep);
					assert(m_utterance != null);
					int len = m_utterance.split(" ").length;
					PhonString result = new PhonString(newDataID(), m_utterance, len, 1.0f, 1.0f, 1, false, null);
					addToWorkingMemory(newDataID(),	result);
					return;
				}

			} // end while running
		}
		catch (Exception e) {
			e.printStackTrace();
		} // end try..catch
	} // end runComponent

	/** Returns a new identifier for a ProcessingData object */

	private String newProcessingDataId() {
		String result = "pd" + pdIdCounter;
		pdIdCounter++;
		return result;
	} // end newProcessingDataId

	static boolean OKBUTTON_PUSHED = false ;
	// =================================================================
	// CORE CLASS METHODS
	// =================================================================

	/**
	 * The method <i>getGUIPhonString</i> opens a simple GUI and asks
	 * the user for a string. This string is then put into working
	 * memory as the one to be analyzed.
	 * 
	 * @return PhonString An object with the typed-in structure
	 */

	public PhonString getGUIPhonString() {

		textField.requestFocusInWindow();

		while (!OKBUTTON_PUSHED) {
			this.sleepComponent(100);
		}

		OKBUTTON_PUSHED = false;

		PhonString result = null;

		result = new PhonString(newDataID(),textField.getText(), 0, 1.0f, 1.0f, 1, false, null);
		textField.setText("");
		return result;
	} // end getGUIPhonString


	protected String tweaks(String text) {
		return text.replaceAll("i ", "I ");
	}

	/**
	 * If a RecogResult has recently been added to the working memory,
	 * retrieve it and propose a ASR_TASK task to the task manager to
	 * handle the given result.
	 */

	protected void newRecogResultReceived(WorkingMemoryChange _wmc) {
		// if we have a logical form collection string,
		try {
			// get the id of the working memory entry
			String id = _wmc.address.id;
			// get the data from working memory and store it
			// with its id
			CASTData< RecogResult> recResultWM = getMemoryEntryWithData (id, RecogResult.class);
			// Propose a new task
			String taskID = newTaskID();
			// store the data we want to process for later
			ProcessingData pd = new ProcessingData(
					newProcessingDataId());
			pd.add(recResultWM);
			m_proposedProcessing.put(taskID, pd);
			// set up the goal
			String taskGoal = "ASR_TASK";
			
			// then ask for permission
			proposeInformationProcessingTask(taskID, taskGoal);
			// store the id with the task type
			m_taskToTaskTypeMap.put(taskID, taskGoal);

		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} // end try..catch

	} // end workingMemoryChanged

	/**
	 * If the ASR_TASK has been accepted by the task manager, and the
	 * data source is set to Nuance speech recognition, then proceed to
	 * the processing of the speech recognition result. A PhonString
	 * object is built and inserted into the working memory.
	 * <p>
	 * If the playback option is set, the string captured by the Nuance
	 * speech recognition is moreover repeated to the user.
	 */

	protected void executeASRTask() {

		System.out.println("(Press Ctrl+C to stop the recognition client)\n");

		String args = "-package " + packageASR + 
		" lm.Addresses=localhost " +
		" audio.Provider=sip" +
		" audio.Device=@any " +  
		" client.AllowBargeIn=TRUE " +  
		" audio.sip.userAgentURI=sip:cosy@localhost " + 
		" audio.sip.UserAgentPort=" + port +
		" client.NoSpeechTimeoutSecs=" + timeout + 
		" rec.GrammarWeight=" + grammarweight + 
		" rec.pass1.gp.WTW=" + WTW +
		" recPPR=" + recPPR + 
		" rec.ConfidenceRejectionThreshold=" + confidencethreshold + 
		" rec.Pruning=" + 1800;

		log("Arguments used for Nuance: " + args);

		while (true) {
			try {
				// Create a NuanceConfig object from the command-line arguments
				NuanceConfig config = new NuanceConfig();
				String[] args2 = args.split(" ");
				String[] extra_args = config.buildFromCommandLine(args2);

				// Create the SpeechChannel object
				NuanceSpeechChannel nsc = new NuanceSpeechChannel(config);

				log("Initialization of speech channel successful");

				// Wait for a call if this is a telephony audio device
				CoreTelephonyControl tc = nsc.getTelephonyControl();

				if (tc != null) {

					System.out.println("\nWaiting for a phone call...");

					// Answer the incoming call
					tc.waitForCall().print();
					tc.answerCall();

					System.out.println("Call answered.\n");

					while(true) {

						RecResult rec_result ;
						try {
							rec_result = nsc.playAndRecognize(".Top");
						}
						catch (SessionEndedException e) {rec_result = null;}

						if (rec_result != null) {
							// Check the returned recognition result.
							if ((rec_result.getNumResults() > 0) &
									(rec_result.isNormalRecognition()))
							{
								String recString = rec_result.getSingleResult(0).getRecognizedString();

								log("Recognized string: " + recString);

								NuanceRecognitionResult result = new NuanceRecognitionResult(
										rec_result.getSingleResult(0).toString());
								PhonString phonString = new PhonString();
								phonString.wordSequence = result.getRecString();
								phonString.confidenceValue = result.getConfidence();
								String[] splits = phonString.wordSequence.split(" ");
								phonString.length = splits.length;
								phonString.id = newDataID();
								addToWorkingMemory(phonString.id, phonString);
								log("PhonString added to the working memory");

							}
							else {
								// We had a NO_SPEECH_TIMEOUT or something similar
								System.out.println("Nothing intelligible was said");

								PhonString phonString = new PhonString();
								phonString.wordSequence = "unrecognized";
								phonString.length = 1;
								phonString.id = newDataID();
								addToWorkingMemory(phonString.id, phonString);
							}
						}   		
					} 

				//	tc.hangup();
				//	nsc.close();
				}
			}
			catch (Exception e) {
				e.printStackTrace();
			}
		}
	} // end executeASRTask


	/**
	 * If the ASR_TASK has been accepted by the task manager, and the
	 * data source is set to a GUI window, then start up a dialogue
	 * window and wait for the user to type his string. A PhonString
	 * object is then built and inserted into the working memory.
	 * <p>
	 * If the playback option is set, the string captured by the Nuance
	 * speech recognition is moreover repeated to the user.
	 */

	protected void executeGUITask() {
		PhonString phonString = getGUIPhonString();

		if (playback) {
			speak("you said ");
			speak(phonString.wordSequence);
		}

		// Set the length
		if (phonString != null && phonString.wordSequence != null) {
			StringTokenizer st = new StringTokenizer(
					phonString.wordSequence);
			log("PhonString length: [" + st.countTokens() + "]");
			phonString.length = st.countTokens();

			// set up a phonological string object for the structural
			// dialogue interpretation

			try {
				if (phonString != null) {
					// lock from external access
					// and then make it available in the comsys working
					// memory
					// addToWorkingMemory(newDataID(), ComsysOntology.PHONSTRING_TYPE, phonString);
					addToWorkingMemory(newDataID(), phonString); // refactored, data type determined from provided object
					// let other stuff happen if necessary
				} // end if.. check for non-null objects
			}
			catch (SubarchitectureComponentException e) {
				e.printStackTrace();
			}
		}
	}

	/*
	 * Given a input string, create a new SpokenOutputItem, and insert
	 * it into the working memory, so as to synthesize it by the TTS
	 * process.
	 */
	protected void speak(String str) {
		try {
			SpokenOutputItem spoi = new SpokenOutputItem();
			spoi.phonString = str;
			addToWorkingMemory(newDataID(), spoi); // refactored
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void configure(Map<String, String> _config) {
	//	_config.list(System.out);
	
		// Source of the data: either speech recognition or a dialogue
		// window
		if (_config.containsKey("--src")) {
			String src = _config.get("--src");

			if (src.equals("gui")) {
				configureGUI(_config);
			}

			else if (src.equals("nuance")) {
				configureNuance(_config);
			}
			else {
				asr_mode = ASR_GUI;
			} // end if..else check for command-line argument
		}
	}

	public void configureGUI(Map<String, String> _config) {
		//nah: try resetting to default incase matlab has
		//messed with it
		try {
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		}
		catch(Exception e) {
			e.printStackTrace();
		}

		Frame frame = null;


		ok = new JButton("OK");
		JLabel l = new JLabel("Please type in the ASR phonological string: ");
		textField = new JTextField();
		textField.setPreferredSize(new Dimension(100,20));

		JPanel panel = new JPanel();
		panel.add(ok);
		JDialog dialog = new JDialog(frame, "ASR Speech Input");
		dialog.setLocation(600, 600);
		dialog.getContentPane().add(l, BorderLayout.NORTH);
		dialog.getContentPane().add(textField, BorderLayout.CENTER);
		dialog.getContentPane().add(panel, BorderLayout.SOUTH);
		dialog.setSize(300,100);
		dialog.setVisible(true);
		textField.requestFocusInWindow();

		KeyboardFocusManager.getCurrentKeyboardFocusManager()
		.addKeyEventDispatcher(new KeyEventDispatcher(){
			public boolean dispatchKeyEvent(KeyEvent ke){
				if(ke.getID() == KeyEvent.KEY_PRESSED)
				{
					if(((KeyEvent) ke).getKeyCode() == KeyEvent.VK_ENTER)
					{
						ok.doClick();
					}
				}
				return false;
			}
		});

		ok.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent e){
				OKBUTTON_PUSHED = true;
			}
		});

		asr_mode = ASR_GUI;
	}

	public void configureNuance(Map<String, String> _config) {

		asr_mode = this.ASR_SPEECHREC; 

		if (_config.containsKey("--timeout")) {
			timeout = _config.get("--timeout");
		}
		else {
			timeout = "8";
		}

		if (_config.containsKey("--port")) {
			port = _config.get("--port");
		}
		else {
			port = "5050";
		}

		// merging sorts option
		if (_config.containsKey("--package")) {
			packageASR = _config.get("--package");
		}
		else {
			System.err.println("ERROR: you have to specify a package for the ASR");
			System.exit(0);
		}


		if (_config.containsKey("--grammarweight")) {
			grammarweight = _config.get("--grammarweight");
		}
		else {
			grammarweight = "8";
		}

		if (_config.containsKey("--WTW")) {
			WTW = _config.get("--WTW");
		}
		else {
			WTW = "-150";
		}

		if (_config.containsKey("--pruning")) {
			pruning = _config.get("--pruning");
		}
		else {
			pruning = "1800";
		}

		if (_config.containsKey("--recPPR")) {
			recPPR = _config.get("recPPR");
		}
		else {
			recPPR = "TRUE";
		}

		if (_config.containsKey("--confidencethreshold")) {
			confidencethreshold = _config.get("confidencethreshold");
		}
		else {
			confidencethreshold = "30";
		}
	}


	/** public static void log(String m) {
		if (logging) {
			System.out.println("[ASR] " + m);
		} // end if
	} // end log
	 */

} // end class
