//=================================================================
//Copyright (C) 2006 Geert-Jan M. Kruijff (gj@dfki.de)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION
//=================================================================
 
package asr;

//=================================================================
//IMPORTS
//=================================================================

//-----------------------------------------------------------------
//CAST IMPORTS
//-----------------------------------------------------------------
import java.util.*;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.*;

import cast.core.CASTData;
import cast.core.CASTUtils;


import speechrec.autogen.PhonString;
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
 * @version 061020 (started 060924)
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
 */

public class ASR extends ManagedComponent {

	// =================================================================
	// GLOBAL VARIABLES
	// =================================================================

	// Constants for the mode -- i.e. the source for input
	final int ASR_SPEECHREC = 0;
	final int ASR_GUI = 1;
	final int ASRconfig = 2;
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

			while (isRunning()) {

				if (asr_mode == ASR_SPEECHREC) {
					executeASRTask();
				}

				// If we are using a dialogue window to retrieve the
				// string to be parsed
				else if (asr_mode == ASR_GUI) {
					executeGUITask();
				}
				else if (asr_mode == ASRconfig) {
					log("sleeping for "+ m_sleep + "ms before adding: \"" + m_utterance + "\"");

					sleepComponent(m_sleep);
					assert(m_utterance != null);
					int len = m_utterance.split(" ").length;
					PhonString result = createNewPhonString (m_utterance, 1, 1, 1);
					addToWorkingMemory(newDataID(),	result);
					return;
				}

			} // end while running
		}
		catch (Exception e) {
			e.printStackTrace();
		} // end try..catch
	} // end runComponent


  private PhonString createNewPhonString(String recogSequence, float confValue, float NLConfValue, int rank) {
    
    PhonString phon = new PhonString();
    phon.wordSequence = recogSequence;
    phon.id = newDataID();
    
    String[] splits = phon.wordSequence.split(" ");
    phon.length = splits.length;
    
    phon.confidenceValue = confValue;
    phon.NLconfidenceValue = NLConfValue;
    phon.rank = rank;
    
    return phon;
  }
  
  
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

		result = createNewPhonString (textField.getText(), 1, 1, 1);
    textField.setText("");
		return result;
	} // end getGUIPhonString


	protected String tweaks(String text) {
		return text.replaceAll("i ", "I ");
	}

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

		log("(Press Ctrl+C to stop the recognition client)\n");

		String args = "-package " + packageASR + 
		" lm.Addresses=localhost " +
		    //	" audio.Provider=sip" +
		" audio.Device=@any " +  
		" client.AllowBargeIn=TRUE " +  
		    //	" audio.sip.userAgentURI=sip:cosy@localhost " + 
		    //	" audio.sip.UserAgentPort=" + port +
		" client.NoSpeechTimeoutSecs=" + timeout + 
		" rec.GrammarWeight=" + grammarweight + 
		" rec.pass1.gp.WTW=" + WTW +
		" recPPR=" + recPPR + 
		" rec.ConfidenceRejectionThreshold=" + confidencethreshold + 
		" rec.Pruning=" + pruning;

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
				//	CoreTelephonyControl tc = nsc.getTelephonyControl();

				//		if (tc != null) {

					log("\nWaiting for a SIP phone call...");
					// Answer the incoming call
					//	tc.waitForCall().print();
					//	tc.answerCall();

					log("SIP call answered.\n");

					boolean connected = true;

					while(connected) {

					    log("\nRecording speech...");

						RecResult rec_result ;
						try {						    				    rec_result = nsc.playAndRecognize(".Top"); 
}
						catch (SessionEndedException e) {rec_result = null;}

						if (rec_result != null) {


							// Check the returned recognition result.
							if ((rec_result.getNumResults() > 0) &
									(rec_result.isNormalRecognition()))
							{
								String recString = rec_result.getSingleResult(0).getRecognizedString();

								log("Recognized utterance: " + recString);

								RecognitionResult result = new RecognitionResult(
										rec_result.getSingleResult(0).toString());
								PhonString phonString = new PhonString();
								phonString.wordSequence = result.getRecString();
								phonString.confidenceValue = result.getConfidence();

								log("Confidence score: " + phonString.confidenceValue);

								String[] splits = phonString.wordSequence.split(" ");
								phonString.length = splits.length;
								phonString.id = newDataID();
								addToWorkingMemory(phonString.id, phonString);
								log("PhonString added to the working memory\n");

							}
							else {
								// We had a NO_SPEECH_TIMEOUT or something similar
								log("Nothing intelligible was said");

								PhonString phonString = new PhonString();
								phonString.wordSequence = "unrecognized";
								phonString.length = 1;
								phonString.id = newDataID();
								addToWorkingMemory(phonString.id, phonString);
							}
							//	}
						//		else {
						//	    connected=false;
							    //	    tc.hangup();
							//	    nsc.close();
							//		}

							this.sleepComponent(100);
						
							//	if (tc==null) {
							//	    connected=false;
							//	    tc.hangup();
							//	    nsc.close();
							//		}
						}   		
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

 

	@Override
	public void configure(Map<String, String> config) {

		super.configure(config);

		// Source of the data: either speech recognition or a dialogue
		// window
		if (config.containsKey("--src")) {
			String src = config.get("--src");

			if (src.equals("gui")) {
				configureGUI(config);
			}

			else if (src.equals("nuance")) {
				configureNuance(config);
			}
			else {
				asr_mode = ASR_GUI;
			} // end if..else check for command-line argument
		}
	}

	public void configureGUI(Map<String, String> config) {
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
				ASR.OKBUTTON_PUSHED = true;
			}
		});

		asr_mode = ASR_GUI;
	}

	public void configureNuance(Map<String, String> config) {

		asr_mode = this.ASR_SPEECHREC; 

		if (config.containsKey("--timeout")) {
			timeout = config.get("--timeout");
		}
		else {
			timeout = "8";
		}

		if (config.containsKey("--timeout")) {
			timeout = config.get("--timeout");
		}
		else {
			timeout = "8";
		}

		if (config.containsKey("--port")) {
			port = config.get("--port");
		}
		else {
			port = "5050";
		}

		// merging sorts option
		if (config.containsKey("--package")) {
			packageASR = config.get("--package");
		}
		else {
			System.err.println("ERROR: you have to specify a package for the ASR");
			System.exit(0);
		}


		if (config.containsKey("--grammarweight")) {
			grammarweight = config.get("--grammarweight");
		}
		else {
			grammarweight = "8";
		}

		if (config.containsKey("--WTW")) {
			WTW = config.get("--WTW");
		}
		else {
			WTW = "-80";
		}

		if (config.containsKey("--pruning")) {
			pruning = config.get("--pruning");
		}
		else {
			pruning = "1800";
		}

		if (config.containsKey("--recPPR")) {
			recPPR = config.get("--recPPR");
		}
		else {
			recPPR = "TRUE";
		}

		if (config.containsKey("--confidencethreshold")) {
			confidencethreshold = config.get("--confidencethreshold");
		}
		else {
			confidencethreshold = "30";
		}
	}


	public static void log(String m) {
		if (logging) {
			System.out.println("[ASR] " + m);
		} // end if
	} // end log

} // end class
