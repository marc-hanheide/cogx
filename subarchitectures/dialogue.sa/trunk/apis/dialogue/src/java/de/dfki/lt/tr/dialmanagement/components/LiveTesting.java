// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@ifi.uio.no)                                                                
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


package de.dfki.lt.tr.dialmanagement.components;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Arrays;

import javax.swing.Timer;

import de.dfki.lt.tr.dialmanagement.components.DialogueManager;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.TNOPolicyReader;
import de.dfki.lt.tr.dialmanagement.utils.XMLPolicyReader;
import de.dfki.lt.tr.dialmanagement.data.ActionSelectionResult;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.AlternativeAction;
import de.dfki.lt.tr.dialmanagement.data.actions.IntentionAction;
import de.dfki.lt.tr.dialmanagement.data.actions.MotorAction;
import de.dfki.lt.tr.dialmanagement.data.actions.PhonstringAction;
import de.dfki.lt.tr.dialmanagement.data.conditions.AbstractCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.DialStateCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.TimeoutCondition;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyEdge;
import de.dfki.lt.tr.dialmanagement.gui.DialogueChatWindow;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;

/**
 * Utility for testing a given policy in a "live" environment, using a chat window
 * 
 * @author Pierre Lison
 * @version 21/12/2010
 *
 */
public class LiveTesting implements ActionListener {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	// the dialogue manager
	private DialogueManager manager;
	
	// the gui window
	private DialogueChatWindow gui;

	// the timer to trigger timeout events
	private Timer timer;

	/**
	 * Start up the live-testing environment, given a policy file
	 * 
	 * @param policyFile the path to the policy file
	 */
	public LiveTesting(String policyFile) {

		try {
			gui = new DialogueChatWindow(this);

			// first try the standard XML reader, then the TNO one
			try {
				manager = new DialogueManager(XMLPolicyReader.constructPolicy(policyFile), null);
			}
			catch (DialogueException e) {
				manager = new DialogueManager(TNOPolicyReader.constructPolicy(policyFile), null);
			}

			ActionSelectionResult r = manager.updateStateAndSelectAction(0);

			for (AbstractAction action : r.getActions()) {
				gui.addUtterance("Robot",action.toString());
			}

			initTimer();

		} catch (Exception e) {
			e.printStackTrace();
		}
	}



	/**
	 * Whenever the text field triggers an event (return key pressed), 
	 * and the resulting text is not empty, the provided text is turned
	 * into a PhonString object and passed on to whatever process is registered
	 * as listener to the class.
	 *  
	 * @param evt The action event triggered by the text field
	 */

	public void actionPerformed (ActionEvent evt)
	{
		String text= gui.getInputText();
		if (!text.equals(""))
		{
			stopTimer();
			gui.addUtterance("Human", text);
			gui.setInputText("");

			try {
				
				ActionSelectionResult r;
				
				// in case the user enters a probability value
				float probValue = 1.0f;
				if (text.contains("(") && text.contains(")") && text.charAt(text.length() -1) == ')') {
					probValue = getProbabilityValueInParenthesis(text);
					text = text.replace("("+probValue+")", "");
				}
				
				// if the entered text is a formula
				if (text.contains("^") || text.contains("<")) {
					r = manager.updateStateAndSelectAction(EpistemicObjectUtils.createSimpleCommunicativeIntention(text, probValue));
				}
				
				// else, consider it to be a phonstring
				else {
					PhonString phon = new PhonString();
					phon.wordSequence= text;
					phon.confidenceValue = probValue;
					r = manager.updateStateAndSelectAction(Arrays.asList(phon));
				}
				
				// if a result is retrieved, add it to the gui window
				if (!r.isVoid()) {
					for (AbstractAction action : r.getActions()) {
						if (!action.toString().equals("")) {
							showActionInWindow(action);
						}
					}	
				}
			} catch (DialogueException e) {
				e.printStackTrace();
			}

			initTimer();

		} // end if
	} // end actionPerformed


	
	/**
	 * If the probability value of a given input is provided in parenthesis,
	 * try to extract it
	 * 
	 * @param text the string where the probability value might be
	 * @return the probability value if a valid one is entered, else 1.0f
	 */
	private float getProbabilityValueInParenthesis (String text) {
		
		try {
		String[] split1 = text.split("\\(");
		String[] split2 = split1[1].split("\\)");
		return Float.parseFloat(split2[0]);
		}
		catch (Exception e) {
			return 1.0f;
		}
	}
	
	/**
	 * Initialise the timer
	 */
	private void initTimer() {

		for (PolicyEdge edge: manager.getAllOutgoingEdges()) {
			
			for (AbstractCondition condition : edge.getConditions()) {
				if (condition instanceof TimeoutCondition) {
					
					int timeout = ((TimeoutCondition)condition).getTimeout();
					
					if (areAllConditionsSatisfied(edge)) {
						createNewTimer(timeout);
					}

				}
			}
		}
	}
	
	
	/**
	 * If an edge has a timeout, we should also verify that the other
	 * conditions (mostly dialogue state conditions) are satisfied
	 * 
	 * @param edge the edge to very
	 * @return true if everything is satified, false otherwise
	 */
	private boolean areAllConditionsSatisfied (PolicyEdge edge) {
		if (edge.getConditions().size() > 1) {
			for (AbstractCondition condition : edge.getConditions()) {
				if (condition instanceof DialStateCondition) {
					if (!((DialStateCondition)condition).matchesPreconditions(manager.getDialogueState())) {
						return false;
					}
				}
			}
		}
		return true;
	}


	/**
	 * Create a new timer for the specific timeout
	 * 
	 * @param timeout
	 */
	private void createNewTimer (int timeout) {

		ActionListener taskPerformer = new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				reactToTimeout();
			}
		};
		log("creating a new timer with delay: " + timeout);
		timer = new Timer(timeout, taskPerformer);
		timer.setRepeats(false);
		timer.start();
	}


	/**
	 * Show the provided action in the GUI window
	 * 
	 * @param action
	 */
	private void showActionInWindow (AbstractAction action) {
		if (action instanceof PhonstringAction) {
			gui.addUtterance("Robot",action.toString());
		}
		else if (action instanceof IntentionAction && 
				((IntentionAction)action).getStatus() == IntentionAction.ATTRIBUTED) {
			gui.addUtterance("FORWARDED",action.toString());
		}
		else if (action instanceof IntentionAction && 
				((IntentionAction)action).getStatus() == IntentionAction.COMMUNICATIVE) {
			gui.addUtterance("Robot",action.toString());
		}
		else if (action instanceof AlternativeAction) {
			AbstractAction selectedAction = ((AlternativeAction)action).selectRandomAction();
			gui.addUtterance("Robot",selectedAction.toString());
		}
		else if (action instanceof MotorAction) {
			gui.addUtterance("ACTION",action.toString());
		}
	}
	
	
	/**
	 * Stop the timer
	 */
	private void stopTimer() {
		if (timer != null) {
			timer.stop();
		}
	}


	/**
	 * React to a timeout event
	 */
	private void reactToTimeout ()
	{
		stopTimer();

		log("Reacting to the timeout");

		ActionSelectionResult r = manager.updateStateAndSelectAction(timer.getDelay());
		if (!r.isVoid()) {
			for (AbstractAction action : r.getActions()) {
				showActionInWindow(action);
			}	
		}

		initTimer();
	}

	
	/**
	 * Start-up the environment, assuming a policy file has been provide as a 
	 * parameter in the command line (or as an Ant option)
	 * 
	 * @param args
	 */
	public static void main (String[] args) {

		String policyFile;
		if (args.length == 1 && !args[0].contains("$")) {
			policyFile = args[0];
			LiveTesting test = new LiveTesting(policyFile);
		}
		else {
			System.out.println("ERROR: no policy file has been provided.  Please provide a policy file in the command line.");
			System.out.println("Command line usage: ant live-testing -Dpolicyfile=/path/to/your/policy/file");
			System.exit(0);
		}

	}




	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[live-testing] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[live-testing] " + s);
		}
	}

}
