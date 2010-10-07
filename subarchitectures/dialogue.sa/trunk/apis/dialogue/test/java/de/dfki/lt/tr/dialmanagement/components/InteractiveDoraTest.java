
// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
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

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.UIManager;


import org.junit.Before;
import org.junit.Test;


import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyAction;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyReader;

/**
 * First attempt at an end-to-end interactive test for a Dora-like interaction
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 04/07/2010
 */
public class InteractiveDoraTest {

	// GUI objects
	JButton ok;
	JTextField textField;
	boolean OKBUTTON_PUSHED = false;

	public static boolean LOGGING = true;

	public static boolean DEBUG = false;


	// configuration files
	public static String POLICYFILE = "subarchitectures/dialogue.sa/config/policies/policyExampleDora.txt";
	public static String OBSFILE = "subarchitectures/dialogue.sa/config/policies/observationsDora.txt";
	public static String ACTIONSFILE = "subarchitectures/dialogue.sa/config/policies/actionsDora.txt";

	// the dialogue manager
	public DialogueManager manager;


	@Test
	public void dummy() { };


	
	private static Observation createSimpleObservation (String s) throws DialogueException {
		return createSimpleObservation(s, 1.0f);
	}
	

	private static Observation createSimpleObservation (String s, float f) throws DialogueException {
		Observation intent = new Observation (Observation.INTENTION);
		intent.addAlternative(s, f);
		intent.addAlternative(new UnknownFormula(0), 1-f);
		return intent;
	}
	
	/**
	 * Construct a new dialogue policy based on the configuration files
	 * 
	 * @throws DialogueException
	 */
	public void constructPolicy() throws DialogueException {

		DialoguePolicy policy = PolicyReader.constructPolicy(POLICYFILE, OBSFILE, ACTIONSFILE);

		policy.ensureWellFormedPolicy();

		manager = new DialogueManager(policy);
	}

	
	
	/**
	 * Configure a GUI window for typing the phonological string
	 */
	public void configureGUI() {
		try {
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		}
		catch(Exception e) {
			e.printStackTrace();
		}

		Frame frame = null;

		ok = new JButton("OK");
		JLabel l = new JLabel("Please type in the phonological string: ");
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

	}


	/**                                                                                                                          
	 * The method <i>getGUIPhonString</i> opens a simple GUI and asks                                                            
	 * the user for a string. This string is then put into working                                                               
	 * memory as the one to be analyzed.                                                                                         
	 *                                                                                                                           
	 * @return PhonString An object with the typed-in structure                                                                  
	 * @throws InterruptedException 
	 */

	public String getGUIPhonString() throws InterruptedException {

		textField.requestFocusInWindow();

		while (!OKBUTTON_PUSHED) {
			Thread.sleep(100);
		}

		OKBUTTON_PUSHED = false;

		textField.setText("");
		return textField.getText();

	} // end getGUIPhonString                                                                                                    



	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialmanager test] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialmanager test] " + s);
		}
	}




	/**
	 * Starts up a GUI window to type speech input, and navigate through the policy with
	 * the resulting interaction
	 * @param args
	 * @throws DialogueException
	 * @throws InterruptedException
	 */
	public static void main(String[] args) throws DialogueException, InterruptedException {

		// construct the test
		InteractiveDoraTest test = new InteractiveDoraTest();
		test.constructPolicy();

		// configure the GUI
		test.configureGUI();

		// navigate through the policy
		while (!test.manager.isFinished()) {
			String text = test.getGUIPhonString();
			Observation intent = createSimpleObservation(text);
			PolicyAction action = test.manager.nextAction(intent);
			System.out.println("REACTION: " + action.toString());
		}

	}
}
