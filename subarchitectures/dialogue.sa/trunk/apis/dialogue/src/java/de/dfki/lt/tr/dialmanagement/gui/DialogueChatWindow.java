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


package de.dfki.lt.tr.dialmanagement.gui;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Insets;

import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;

import de.dfki.lt.tr.dialmanagement.components.LiveTesting;


/**
 * Dialogue chat window for the dialogue manager
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class DialogueChatWindow extends JFrame
{

	private static final long serialVersionUID = 1L;
	
	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	// main chat window
	final JTextArea utterances;

	// input field
	JTextField inputField;

	/**
	 * Start up the window
	 * 
	 * @param tester reference to the live-testing environment
	 */
	public DialogueChatWindow (LiveTesting tester) 
	{
		super("Dialogue Window");

		// Create the area where the utterances appear
		utterances = new JTextArea (10,50);
		utterances.setMargin(new Insets(5,5,5,5));
		utterances.setEditable(false);
		JScrollPane utterancesScrollPane = new JScrollPane(utterances);

		inputField = new JTextField(50);
		inputField.addActionListener(tester);

		// Add the text field and the utterances to the frame
		Container contentPane = getContentPane();
		contentPane.add (inputField, BorderLayout.SOUTH);
		contentPane.add (utterancesScrollPane, BorderLayout.CENTER);

		addWindowListener(new WindowAdapter() 
		{
			public void windowClosing(WindowEvent e) 
			{
				System.exit(0);
			}
		}
		); // end addWindowListener
		pack();
		setVisible(true);
	} // end constructor


	/**
	 * Returns the text in the input field
	 * @return
	 */
	public String getInputText() {
		return inputField.getText();
	}
	
	
	/**
	 * Sets the text in the input field to a particular value
	 * @param text
	 */
	public void setInputText(String text) {
		inputField.setText(text);
	}



	/**
	 * Add an utterance to the chat window, together with
	 * the agent name
	 * 
	 * @param agent the agent name
	 * @param utt the utterance
	 */
	public void addUtterance(String agent, String utt) {
		utterances.append(agent + ": " + utt+"\n");
	}


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[chatwindow] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[chatwindow] " + s);
		}
	}

} 
