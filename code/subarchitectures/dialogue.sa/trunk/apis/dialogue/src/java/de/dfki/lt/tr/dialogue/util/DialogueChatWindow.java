// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
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
package de.dfki.lt.tr.dialogue.util;


//=================================================================
// IMPORTS

// Java
import de.dfki.lt.tr.dialogue.slice.asr.InitialPhonString;
import de.dfki.lt.tr.dialogue.slice.asr.Noise;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.text.AttributeSet;
import javax.swing.text.SimpleAttributeSet;
import javax.swing.text.StyleConstants;
import javax.swing.text.StyleContext;

import java.util.StringTokenizer;
import java.util.Vector;

// Meta
import de.dfki.lt.tr.meta.TRResultListener;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;
import de.dfki.lt.tr.dialogue.slice.time.Interval;
import de.dfki.lt.tr.dialogue.slice.time.TimePoint;
import de.dfki.lt.tr.dialogue.time.Point;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import java.util.LinkedList;


/**
 * Implements a simple chat window. The user can type in text, for 
 * which a <tt>PhonString</tt> object is then constructed. This 
 * object is made available to processes registered as <tt>TRResultListener</tt> 
 * with the GUI. The chat window can also take input in the form 
 * of a <tt>SpokenOutputItem</tt> which then gets displayed in the 
 * chat window. Taken together, this provides a simple display of the 
 * dialogue as it is taking place between a user and the system. 
 * 
 * @author 	Geert-Jan M. Kruijff
 * @version 100622
 * @started 100622
 */

public class DialogueChatWindow 
extends JFrame
implements ActionListener {

	LinkedList<String> blocked = new LinkedList<String>();
	
	private int idCounter = 0;

	// Process that listen for / are to be notified of results of the engine
	private Vector<TRResultListener> listeners = new Vector<TRResultListener>();

	// Aspects of the GUI, global within class for accessibility
	JTextField inputField;
	final JTextArea utterances;
	
	/**
	 *  Constructor. Sets up the frame with the area for displaying utterances, 
	 *  and the text field for typing in an utterance. 
	 */
	
	public DialogueChatWindow () 
	{
		super("Dialogue Window");
		// Create the area where the utterances appear
		utterances = new JTextArea (10,50);
		utterances.setMargin(new Insets(5,5,5,5));
		utterances.setEditable(false);
		JScrollPane utterancesScrollPane = new JScrollPane(utterances);
		
		inputField = new JTextField(50);
		inputField.addActionListener(this);
		
		// Add the text field and the utterances to the frame
		Container contentPane = getContentPane();
		contentPane.add (inputField, BorderLayout.SOUTH);
		contentPane.add (utterancesScrollPane, BorderLayout.CENTER);
		
	} // end constructor
	
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
		String text= inputField.getText();
		if (!text.equals(""))
		{
			utterances.append("Human: " + text + "\n");
			StringTokenizer tokenizer = new StringTokenizer(text);

			long now = System.currentTimeMillis();
			long past = now - 50;
			TimeInterval ival = new TimeInterval(new Point(past), new Point(now));
			PhonString ps = new PhonString(newId(), text, tokenizer.countTokens(), 1.0f, 0.0f, 1, false, ival.toIce());
			InitialPhonString ips = new InitialPhonString(ps);

			blocked.add(ps.id);
			notify(ips);
			inputField.setText("");
		} // end if
	} // end actionPerformed
	
	/**
	 * Adds a spoken output as string to the dialogue window, assuming
	 * the provided object is not null. Else nothing else is added. The
	 * string is added as "Robot: "+string.   
	 * 
	 * @param soi	The spoken output to be added (as string)
	 */
	
	public void publishSOI (SpokenOutputItem soi)
	{
		if (soi != null)
		{
			utterances.append("Robot: " + soi.phonString + "\n");
		} // end if
	}

	public void publishPhonString (PhonString ps)
	{
		if (ps != null) {
			boolean found = false;
			for (String i : blocked) {
				if (i.equals(ps.id)) {
					found = true;
				}
			}
			if (!found) {
				utterances.append("Human: "+ps.wordSequence + "\n");
			}
		}
	}

	public void publishNoise (Noise n)
	{
		if (n != null) {
			utterances.append("Human: [...]\n");
		}
	}

	/**
	 * Registers a process with the engine, to be notified whenever
	 * the engine produces a new result
	 * @param listener		The process to be informed
	 * @throws UnsupportedOperationException Thrown if the process is null
	 */
	
	public void registerNotification (TRResultListener listener)
	throws UnsupportedOperationException
	{
		if (listener == null) 
		{
			throw new UnsupportedOperationException("Cannot register process to be notified: "
					+"Provided process is null");	
		} else {
			this.listeners.add(listener); 
		}		
	} // end registerNotification
	
	/**
	 * Notify all the registered listeners of the obtained result
	 * @param result
	 */
	
	private void notify (Object result)
	{
		// Notify all the registered listeners
		for (TRResultListener listener : listeners)
		{
			listener.notify(result);
		} // end for 
	} // end notify
	
	
	/**
	 * Creates a new String identifier of the form "asr"+integer
	 * @return String the new identifier
	 */
	private String newId() 
	{ 
		idCounter++;
		return "asr"+idCounter;
	}
	
	
	public static void main (String[] args)
	{
		JFrame frame = new DialogueChatWindow();
		frame.addWindowListener(new WindowAdapter() 
		{
			public void windowClosing(WindowEvent e) 
			{
				System.exit(0);
			}
		}
		); // end addWindowListener
		frame.pack();
		frame.setVisible(true);
	} // end main
	
} // end class
