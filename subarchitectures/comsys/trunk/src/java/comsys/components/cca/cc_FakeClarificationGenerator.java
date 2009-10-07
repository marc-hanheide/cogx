//=================================================================
//Copyright (C) 2009 Geert-Jan M. Kruijff (gj@dfki.de)

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

package comsys.components.cca; 


//=================================================================
//IMPORTS
//=================================================================

//-----------------------------------------------------------------
// BELIEFMODELS IMPORTS
//-----------------------------------------------------------------

import beliefmodels.adl.*;
import beliefmodels.clarification.ClarificationRequest; 

//-----------------------------------------------------------------
// CAST IMPORTS
//-----------------------------------------------------------------

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.SubarchitectureComponentException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;


public class cc_FakeClarificationGenerator 
	extends ManagedComponent 
	
{

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

	static boolean OKBUTTON_PUSHED = false ;
	JTextField aboutField;	
	JTextField needField;		
	JTextField modalityField;
	JTextField sourceField;	
	JTextField formulaField;	
	
	JButton ok;
	
	private void init() {
		// general information processing structures
		m_proposedProcessing = new Hashtable<String, ProcessingData>();
		m_dataToProcessingGoalMap = new Hashtable<String, String>();
		m_taskToTaskTypeMap = new Hashtable<String, String>();
		m_dataObjects = new Vector<ProcessingData>();
		pdIdCounter = 0;
		configureGUI();
	} // end init
	
	@Override
	public void start() {
		init();
	}	
	
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
	
	
	@Override
	public void runComponent() {
		try {
			log("Run FakeClarificationGenerator...");
			lockComponent();
			sleepComponent(100);
			unlockComponent(); 
			while (this.isRunning()) {
				executeGUITask();
			} // end while running
		} catch (Exception e) {
			e.printStackTrace();
		} // end try..catch
	} // end runComponent	
	
	private String newProcessingDataId() {
		String result = "pd" + pdIdCounter;
		pdIdCounter++;
		return result;
	} // end newProcessingDataId	
	
	protected void executeGUITask() {
		ClarificationRequest request = getGUIRequest();
		try {
			if (request != null) {
				log("Storing clarification request on working memory");
				addToWorkingMemory(newDataID(), request);
			}
		} catch (SubarchitectureComponentException e) {
				e.printStackTrace();
		} // end try .. catch
	} // end method	
	
	public ClarificationRequest getGUIRequest() {
		
		textField.requestFocusInWindow();
		
		while (!OKBUTTON_PUSHED) {
			this.sleepComponent(100);
		}
		
		OKBUTTON_PUSHED = false;
		
		// construct the clarification request
		
		ClarificationRequest result = new ClarificationRequest();
		result.id = newDataID();
		result.about = constructFormula(aboutField.getText());

		result.sourceModality = modalityField.getText();
		result.sourceEntityID = sourceField.getText();
		
		result.clarificationNeed = constructFormula(needField.getText());
		
		// return the result
		textField.setText("");
		return result;
	} // end getGUIPhonString	
	
	
	public void configureGUI() {
		try {
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		} catch(Exception e) {
			e.printStackTrace();
		}
		Frame frame = null;
		ok = new JButton("OK");
		JLabel l = new JLabel("Please provide info for the clarification request");
		aboutField = new JTextField();
		aboutField.setPreferredSize(new Dimension(100,20));
		needField = new JTextField();
		needField.setPreferredSize(new Dimension(100,20));
		modalityField = new JTextField();
		modalityField.setPreferredSize(new Dimension(100,20));
		sourceField = new JTextField();
		sourceField.setPreferredSize(new Dimension(100,20));		
		
		JPanel panel = new JPanel();
		panel.add(ok);
		JDialog dialog = new JDialog(frame, "Clarification Request");
		dialog.setLocation(600, 600);
		dialog.getContentPane().add(l, BorderLayout.NORTH);
		dialog.getContentPane().add(aboutField, BorderLayout.CENTER);
		dialog.getContentPane().add(needField, BorderLayout.CENTER);		
		dialog.getContentPane().add(modalityField, BorderLayout.CENTER);		
		dialog.getContentPane().add(sourceField, BorderLayout.CENTER);				
		dialog.getContentPane().add(panel, BorderLayout.SOUTH);
		dialog.setSize(500,100);
		dialog.setVisible(true);
		aboutField.requestFocusInWindow();
		
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
							 cc_FakeClarificationGenerator.OKBUTTON_PUSHED = true;
							 }
							 });
		
	} 
	
	
	private ComplexFormula constructFormula (String phi) { 
	
	
	
	
	} // end 
	
	
} // end class
