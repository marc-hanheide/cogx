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
import beliefmodels.domainmodel.cogx.*;

//-----------------------------------------------------------------
// CAST IMPORTS
//-----------------------------------------------------------------

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

//-----------------------------------------------------------------
// COMSYS IMPORTS
//-----------------------------------------------------------------

import comsys.arch.ProcessingData;
import comsys.processing.cca.BeliefUtils;
import comsys.processing.reference.belieffactories.AbstractBeliefFactory;
import binder.abstr.BeliefModelInterface;
import binder.components.Binder;

//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------

import java.util.*;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;


public class cc_FakeVerificationGenerator 
	extends BeliefModelInterface 
	
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
	JTextField entityField;
	JTextField modalityField;
	JTextField probField;
	JDialog dialog;
	String defaultModality = "";
	float defaultProb = 1.0f;
	
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
		super.start();
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
		GroundedBelief request = getGUIRequest();
		try {
			if (request != null) {
				log("Storing grounded belief on binder WM");
				overwriteWorkingMemory(request.id, Binder.BINDER_SA, request);
			}
			else {
				log("null request, ignoring");
			}
		} catch (SubarchitectureComponentException e) {
				e.printStackTrace();
		} // end try .. catch
	} // end method	
	
	public GroundedBelief getGUIRequest() {
		
		entityField.requestFocusInWindow();
		
		while (!OKBUTTON_PUSHED) {
			this.sleepComponent(100);
		}
		
		OKBUTTON_PUSHED = false;
		
		Belief b = null;
		String beliefId = entityField.getText();
		log("retrieving belief id '" + beliefId + "'");
		try {
			b = getBelief(beliefId);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		if (b == null) {
			log("no such belief found!");
			return null;
		}
		
		float prob;
		String probStr = probField.getText();
		try {
			prob = Float.parseFloat(probStr);
		}
		catch (NumberFormatException e) {
			prob = 1.0f;
			log("not a valid float: \"" + probStr + "\", using " + Float.toString(prob));
		}

		Ground g = new Ground();
		g.gstatus = GroundStatus.assertionVerified;
		g.modality = modalityField.getText();
		g.indexSet = new String[] { };
		g.reason = new SuperFormula(); // truth -> empty formula

		GroundedBelief verif = new GroundedBelief();
		verif.ags = AbstractBeliefFactory.createMutualAgentStatus(new String[] {"robot", "human"}); // XXX
		verif.phi = BeliefUtils.changeAssertionsToPropositions((SuperFormula) b.phi, prob);
		verif.sigma = b.sigma;
		verif.timeStamp = getCASTTime();
		verif.id = b.id;
		verif.grounding = g;

		//use entityField.getText();
		
		// return the result
		entityField.setText("");
		return verif;
	} // end getGUIRequest	
		
	public void configureGUI() {
		try {
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		} catch(Exception e) {
			e.printStackTrace();
		}
		Frame frame = null;
				
        JPanel dialogPanel = new JPanel();
        dialogPanel.setLayout(new BoxLayout(dialogPanel, BoxLayout.Y_AXIS));

        JPanel modalityPanel = new JPanel();
        modalityPanel.setLayout(new BoxLayout(modalityPanel, BoxLayout.LINE_AXIS));
        JLabel modalityLabel = new JLabel("Modality:");
        modalityPanel.add(modalityLabel);
        modalityField = new JTextField();
        modalityField.setText(defaultModality);
		modalityPanel.add(modalityField);
		dialogPanel.add(modalityPanel);

        JPanel entityPanel = new JPanel();
        entityPanel.setLayout(new BoxLayout(entityPanel, BoxLayout.LINE_AXIS));
        JLabel entityLabel = new JLabel("Belief ID:");
        entityPanel.add(entityLabel);
        entityField = new JTextField();
		entityPanel.add(entityField);
		dialogPanel.add(entityPanel);
		
        JPanel probPanel = new JPanel();
        probPanel.setLayout(new BoxLayout(probPanel, BoxLayout.LINE_AXIS));
        JLabel probLabel = new JLabel("Probability:");
        probPanel.add(probLabel);
        probField = new JTextField();
        probField.setText(Float.toString(defaultProb));
		probPanel.add(probField);
		dialogPanel.add(probPanel);

		dialogPanel.add(Box.createVerticalGlue());
		
		JPanel buttonPanel = new JPanel();
		buttonPanel.setLayout(new BoxLayout(buttonPanel, BoxLayout.LINE_AXIS));
		buttonPanel.add(Box.createHorizontalGlue());
		ok = new JButton("Verify all assertions");
		buttonPanel.add(ok);
		dialogPanel.add(buttonPanel);

		dialog = new JDialog(frame, "Assertion verification");
		dialog.add(dialogPanel);

		dialog.setLocation(600, 220);

		Dimension dim = dialogPanel.getPreferredSize();
		dialog.setSize(200, dim.height + 25);
		dialog.setVisible(true);
		entityField.requestFocusInWindow();
		
		KeyboardFocusManager.getCurrentKeyboardFocusManager()
		.addKeyEventDispatcher(new KeyEventDispatcher(){
							   public boolean dispatchKeyEvent(KeyEvent ke){
							   if(ke.getID() == KeyEvent.KEY_PRESSED)
							   {
							   if(((KeyEvent) ke).getKeyCode() == KeyEvent.VK_ENTER)
							   {
							   if (dialog.isActive()) ok.doClick();
							   }
							   }
							   return false;
							   }
							   });
		
		ok.addActionListener(new ActionListener(){
							 public void actionPerformed(ActionEvent e){
							 cc_FakeVerificationGenerator.OKBUTTON_PUSHED = true;
							 }
							 });
		
	} 
	
	@Override
    public void configure(Map<String, String> _config) {
		if (_config.containsKey("--modality")) {
			defaultModality = _config.get("--modality");
		}
	}

} // end class
