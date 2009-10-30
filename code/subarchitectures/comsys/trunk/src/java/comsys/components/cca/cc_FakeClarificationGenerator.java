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
import binder.autogen.core.UnionConfiguration;

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
// COMSYS IMPORTS
//-----------------------------------------------------------------

import comsys.arch.ProcessingData;

//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------

import java.util.*;

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
//	JTextField sourceField;
	JComboBox unionBox;
	JTextField formulaField;	
	JDialog dialog;
	
	Vector<String> currentUnions = new Vector<String>();
	
	private String defaultModality = "";
	private String defaultSourceId = "";
	
	JButton ok;

	public cc_FakeClarificationGenerator() {
		init();
	}
	
	
	private void init() {
		// general information processing structures
		m_proposedProcessing = new Hashtable<String, ProcessingData>();
		m_dataToProcessingGoalMap = new Hashtable<String, String>();
		m_taskToTaskTypeMap = new Hashtable<String, String>();
		m_dataObjects = new Vector<ProcessingData>();
		pdIdCounter = 0;
        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
		configureGUI();
	} // end init
	
	@Override
	public void start() {
		
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(UnionConfiguration.class, WorkingMemoryOperation.WILDCARD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							UnionConfiguration config = 
								getMemoryEntry(_wmc.address, UnionConfiguration.class);
							updateWithConfiguration(config);
						}
						catch (Exception e) {
							e.printStackTrace();
						}
					}
				});
	}

	private void updateWithConfiguration(UnionConfiguration config) {
		Vector<String> newItems = new Vector<String>();

		for (int i = 0; i < config.includedUnions.length; i++) {
			String unionId = config.includedUnions[i].entityID;
			newItems.add(unionId);
		}
		
		for (int i = 0; i < newItems.size(); i++) {
			String unionId = newItems.elementAt(i);
			if (!currentUnions.contains(unionId)) {
				unionBox.addItem(unionId);
			}
		}

		for (int i = 0; i < currentUnions.size(); i++) {
			String unionId = currentUnions.elementAt(i);
			if (!newItems.contains(unionId)) {
				unionBox.removeItem(unionId);
			}
		}
		currentUnions = newItems;
		ok.setEnabled(currentUnions.size() > 0);

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
		
		aboutField.requestFocusInWindow();
		
		while (!OKBUTTON_PUSHED) {
			this.sleepComponent(100);
		}
		
		OKBUTTON_PUSHED = false;
		
		// construct the clarification request

		if (needField.getText().equals("") || modalityField.equals("")) {
			return null;
		}
		
		ClarificationRequest result = new ClarificationRequest();
		result.id = newDataID();
		
		result.about = constructFormula(aboutField.getText());
		result.clarificationNeed = constructFormula(needField.getText());

		result.sourceModality = modalityField.getText();
		
		Object curItem = unionBox.getSelectedItem();
		if (curItem == null) {
			return null;
		}
		result.sourceEntityID = curItem.toString();

		// return the result
		aboutField.setText("");
		needField.setText("");
		return result;
	} // end getGUIPhonString	
	
	
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
		//modalityField.setPreferredSize(new Dimension(100,20));
		modalityPanel.add(modalityField);
		dialogPanel.add(modalityPanel);

        JPanel unionIdPanel = new JPanel();
        unionIdPanel.setLayout(new BoxLayout(unionIdPanel, BoxLayout.LINE_AXIS));
        JLabel unionIdLabel = new JLabel("Union ID:");
        unionIdPanel.add(unionIdLabel);
        unionBox = new JComboBox();
		unionIdPanel.add(unionBox);
		dialogPanel.add(unionIdPanel);

/*
        JPanel sourcePanel = new JPanel();
        sourcePanel.setLayout(new BoxLayout(sourcePanel, BoxLayout.LINE_AXIS));
        JLabel sourceLabel = new JLabel("Source ID:");
        sourcePanel.add(sourceLabel);
        sourceField = new JTextField();
        sourceField.setText(defaultSourceId);
		//sourceField.setPreferredSize(new Dimension(100,20));
		sourcePanel.add(sourceField);
		dialogPanel.add(sourcePanel);
*/

        JPanel aboutPanel = new JPanel();
        aboutPanel.setLayout(new BoxLayout(aboutPanel, BoxLayout.LINE_AXIS));
        JLabel aboutLabel = new JLabel("About:");
        aboutPanel.add(aboutLabel);
		aboutField = new JTextField();
		//aboutField.setPreferredSize(new Dimension(100,20));
		aboutPanel.add(aboutField);
		dialogPanel.add(aboutPanel);

        JPanel needPanel = new JPanel();
        needPanel.setLayout(new BoxLayout(needPanel, BoxLayout.LINE_AXIS));
        JLabel needLabel = new JLabel("Need:");
        needPanel.add(needLabel);
        needField = new JTextField();
		//needField.setPreferredSize(new Dimension(100,20));
		needPanel.add(needField);
		dialogPanel.add(needPanel);
		
		dialogPanel.add(Box.createVerticalGlue());
		
		JPanel buttonPanel = new JPanel();
		buttonPanel.setLayout(new BoxLayout(buttonPanel, BoxLayout.LINE_AXIS));
		buttonPanel.add(Box.createHorizontalGlue());
		ok = new JButton("Issue request");
		ok.setEnabled(false);  // disabled at the beginning
		buttonPanel.add(ok);
		dialogPanel.add(buttonPanel);

		dialog = new JDialog(frame, "Clarification Request");
		dialog.add(dialogPanel);

		dialog.setLocation(600, 400);

		Dimension dim = dialogPanel.getPreferredSize();
		dialog.setSize(300, dim.height + 25);
		dialog.setVisible(true);
		aboutField.requestFocusInWindow();
		
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
							 cc_FakeClarificationGenerator.OKBUTTON_PUSHED = true;
							 }
							 });
		
	} 
	
	// 	
	
	private ComplexFormula constructFormula (String phi) { 
		ComplexFormula formula = new ComplexFormula();
		formula.id = newDataID();
		formula.prob = 1.0f;
		// establish simple predicate/argument structure
		String predicate = "";
		String argument = null; 
		if (phi.indexOf("(") != -1) { 
			predicate = phi.substring(0,phi.indexOf("("));
			argument  = phi.substring(phi.indexOf("(")+1,phi.indexOf(")"));
			log("Predicate[argument]: "+predicate+"["+argument+"]");
		} else { 
			predicate = phi;
		} // end if..else check for predicate argument
		// construct the formula
		if (predicate.startsWith("colo")) { 
			ColorProperty color = new ColorProperty();
			color.cstatus = ContinualStatus.proposition;
			color.prob = 1.0f;
			if (argument != null) { 
				if (argument.equals("red")) { 
					color.colorValue = beliefmodels.domainmodel.cogx.Color.red;
				} else if (argument.equals("blue")) { 
					color.colorValue = beliefmodels.domainmodel.cogx.Color.blue;					
				} else if (argument.equals("green")) { 
					color.colorValue = beliefmodels.domainmodel.cogx.Color.green;									
				} else if (argument.equals("yellow")) { 
					color.colorValue = beliefmodels.domainmodel.cogx.Color.yellow;										
				} else { 	
					color.colorValue = beliefmodels.domainmodel.cogx.Color.unknownColor;					
				} // end if..else check for specified color
			} else { 
				color.colorValue = beliefmodels.domainmodel.cogx.Color.unknownColor;		
			} // end if..else check whether specified color
			SuperFormula[] frms = new SuperFormula[1];
			frms[0] = color;
			formula.op = LogicalOp.none;
			formula.formulae = frms;
		} else if (predicate.startsWith("shape")) { 
			ShapeProperty shape = new ShapeProperty();
			shape.cstatus = ContinualStatus.proposition;
			shape.prob = 1.0f;
			if (argument != null) { 
				if (argument.equals("cylindrical")) { 
					shape.shapeValue = beliefmodels.domainmodel.cogx.Shape.cylindrical;
				} else if (argument.equals("spherical")) { 
					shape.shapeValue = beliefmodels.domainmodel.cogx.Shape.spherical;
				} else if (argument.equals("cubic")) { 
					shape.shapeValue = beliefmodels.domainmodel.cogx.Shape.cubic;									
				} else { 	
					shape.shapeValue = beliefmodels.domainmodel.cogx.Shape.unknownShape;					
				} // end if..else check for specified shape
			} else { 
				shape.shapeValue = beliefmodels.domainmodel.cogx.Shape.unknownShape;		
			} // end if..else check whether specified shape
			SuperFormula[] frms = new SuperFormula[1];
			frms[0] = shape;
			formula.op = LogicalOp.none;
			formula.formulae = frms;			
		} else if (predicate.startsWith("type")) { 
			ObjectTypeProperty type = new ObjectTypeProperty();
			type.cstatus = ContinualStatus.proposition;
			type.prob = 1.0f;
			if (argument != null) { 
				if (argument.equals("box")) { 
					type.typeValue = beliefmodels.domainmodel.cogx.ObjectType.box;
				} else if (argument.equals("ball")) { 
					type.typeValue = beliefmodels.domainmodel.cogx.ObjectType.ball;					
				} else if (argument.equals("cube")) { 
					type.typeValue = beliefmodels.domainmodel.cogx.ObjectType.cube;								
				} else if (argument.equals("mug")) { 
					type.typeValue = beliefmodels.domainmodel.cogx.ObjectType.mug;
				} else { 	
					type.typeValue = beliefmodels.domainmodel.cogx.ObjectType.unknownObjectType;					
				} // end if..else check for specified type
			} else { 
				type.typeValue = beliefmodels.domainmodel.cogx.ObjectType.unknownObjectType;		
			} // end if..else check whether specified type
			SuperFormula[] frms = new SuperFormula[1];
			frms[0] = type;
			formula.op = LogicalOp.none;
			formula.formulae = frms;			
		} else { 
			log("Unknown predicate: ["+predicate+"]");
		} 
		return formula;
	} // end 

	@Override
    public void configure(Map<String, String> _config) {
		if (_config.containsKey("--modality")) {
			defaultModality = _config.get("--modality");
		}
		if (_config.containsKey("--sourceId")) {
			defaultSourceId = _config.get("--sourceId");
		}
	}
	
} // end class
