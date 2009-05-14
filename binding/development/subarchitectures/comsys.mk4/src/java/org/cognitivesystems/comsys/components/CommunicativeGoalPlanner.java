//=================================================================
// Copyright (C) 2008 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
//=================================================================

package org.cognitivesystems.comsys.components;
//=================================================================
// IMPORTS
//=================================================================

// ----------------------------------------------------------------
// BINDING imports
// ----------------------------------------------------------------
import binding.abstr.AbstractBindingReader;
import binding.util.BindingUtils;
import BindingData.BindingProxy;
import BindingFeatures.Colour;
import BindingFeatures.Concept;
import BindingFeatures.RelationLabel;
import BindingFeatures.Shape;
import BindingFeatures.Size;
import BindingFeatures.SourceID;

// ----------------------------------------------------------------
// CAST imports
// ----------------------------------------------------------------
import cast.architecture.subarchitecture.AlreadyExistsOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.cdl.*;
import cast.core.data.CASTData;
import cast.core.CASTUtils;


// ----------------------------------------------------------------
// COMSYS imports
// ----------------------------------------------------------------
import org.cognitivesystems.comsys.autogen.ComsysEssentials.*;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.*;
import org.cognitivesystems.comsys.data.ProcessingData;
import org.cognitivesystems.comsys.general.ComsysException;
import org.cognitivesystems.comsys.ontology.ComsysGoals;
import org.cognitivesystems.comsys.general.ComsysUtils;

// ----------------------------------------------------------------
// JAVA imports
// ----------------------------------------------------------------
import java.util.Arrays;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Properties;
import java.util.Vector;

// -----------------------------------------------------------------
// LF IMPORTS
// -----------------------------------------------------------------
import org.cognitivesystems.repr.lf.utils.LFUtils;

// ----------------------------------------------------------------
// MOTIVATION imports
// ----------------------------------------------------------------
import motivation.components.abstr.AbstractActionResponder;
import motivation.idl.AcknowledgeMotive;
import motivation.idl.AttentionMotive;
import motivation.idl.GainAttentionMotive;
import motivation.idl.MotiveAcknowledgement;
import motivation.idl.MotiveType;
import motivation.idl.AnswerSet;
import motivation.util.CompetenceRegistration;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/**
 The class <b>CommunicativeGoalPlanner</b> creates communicative goals, on the 
 basis of dialogue moves (language system-internal) and motivation-process actions
 (language system-external). These goals are represented on the comsys WM as
 <tt>ContentPlanningGoal</tt> structs, feeding into the content planning process. 
 
 @started 080821
 @version 080821
 @author  Nick Hawes
 @author  Geert-Jan M. Kruijff (gj@dfki.de)
 
 */

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class CommunicativeGoalPlanner 
	extends AbstractActionResponder {

		
	//=================================================================
	// INNER CLASSES
	//=================================================================

			
	//=================================================================
	// CLASS-INTERNAL GLOBAL VARIABLES
	//=================================================================

		
		String bindingSA; 
		
		
	/* Process identifier counter, used for proposed tasks **/
	int pdIdCounter;

	// Hashtable used to record the tasks we want to carry out
	private Hashtable<String, ProcessingData> m_proposedProcessing;
	
	// Hashtable linking data IDs to goal IDs
	private Hashtable<String, String> m_dataToProcessingGoalMap;
	
	// Vector with objects to be processed, can be ComSys:PhonString,...
	private Vector<ProcessingData> m_dataObjects;	

	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================
		
	/**
	 * @param _id
	 */
	public CommunicativeGoalPlanner(String _id) {
		super(_id);
	}
		
	private void init () { 
		pdIdCounter = 0;
		// Make all the comsys queue changes to avoid missing updates
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
		// Data processing
		m_proposedProcessing = new Hashtable<String, ProcessingData>();
		m_dataToProcessingGoalMap = new Hashtable<String, String>();
		m_dataObjects = new Vector<ProcessingData>();	
	} // end init		
		
		
	//=================================================================
	// TASK METHODS
	//=================================================================
		
	@Override
	protected void runComponent() {
		// let the world know we're capable dealing with these things
		try {
			CompetenceRegistration.registerFeatureGenerationCompetence(this,
																	Colour.class);
			CompetenceRegistration.registerFeatureGenerationCompetence(this,
																	   Shape.class);

		}
		catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
		// start the component loop
		try {
			log("Entering loop checking for data in communicative goal planner component");
			while (m_status == ProcessStatus.RUN) {
				// lock from external access
				lockProcess();
				// check (synchronised) data objects queue
				ListIterator<ProcessingData> i = m_dataObjects
				.listIterator();
				// for the data on the queue, tasks have been approved, so
				// iterate over data objects, and execute the tasks.
				
				while (i.hasNext()) {
					// get the data from the queue
					ProcessingData data = i.next();
					// check its ID, get the corresponding task ID
					String dataID = data.getID();
					String taskID = m_dataToProcessingGoalMap
					.get(dataID);
					log("For data [" + dataID + "/" + data.getTypes()
						+ "] do [" + taskID + "]");
					// make sure we know what to do, and have something
					// to do it with
					if (data != null) {
						// execution tasks throw comsys exceptions
						try {
							executeGoalPlanningTask(data);
							// inform the goal manager that the task has
							// been completed
							try {
								taskComplete(
											 taskID,
											 TaskOutcome.PROCESSING_COMPLETE_SUCCESS);
							}
							catch (SubarchitectureProcessException e) {
								e.printStackTrace();
							} // end try..catch
						}
						catch (ComsysException e) {
							log("Exception while executing a task in utterance realization: "
								+ e.getMessage());
							e.printStackTrace();
							// inform the goal manager that the task has
							// been completed, but unsuccessfully
							// we may want to make this more specific
							try {
								taskComplete(
											 taskID,
											 TaskOutcome.PROCESSING_COMPLETE_FAILURE);
							}
							catch (SubarchitectureProcessException ex) {
								ex.printStackTrace();
							} // end try..catch
						} // end try..catch for processing exceptions
					} else {
						log("Nothing to process in utterance realization: data null");
					} // end
					// Clean up!
					// no matter what happened, remove the data item
					// from the queue
					i.remove();
				} // end while
				// Free the process
				unlockProcess();
				sleepProcess(20);
				// wait for new tasks!
				waitForNotifications();
			} // end while running
		}
		catch (Exception e) {
			e.printStackTrace();
		} // end try..catch
	} // end runComponent


	/**
	 * The method <i>taskAdopted</i> processes a parsing task once the
	 * task manager has informed the parser component it can run. 
	 * This method does <b>not</b> inform the task manager, whether the
	 * task has been succesfully completed. This notification happens in
	 * the <i>runComponent</i> method, which executes the task (run on its 
	 * associated data). 
	 * 
	 * @param _goalID The identifier of the task that has been adopted
	 * 
	 * @see #runComponent
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _goalID) {
		// get the data we stored for this goal
		ProcessingData data = m_proposedProcessing.remove(_goalID);
		// if we have stored this goal earlier
		if (data != null) {
			// add the data item to the data objects queue
			m_dataObjects.addElement(data);
			// get the identifier of the CAST data type
			String dataID = data.getID();
			// link the data ID to the goal ID, for future reference
			// on task completion (done in runComponent)
			m_dataToProcessingGoalMap.put(dataID, _goalID);
		}
		else {
			log("ERROR: Goal without data: " + _goalID);
		} // end if..else
	} // end taskAdopted

	/**
	 * The method <i>taskRejected</i> prints out a log message that the proposed task 
	 * has been rejected, and removes the goal from the queue of proposed tasks (by ID). 
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _goalID) {
		log("WARNING: The goal with ID [" + _goalID
		+ "] has been rejected.");
		m_proposedProcessing.remove(_goalID);
	} // end taskRejected


	/**
	 * The method "start" adds a working memory change filter on PhonString and PackedLFs objects. If an object has been added to working memory, 
	 * the method <i>handleWorkingMemoryChange</i> is being called. <p>
	 * Furthermore, the method registers change listeners for all the data types that have been registered for the active processes run within the component. These change
	 * listeners all call the method <i>handleActiveDataWorkingMemoryChange</i>. 
	 * 
	 * @see #handleWorkingMemoryChange(WorkingMemoryChange _wmc)
	 * @see #handleActiveDataWorkingMemoryChange(WorkingMemoryChange _wmc)
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged(cast.corba.autogen.SubArchitecture.WorkingMemoryChange[])
	 */
	@Override
	public void start() {
		super.start();
		// now call the initialization method for the object
		init();
		// now do the rest
		try {
		// register change filters for ProductionLF, which triggers realization
			addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(DialogueMove.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				handleWorkingMemoryChange(_wmc);
			} });
			addChangeFilter(
							ChangeFilterFactory.createLocalTypeFilter(BoggleWoggle.class,  WorkingMemoryOperation.ADD),
							new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
							handleMotivationChange(_wmc);
							} });
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		} // end try..catch
	}// end start

		
	private void handleMotivationChange(WorkingMemoryChange _wmc) {
		try {
			// get the id of the working memory entry
			String id = _wmc.m_address.m_id;
			// get the data from working memory and store it with its id
			CASTData data = new CASTData(id,getWorkingMemoryEntry(id));
			// get a new id for the task
			String taskID = newTaskID();
			// store the data we want to process for later
			String dataType = data.getType();
			if (data != null) {
				log("Now try to start content goal plan formulation from motivation structure");
				// Get the struct
				BoggleWoggle bw = (BoggleWoggle) data.getData();
				// Initialize the logical form for the goal
				LogicalForm comGoal = LFUtils.convertFromString(bw.m_string);
				// Send off the comgoal
				sendCommunicativeGoal(comGoal);
			} // end if.. check we have data
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		} // end try..catch
	} // end handleWorkingMemoryChange	
		
		
		
	/**
	 * The method <i>handleWorkingMemoryChange</i> retrieves a CASTData object from the working memory, 
	 * and stores it in the <tt>m_proposedProcessing</tt> table with a parsing task. The CASTData object
	 * can be either a PhonString object (beginning a new utterance processing task) or a PackedLFs object
	 * (continuing an already ongoing processing task).  
	 * 
	 * If one of these objects turns out to provide contextual information used in a registered active
	 * subprocess, then this information needs to be passed on <b>before</b> any further tasks are being
	 * proposed. This is checked here. The change is passed on to the method <i>handleActiveDataWorkingMemoryChange</i>,
	 * and carried out (waiting till the called update methods return) to ensure that all active processes 
	 * are up-to-date before any tasks methods are called. 
	 * 
	 * @param _wmc The working memory change -- a recently added CASTData object
	 */
	private void handleWorkingMemoryChange(WorkingMemoryChange _wmc) {
		try {
			// get the id of the working memory entry
			String id = _wmc.m_address.m_id;
			// get the data from working memory and store it with its id
			CASTData data = new CASTData(id,getWorkingMemoryEntry(id));
			// get a new id for the task
			String taskID = newTaskID();
			// store the data we want to process for later
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			m_proposedProcessing.put(taskID, pd);
			// set the goal
			String taskGoal = ComsysGoals.DIALPLAN_TASK;
			// store the goal with its information
			proposeInformationProcessingTask(taskID, taskGoal);
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		} // end try..catch
	} // end handleWorkingMemoryChange			
	
	/** Returns a new identifier for a ProcessingData object 
	 @return String The new (unique) identifier
	 */
	
	private String newProcessingDataId() {
		String result = "pd" + pdIdCounter;
		pdIdCounter++;
		return result;
	} // end newProcessingDataId			

		
	protected void sendCommunicativeGoal (LogicalForm comGoal) { 	
		// Store the content planning goal on working memory
		log("Sending the following communicative goal: "+LFUtils.lfToString(comGoal)); 
		try { 
			ContentPlanningGoal cpg = ComsysUtils.newContentPlanningGoal();
			cpg.lf = comGoal;
			addToWorkingMemory(newDataID(), cpg); 
		} catch (AlreadyExistsOnWMException ioe) { 
			log("ERROR: "+ioe.getMessage());
		} catch (SubarchitectureProcessException se) { 
			log("ERROR: "+se.getMessage());						
		} // end try.. catch		
	} // end sendCommunicativeGoal

		
		protected String encode (String proxyID) { 	
			return proxyID.replaceAll(":","_");
		} 
		
		protected String decode (String proxyID) { 
			return proxyID.replaceAll("_",":");
		} 
		
	//=================================================================
	// COMPUTATION METHODS
	//=================================================================
		
	/**	
	The method <i>executeGoalPlanningTask</i> is triggered when a dialogue move has been posted to working memory. 
	This method checks whether there is a purely linguistic / communicative response needed to that move, e.g. 
	a greeting, and produces such a goal. 
	 
	*/  
	protected void executeGoalPlanningTask (ProcessingData pd) 	
	throws ComsysException {
		Vector pdTypes = pd.getTypes();
		if (pdTypes.contains(CASTUtils.typeName(DialogueMove.class))) {
			CASTData data = pd
			.getByType(CASTUtils.typeName(DialogueMove.class));
			String dataType = data.getType();
			log("Execute content goal planning task on data item [" + dataType + "]");
			if (data != null) {
				log("Now try to start content goal planning");
				// Initialize the logical form for the goal
				LogicalForm comGoal = null;
				// Get the input object
				DialogueMove dialogueMove = (DialogueMove) data.getData();
				// Decide what to do
				if (dialogueMove.mType == MoveType.OPENING)  {
					comGoal = LFUtils.convertFromString("(@d1:dvp(c-goal ^ <SpeechAct>greeting))");
				} // end if else ... check for linguistic moves
				// Store the content planning goal on working memory
				if (comGoal != null) { 
					log("Sending the following communicative goal: "+LFUtils.lfToString(comGoal)); 
					sendCommunicativeGoal(comGoal);
				}
			} // end if.. check for data
		} // end if.. check for a dialogue move
	} // end executeGoalPlanningTask
		
	/**	
	 The method <i>acknowledgeMotive<i> produces a communicative goal on the basis of an 
	 acknowledgement received externally, from the Motivation subarchitecture. 
	 */  
	@Override
	protected void acknowledgeMotive(String _actionID, AcknowledgeMotive _ack)
		throws SubarchitectureProcessException 
	{
		// Initialize the logical form for the communicative goal
		LogicalForm comGoal = LFUtils.newLogicalForm();		
		
		// Check for what we are about to acknowledge
		if (_ack.m_motiveType.equals(MotiveType.PHYSICAL_ACTION)) {
			// Check for type of acknowledgement		
			if (_ack.m_ack == MotiveAcknowledgement.MOTIVE_ACK_ACCEPTED) {
				log("The last motive from " + getSubarchitectureID()
						+ " was accepted.");
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>accept ^ <AcknoModality>action)");			
			} else if (_ack.m_ack == MotiveAcknowledgement.MOTIVE_ACK_REJECTED) {
				log("The last motive from " + getSubarchitectureID()
						+ " was rejected.");
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>reject ^ <AcknoModality>action)");						
			} else if (_ack.m_ack == MotiveAcknowledgement.MOTIVE_ACK_ALREADY_HOLDS) {
				log("The last motive from " + getSubarchitectureID()
						+ " was already the case.");
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>alreadyTrue)");						
			} // end if..else check for ackno type
		} else if (_ack.m_motiveType.equals(MotiveType.FACTUAL_QUESTION) || _ack.m_motiveType.equals(MotiveType.POLAR_QUESTION)) {
			if (_ack.m_ack == MotiveAcknowledgement.MOTIVE_ACK_ACCEPTED) {
				log("The last motive from " + getSubarchitectureID()
					+ " was accepted.");
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>accept ^ <AcknoModality>vision)");
			} else if (_ack.m_ack == MotiveAcknowledgement.MOTIVE_ACK_REJECTED) {
				log("The last motive from " + getSubarchitectureID()
					+ " was rejected.");
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>reject ^ <AcknoModality>vision)");						
			} else if (_ack.m_ack == MotiveAcknowledgement.MOTIVE_ACK_ALREADY_HOLDS) {
				log("The last motive from " + getSubarchitectureID()
					+ " was already the case.");
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>alreadyTrue)");						
			} // end if..else check for ackno type		
		
		} else { 
			if (_ack.m_ack == MotiveAcknowledgement.MOTIVE_ACK_ACCEPTED) {
				log("The last motive from " + getSubarchitectureID()
					+ " was accepted.");
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>accept)");			
			} else if (_ack.m_ack == MotiveAcknowledgement.MOTIVE_ACK_REJECTED) {
				log("The last motive from " + getSubarchitectureID()
					+ " was rejected.");
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>reject)");						
			} else if (_ack.m_ack == MotiveAcknowledgement.MOTIVE_ACK_ALREADY_HOLDS) {
				log("The last motive from " + getSubarchitectureID()
					+ " was already the case.");
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>alreadyTrue)");						
			} // end if..else check for ackno type		
		} 
		// Now send the communicative goal to the comsys WM
		sendCommunicativeGoal(comGoal);
		// Do the acknowledgement admin
		for (WorkingMemoryPointer _source : _ack.m_motiveCause) {
			// these are all proxies
			assert (_source.m_type.equals(CASTUtils
					.typeName(BindingProxy.class)));
		} // end for
		log("The motive involved these motive.sa proxies: "
				+ Arrays.toString(_ack.m_motiveCause));
		// now tell the planner we've acknowledged successfully, note that this
		// doesn't have to be called in this method. It should be called once
		// the acknowledgement is done, i.e. after something has been said
		responseComplete(_actionID, TriBool.triTrue);
	} // end acknowledgeMotive

	@Override
	protected void factualQuery(String _actionID, String _proxyID,
			Class<?> _queryClass, String _fromSA)
			throws SubarchitectureProcessException 
	{
		log("Generating communicative goal for a factual question");		
		// Initialize the proxyAddress feature, and the concept of the object (used if non-comsys proxy)
		String proxyAddress = null; 
		String concept = null;
		// Initialize the logical form for the communicative goal
		LogicalForm comGoal = LFUtils.newLogicalForm();			
		// Check for the proxy and what subarch it is from 		
		BindingProxy _intdRefProxy = null;
		_intdRefProxy = BindingUtils.getProxy(this, bindingSA, _proxyID); // this will throw an error if failing; passed upwards
		String sourceMonitor = _intdRefProxy.m_sourceID;
		if (sourceMonitor.contains("comsys")) { 
			proxyAddress = _proxyID;
		} else if (sourceMonitor.contains("vision") ){ 
			for (Concept refConcept : BindingUtils.getBindingFeatures(this, bindingSA, _intdRefProxy, Concept.class)) {
				concept = refConcept.m_concept;
			}
		} else {	
			System.err.println("ERROR: comsys.sa:CommunicativeGoalPlanner cannot determine factual queries for content other than from vision or comsys");
		} // end if..else 
		// Check what kind of property we are dealing with 
		if (_queryClass.equals(BindingFeatures.Concept.class)) { 
			
		} else if (_queryClass.equals(BindingFeatures.Colour.class)) { 
			if (proxyAddress != null) { 
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>question  ^  <Content>(e1:ascription ^ <Target>("+encode(proxyAddress)+":rfx ^ <ProxyAddress>true) ^ <Color>(b3:quality ^ color ^ <Questioned>true)))");
			} else if (concept != null) { 
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>question  ^  <Content>(e1:ascription ^ <Target>(b2:entity ^ "+concept+" ^ <ProxyAddress>false) ^ <Color>(b3:quality ^ color ^ <Questioned>true)))");
			} else {
				log("Cannot determine comgoal for Colour");
			} // end if..else check how to provide reference to object
		} else if (_queryClass.equals(BindingFeatures.Shape.class)) { 
			if (proxyAddress != null) { 
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>question  ^  <Content>(e1:ascription ^ <Target>("+encode(proxyAddress)+":rfx ^ <ProxyAddress>true) ^ <Color>(b3:quality ^ shape ^ <Questioned>true)))");
			} else if (concept != null) { 
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>question  ^  <Content>(e1:ascription ^ <Target>(b2:entity ^ "+concept+" ^ <ProxyAddress>false) ^ <Color>(b3:quality ^ shape ^ <Questioned>true)))");
			} else {
				log("Cannot determine comgoal for Shape");
			} // end if..else check how to provide reference to object			
		} else if (_queryClass.equals(BindingFeatures.Size.class)) { 
			if (proxyAddress != null) { 
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>question  ^  <Content>(e1:ascription ^ <Target>("+encode(proxyAddress)+":rfx ^ <ProxyAddress>true) ^ <Color>(b3:quality ^ size ^ <Questioned>true)))");
			} else if (concept != null) { 
				comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>question  ^  <Content>(e1:ascription ^ <Target>(b2:entity ^ "+concept+" ^ <ProxyAddress>false) ^ <Color>(b3:quality ^ size ^ <Questioned>true)))");
			} else {
				log("Cannot determine comgoal for Size");
			} // end if..else check how to provide reference to object				
		}
		// Now send the communicative goal to the comsys WM
		sendCommunicativeGoal(comGoal);
		
		// now tell the planner we've acknowledged successfully, note that this
		// doesn't have to be called in this method. It shouuld be called once
		// the query has been generated and the answer process resulting in new
		// data on binding wm
		responseComplete(_actionID, TriBool.triTrue);
	}
		
		/** 
		 Generates a communicative goal for an assertion. Again, make a difference between vision and comsys proxies. 
		*/ 
		
		@Override
		protected <FeatureT> void factualAssertion(String _actionID,
												   String _proxyID, FeatureT _feature)
		throws SubarchitectureProcessException 
		{
			
			log("Generating communicative goal for a factual assertion");
			// Initialize the proxyAddress feature, and the concept of the object (used if non-comsys proxy)
			String proxyAddress = null; 
			String concept = null;
			// Initialize the logical form for the communicative goal
			LogicalForm comGoal = LFUtils.newLogicalForm();			
			// Check for the proxy and what subarch it is from 		
			BindingProxy _intdRefProxy = null;
			_intdRefProxy = BindingUtils.getProxy(this, bindingSA, _proxyID); // this will throw an error if failing; passed upwards
			String sourceMonitor = _intdRefProxy.m_sourceID;
			if (sourceMonitor.contains("comsys")) { 
				proxyAddress = _proxyID;
				/**for (Concept refConcept : BindingUtils.getBindingFeatures(this, bindingSA, _intdRefProxy, Concept.class)) {
					concept = refConcept.m_concept;
				}*/
			} else if (sourceMonitor.contains("vision") ){ 
				for (Concept refConcept : BindingUtils.getBindingFeatures(this, bindingSA, _intdRefProxy, Concept.class)) {
					concept = refConcept.m_concept;
				}
			} else {	
				System.err.println("ERROR: comsys.sa:CommunicativeGoalPlanner cannot determine factual answers for content other than from vision or comsys");
			} // end if..else 
			// Check what kind of property we are dealing with 			
			if(_feature.getClass().equals(Concept.class)) {
				// Get the concept
				concept = ((Concept) _feature).m_concept;
				// check whether the concept is a compound or not
				if (concept.indexOf("_") != -1) { 
					String mod  = concept.substring(0,concept.indexOf("_"));
					String head = concept.substring(concept.indexOf("_")+1,concept.length());
					if (mod.equals("colour")) { mod = "color"; }
					concept = head+"^ <Unique>true ^ <Name>"+mod;
					comGoal = LFUtils.convertFromString("@d1:dvp(c-goal ^ <SpeechAct>assertion ^ <Content>(b3:entity ^ "+concept+")   ) ");
				} else { 
					// create a fancy visual response
					comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>answer  ^  <Modality>vision ^ <Content>(b1:entity ^ "+concept+"))");
				} // end if..else check for compounds
			} else if(_feature.getClass().equals(Colour.class)) {
				String color = ((Colour) _feature).m_colour;
				if (proxyAddress != null) { 
					comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>answer  ^  <Content>(e1:ascription ^ <Target>("+encode(proxyAddress)+":rfx ^ <ProxyAddress>true) ^ <Color>(b3:quality ^ "+color+")))");
					//comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>answer  ^  <Content>(e1:ascription ^ be ^ <Target>(b2:entity ^ "+concept+" ^ <Unique>true ^ <ProxyAddress>false) ^ <Color>(b3:quality ^ "+color+")))");
				} else if (concept != null) { 
					comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>answer  ^  <Content>(e1:ascription ^ be ^ <Target>(b2:entity ^ "+concept+" ^ <ProxyAddress>false ^ <Unique>true) ^ <Color>(b3:quality ^ "+color+")))");
				} else {
					log("Cannot determine comgoal for Colour");
				} // end if..else check how to provide reference to object
			} else if (_feature.getClass().equals(BindingFeatures.Shape.class)) { 
				String shape = ((Shape) _feature).m_shape;
				if (proxyAddress != null) { 
					comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>answer  ^  <Content>(e1:ascription ^ <Target>("+encode(proxyAddress)+":rfx ^ <ProxyAddress>false) ^ <Shape>(b3:quality ^ "+shape+")))");
//					comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>answer  ^  <Content>(e1:ascription ^ <Target>(b2:entity ^ "+concept+" ^ <ProxyAddress>false ^ <Unique>true) ^ <Shape>(b3:quality ^ "+shape+")))");
				} else if (concept != null) { 
					comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>answer  ^  <Content>(e1:ascription ^ <Target>(b2:entity ^ "+concept+" ^ <ProxyAddress>false ^ <Unique>true) ^ <Shape>(b3:quality ^ "+shape+")))");
				} else {
					log("Cannot determine comgoal for Shape");
				} // end if..else check how to provide reference to object			
			} else if (_feature.getClass().equals(BindingFeatures.Size.class)) { 
				String size = ((Size) _feature).m_size;
				if (proxyAddress != null) { 
					comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>answer  ^  <Content>(e1:ascription ^ <Target>("+encode(proxyAddress)+":rfx ^ <ProxyAddress>false) ^ <Size>(b3:quality ^ "+size+")))");
//					comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>answer  ^  <Content>(e1:ascription ^ <Target>(b2:entity ^ "+concept+" ^ <ProxyAddress>false ^ <Unique>true) ^ <Size>(b3:quality ^ "+size+")))");
				} else if (concept != null) { 
					comGoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>assertion  ^ <Relation>answer  ^  <Content>(e1:ascription ^ <Target>(b2:entity ^ "+concept+" ^ <ProxyAddress>false ^ <Unique>true) ^ <Size>(b3:quality ^ "+size+")))");
				} else {
					log("Cannot determine comgoal for Size");
				} // end if..else check how to provide reference to object				
			}
			// Now send the communicative goal to the comsys WM
			sendCommunicativeGoal(comGoal);
			
			// now tell the planner we've asked unsuccessfully, note that this
			// doesn't have to be called in this method. It shouuld be called once
			// the query has been generated and the answer process resulting in new
			// data on binding wm
			responseComplete(_actionID, TriBool.triTrue);
		}
	
		//@Override
		protected void answerSet(String _actionID, AnswerSet _as)	
			throws SubarchitectureProcessException
		{ 
			// Initialize the goal LF
			LogicalForm comGoal = LFUtils.newLogicalForm();			

			// Create a vector over the objects
			Vector<String> objects = new Vector<String>();
			
			// Cycle over the proxies
			WorkingMemoryPointer[] proxies = _as.m_answerSet;
			for (WorkingMemoryPointer pointer : proxies) { 
				// Check for the proxy and what subarch it is from 		
				BindingProxy _intdRefProxy = null;
				String _proxyID = pointer.m_address.m_id;
				_intdRefProxy = BindingUtils.getProxy(this, bindingSA, _proxyID); // this will throw an error if failing; passed upwards
				for (Concept refConcept : BindingUtils.getBindingFeatures(this, bindingSA, _intdRefProxy, Concept.class)) {
					objects.addElement(refConcept.m_concept);
				}
			} // end for
			log("Planning a scene description including ["+objects.size()+"] objects, "+objects.toString());
			
			
			String listLF = "";
			if (objects.size() == 0) { 
				listLF = "@d1:dvp(cg ^ <Content>(s1:perception ^ see ^ <Mood>ind ^ <Polarity>neg ^ <Tense>pres ^ <Actor>(i1:person ^ I) ^ <Patient>(c1:thing ^ context ^ <Delimitation>variable ) ^ <Subject>i1:person))";
			} else if (objects.size() == 1) { 	
				listLF = "@a1:entity("+objects.elementAt(0)+")";
			} else if (objects.size() == 2) { 
				listLF = "@a1:entity(<List>conjunction ^ <First>(b1:entity ^ "+objects.elementAt(0)+") ^ <Next>(m1:entity ^ "+objects.elementAt(1)+"))";
			} else if (objects.size() == 3) { 
				listLF = "@a1:entity(<List>conjunction ^ <First>(b1:entity ^ "+objects.elementAt(0)+") ^ <Next>(e2:entity ^ and <First>(b2:entity ^ "+objects.elementAt(1)+" ^ <Next>(b3:entity ^ "+objects.elementAt(2)+")))";
		    } else { 		   
				System.err.println("CURSES!!: Cannot deal with lists with more than 3 elements at the moment");	   
			} // 
			comGoal = LFUtils.convertFromString(listLF);
			// Now send the communicative goal to the comsys WM
			sendCommunicativeGoal(comGoal);
			// now tell the planner we've asked unsuccessfully, note that this
			// doesn't have to be called in this method. It shouuld be called once
			// the query has been generated and the answer process resulting in new
			// data on binding wm
			responseComplete(_actionID, TriBool.triTrue);			
		}
			
		
		
		@Override
		protected void factualRelationQuery(String _actionID, String _proxyID, RelationLabel _relationLabel, String _fromSA) 
			throws SubarchitectureProcessException 
		{
			// Get the relation
			String relation = _relationLabel.m_label;
			// Initialize the communicative goal LF
			LogicalForm comgoal = LFUtils.newLogicalForm();
			// Check for the proxy and what subarch it is from 		
			BindingProxy _intdRefProxy = null;
			_intdRefProxy = BindingUtils.getProxy(this, bindingSA, _proxyID); // this will throw an error if failing; passed upwards
			String concept = "thing";
			for (Concept refConcept : BindingUtils.getBindingFeatures(this, bindingSA, _intdRefProxy, Concept.class)) {
				concept = refConcept.m_concept;
				if (concept.indexOf("_") != -1) { 
					String mod  = concept.substring(0,concept.indexOf("_"));
					String head = concept.substring(concept.indexOf("_")+1,concept.length());
					if (mod.equals("colour")) { mod = "color"; }
					concept = head+"^ <Unique>true ^ <Name>"+mod;
				}
				
			}
			if (relation.startsWith("Position") || relation.startsWith("Loc")) { 
				comgoal = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>question  ^  <Content>(e1:ascription ^ <Target>(b2:entity ^ "+concept+" ^ <Unique>true) ^ <Location>(b3:m-location ^ <Questioned>true)) )");
			} else { 
				System.err.println("ERROR: Cannot determine comgoal for relational property ["+relation+"] in Communicative Goal Planner");
			} 
			// Send off the communicative goal
			sendCommunicativeGoal(comgoal);			
			// now tell the planner we've asked unsuccessfully, note that this
			// doesn't have to be called in this method. It shouuld be called once
			// the query has been generated and the answer process resulting in new
			// data on binding wm
			responseComplete(_actionID, TriBool.triFalse);
		}
		
	/** 	
		The method <i>gainAttentionMotive</i> triggers the generation of one or more 
		utterances 
		
	 
	*/ 
		
	@Override
	protected void gainAttentionMotive (String _actionID, GainAttentionMotive _att) 
		throws SubarchitectureProcessException 
	{ 
		// Initialize the logical form for the communicative goal
		LogicalForm comGoal = LFUtils.newLogicalForm();		
		// Check for type of attention we need to do 		
		if (_att.m_attention == AttentionMotive.ATTENTION_HELP) {
			log("The robot needs to try and get help");		
			comGoal = LFUtils.convertFromString("@d1:dvp(c-goal ^ <SpeechAct>gainAttention ^ <AttentionType>help)");
		} else if (_att.m_attention == AttentionMotive.ATTENTION_GREET) {
			log("The robot should just call to say hello ");		
			comGoal = LFUtils.convertFromString("@d1:dvp(c-goal ^ <SpeechAct>gainAttention ^ <AttentionType>greet)");			
		} else if (_att.m_attention == AttentionMotive.ATTENTION_CLOSE) {
			log("The robot should say goodbye to everyody");		
			comGoal = LFUtils.convertFromString("@d1:dvp(c-goal ^ <SpeechAct>gainAttention ^ <AttentionType>close)");						
		} else if (_att.m_attention == AttentionMotive.ATTENTION_KILL) {
			log("The robot is coming to get you");		
			comGoal = LFUtils.convertFromString("@d1:dvp(c-goal ^ <SpeechAct>gainAttention ^ <AttentionType>kill)");									
		} // 
		// Send the communicative goal to the comsys WM
		sendCommunicativeGoal(comGoal);		
		// Push the goal onto the stack, wait until we get a reaction to see how to indicate response completion
		
		
		
		
	} // end gain attention

	
	public void configure(Properties _config) {
		super.configure(_config);
		if (_config.containsKey("--bsa")) {
			this.bindingSA = _config.getProperty("--bsa");
		} else {
			throw new RuntimeException("You must specify the binding SA ID: --bsa <SA-ID>!");
		}
	}
	
	
	
		
}
