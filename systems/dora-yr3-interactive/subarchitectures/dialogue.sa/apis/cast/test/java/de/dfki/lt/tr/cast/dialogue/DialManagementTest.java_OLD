
package de.dfki.lt.tr.cast.dialogue;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.events.Event;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;


/**
 * Test class for the integration of the dialogue manager with the other CAST components
 * 
 * @author Pierre Lison
 * @version 19/07/2010
 *
 */
public class DialManagementTest extends ManagedComponent {
 

	@Override
	public void runComponent() {

		String utterance = "this is a ball";
		PhonString result = new PhonString(newDataID(), utterance, 4, 1.0f, 1.0f, 1);
		try {
			addToWorkingMemory(newDataID(), result);
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	
	
	
	/*
	 * Starting up the CAST component by registering filters on the insertion of new intentions and events. 
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {

		//try {
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(Intention.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								Intention intention = getMemoryEntry(_wmc.address, Intention.class);
								if (intention.estatus instanceof PrivateEpistemicStatus) {
									log("Intention received: " + 
											FormulaUtils.getString(intention.content.get(0).postconditions));
								}
							} catch (DoesNotExistOnWMException e) {
								e.printStackTrace();
							} catch (UnknownSubarchitectureException e) {
								e.printStackTrace();
							}
						}
					});
	}
	
	
	public void log(String s) {
		System.out.println("[DialManagementTest] " + s);
	}
	
}
