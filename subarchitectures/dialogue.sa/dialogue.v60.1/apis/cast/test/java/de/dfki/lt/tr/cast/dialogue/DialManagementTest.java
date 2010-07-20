
package de.dfki.lt.tr.cast.dialogue;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

import cast.AlreadyExistsOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;


/**
 * Test class for the integration of the dialogue manager with the other CAST components
 * 
 * @author Pierre Lison
 * @version 19/07/2010
 *
 */
public class DialManagementTest extends ManagedComponent {

	Intention extractedIntention ;

	@Before
	@Override
	public void start() {

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(Intention.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(
							WorkingMemoryChange _wmc) {
						try {
							extractedIntention = getMemoryEntry(_wmc.address, Intention.class);
							if (extractedIntention!=null && extractedIntention.content.size() > 0)  {
								String formAsString = FormulaUtils.getString(extractedIntention.content.get(0).postconditions);
								
								debug("raw extracted intention: " + formAsString);

								if (extractedIntention.content.get(0).postconditions instanceof ModalFormula && 
										((ModalFormula)extractedIntention.content.get(0).postconditions).op.equals("Belief")) {
									String beliefID = FormulaUtils.getString(
											((ModalFormula)extractedIntention.content.get(0).postconditions).form);
									
										dBelief b = getMemoryEntry(beliefID, dBelief.class);
									
										log("type of distrib: " + b.content);
									
								//		Intention forgedIntention = EpistemicObjectUtils.createAttributedIntention(postconditions)
									
										log("augmented extracted intention");
										
									
								}
							}
						}
						catch (Exception e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} 
					}
				});

	}

	/**
	@Override
	public void runComponent() {

		try {
			dFormula formula = FormulaUtils.constructFormula("\"this is a ball\"");
			Intention intent = EpistemicObjectUtils.createSimpleAttributedIntention(formula, 1.0f);
			addToWorkingMemory(newDataID(), intent);
		}
		catch (Exception e) {
			e.printStackTrace();
		}

		int count = 0;
		while (count < 100 && extractedIntention==null) {
			sleepComponent(100);
			count++;
		}
		if (extractedIntention!=null && extractedIntention.content.size() > 0)  {
			String formAsString = FormulaUtils.getString(extractedIntention.content.get(0).postconditions);
			log("PRODUCED INTENTION: " + formAsString);
		}
		else {
			log("WARNING: NO INTENTION PRODUCED");
		}
	}
	*/
}
