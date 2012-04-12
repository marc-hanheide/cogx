package de.dfki.lt.tr.cast.dialogue;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import autogen.Planner.Action;
import autogen.Planner.Link;
import autogen.Planner.POPlan;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.cast.dialogue.util.POPlanUtils;

public class POPlanMonitor extends ManagedComponent {

	protected void configure(Map<String, String> args) {
		// TODO Auto-generated method stub
		super.configure(args);
	}


	protected void start() {
		// add CFs

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(POPlan.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processAddedPOPlan(_wmc);
			}
		});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(POPlan.class, WorkingMemoryOperation.OVERWRITE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processOverwrittenPOPlan(_wmc);
			}
		});		
	}
	
	private void processAddedPOPlan(WorkingMemoryChange _wmc) {
		POPlan _newPOPlan;
		try {
			_newPOPlan = getMemoryEntry(_wmc.address, POPlan.class);
		} catch (DoesNotExistOnWMException e) {
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		log("ADDED POPlan: " + poplanToString(_newPOPlan));
	}
	
	private void processOverwrittenPOPlan(WorkingMemoryChange _wmc) {
		POPlan _oldPOPlan;
		try {
			_oldPOPlan = getMemoryEntry(_wmc.address, POPlan.class);
		} catch (DoesNotExistOnWMException e) {
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		log("OVERWRITTEN POPlan: " + poplanToString(_oldPOPlan));
	}

	private String poplanToString(POPlan _poPlan) {

		return POPlanUtils.POPlanToString(_poPlan);
	}
	
	
	protected void runComponent() {
		log("POPlanMonitor running...");
	}

}
