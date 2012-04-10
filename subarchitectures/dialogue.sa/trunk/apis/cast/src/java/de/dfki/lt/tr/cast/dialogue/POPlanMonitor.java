package de.dfki.lt.tr.cast.dialogue;

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
		StringBuilder action_sb = new StringBuilder(22 * _poPlan.actions.length);
		for (Action _ac : _poPlan.actions) {
			action_sb.append("Action fullName = ");
			action_sb.append(_ac.fullName);
		}
		
		StringBuilder link_sb = new StringBuilder(42 * _poPlan.links.length);
		for (Link _ln : _poPlan.links) {
			link_sb.append("Link  =");
			link_sb.append(" src: " + _ln.src);
			link_sb.append(" dest: " + _ln.dest);
			link_sb.append(" reason fact name: " + _ln.reason.name);
		}

		StringBuilder return_sb = new StringBuilder(40 + action_sb.length() + link_sb.length());
		return_sb.append("processAddedPOPlan() called: ");
		return_sb.append("Task ID = ");
		return_sb.append(_poPlan.taskID);
		return_sb.append(" Status name = ");
		return_sb.append(_poPlan.status.name());
		return_sb.append(" Actions: ");
		return_sb.append(action_sb);
		return_sb.append(" Links: ");
		return_sb.append(link_sb);
		
		return return_sb.toString();
	}
	
	
	protected void runComponent() {
		log("POPlanMonitor running...");
	}

}
