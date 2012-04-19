package de.dfki.lt.tr.cast.dialogue;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import autogen.Planner.POPlan;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.cast.dialogue.util.POPlanUtils;
import de.dfki.lt.tr.cast.dialogue.util.VerbalisationUtils;

public class POPlanMonitor extends ManagedComponent {

	private Map<Integer,List<POPlan>> runningMap = new HashMap<Integer,List<POPlan>>();
	private Map<Integer,List<POPlan>> finishedMap = new HashMap<Integer,List<POPlan>>();
	private String pddldomain = "";
	private String grammarpath = "";
	
	protected void configure(Map<String, String> args) {
		// TODO Auto-generated method stub
		super.configure(args);
		if (args.containsKey("--pddldomain")) {
			pddldomain = args.get("--pddldomain");
			log(pddldomain);
		}
		if (args.containsKey("--grammarpath")) {
			grammarpath = args.get("--grammarpath");
			log(grammarpath);
		}
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
		VerbalisationUtils.verbaliseString(this, "Got a new POPlan");
		if (_newPOPlan.status.name().equals("RUNNING")) {
			runningMap.put(_newPOPlan.taskID, new LinkedList<POPlan>());
			runningMap.get(_newPOPlan.taskID).add(_newPOPlan);
		}
		else {
			finishedMap.put(_newPOPlan.taskID, new LinkedList<POPlan>());
			finishedMap.get(_newPOPlan.taskID).add(_newPOPlan);
		}
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
		VerbalisationUtils.verbaliseString(this, "Got an overwritten POPlan");
		
		if (_oldPOPlan.status.name().equals("RUNNING")) {
			runningMap.get(_oldPOPlan.taskID).add(_oldPOPlan);
		}
		else {
			finishedMap.get(_oldPOPlan.taskID).add(_oldPOPlan);
		}
		
		/*for (Map.Entry<Integer, List<POPlan>> entry : runningMap.entrySet()) {
			for (POPlan _plan : entry.getValue()) {
				log(entry.getKey() + ":" + poplanToString(_plan));
			}
		}
		
		for (Map.Entry<Integer, List<POPlan>> entry : finishedMap.entrySet()) {
			for (POPlan _plan : entry.getValue()) {
				log(entry.getKey() + ":" + poplanToString(_plan));
			}
		}*/
	}

	private String poplanToString(POPlan _poPlan) {

		return POPlanUtils.POPlanToString(_poPlan);
	}
	
	
	protected void runComponent() {
		log("POPlanMonitor running...");
	}

}

/*
 * Behavior: 1. Goal: Goto-Place ( (= (is-in ROBOT) Place) ) : Adds a new task, does what it should and that's it. No confirmation after task succeeded.
 * 			 2. Give a new Goal while old one is still active: Adds a new Task and only the newer task is overwritten, updated, etc.
 */

