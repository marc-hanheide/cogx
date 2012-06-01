package de.dfki.lt.tr.cast.dialogue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import jline.History;

import autogen.Planner.Action;
import autogen.Planner.Goal;
import autogen.Planner.POPlan;
import autogen.Planner.PlanningTask;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.cast.dialogue.planverb.PlanVerbalizer;
import de.dfki.lt.tr.cast.dialogue.util.POPlanUtils;
import de.dfki.lt.tr.cast.dialogue.util.VerbalisationUtils;
import de.dfki.lt.tr.planverb.history.Step;

public class POPlanMonitor extends ManagedComponent {

	private Map<Integer,List<POPlan>> runningMap = new HashMap<Integer,List<POPlan>>();
	private Map<Integer,List<POPlan>> finishedMap = new HashMap<Integer,List<POPlan>>();
	private Map<Integer,PlanningTask> planningTaskMap = new HashMap<Integer,PlanningTask>();

	private PlanVerbalizer pevModule; 
	
	protected void configure(Map<String, String> args) {
		String pddldomain = "";
		String grammarpath = "";
		String domainannotation = "";
		
		// TODO Auto-generated method stub
		super.configure(args);
		if (args.containsKey("--pddldomain")) {
			pddldomain = args.get("--pddldomain");
			log("pddldomain=" + pddldomain);
		}
		if (args.containsKey("--grammarpath")) {
			grammarpath = args.get("--grammarpath");
			log("grammarpath=" + grammarpath);
		}
		if (args.containsKey("--domainannotation")) {
			domainannotation = args.get("--domainannotation");
			log("domainannotation=" + domainannotation);
		}
		
		try {
			log("trying to create PEV Module with domainannotation=" + domainannotation + "pddldomain=" + pddldomain + "grammarpath=" + grammarpath);
			pevModule = new PlanVerbalizer(domainannotation, pddldomain, grammarpath, this);
			log("created PEV Module with domainannotation=" + domainannotation + "pddldomain=" + pddldomain + "grammarpath=" + grammarpath);
		} catch (IOException e) {
			logException(e);
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
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(PlanningTask.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processAddedPlanningTask(_wmc);
			}
		});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(PlanningTask.class, WorkingMemoryOperation.OVERWRITE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processOverwrittenPlanningTask(_wmc);
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
		// VerbalisationUtils.verbaliseString(this, "Got a new POPlan");
		if (_newPOPlan.status.name().equals("RUNNING")) {
			runningMap.put(_newPOPlan.taskID, new LinkedList<POPlan>());
			runningMap.get(_newPOPlan.taskID).add(_newPOPlan);
		}
		else {
			finishedMap.put(_newPOPlan.taskID, new LinkedList<POPlan>());
			finishedMap.get(_newPOPlan.taskID).add(_newPOPlan);
			//reportFinishedPOPlan(_newPOPlan);
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
		// VerbalisationUtils.verbaliseString(this, "Got an overwritten POPlan");
		
		
		if (_oldPOPlan.status.name().equals("RUNNING")) {
			runningMap.get(_oldPOPlan.taskID).add(_oldPOPlan);
		}
		else {
			finishedMap.get(_oldPOPlan.taskID).add(_oldPOPlan);
			//reportFinishedPOPlan(_oldPOPlan);
		}
	}
	
	private void processAddedPlanningTask(WorkingMemoryChange _wmc) {
		PlanningTask _newPlanningTask;
		try {
			_newPlanningTask = getMemoryEntry(_wmc.address, PlanningTask.class);
		} catch (DoesNotExistOnWMException e) {
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		log("ADDED PlanningTask: " + planningTaskToString(_newPlanningTask));
		// VerbalisationUtils.verbaliseString(this, "Got a new POPlan");
		
		planningTaskMap.put(_newPlanningTask.id, _newPlanningTask);
	}
	
	private void processOverwrittenPlanningTask(WorkingMemoryChange _wmc) {
		PlanningTask _oldPlanningTask;
		try {
			_oldPlanningTask = getMemoryEntry(_wmc.address, PlanningTask.class);
		} catch (DoesNotExistOnWMException e) {
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		log("OVERWRITTEN PlanningTask: " + planningTaskToString(_oldPlanningTask));
		// VerbalisationUtils.verbaliseString(this, "Got an overwritten POPlan");
		
		planningTaskMap.put(_oldPlanningTask.id, _oldPlanningTask);
		
		StringBuilder log_sb = new StringBuilder();
		
		if (finishedMap.containsKey(_oldPlanningTask.id)) {
			log_sb.append("History:\n");
			log_sb.append(getHistory(_oldPlanningTask.id));
			log_sb.append("History End");
			log(log_sb.toString());
		}
		
		if (_oldPlanningTask.executionStatus.toString().equals("SUCCEEDED")) {
			reportFinishedHistory(_oldPlanningTask.id);
		}
	}

	private void reportFinishedPOPlan(POPlan pp) {
		log("************ reportFinishedPOPlan() called ************");

		/*List<Step<String>> actionList = POPlanUtils.extractSteps(pp);
		
		List<String> linkList = POPlanUtils.extractLinks(pp);
		StringBuilder linksSection = new StringBuilder();
		for (String link : linkList) {
			linksSection.append(link + "\n");
		}
		
		// log("links: " + linksSection);
		// log("constructing a de.dfki.lt.tr.planverb.planning.pddl.POPlan from the autogen.Planner.POPlan");
		de.dfki.lt.tr.planverb.planning.pddl.POPlan pevPOPlan = 
				new de.dfki.lt.tr.planverb.planning.pddl.
				POPlan(new Integer(pp.taskID).toString(), actionList, linksSection.toString());*/
		
		de.dfki.lt.tr.planverb.planning.pddl.POPlan pevPOPlan = POPlanUtils.convertPOPlan(pp);
		
		log("calling PEV Module verbalizePOPlan()");
		String report = this.pevModule.verbalizePOPlan(pevPOPlan);
		log("REPORTING FINISHED POPLAN: \n" + report);
		//VerbalisationUtils.verbaliseString(this, report);
	}
	
	private void reportFinishedHistory(int taskID) {
		log("************ reportFinishedHistory() called ************");

		List<de.dfki.lt.tr.planverb.planning.pddl.POPlan> hlist = new ArrayList<de.dfki.lt.tr.planverb.planning.pddl.POPlan>();
		
		if (finishedMap.containsKey(taskID)) {
			for (POPlan pp : finishedMap.get(taskID)) {
				de.dfki.lt.tr.planverb.planning.pddl.POPlan pevPOPlan = POPlanUtils.convertPOPlan(pp);
				hlist.add(pevPOPlan);
			}
		}
		
		log("calling PEV Module verbalizeHistory()");
		String report = this.pevModule.verbalizeHistory(hlist);
		log("REPORTING FINISHED HISTORY: \n" + report);
		VerbalisationUtils.verbaliseString(this, report);
		
	}
	
	
	private String poplanToString(POPlan _poPlan) {
		return POPlanUtils.POPlanToString(_poPlan);
	}
	
	private String planningTaskToString(PlanningTask _planningTask) {
		return POPlanUtils.PlanningTaskToString(_planningTask);
	}
	
	private String getHistory(int id) {
		StringBuilder history_sb = new StringBuilder();
		
		for (POPlan _plan : finishedMap.get(id)) {
			history_sb.append(poplanToString(_plan));
		}
		
		return history_sb.toString();
	}
	
	
	protected void runComponent() {
		log("POPlanMonitor running...");
	}

}

/*
 * Behavior: 1. Goal: Goto-Place ( (= (is-in ROBOT) Place) ) : Adds a new task but no confirmation after task succeeded.
 * 			 2. Give a new goal while old one is still active: Adds a new task and only the newer task is overwritten, updated, etc.
 */

