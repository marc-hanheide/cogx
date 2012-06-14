package de.dfki.lt.tr.cast.dialogue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;

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
	
	private Set<Integer> reportedTasks = new HashSet<Integer>();

	private PlanVerbalizer pevModule; 
	
	protected void configure(Map<String, String> args) {
		String pddldomain = "";
		String domainannotation = "";
		String grammarFile = "";
		String ngramFile = "";
		String hostname = "";
		Integer port = null;
		
		// TODO Auto-generated method stub
		super.configure(args);
		if (args.containsKey("--pddldomain")) {
			pddldomain = args.get("--pddldomain");
			log("pddldomain=" + pddldomain);
		}
		if (args.containsKey("--domainannotation")) {
			domainannotation = args.get("--domainannotation");
			log("domainannotation=" + domainannotation);
		}
		if (args.containsKey("--grammarpath")) {
			grammarFile = args.get("--grammarpath");
			log("grammarpath=" + grammarFile);
		}
		if (args.containsKey("--ngrampath")) {
			ngramFile = args.get("--ngrampath");
			log("ngrampath=" + ngramFile);
		}
		if (args.containsKey("--hostname")) {
			hostname = args.get("--hostname");
			log("hostname=" + hostname);
		}
		if (args.containsKey("--port")) {
			port = Integer.parseInt(args.get("--port"));
			log("port=" + port);
		}
		
		try {
			log("trying to create PEV Module with domainannotation=" + domainannotation + ", pddldomain=" + pddldomain + 
					", grammarpath=" + grammarFile + ", ngrampath=" + ngramFile + 
					", hostname=" + hostname + ", port=" + (port!=null ? port : "null") );
			pevModule = new PlanVerbalizer(domainannotation, pddldomain, grammarFile, ngramFile, hostname, port, this);
			log("created PEV Module");
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
		log("received ADD for POPlan: \n " + poplanToString(_newPOPlan));
		if (_newPOPlan.status.name().equals("RUNNING")) {
			runningMap.put(_newPOPlan.taskID, new LinkedList<POPlan>());
			runningMap.get(_newPOPlan.taskID).add(_newPOPlan);
		}
		else {
			finishedMap.put(_newPOPlan.taskID, new LinkedList<POPlan>());
			finishedMap.get(_newPOPlan.taskID).add(_newPOPlan);
			String report = generateHistoryReport(_newPOPlan.taskID);
			log("received ADD for FINISHED POPlan with taskID " + _newPOPlan.taskID + " -- verbalizing past history of this task so far: \n " + report);
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
		log("received OVERWRITE for POPlan: \n " + poplanToString(_oldPOPlan));
		
		if (_oldPOPlan.status.name().equals("RUNNING")) {
			runningMap.get(_oldPOPlan.taskID).add(_oldPOPlan);
		}
		else {
			finishedMap.get(_oldPOPlan.taskID).add(_oldPOPlan);
			String report = generateHistoryReport(_oldPOPlan.taskID);
			log("received OVERWRITE for FINISHED POPlan with taskID " + _oldPOPlan.taskID + " -- verbalizing past history of this task so far: \n " + report);
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
		
		planningTaskMap.put(_oldPlanningTask.id, _oldPlanningTask);
		
		StringBuilder log_sb = new StringBuilder();
		
		if (finishedMap.containsKey(_oldPlanningTask.id)) {
			log_sb.append("History:\n");
			log_sb.append(getHistory(_oldPlanningTask.id));
			log_sb.append("History End");
			log(log_sb.toString());
		}
		
		switch (_oldPlanningTask.executionStatus) {
			case INPROGRESS:
				log("PlanningTask " + _oldPlanningTask.id + " is still INPROGRESS. Not reporting verbally.");
				break;
			case PENDING:
				log("PlanningTask " + _oldPlanningTask.id + " is still PENDING. Not reporting verbally.");
				break;
			case ABORTED:
				if (reportedTasks.contains(_oldPlanningTask.id)) break;
				reportedTasks.add(_oldPlanningTask.id);
				log("PlanningTask " + _oldPlanningTask.id + " is ABORTED. Reporting verbally.");
				VerbalisationUtils.verbaliseString(this, "I aborted my previous task. I will tell you what I did in a moment.");
				reportFinishedHistory(_oldPlanningTask.id);
				break;
			case FAILED:
				if (reportedTasks.contains(_oldPlanningTask.id)) break;
				reportedTasks.add(_oldPlanningTask.id);
				log("PlanningTask " + _oldPlanningTask.id + " is FAILED. Reporting verbally.");
				VerbalisationUtils.verbaliseString(this, "My task failed. I will tell you what I did in a moment.");
				reportFinishedHistory(_oldPlanningTask.id);
				break;
			case SUCCEEDED:
				if (reportedTasks.contains(_oldPlanningTask.id)) break;
				reportedTasks.add(_oldPlanningTask.id);
				log("PlanningTask " + _oldPlanningTask.id + " is SUCCEEDED. Reporting verbally.");
				VerbalisationUtils.verbaliseString(this, "I finished my task successfully. I will tell you what I did in a moment.");
				reportFinishedHistory(_oldPlanningTask.id);
				break;
			default:
				break;
		}
		
	}
	
	private String generateHistoryReport(int taskID) {
		log("************ generateHistoryReport(" + taskID + ") called ************");

		// construct history in our POPlan representation (from planner.sa POPlan objects)
		List<de.dfki.lt.tr.planverb.planning.pddl.POPlan> hlist = new ArrayList<de.dfki.lt.tr.planverb.planning.pddl.POPlan>();
		if (finishedMap.containsKey(taskID)) {
			for (POPlan pp : finishedMap.get(taskID)) {
				de.dfki.lt.tr.planverb.planning.pddl.POPlan pevPOPlan = POPlanUtils.convertPOPlan(pp);
				hlist.add(pevPOPlan);
			}
		}

		// hand history over to PEV
		log("calling PEV Module verbalizeHistory()");
		return this.pevModule.verbalizeHistory(hlist);
	}

	private void reportFinishedHistory(int taskID) {
		String report = generateHistoryReport(taskID);
		log("REPORTING FINISHED HISTORY: \n" + report);
		VerbalisationUtils.verbaliseString(this, report);
	}

	
	private void reportFinishedPOPlan(POPlan pp) {
		log("************ reportFinishedPOPlan() called ************");
		
		de.dfki.lt.tr.planverb.planning.pddl.POPlan pevPOPlan = POPlanUtils.convertPOPlan(pp);
		
		log("calling PEV Module verbalizePOPlan()");
		String report = this.pevModule.verbalizePOPlan(pevPOPlan);
		log("REPORTING FINISHED POPLAN: \n" + report);
		//VerbalisationUtils.verbaliseString(this, report);
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

