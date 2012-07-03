package de.dfki.lt.tr.cast.dialogue;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;

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
import castutils.castextensions.IceXMLSerializer;
import de.dfki.lt.tr.cast.dialogue.planverb.PlanVerbalizer;
import de.dfki.lt.tr.cast.dialogue.util.POPlanUtils;
import de.dfki.lt.tr.cast.dialogue.util.VerbalisationUtils;
import de.dfki.lt.tr.dialogue.production.PlanVerbalizationRequest;
import eu.cogx.beliefs.slice.GroundedBelief;

public class POPlanMonitor extends ManagedComponent {

  private String fileName = "logs" + File.separator + "Verbalisations.txt";

	private Map<Integer,List<POPlan>> runningMap = new HashMap<Integer,List<POPlan>>();
	private Map<Integer,List<POPlan>> finishedMap = new HashMap<Integer,List<POPlan>>();
	private Map<Integer,PlanningTask> planningTaskMap = new HashMap<Integer,PlanningTask>();
    private Map<Integer,String> generatedReportsMap = new HashMap<Integer,String>();
	
	private Set<Integer> reportedTasks = new HashSet<Integer>();

	private PlanVerbalizer pevModule;
	
	private boolean saveGBHistoryFile = false;

    private boolean suppressVerbalisationUntilRequested;
	
	protected void configure(Map<String, String> args) {
		String pddldomain = "";
		String domainannotation = "";
		String grammarFile = "";
		String ngramFile = "";
		String hostname = "";
		Integer port = null;
		
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
		if (args.containsKey("--only-verbalise-on-request")) {
			suppressVerbalisationUntilRequested = true;
		} else {
			suppressVerbalisationUntilRequested = false;
		}
		if (args.containsKey("--saveGBHistoryFile")) {
			saveGBHistoryFile = true;
		} else {
			saveGBHistoryFile = false;
		}

		try {
			log("trying to create PEV Module with domainannotation=" + domainannotation + ", pddldomain=" + pddldomain + 
					", grammarpath=" + grammarFile + ", ngrampath=" + ngramFile + 
					", hostname=" + hostname + ", port=" + (port!=null ? port : "null") +
          ", " + (suppressVerbalisationUntilRequested ? "verbalisation on request only" : "") );
			pevModule = new PlanVerbalizer(domainannotation, pddldomain, grammarFile, ngramFile, hostname, port, this);
			log("created PEV Module");
		} catch (IOException e) {
			logException(e);
		}
		
    deleteVerbalisationsFile();
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
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(PlanVerbalizationRequest.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processPlanVerbRequest(_wmc);
			}
		});
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(GroundedBelief.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processAddedGroundedBelief(_wmc);
			}
		});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(GroundedBelief.class, WorkingMemoryOperation.OVERWRITE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processOverwrittenGroundedBelief(_wmc);
			}
		});
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(GroundedBelief.class, WorkingMemoryOperation.DELETE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processDeletedGroundedBelief(_wmc);
			}
		});
		
	}
	
	private void processPlanVerbRequest(WorkingMemoryChange _wmc) {
		PlanVerbalizationRequest _pevReq;
		try {
			_pevReq = getMemoryEntry(_wmc.address, PlanVerbalizationRequest.class);
		} catch (DoesNotExistOnWMException e) {
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		log("received ADD for PlanVerbalizationRequest: " + _pevReq.taskID);
		reportFinishedHistory(_pevReq.taskID, false);
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
			
			pevModule.m_gbmemory.addTimeStamp(_newPOPlan.taskID, runningMap.get(_newPOPlan.taskID).size()-1, _wmc.timestamp);
			log(pevModule.m_gbmemory.getTimeStampMap());
			log("added timestamp to GBeliefMemory: " + _newPOPlan.taskID + ", " + (runningMap.get(_newPOPlan.taskID).size()-1) + ", " + _wmc.timestamp);
			writeToFile();
		}
		else {
			finishedMap.put(_newPOPlan.taskID, new LinkedList<POPlan>());
			finishedMap.get(_newPOPlan.taskID).add(_newPOPlan);
			writeToFile();
//			String report = generateHistoryReport(_newPOPlan.taskID);
//			log("received ADD for FINISHED POPlan with taskID " + _newPOPlan.taskID + " -- verbalizing past history of this task so far: \n " + report);
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
			pevModule.m_gbmemory.addTimeStamp(_oldPOPlan.taskID, runningMap.get(_oldPOPlan.taskID).size()-1, _wmc.timestamp);
			log(pevModule.m_gbmemory.getTimeStampMap());
			log("added timestamp to GBeliefMemory: " + _oldPOPlan.taskID + ", " + (runningMap.get(_oldPOPlan.taskID).size()-1) + ", " + _wmc.timestamp);
			writeToFile();
		}
		else {
			finishedMap.get(_oldPOPlan.taskID).add(_oldPOPlan);
			writeToFile();
//			String report = generateHistoryReport(_oldPOPlan.taskID);
//			log("received OVERWRITE for FINISHED POPlan with taskID " + _oldPOPlan.taskID + " -- verbalizing past history of this task so far: \n " + report);
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
				VerbalisationUtils.verbaliseString(this, "I aborted my previous task.");
				log("PlanningTask " + _oldPlanningTask.id + " is ABORTED. Reporting verbally.");
				reportFinishedHistory(_oldPlanningTask.id, suppressVerbalisationUntilRequested);
				break;
			case FAILED:
				if (reportedTasks.contains(_oldPlanningTask.id)) break;
				reportedTasks.add(_oldPlanningTask.id);
				log("PlanningTask " + _oldPlanningTask.id + " is FAILED. Reporting verbally.");
				VerbalisationUtils.verbaliseString(this, "My task failed.");
				reportFinishedHistory(_oldPlanningTask.id, suppressVerbalisationUntilRequested);
				break;
			case SUCCEEDED:
				if (reportedTasks.contains(_oldPlanningTask.id)) break;
				reportedTasks.add(_oldPlanningTask.id);
				log("PlanningTask " + _oldPlanningTask.id + " is SUCCEEDED. Reporting verbally.");
				VerbalisationUtils.verbaliseString(this, "I finished my task successfully.");
				reportFinishedHistory(_oldPlanningTask.id, suppressVerbalisationUntilRequested);
				break;
			default:
				break;
		}
		
	}
	
	private void processAddedGroundedBelief(WorkingMemoryChange _wmc) {
		GroundedBelief _newGroundedBelief;
		try {
			_newGroundedBelief = getMemoryEntry(_wmc.address, GroundedBelief.class);
			
			pevModule.m_gbmemory.addGBelief(_wmc.address, getCASTTime(), _newGroundedBelief);
			
			writeToFile();
			
		} catch (DoesNotExistOnWMException e) {
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		//log("Received new GroundedBelief");
	}
	
	private void processOverwrittenGroundedBelief(WorkingMemoryChange _wmc) {
		GroundedBelief _oldGroundedBelief;
		try {
			_oldGroundedBelief = getMemoryEntry(_wmc.address, GroundedBelief.class);
			
			pevModule.m_gbmemory.addGBelief(_wmc.address, getCASTTime(), _oldGroundedBelief);
			
			writeToFile();
			
		} catch (DoesNotExistOnWMException e) {
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		//log("Received overwritten GroundedBelief");
	}
	
	private void processDeletedGroundedBelief(WorkingMemoryChange _wmc) {
		
		pevModule.m_gbmemory.addGBelief(_wmc.address, getCASTTime(), null);
		writeToFile();
		//log("Received deleted GroundedBelief");
	}
	
	/**
	 * Write the GBeliefMemory object to the given file
	 * 
	 */
	private void writeToFile() {
		if (saveGBHistoryFile) {
			log("Writing GBeliefHistory ...");
			
			File file;
		    file = new File("GBeliefHistory.xml");
		
		    try {
		    	String gbMemXMLString = IceXMLSerializer.toXMLString(pevModule.m_gbmemory);
		    	
		    	Writer out = new OutputStreamWriter(new FileOutputStream(file));
		        try {
		          out.write(gbMemXMLString);
		        }
		        finally {
		          out.close();
		        }			
		    } catch (FileNotFoundException e) {
		    	// TODO Auto-generated catch block
		    	e.printStackTrace();
		    }
		    catch (IOException e) {
		    	// TODO Auto-generated catch block
		    	e.printStackTrace();
		    }
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
		} else {
			// check if we know about the given taskID -- necessary when listening to external PEV Request triggers that might be illegal
			return "I am sorry. Task ID " + taskID + " is not known to me.";
		}

		// hand history over to PEV
		log("calling PEV Module verbalizeHistory()");
		return this.pevModule.verbalizeHistory(hlist, taskID);
	}

  // Does this method need to be synchronized to handle multiple requests at the same time as 
  // generation due to task completion? I think it is handled by the event queue.
	private void reportFinishedHistory(int taskID, boolean suppress_verbalisation) {
    String report = null;
    if (generatedReportsMap.containsKey(taskID)) {
      report = generatedReportsMap.get(taskID);
    }

    // If the PlanVerbalisationControllerGUI belonged to this component then we could
    // indicate in the GUI when a completed task was ready to verbalise (i.e. report generated)

    if (suppress_verbalisation == false && report == null && finishedMap.containsKey(taskID)) {
		  VerbalisationUtils.verbaliseString(this, "I will tell you what I did in a moment.");		
    }

    if (report == null) {		
      if (finishedMap.containsKey(taskID)) {     
        report = generateHistoryReport(taskID);
        // only store the report if the task has finished
        // might get a verbalisation request earlier
        PlanningTask planning_task = planningTaskMap.get(taskID);
        switch (planning_task.executionStatus) {
          case SUCCEEDED:
          case ABORTED:
          case FAILED:
           generatedReportsMap.put(taskID, report);
           break;
          case INPROGRESS:
          case PENDING:
          default:
           break;
        }
      } else {
        report = "I am sorry. Task ID " + taskID + " is not known to me.";
      }

      if (suppress_verbalisation == true) {
        log("GENERATED FINISHED HISTORY: \n" + report);
      }
    }
	
    if (suppress_verbalisation == false) {
      log("VERBALISING FINISHED HISTORY: \n" + report);
  		VerbalisationUtils.verbaliseString(this, report);

      writeVerbalisationToFile(taskID, report);
    }
	}

  private void deleteVerbalisationsFile() {
    try {
      File verbalisations_file = new File(fileName);
      if (verbalisations_file.exists()) {
        verbalisations_file.delete();
      }
    } catch (SecurityException e) {
      // ignore silently
    }
  }

  private void writeVerbalisationToFile(int taskID, String report) {
    try {
      BufferedWriter bw = new BufferedWriter(new FileWriter(fileName, true));
      bw.write(planningTaskToString(planningTaskMap.get(taskID)));
      bw.write("\n\n ****** Verbalisation ******\n");
      bw.write(report);
      bw.newLine();
      bw.newLine();
      bw.flush();
      bw.close();
    } catch (IOException e) {
      log("WARN: unable to write verbalisation to file");
    }
  }
	
//	private void reportFinishedPOPlan(POPlan pp) {
//		log("************ reportFinishedPOPlan() called ************");
//		
//		de.dfki.lt.tr.planverb.planning.pddl.POPlan pevPOPlan = POPlanUtils.convertPOPlan(pp);
//		
//		log("calling PEV Module verbalizePOPlan()");
//		String report = this.pevModule.verbalizePOPlan(pevPOPlan);
//		log("REPORTING FINISHED POPLAN: \n" + report);
//		//VerbalisationUtils.verbaliseString(this, report);
//	}
	
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

