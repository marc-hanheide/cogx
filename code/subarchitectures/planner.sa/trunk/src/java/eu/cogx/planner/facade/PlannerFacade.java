/**
 * 
 */
package eu.cogx.planner.facade;

import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.FutureTask;

import autogen.Planner.Action;
import autogen.Planner.Completion;
import autogen.Planner.CppServer;
import autogen.Planner.CppServerPrx;
import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEntryQueue;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;
import castutils.experimentation.StopWatch;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * @author marc
 * 
 */
public class PlannerFacade {

	public static final String PLANNER_CPP_SERVER = "Planner";
	private static PlannerFacade singleton=null;
	
	public static synchronized PlannerFacade get(ManagedComponent component ) {
		if (singleton==null) {
			singleton = new PlannerFacade(component);
		}
		return singleton;
	}
	
	public class FuturePlanningTask extends
			FutureTask<WMEntryQueueElement<PlanningTask>> {

		public FuturePlanningTask(PlanningTask task) {
			super(new PlanningCallable(task));
		}

	}

	public class PlanningCallable implements
			Callable<WMEntryQueueElement<PlanningTask>> {
		final PlanningTask task;

		/**
		 * @param task
		 */
		public PlanningCallable(PlanningTask task) {
			super();
			this.task = task;
		}

		@Override
		public WMEntryQueueElement<PlanningTask> call() throws Exception {
			WMEntryQueue<PlanningTask> planQueue = new WMEntryQueue<PlanningTask>(
					component, PlanningTask.class);
			WMEntryQueueElement<PlanningTask> pt = null;

			// if we don't have anything to do... just quit.
			if (task.goals.length <= 0)
				return null;

			try {
				String id = component.newDataID();

				component.addChangeFilter(ChangeFilterFactory.createIDFilter(
						id, WorkingMemoryOperation.OVERWRITE), planQueue);
				component.addChangeFilter(ChangeFilterFactory.createIDFilter(
						id, WorkingMemoryOperation.DELETE), planQueue);
				// if (MEASURE_TIMES) {
				// component.addChangeFilter(ChangeFilterFactory.createIDFilter(
				// id, WorkingMemoryOperation.OVERWRITE), monitorTask);
				// component.addChangeFilter(ChangeFilterFactory.createIDFilter(
				// id, WorkingMemoryOperation.ADD), monitorTask);
				// }

				// start the stop watch
//				watch.tic();
				component.addToWorkingMemory(id, task);

				// wait for the plan to be generated
				boolean continueWaiting = true;
				while (continueWaiting) {
					component.log("waiting for planner to answer");
					pt = planQueue.take();
					PlanningTask taskEntry = (PlanningTask) pt.getEntry();
					if (pt.getEntry() == null) {
						pt = null;
						component
								.println("the Planning task has been removed before we actually received a valid plan...");
						break;
					}
					switch (taskEntry.planningStatus) {
					case SUCCEEDED:
//						watch.toc("plan succeded");
						component.log("we have a plan right now: "
								+ ((PlanningTask) pt.getEntry()).firstActionID);
						// stop waiting for further changes
						continueWaiting = false;
						break;
					case ABORTED:
					case FAILED:
//						watch.toc("plan failed");
						component
								.log("could not generate a plan... we have to abort for this time and remove the listener");
						component.removeChangeFilter(planQueue);
						planQueue = null;
						try {
							component
									.deleteFromWorkingMemory(pt.getEvent().address);
						} catch (CASTException e) {
							component.println("CASTException: " + e.message);
						}
						pt = null;
						// stop waiting for further changes
						continueWaiting = false;
						break;
					default:
						component
								.log("still planning... continue waiting with status "
										+ taskEntry.planningStatus.name());
					}
				}

			} catch (SubarchitectureComponentException e) {
				e.printStackTrace();
			} finally {
				try {
					if (planQueue != null) {
						component.log("remove listener");
						component.removeChangeFilter(planQueue);
					}
				} catch (SubarchitectureComponentException e) {
					component.println("SubarchitectureComponentException");
					e.printStackTrace();
				}
			}
			return pt;
		}

	}

	/**
	 * Create task with non-crashy default values.
	 * 
	 * @return
	 */
	public static PlanningTask newPlanningTask() {
		return new PlanningTask(0, null, false, new Action[0], 0, "",
                                Completion.PENDING, 0, Completion.PENDING, 0, new dBelief[0]);
	}

	public static PlanningTask newPlanningTask(List<Goal> goals) {
		PlanningTask plan = newPlanningTask();
		plan.goals = new Goal[goals.size()];
		int count = 0;
		for (Goal m : goals) {
			plan.goals[count++] = m;
		}
		return plan;
	}

	/**
	 * set to true if monitors should be registered for each PlanningTask to
	 * measure planning time.
	 */
	// private static final boolean MEASURE_TIMES = true;

	private final ManagedComponent component;

//	private final StopWatch watch = new StopWatch("plannerStopWatch");;

	private final static ExecutorService executorService = Executors
			.newCachedThreadPool();

	CppServerPrx cppServer = null;

	/**
	 * @param motives
	 */
	public PlannerFacade(final ManagedComponent component) {
		super();
		this.component = component;

		// taskWatches = new HashMap<String, StopWatch>();
		//
		// // We monitor planning tasks to measure planning time
		// monitorTask = new WorkingMemoryChangeReceiver() {
		//
		// @Override
		// public void workingMemoryChanged(WorkingMemoryChange wmc)
		// throws CASTException {
		// PlanningTask plan = component.getMemoryEntry(wmc.address,
		// PlanningTask.class);
		// StopWatch watch = taskWatches.get(wmc.address.id);
		// if (watch == null) {
		// watch = new StopWatch("PlanningTask." + wmc.address.id);
		// taskWatches.put(wmc.address.id, watch);
		// }
		// watch.info(CASTUtils.toString(wmc));
		// watch.info("STATUS = " + plan.planningStatus.toString());
		// switch (wmc.operation) {
		// case ADD:
		// watch.tic();
		// break;
		// case OVERWRITE:
		// switch (plan.planningStatus) {
		// case ABORTED:
		// case FAILED:
		// case SUCCEEDED:
		// if (watch.isRunning())
		// watch.toc(plan.planningStatus.toString() + " / "
		// + plan.executionStatus.toString() + "("
		// + plan.planningRetries + ")");
		// else
		// component.log("watch is not running");
		// break;
		// default:
		// if (!watch.isRunning())
		// watch.tic();
		// }
		// break;
		// case DELETE:
		// watch.toc("DELETE");
		// break;
		// }
		//
		// }
		// };
	}

	public boolean isGoalAchieved(String goalString) {
		if (cppServer == null) {
			try {
				cppServer = component.getIceServer(PLANNER_CPP_SERVER,
						CppServer.class, CppServerPrx.class);
			} catch (CASTException e) {
				component.logException("failed to contact "
						+ PLANNER_CPP_SERVER + ": ", e);
				return false;
			}
		}
		return cppServer.queryGoal(goalString);
	}

	public Future<WMEntryQueueElement<PlanningTask>> plan(List<Goal> goals) {
		return plan(newPlanningTask(goals));
	}

	public Future<WMEntryQueueElement<PlanningTask>> plan(List<Goal> goals,
			boolean execute) {
		PlanningTask task = newPlanningTask(goals);
		task.executePlan = execute;
		return plan(task);
	}

	public Future<WMEntryQueueElement<PlanningTask>> plan(PlanningTask task) {
		FuturePlanningTask futurePlan = new FuturePlanningTask(task);
		executorService.execute(futurePlan);
		return futurePlan;
	}
}
