/**
 * 
 */
package eu.cogx.planner.facade;

import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Callable;

import autogen.Planner.Action;
import autogen.Planner.Completion;
import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEntryQueue;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;
import castutils.experimentation.StopWatch;

/**
 * @author marc
 * 
 */
public class PlannerFacade implements Callable<WMEntryQueueElement<PlanningTask>> {

	/**
	 * set to true if monitors should be registered for each PlanningTask to
	 * measure planning time.
	 */
	// private static final boolean MEASURE_TIMES = true;

//	public static class GoalTranslator {
//
//		public static String motive2PlannerGoal(HomingMotive m,
//				String placeUnion, String robotUnion) {
//			return new String("(located '" + robotUnion + "' '" + placeUnion
//					+ "')");
//		}
//
//		public static String motive2PlannerGoal(PatrolMotive m,
//				String placeUnion, String robotUnion) {
//			return new String("(located '" + robotUnion + "' '" + placeUnion
//					+ "')");
//		}
//
//		public static String motive2PlannerGoal(CategorizeRoomMotive m,
//				String roomUnion, String robotUnion) {
//			return "(kval '" + robotUnion + "' (areaclass '" + roomUnion
//					+ "'))";
//		}
//
//		public static String motive2PlannerGoal(GeneralGoalMotive m) {
//			return m.internalGoal;
//		}
//
//		public static String motive2PlannerGoal(ExploreMotive m,
//				String placeUnion) {
//			// String placeStr = Long.toString(m.placeID);
//			// return "(exists (?p - place)  (and (= (place_id ?p) place_id_"
//			// + placeUnion + ") (= (explored ?p) true)))";
//			return "(= (explored '" + placeUnion + "') true)";
//			// return new String ("(explored place_id_" + m.placeID+")");
//		}
//
//		public static String motive2PlannerGoal(CategorizePlaceMotive m,
//				String placeUnion, String robotUnion) {
//			return "(kval '" + robotUnion + "' (place_category '" + placeUnion
//					+ "'))";
//
//		}
//	}

	List<Goal> motives;

	private ManagedComponent component;
	private StopWatch watch;

	private boolean executePlan=false;
	/**
	 * @param motives
	 */
	public PlannerFacade(final ManagedComponent component) {
		super();
		this.component = component;
		watch = new StopWatch("plannerStopWatch");
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

	@Override
	public WMEntryQueueElement<PlanningTask> call() throws Exception {
		return generatePlan(motives);
	}

	/**
	 * The monitorTask is used to measure planning time
	 * 
	 */
	// private WorkingMemoryChangeReceiver monitorTask;

	public void setGoals(Collection<Goal> m) {
		motives = new LinkedList<Goal>(m);
	}

	public void setExecutePlan(boolean b) {
		this.executePlan=b;
	}
	
	/**
	 * @throws InterruptedException
	 * @throws InterruptedException
	 * 
	 */
	private synchronized WMEntryQueueElement<PlanningTask> generatePlan(
			List<Goal> activeMotives) throws InterruptedException {
		WMEntryQueue<PlanningTask> planQueue = new WMEntryQueue<PlanningTask>(component, PlanningTask.class);
		WMEntryQueueElement<PlanningTask> pt = null;

		// if we don't have anything to do... just quit.
		if (activeMotives.size() < 0)
			return null;

		try {
			PlanningTask plan = generatePlanningTask(activeMotives);
			if (plan == null) {// we couldn't generate a proper goal... there is
				// nothing to be done
				component
						.println("the goal is empty... there is nothing to plan for");
				return null;
			}
			String id = component.newDataID();

			component.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
					WorkingMemoryOperation.OVERWRITE), planQueue);
			component.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
					WorkingMemoryOperation.DELETE), planQueue);
			// if (MEASURE_TIMES) {
			// component.addChangeFilter(ChangeFilterFactory.createIDFilter(
			// id, WorkingMemoryOperation.OVERWRITE), monitorTask);
			// component.addChangeFilter(ChangeFilterFactory.createIDFilter(
			// id, WorkingMemoryOperation.ADD), monitorTask);
			// }

			// start the stop watch
			watch.tic();
			component.addToWorkingMemory(id, plan);

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
					watch.toc("plan succeded");
					component
							.log("we have a plan right now: "
									+ ((PlanningTask) pt.getEntry()).firstActionID);
					// stop waiting for further changes
					continueWaiting = false;
					break;
				case ABORTED:
				case FAILED:
					watch.toc("plan failed");
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

	private PlanningTask generatePlanningTask(List<Goal> activeMotives)
			throws UnknownSubarchitectureException {
		PlanningTask plan = newPlanningTask();

		plan.goals = new Goal[activeMotives.size()];
		int count = 0; 
		for (Goal m : activeMotives) {
			plan.goals[count++] = m;
		}
		return plan;
	}

	/**
	 * Create task with non-crashy default values.
	 * 
	 * @return
	 */
	private PlanningTask newPlanningTask() {
		return new PlanningTask(0, null, executePlan, new Action[0], 0, "", Completion.PENDING,
				0, Completion.PENDING, 0);
	}
}
