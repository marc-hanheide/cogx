/**
 * 
 */
package castutils.facades;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.Callable;

import autogen.Planner.Completion;
import autogen.Planner.PlanningTask;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.StringValue;
import beliefmodels.autogen.featurecontent.IntegerValue;

import motivation.slice.CategorizePlaceMotive;
import motivation.slice.CategorizeRoomMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.GeneralGoalMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.slice.PatrolMotive;
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
public class PlannerFacade implements Callable<WMEntryQueueElement> {

	/**
	 * set to true if monitors should be registered for each PlanningTask to
	 * measure planning time.
	 */
//	private static final boolean MEASURE_TIMES = true;

	public static class GoalTranslator {

		public static String motive2PlannerGoal(HomingMotive m,
				String placeUnion, String robotUnion) {
			return new String("(located '" + robotUnion + "' '" + placeUnion
					+ "')");
		}

		public static String motive2PlannerGoal(PatrolMotive m,
				String placeUnion, String robotUnion) {
			return new String("(located '" + robotUnion + "' '" + placeUnion
					+ "')");
		}

		public static String motive2PlannerGoal(CategorizeRoomMotive m,
				String roomUnion, String robotUnion) {
			return "(kval '" + robotUnion + "' (areaclass '" + roomUnion
					+ "'))";
		}

		public static String motive2PlannerGoal(GeneralGoalMotive m,
				String robotUnion) {
			return m.internalGoal.replace("@R", robotUnion);
		}

		
		public static String motive2PlannerGoal(ExploreMotive m,
				String placeUnion) {
			// String placeStr = Long.toString(m.placeID);
			// return "(exists (?p - place)  (and (= (place_id ?p) place_id_"
			// + placeUnion + ") (= (explored ?p) true)))";
			return "(= (explored '" + placeUnion + "') true)";
			// return new String ("(explored place_id_" + m.placeID+")");
		}

		public static String motive2PlannerGoal(CategorizePlaceMotive m,
				String placeUnion, String robotUnion) {
			return "(kval '" + robotUnion + "' (place_category '" + placeUnion
					+ "'))";

		}
	}


	Map<String, FeatureValue> unionsWithPlaceID;
	Map<String, FeatureValue> unionsWithRoomId;
//	Map<String, StopWatch> taskWatches;

	private BinderFacade binderFacade;

	/**
	 * @param motives
	 */
	public PlannerFacade(final ManagedComponent component,
			BinderFacade binderFacade) {
		super();
		this.component = component;
		this.binderFacade = binderFacade;
		watch = new StopWatch("plannerStopWatch");
//		taskWatches = new HashMap<String, StopWatch>();
//
//		// We monitor planning tasks to measure planning time
//		monitorTask = new WorkingMemoryChangeReceiver() {
//
//			@Override
//			public void workingMemoryChanged(WorkingMemoryChange wmc)
//					throws CASTException {
//				PlanningTask plan = component.getMemoryEntry(wmc.address,
//						PlanningTask.class);
//				StopWatch watch = taskWatches.get(wmc.address.id);
//				if (watch == null) {
//					watch = new StopWatch("PlanningTask." + wmc.address.id);
//					taskWatches.put(wmc.address.id, watch);
//				}
//				watch.info(CASTUtils.toString(wmc));
//				watch.info("STATUS = " + plan.planningStatus.toString());
//				switch (wmc.operation) {
//				case ADD:
//					watch.tic();
//					break;
//				case OVERWRITE:
//					switch (plan.planningStatus) {
//					case ABORTED:
//					case FAILED:
//					case SUCCEEDED:
//						if (watch.isRunning())
//							watch.toc(plan.planningStatus.toString() + " / "
//									+ plan.executionStatus.toString() + "("
//									+ plan.planningRetries + ")");
//						else
//							component.log("watch is not running");
//						break;
//					default:
//						if (!watch.isRunning())
//							watch.tic();
//					}
//					break;
//				case DELETE:
//					watch.toc("DELETE");
//					break;
//				}
//
//			}
//		};
	}

	List<Motive> motives;
	private ManagedComponent component;
	private String agentUnionID = null;
	private StopWatch watch;

	/**
	 * The monitorTask is used to measure planning time
	 * 
	 */
//	private WorkingMemoryChangeReceiver monitorTask;

	public void setGoalMotives(List<Motive> m) {
		motives = new LinkedList<Motive>(m);
	}

	@Override
	public WMEntryQueueElement call() throws Exception {
		unionsWithPlaceID = binderFacade.findFeaturesInUnion("place_id");
		unionsWithRoomId = binderFacade.findFeaturesInUnion("roomID");

		return generatePlan(motives);
	}

	/**
	 * Create task with non-crashy default values.
	 * 
	 * @return
	 */
	private PlanningTask newPlanningTask() {
		return new PlanningTask(0, null, null, null, null, Completion.PENDING,
				0, Completion.PENDING, 0);
	}

	public PlanningTask generatePlanningTask(List<Motive> activeMotives)
			throws UnknownSubarchitectureException {
		PlanningTask plan = newPlanningTask();

		// create a conjunction of motives
		String goalString = "(and ";
		int countGoals = 0;
		for (Motive m : activeMotives) {
			String conjunctiveGoal = null;
			if (m instanceof ExploreMotive) {
				component.log("plan for ExploreMotive");
				ExploreMotive em = (ExploreMotive) m;
				em.correspondingUnion = resolveMotive(em);
				if (em.correspondingUnion != null) {
					component
							.log("found a corresponding union "
									+ em.correspondingUnion
									+ "/"
									+ binderFacade
											.getBelief(em.correspondingUnion).id);

					conjunctiveGoal = GoalTranslator
							.motive2PlannerGoal(em, binderFacade
									.getBelief(em.correspondingUnion).id);
				}
			} else if (m instanceof CategorizeRoomMotive) {
				CategorizeRoomMotive crm = (CategorizeRoomMotive) m;
				crm.correspondingUnion = resolveMotive(crm);
				if (crm.correspondingUnion != null)
					conjunctiveGoal = GoalTranslator
							.motive2PlannerGoal(crm, binderFacade
									.getBelief(crm.correspondingUnion).id,
									getAgentUnion());
			} else if (m instanceof HomingMotive) {
				HomingMotive crm = (HomingMotive) m;
				crm.correspondingUnion = resolveMotive(crm);
				conjunctiveGoal = GoalTranslator.motive2PlannerGoal(crm,
						binderFacade.getBelief(crm.correspondingUnion).id,
						getAgentUnion());
			} else if (m instanceof GeneralGoalMotive) {
				GeneralGoalMotive ggm = (GeneralGoalMotive) m;
				conjunctiveGoal = GoalTranslator.motive2PlannerGoal(ggm,
						getAgentUnion());
			} else if (m instanceof PatrolMotive) {
				PatrolMotive pm = (PatrolMotive) m;
				pm.correspondingUnion = resolveMotive(pm);
				conjunctiveGoal = GoalTranslator.motive2PlannerGoal(pm,
						binderFacade.getBelief(pm.correspondingUnion).id,
						getAgentUnion());
			} else if (m instanceof CategorizePlaceMotive) {
				conjunctiveGoal = GoalTranslator.motive2PlannerGoal(
						(CategorizePlaceMotive) m, binderFacade
								.getBelief(m.correspondingUnion).id,
						getAgentUnion());
			}
			if (conjunctiveGoal != null) {
				if (conjunctiveGoal.length() > 0) {
					countGoals++;
					goalString = goalString + conjunctiveGoal;
				}
			}

		}

		if (countGoals > 0) {
			goalString = goalString + ")";
			component.log("generated goal string: " + goalString);
			plan.goal = goalString;
			return plan;
		} else {
			return null; // generated empty plan
		}

	}

	/**
	 * resolves references in all the motives to prepare for planning
	 * 
	 * @throws CASTException
	 */
	String resolveMotive(Motive m) {
		component.log("resolve references in motives");
		// resolve place_ids for union ids
		if (m instanceof ExploreMotive) { // resolve
			// place_ids for
			// union ids
			ExploreMotive em = (ExploreMotive) m;
			component.log("resolving place_id=" + em.placeID
					+ " in ExploreMotive against " + unionsWithPlaceID.size()
					+ " unions that have a place_id");
			// search the unions for correct placeID
			for (Entry<String, FeatureValue> entry : unionsWithPlaceID
					.entrySet()) {
				StringValue placeID = (StringValue) entry.getValue();
				if (Integer.parseInt(placeID.val) == em.placeID) {
					component.log("found a corresponding union for place_id "
							+ em.placeID + ": " + entry.getKey());
					// TODO: we hope, that there is only ONE union with this
					// place_id, so let's break
					return entry.getKey();
				}
			}
		} else if (m instanceof CategorizeRoomMotive) {
			// place_ids for
			// union ids
			CategorizeRoomMotive crm = (CategorizeRoomMotive) m;
			component.log("resolving roomId=" + crm.roomId
					+ " in CategorizeRoomMotive against "
					+ unionsWithRoomId.size() + " unions that have a roomId");
			// search the unions for correct placeID
			for (Entry<String, FeatureValue> entry : unionsWithRoomId
					.entrySet()) {
				IntegerValue roomID = (IntegerValue) entry.getValue();
				if (roomID.val == crm.roomId) {
					component.log("found a corresponding union for roomId "
							+ crm.roomId + ": " + entry.getKey());
					// TODO: we hope, that there is only ONE union with this
					// roomId, so let's break
					return entry.getKey();
				}
			}
		} else if (m instanceof CategorizePlaceMotive) {
			// place_ids for
			// union ids
			CategorizePlaceMotive cpm = (CategorizePlaceMotive) m;
			component.log("resolving roomId=" + cpm.placeID
					+ " in CategorizeRoomMotive against "
					+ unionsWithRoomId.size() + " unions that have a roomId");
			// search the unions for correct placeID
			for (Entry<String, FeatureValue> entry : unionsWithPlaceID
					.entrySet()) {
				int roomID = Integer
						.parseInt(((StringValue) entry.getValue()).val);
				if (roomID == cpm.placeID) {
					component.log("found a corresponding union for roomId "
							+ cpm.placeID + ": " + entry.getKey());
					// TODO: we hope, that there is only ONE union with this
					// roomId, so let's break
					return entry.getKey();
				}
			}

		} else if (m instanceof HomingMotive) {
			// place_ids for
			// union ids
			for (Entry<String, FeatureValue> entry : unionsWithPlaceID
					.entrySet()) {
				int placeID = Integer
						.parseInt(((StringValue) entry.getValue()).val);
				if (placeID == 0) {
					component.log("found a corresponding union for placeId "
							+ 0 + ": " + entry.getKey());
					// TODO: we hope, that there is only ONE union with this
					// placeId, so let's break
					return entry.getKey();
				}
			}

		} else if (m instanceof PatrolMotive) {
			// place_ids for
			// union ids
			for (Entry<String, FeatureValue> entry : unionsWithPlaceID
					.entrySet()) {
				int placeID = Integer
						.parseInt(((StringValue) entry.getValue()).val);
				if (placeID == ((PatrolMotive) m).placeID) {
					component.log("found a corresponding union for placeID "
							+ placeID + ": " + entry.getKey());
					// TODO: we hope, that there is only ONE union with this
					// placeId, so let's break
					return entry.getKey();
				}
			}

		}

		return null;
	}

	/**
	 * try to get the agent's union ID (does some rather brute force search)
	 * 
	 * @return the agent's union ID
	 */
	private String getAgentUnion() {
		if (agentUnionID != null)
			return agentUnionID;
		else {
			Map<String, FeatureValue> features = binderFacade
					.findFeaturesInUnion("category");
			for (Entry<String, FeatureValue> entry : features.entrySet()) {
				component.log("  finding right values...");
				if (entry.getValue() instanceof StringValue) {

					String value = ((StringValue) entry.getValue()).val;
					component.log("  check String value " + value);
					if (value.equals("robot")) {
						component.log("  found agent: " + entry.getKey());
						agentUnionID = entry.getKey();
						return agentUnionID;
					}
				}
			}
			// for (Union u : unions) {
			// for (Feature f : u.features) {
			// if (f.featlabel.equals("category")) {
			// if (f.alternativeValues.length > 0)
			// if (((StringValue) f.alternativeValues[0]).val
			// .equals("robot")) {
			// agentUnion = u;
			// return agentUnion;
			// }
			// }
			// }
			// }
		}
		return null;

	}

	/**
	 * @throws InterruptedException
	 * @throws InterruptedException
	 * 
	 */
	private synchronized WMEntryQueueElement generatePlan(
			List<Motive> activeMotives) throws InterruptedException {
		WMEntryQueue planQueue = new WMEntryQueue(component);
		WMEntryQueueElement pt = null;

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
//			if (MEASURE_TIMES) {
//				component.addChangeFilter(ChangeFilterFactory.createIDFilter(
//						id, WorkingMemoryOperation.OVERWRITE), monitorTask);
//				component.addChangeFilter(ChangeFilterFactory.createIDFilter(
//						id, WorkingMemoryOperation.ADD), monitorTask);
//			}

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
					component.log("we have a plan right now: "
							+ ((PlanningTask) pt.getEntry()).goal);
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
}
