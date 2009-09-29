/**
 * 
 */
package motivation.util.facades;

import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.Callable;

import motivation.slice.CategorizePlaceMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.util.WMEntryQueue;
import motivation.util.WMEntryQueue.WMEntryQueueElement;
import autogen.Planner.Completion;
import autogen.Planner.PlanningTask;
import binder.autogen.core.FeatureValue;
import binder.autogen.featvalues.StringValue;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 * 
 */
public class PlannerFacade implements Callable<WMEntryQueueElement> {

	public static class GoalTranslator {

		public static String motive2PlannerGoal(HomingMotive m) {
			// TODO: this has to be implemented with lookup to the unions
			return new String("");
		}

		public static String motive2PlannerGoal(ExploreMotive m) {
			String placeStr = Long.toString(m.placeID);
			return "(exists (?p - place)  (and (= (place_id ?p) place_id_"
					+ placeStr + ") (= (explored ?p) true)))";
			// return new String ("(explored place_id_" + m.placeID+")");
		}

		public static String motive2PlannerGoal(CategorizePlaceMotive m,
				String union) {
			String placeStr = Long.toString(m.placeID);
			return "(exists (?p - place)  (and (= (place_id ?p) place_id_"
					+ placeStr + ") (kval '" + union
					+ "' (place_category ?p))))";

		}
	}

	private BinderFacade binderFacade;

	/**
	 * @param motives
	 */
	public PlannerFacade(ManagedComponent component, BinderFacade binderFacade) {
		super();
		this.component = component;
		this.binderFacade = binderFacade;

	}

	List<Motive> motives;
	private ManagedComponent component;
	private String agentUnionID = null;

	public void setGoalMotives(List<Motive> m) {
		motives = m;
	}

	@Override
	public WMEntryQueueElement call() throws Exception {
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
		for (Motive m : activeMotives) {
			if (m instanceof ExploreMotive) {
				goalString = goalString
						+ GoalTranslator.motive2PlannerGoal((ExploreMotive) m);
			} else if (m instanceof HomingMotive) {
				goalString = goalString
						+ GoalTranslator.motive2PlannerGoal((HomingMotive) m);
			} else if (m instanceof CategorizePlaceMotive) {
				goalString = goalString
						+ GoalTranslator.motive2PlannerGoal(
								(CategorizePlaceMotive) m, getAgentUnion());
			}
		}

		goalString = goalString + ")";
		component.log("generated goal string: " + goalString);
		plan.goal = goalString;
		return plan;

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
			String id = component.newDataID();

			component.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
					WorkingMemoryOperation.OVERWRITE), planQueue);
			component.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
					WorkingMemoryOperation.DELETE), planQueue);
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
					component.log("we have a plan right now: "
							+ ((PlanningTask) pt.getEntry()).goal);
					// stop waiting for further changes
					continueWaiting = false;
					break;
				case ABORTED:
				case FAILED:
					component
							.log("could not generate a plan... we have to abort for this time and remove the listener");
					component.removeChangeFilter(planQueue);
					planQueue = null;
					component.deleteFromWorkingMemory(pt.getEvent().address);
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
