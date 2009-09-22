/**
 * 
 */
package motivation.components.managers;

import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import motivation.components.managers.comparators.AgeComparator;
import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.slice.PlanProxy;
import motivation.util.GoalTranslator;
import motivation.util.WMEntryQueue;
import motivation.util.WMEntryQueue.QueueElement;
import autogen.Planner.Completion;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 * 
 */
public class PlanAllManager extends MotiveManager {

	Set<Motive> managedMotives;

	private int maxPlannedMotives;
	private Comparator<? super Motive> motiveComparator;

	/**
	 * @param specificType
	 */
	public PlanAllManager() {
		super(Motive.class);
		managedMotives = Collections.synchronizedSet(new HashSet<Motive>());
		// defaults
		motiveComparator = new AgeComparator();
		maxPlannedMotives = 3;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected void configure(Map<String, String> arg0) {
		// TODO Auto-generated method stub
		super.configure(arg0);

		String arg;
		// parsing stuff
		arg = arg0.get("--orderPolicy");
		if (arg != null) {
			String className = this.getClass().getPackage().getName()
					+ ".comparators." + arg.trim()+"Comparator";
			try {
				this.getClass().getPackage().getName();
				println("add type '" + className + "'");
				ClassLoader.getSystemClassLoader().loadClass(className);
				Class<Comparator<? super Motive>> cl = (Class<Comparator<? super Motive>>) Class
						.forName(className);
				motiveComparator = cl.newInstance();
			} catch (ClassNotFoundException e) {
				println("trying to register for a class that doesn't exist.");
				e.printStackTrace();
			} catch (InstantiationException e) {
				println("failed to get an instance.");
				e.printStackTrace();
			} catch (IllegalAccessException e) {
				println("failed to get an instance.");
				e.printStackTrace();
			}
		}

		arg = arg0.get("--maxPlannedMotives");
		if (arg != null) {
			maxPlannedMotives = Integer.parseInt(arg);
		}
		println("maxPlannedMotives is " + maxPlannedMotives);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		super.runComponent();
		try {
			while (isRunning()) {
				log("runComponent loop");
				if (managedMotives.size() > 0) { // if we have motives to manage
					// this waits until we have a plan or return null, if it
					// fails
					log("we have some motives that should be planned for");
					QueueElement pt = generatePlan();
					if (pt != null) { // if we got a plan...
						log("a plan has been generated. it's time to execute it");
						PlanProxy pp = new PlanProxy();
						pp.planAddress = pt.getEvent().address;
						executePlan(pp);
						log("execution finished... wait 1 sec to let state changes propagate");
					}
					sleepComponent(1000); // TODO: wait for motives to
					// update
				} else { // if we have no motives yet, we wait until something
					// happens
					waitForChanges();
				}
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

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

	/**
	 * @throws InterruptedException
	 * 
	 */
	private synchronized QueueElement generatePlan()
			throws InterruptedException {
		WMEntryQueue planQueue = new WMEntryQueue(this);
		QueueElement pt = null;

		// if we don't have anything to do... just quit.
		if (managedMotives.size() < 0)
			return null;

		try {
			String id = newDataID();
			PlanningTask plan = newPlanningTask();

			List<Motive> sortedMotives = new LinkedList<Motive>(managedMotives);
			// rank the motives according to the comparator
			Collections.sort(sortedMotives, motiveComparator);
			
			// create a conjunction of motives
			String goalString = "(and ";
			int remainingCapacity = maxPlannedMotives;
			for (Motive m : sortedMotives) {
				if (remainingCapacity <= 0) // we reached the limit of motives
					// to consider
					break;
				m.tries++;
				overwriteWorkingMemory(m.thisEntry, m);
				if (m instanceof ExploreMotive) {
					log("---> add goal to explore node "
							+ ((ExploreMotive) m).placeID);
					goalString = goalString
							+ GoalTranslator.motive2PlannerGoal(
									(ExploreMotive) m, getOriginMap());
					remainingCapacity--;
				} else if (m instanceof HomingMotive) {
					log("---> add goal to go home "
							+ ((HomingMotive) m).homePlaceID);
					goalString = goalString
							+ GoalTranslator.motive2PlannerGoal(
									(HomingMotive) m, getOriginMap());
					remainingCapacity--;
				}
			}

			goalString = goalString + ")";
			log("generated goal string: " + goalString);
			plan.goal = goalString;
			// plan.goal = "(forall (?p - place) (= (explored ?p) true))";

			addChangeFilter(ChangeFilterFactory.createIDFilter(id,
					WorkingMemoryOperation.OVERWRITE), planQueue);
			addChangeFilter(ChangeFilterFactory.createIDFilter(id,
					WorkingMemoryOperation.DELETE), planQueue);
			log("submitting plan to WM");
			addToWorkingMemory(id, plan);
			log("has been submitted");

			// wait for the plan to be generated
			boolean continueWaiting = true;
			while (continueWaiting) {
				log("waiting for planner to answer");
				pt = planQueue.take();
				PlanningTask taskEntry = (PlanningTask) pt.getEntry();
				if (pt.getEntry() == null) {
					pt = null;
					println("the Planning task has been removed before we actually received a valid plan...");
					break;
				}
				switch (taskEntry.planningStatus) {
				case SUCCEEDED:
					log("we have a plan right now: "
							+ ((PlanningTask) pt.getEntry()).goal);
					// stop waiting for further changes
					continueWaiting = false;
					break;
				case ABORTED:
				case FAILED:
					log("could not generate a plan... we have to abort for this time and remove the listener");
					removeChangeFilter(planQueue);
					planQueue = null;
					deleteFromWorkingMemory(pt.getEvent().address);
					pt = null;
					// stop waiting for further changes
					continueWaiting = false;
					break;
				default:
					log("still planning... continue waiting with status "
							+ taskEntry.planningStatus.name());
				}
			}

		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} finally {
			try {
				if (planQueue != null) {
					log("remove listener");
					removeChangeFilter(planQueue);
				}
			} catch (SubarchitectureComponentException e) {
				println("SubarchitectureComponentException");
				e.printStackTrace();
			}
		}
		return pt;
	}

	private void executePlan(PlanProxy pp) {
		String id = newDataID();
		WMEntryQueue planProxyQueue = new WMEntryQueue(this);
		addChangeFilter(ChangeFilterFactory.createIDFilter(id,
				WorkingMemoryOperation.DELETE), planProxyQueue);
		// submit the planProxy
		try {
			addToWorkingMemory(id, pp);

			// wait for the PlanProxy to be deleted
			// TODO: we should move to status information in here and not only
			// listen for deletion
			planProxyQueue.take();
			removeChangeFilter(planProxyQueue);
		} catch (CASTException e) {
			println("CASTException");
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.managers.MotiveManager#manageMotive(motivation.
	 * slice.Motive)
	 */
	@Override
	protected void manageMotive(Motive motive) {
		log("have a new motive to manage... type is "
				+ Motive.class.getSimpleName());
		// TODO: this is not the most efficient way to do it... should implement a kind of LinkedListSet someday
		
		// remove the motive before actually re-adding it. It's safe to do,
		// even if it is not in yet
		managedMotives.add(motive);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.managers.MotiveManager#retractMotive(motivation
	 * .slice.Motive)
	 */
	@Override
	protected void retractMotive(Motive motive) {
		managedMotives.remove(motive);

	}

}
