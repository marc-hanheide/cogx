/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.generators;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import motivation.slice.Motive;
import autogen.Planner.Completion;
import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;
import castutils.castextensions.WMEventQueue;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.planner.facade.PlannerFacade;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
public abstract class AbstractBeliefMotiveGenerator<M extends Motive, T extends dBelief>
		extends AbstractEpistemicObjectMotiveGenerator<M, T> {

	WMEventQueue wmcQueue = new WMEventQueue();
	String beliefType;

	/**
	 * 
	 */
	protected AbstractBeliefMotiveGenerator(String beliefType,
			Class<M> motiveClass, Class<T> beliefClass) {
		super(motiveClass, beliefClass);
		this.beliefType = beliefType;
	}

	public void beliefChanged(WorkingMemoryChange wmc) throws CASTException {
		M motive = null;
		switch (wmc.operation) {
		case ADD: {
			T newEntry = getMemoryEntry(wmc.address, epistemicClass);
			// if we don't deal with this type, bugger off
			if (!newEntry.type.equals(beliefType))
				break;
			log("relevant ADD event: " + CASTUtils.toString(wmc));
			motive = checkForAddition(wmc.address, newEntry);

			if (motive != null) {
				motive.thisEntry = new WorkingMemoryAddress(newDataID(),
						getSubarchitectureID());
				assignCosts(motive);
				addToWorkingMemory(motive.thisEntry, motive);
				bel2motiveMap.put(wmc.address, motive.thisEntry);
			}

			break;
		}
		case OVERWRITE: {
			T newEntry = getMemoryEntry(wmc.address, epistemicClass);
			// if we don't deal with this type, bugger off
			if (!newEntry.type.equals(beliefType))
				break;
			log("relevant OVERWRITE event: " + CASTUtils.toString(wmc));
			WorkingMemoryAddress correspondingWMA = bel2motiveMap
					.get(wmc.address);
			if (correspondingWMA == null) {
				motive = checkForAddition(wmc.address, newEntry);
				if (motive != null) {
					motive.thisEntry = new WorkingMemoryAddress(newDataID(),
							getSubarchitectureID());
					assignCosts(motive);
					addToWorkingMemory(motive.thisEntry, motive);
					bel2motiveMap.put(wmc.address, motive.thisEntry);
				}

			} else {
				try {
					getLogger().debug(
							"AbstractBeliefMotiveGenerator: locking and reading corresponding motive "
									+ correspondingWMA.id);
					lockEntry(correspondingWMA,
							WorkingMemoryPermissions.LOCKEDOD);
					motive = getMemoryEntry(correspondingWMA, motiveClass);
					getLogger().debug(
							"AbstractBeliefMotiveGenerator: motive read, status is "
									+ motive.status.name());

					motive = checkForUpdate(newEntry, motive);

					if (motive == null) {
						deleteFromWorkingMemory(correspondingWMA);
						bel2motiveMap.remove(wmc.address);
					} else {
						assignCosts(motive);
						overwriteWorkingMemory(correspondingWMA, motive);
					}

				} catch (CASTException e) {
					logException(e);
				} finally {
					if (holdsLock(correspondingWMA.id,
							correspondingWMA.subarchitecture))
						try {
							unlockEntry(correspondingWMA);
						} catch (CASTException e) {
							getLogger().warn(
									"when unlocking an object, can be ignored",
									e);
						}
				}
			}
			break;
		}
		case DELETE: {
			WorkingMemoryAddress correspondingWMA = bel2motiveMap
					.get(wmc.address);
			if (correspondingWMA != null) {
				log("relevant DELETE event: " + CASTUtils.toString(wmc)
						+ " causing DELETION of "
						+ CASTUtils.toString(correspondingWMA));
				try {
					lockEntry(correspondingWMA,
							WorkingMemoryPermissions.LOCKEDOD);
					deleteFromWorkingMemory(correspondingWMA);
					bel2motiveMap.remove(wmc.address);
				} catch (CASTException e) {
					logException(e);
					throw (e);
				} finally {
					if (holdsLock(correspondingWMA.id,
							correspondingWMA.subarchitecture))
						try {
							unlockEntry(correspondingWMA);
						} catch (CASTException e) {
							getLogger().warn(
									"when unlocking an object, can be ignored",
									e);
						}
				}
			}
			break;
		}
		}

	}

	protected abstract M checkForAddition(WorkingMemoryAddress addr, T newEntry);

	protected abstract M checkForUpdate(T newEntry, M motive);

	/*
	 * (non-Javadoc)
	 * 
	 * @see motivation.components.generators.AbstractMotiveGenerator#start()
	 */
	@Override
	protected void runComponent() {
		while (isRunning()) {
			try {
				WorkingMemoryChange wmc = wmcQueue.take();
				if (wmc.type.equals(CASTUtils.typeName(epistemicClass))) {
					// lockComponent();
					getLogger().trace(
							"new change for "
									+ AbstractBeliefMotiveGenerator.class
											.getSimpleName() + ": "
									+ CASTUtils.toString(wmc));
					beliefChanged(wmc);
				}
			} catch (InterruptedException e) {
				logException(e);
			} catch (CASTException e) {
				logException(e);
			} finally {
				// unlockComponent();
			}
		}
	}

	/**
	 * ask the planner to come up with costs just for this goal.
	 * 
	 * @param motive
	 * @return costs of the goal alone or Float.POSITIVE_INFINITY in case no
	 *         plan could be found.
	 */
	public float askPlannerForCosts(Motive motive) {
		PlannerFacade pf = PlannerFacade.get(this);
		List<Goal> goals = new ArrayList<Goal>(1);
		Goal hardGoal = new Goal(-1, motive.goal.goalString, false);
		goals.add(hardGoal);
		Future<WMEntryQueueElement<PlanningTask>> planningTask = pf.plan(goals,
				false);
		WMEntryQueueElement<PlanningTask> result;
		try {
			result = planningTask.get();
			if (result == null) {
				getLogger().warn(
						"failed to create a plan at all for goal: "
								+ motive.goal);
				return Float.POSITIVE_INFINITY;
			}
			if (result.getEntry().planningStatus != Completion.SUCCEEDED) {
				getLogger().warn(
						"planner returned with result "
								+ result.getEntry().planningStatus.name());
				return Float.POSITIVE_INFINITY;
			}

			return result.getEntry().costs;
		} catch (InterruptedException e) {
			logException(e);
		} catch (ExecutionException e) {
			logException(e);
		}
		return Float.POSITIVE_INFINITY;

	}

	@Override
	protected void start() {
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(epistemicClass),
				wmcQueue);
	}

	protected void recheckAllMotives() {
		try {
			// lockComponent();

			List<WorkingMemoryAddress> addresses = new ArrayList<WorkingMemoryAddress>(
					bel2motiveMap.keySet());

			for (WorkingMemoryAddress wma : addresses) {
				WorkingMemoryChange wmc = new WorkingMemoryChange(
						WorkingMemoryOperation.OVERWRITE,
						"SpatialFacade.PlaceChangedHandler", wma,
						CASTUtils.typeName(epistemicClass),
						new String[] { CASTUtils.typeName(epistemicClass) },
						getCASTTime());
				try {
					wmcQueue.put(wmc);
				} catch (InterruptedException e) {
					logException(e);
				}
			}
		} finally {
			// unlockComponent();
		}
	}

	/**
	 * assigns costs to the motive
	 * 
	 * @param motive
	 */
	protected void assignCosts(Motive motive) {
		motive.costs = -1;
		// TODO: we should ask the planner, which is not yet possible due to bug
		// https://codex.cs.bham.ac.uk/trac/cogx/ticket/246
		// motive.costs = askPlannerForCosts(motive);
		// log("asking planner for cost got me: " + motive.costs);
	}

}
