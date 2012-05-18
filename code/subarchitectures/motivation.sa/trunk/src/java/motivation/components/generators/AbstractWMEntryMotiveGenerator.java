/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.generators;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import autogen.Planner.Completion;
import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;
import castutils.castextensions.WMEventQueue;
import eu.cogx.planner.facade.PlannerFacade;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
public abstract class AbstractWMEntryMotiveGenerator<M extends Motive, T extends Ice.Object>
		extends AbstractEpistemicObjectMotiveGenerator<M, T> {

	WMEventQueue wmcQueue = new WMEventQueue();
	List<M> newAdditions;

	/**
	 * 
	 */
	protected AbstractWMEntryMotiveGenerator(Class<M> motiveClass,
			Class<T> beliefClass) {
		super(motiveClass, beliefClass);
		newAdditions = new ArrayList<M>();
	}

	protected boolean processEntry(T _entry) {
		return true;
	}

	protected String conjoinGoalStrings(String[] _goalStrings) {
		StringBuilder sb = new StringBuilder("(and ");
		for (String goalString : _goalStrings) {
			sb.append(goalString);
		}
		sb.append(")");
		return sb.toString();
	}

	public void beliefChanged(WorkingMemoryChange wmc) throws CASTException {
		switch (wmc.operation) {
		case ADD: {
			T newEntry = getMemoryEntry(wmc.address, epistemicClass);
			// if we don't deal with this type, bugger off
			if (!processEntry(newEntry)) {
				break;
			}
			log("relevant ADD event: " + CASTUtils.toString(wmc));

			checkForAdditions(wmc, newEntry);

			break;
		}
		case OVERWRITE: {
			T newEntry = getMemoryEntry(wmc.address, epistemicClass);
			// if we don't deal with this type, bugger off
			if (!processEntry(newEntry)) {
				break;
			}
			log("relevant OVERWRITE event: " + CASTUtils.toString(wmc));
			List<WorkingMemoryAddress> correspondingWMAs = bel2motivesMap
					.get(wmc.address);
			if (correspondingWMAs == null) {
				checkForAdditions(wmc, newEntry);
			} else {

				// need an explicit iterator to allow removal
				for (Iterator<WorkingMemoryAddress> i = correspondingWMAs
						.iterator(); i.hasNext();) {
					WorkingMemoryAddress correspondingWMA = i.next();
					try {
						getLogger().debug(
								"AbstractBeliefMotiveGenerator: locking and reading corresponding motive "
										+ correspondingWMA.id);
						lockEntry(correspondingWMA,
								WorkingMemoryPermissions.LOCKEDOD);
						M motive = getMemoryEntry(correspondingWMA, motiveClass);
						getLogger().debug(
								"AbstractBeliefMotiveGenerator: motive read, status is "
										+ motive.status.name());

						motive = checkForUpdate(newEntry, motive);

						if (motive == null) {
							deleteFromWorkingMemory(correspondingWMA);
							// remove the motive from the list
							i.remove();
							if (bel2motivesMap.get(wmc.address).isEmpty()) {
								bel2motivesMap.remove(wmc.address);
								log("removing belief ref from map");

							}

							println("bel2mm size: " + bel2motivesMap.size());
							for (Entry<WorkingMemoryAddress, List<WorkingMemoryAddress>> workingMemoryAddress : bel2motivesMap
									.entrySet()) {
								println("bel2mm size: "
										+ workingMemoryAddress.getValue()
												.size());
							}

						} else {
							assignCosts(motive);
							overwriteWorkingMemory(correspondingWMA, motive);
						}

					} catch (CASTException e) {
						getLogger()
								.warn("this is probably safe, particularly after a goal completion",
										e);
					} finally {
						if (holdsLock(correspondingWMA.id,
								correspondingWMA.subarchitecture))
							try {
								unlockEntry(correspondingWMA);
							} catch (CASTException e) {
								getLogger()
										.warn("when unlocking an object, can be ignored",
												e);
							}
					}
				}
			}
			break;
		}
		case DELETE: {
			List<WorkingMemoryAddress> correspondingWMAs = bel2motivesMap
					.get(wmc.address);
			if (correspondingWMAs != null) {
				for (WorkingMemoryAddress correspondingWMA : correspondingWMAs) {
					if (correspondingWMA != null) {
						log("relevant DELETE event: " + CASTUtils.toString(wmc)
								+ " causing DELETION of "
								+ CASTUtils.toString(correspondingWMA));
						try {
							lockEntry(correspondingWMA,
									WorkingMemoryPermissions.LOCKEDOD);
							deleteFromWorkingMemory(correspondingWMA);

						} catch (CASTException e) {
							logException(e);
							throw (e);
						} finally {
							if (holdsLock(correspondingWMA.id,
									correspondingWMA.subarchitecture))
								try {
									unlockEntry(correspondingWMA);
								} catch (CASTException e) {
									getLogger()
											.warn("when unlocking an object, can be ignored",
													e);
								}
						}
					}
				}

				bel2motivesMap.remove(wmc.address);
			} else {
				log("deleted belief had no corresponding motives, even though it did have earlier (i think)");
			}
			break;
		}
		}

	}

	/**
	 * Registered to return the motive after it was deleted, useful for cleanup
	 * on planner-determine successful cases.
	 * 
	 * @author nah
	 * 
	 */
	private class MotiveCompletionChangeReceiver implements
			WorkingMemoryChangeReceiver {

		@Override
		public void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {
			assert (_wmc.operation == WorkingMemoryOperation.OVERWRITE);
			try {
				lockEntry(_wmc.address, WorkingMemoryPermissions.LOCKEDOD);
				M motive = getMemoryEntry(_wmc.address, motiveClass);
				assert (CASTUtils.typeName(motive).equals(_wmc.type));
				if (motive.status == MotiveStatus.COMPLETED) {
					try {
						if (isMonitoringMotivesForReactivation()) {
							reactivateMotive(_wmc, motive);
						} else {
							motiveWasCompleted(motive);
						}
					} catch (SubarchitectureComponentException e) {
						logException(e);
					} finally {
						unlockEntry(_wmc.address);
					}
					if (!isMonitoringMotivesForReactivation())
						removeChangeFilter(this);
				}
			} finally {
				unlockEntry(_wmc.address);
			}
		}

	}

	/**
	 * Registered to return the motive after it was completed, useful for
	 * cleanup on planner-determine successful cases.
	 * 
	 * @param _motive
	 *            The motive that was completed
	 * @throws SubarchitectureComponentException
	 */
	protected void motiveWasCompleted(M _motive)
			throws SubarchitectureComponentException {
		log("goal " + _motive.goal.goalString + " completed.");
	}

	void reactivateMotive(WorkingMemoryChange _wmc, M motive) {
		log("reactivate goal " + motive.goal.goalString);
		motive.status = MotiveStatus.UNSURFACED;
		motive.priority = MotivePriority.UNSURFACE;
		try {
			overwriteWorkingMemory(_wmc.address, motive);
			log("goal " + motive.goal.goalString + " reactivated.");
		} catch (CASTException e) {
			logException(e);
		}
	}

	private boolean m_monitorMotivesForCompletion = false;
	private boolean m_monitorMotivesForReactivation = false;

	public boolean isMonitoringMotivesForCompletion() {
		return m_monitorMotivesForCompletion;
	}

	public boolean isMonitoringMotivesForReactivation() {
		return m_monitorMotivesForReactivation;
	}

	public void monitorMotivesForDeletion(boolean _monitorMotivesForDeletion) {
		this.m_monitorMotivesForCompletion = _monitorMotivesForDeletion;
	}

	public void reactivateCompleteMotives(boolean _monitorMotivesForReactivation) {
		println("reactivateCompleteMotives set to "
				+ _monitorMotivesForReactivation);
		this.m_monitorMotivesForReactivation = _monitorMotivesForReactivation;
	}

	private void checkForAdditions(WorkingMemoryChange wmc, T newEntry)
			throws AlreadyExistsOnWMException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {

		assert (newAdditions != null);
		assert (newAdditions.isEmpty());

		checkForAdditions(wmc.address, newEntry, newAdditions);

		if (!newAdditions.isEmpty()) {
			ArrayList<WorkingMemoryAddress> motiveAddresses = new ArrayList<WorkingMemoryAddress>(
					newAdditions.size());
			for (M motive : newAdditions) {
				if (motive != null) {
					motive.thisEntry = new WorkingMemoryAddress(newDataID(),
							getSubarchitectureID());
					assignCosts(motive);
					if (isMonitoringMotivesForCompletion()) {
						addChangeFilter(
								ChangeFilterFactory.createAddressFilter(
										motive.thisEntry,
										WorkingMemoryOperation.OVERWRITE),
								new MotiveCompletionChangeReceiver());
					}
					addToWorkingMemory(motive.thisEntry, motive);
					motiveAddresses.add(motive.thisEntry);
				}
			}

			if (!motiveAddresses.isEmpty()) {
				bel2motivesMap.put(wmc.address, motiveAddresses);
			}

			newAdditions.clear();
		}
	}

	protected void checkForAdditions(WorkingMemoryAddress addr, T newEntry,
			List<M> newAdditions) {
		newAdditions.add(checkForAddition(addr, newEntry));
	}

	protected abstract M checkForAddition(WorkingMemoryAddress addr, T newEntry);

	protected abstract M checkForUpdate(T newEntry, M existingMotive);

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
				lockComponent();
				getLogger().trace(
						"new change for "
								+ AbstractWMEntryMotiveGenerator.class
										.getSimpleName() + ": "
								+ CASTUtils.toString(wmc));
				beliefChanged(wmc);
			} catch (InterruptedException e) {
				logException(e);
			} catch (CASTException e) {
				logException(e);
			} finally {
				unlockComponent();
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
		Goal hardGoal = new Goal(-1, -1, motive.goal.goalString, false);
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
					bel2motivesMap.keySet());

			for (WorkingMemoryAddress wma : addresses) {
				WorkingMemoryChange wmc = new WorkingMemoryChange(
						WorkingMemoryOperation.OVERWRITE, getComponentID(),
						wma, CASTUtils.typeName(epistemicClass),
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
