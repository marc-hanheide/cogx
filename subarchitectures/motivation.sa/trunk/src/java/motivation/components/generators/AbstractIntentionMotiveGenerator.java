/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.generators;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import motivation.slice.Motive;
import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.planner.facade.PlannerFacade;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public abstract class AbstractIntentionMotiveGenerator<M extends Motive, T extends Intention>
		extends AbstractEpistemicObjectMotiveGenerator<M, T> implements
		WorkingMemoryChangeReceiver {

	final Map<WorkingMemoryAddress, WorkingMemoryAddress> intent2motiveMap = new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>();
	private WorkingMemoryAddress robotBeliefAddr = null;

	/**
	 * 
	 */
	protected AbstractIntentionMotiveGenerator(Class<M> motiveClass,
			Class<T> intentClass) {
		super(motiveClass, intentClass);
	}

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(Intention.class,
				WorkingMemoryOperation.ADD), this);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * cast.architecture.WorkingMemoryChangeReceiver#workingMemoryChanged(cast
	 * .cdl.WorkingMemoryChange)
	 */
	@Override
	public void workingMemoryChanged(WorkingMemoryChange wmc)
			throws CASTException {
		M motive = null;
		switch (wmc.operation) {
		case ADD: {
			T intent = getMemoryEntry(wmc.address, epistemicClass);
			motive = checkForAddition(wmc.address, intent);
			if (motive != null
					&& PlannerFacade.get(this).isGoalAchieved(
							motive.goal.goalString)) {
				motive = null;
			}
			if (motive != null) {
				motive.thisEntry = new WorkingMemoryAddress(newDataID(),
						getSubarchitectureID());
				addToWorkingMemory(motive.thisEntry, motive);
				intent2motiveMap.put(wmc.address, motive.thisEntry);
			}
			break;
		}
		case OVERWRITE: {
			WorkingMemoryAddress correspondingWMA = intent2motiveMap
					.get(wmc.address);
			T intent = getMemoryEntry(wmc.address, epistemicClass);
			if (correspondingWMA == null) {
				motive = checkForAddition(wmc.address, intent);
			} else {
				try {
					lockEntry(correspondingWMA,
							WorkingMemoryPermissions.LOCKEDOD);
					motive = getMemoryEntry(correspondingWMA, motiveClass);
					motive = checkForUpdate(intent, motive);
					if (motive != null
							&& PlannerFacade.get(this).isGoalAchieved(
									motive.goal.goalString)) {
						motive = null;
					}

					if (motive == null) {
						deleteFromWorkingMemory(correspondingWMA);
						intent2motiveMap.remove(wmc.address);
					} else {
						overwriteWorkingMemory(correspondingWMA, motive);
					}
				} finally {
					unlockEntry(correspondingWMA);
				}
			}
			break;
		}
		case DELETE: {
			WorkingMemoryAddress correspondingWMA = intent2motiveMap
					.get(wmc.address);
			if (correspondingWMA != null) {
				try {
					lockEntry(correspondingWMA,
							WorkingMemoryPermissions.LOCKEDOD);
					deleteFromWorkingMemory(correspondingWMA);
				} finally {
					unlockEntry(correspondingWMA);
				}
				intent2motiveMap.remove(wmc.address);
			}
			break;
		}
		}

	}

	protected abstract M checkForAddition(WorkingMemoryAddress addr, T newEntry);

	protected abstract M checkForUpdate(T newEntry, M motive);

	protected WorkingMemoryAddress getRobotBeliefAddr() {
		if (robotBeliefAddr == null) {
			List<CASTData<GroundedBelief>> groundedBeliefs = new ArrayList<CASTData<GroundedBelief>>();
			try {
				getMemoryEntriesWithData(GroundedBelief.class, groundedBeliefs,
						BINDER_SA, 0);
			} catch (UnknownSubarchitectureException e) {
				logException(e);
				return null;
			}

			for (CASTData<GroundedBelief> beliefEntry : groundedBeliefs) {
				if (beliefEntry.getData().type.equals(ROBOT_BELIEF_TYPE)) {
					robotBeliefAddr = new WorkingMemoryAddress(beliefEntry
							.getID(), BINDER_SA);
					break;
				}
			}
			getLogger().warn("unable to find belief 'Robot'");
		}
		return robotBeliefAddr;
	}

}
