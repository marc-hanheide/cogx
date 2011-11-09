/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.generators;

import java.util.HashMap;
import java.util.Map;

import motivation.slice.Motive;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import eu.cogx.planner.facade.PlannerFacade;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public abstract class AbstractIntentionMotiveGenerator<M extends Motive, T extends Intention>
		extends AbstractEpistemicObjectMotiveGenerator<M, T> implements
		WorkingMemoryChangeReceiver {

	final Map<WorkingMemoryAddress, WorkingMemoryAddress> intent2motiveMap = new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>();


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


}
