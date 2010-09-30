/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.generators;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryPermissions;
import castutils.castextensions.WMView;
import castutils.castextensions.WMView.ChangeHandler;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.WMBeliefView;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public abstract class AbstractBeliefMotiveGenerator<M extends Motive, T extends dBelief>
		extends ManagedComponent implements ChangeHandler<T> {

	private static final int DEFAULT_MAX_EXECUTION_TIME = 60 * 5;
	private static final int DEFAULT_MAX_PLANNING_TIME = 10;
	final WMView<T> beliefView;
	final Map<WorkingMemoryAddress, WorkingMemoryAddress> bel2motiveMap = new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>();
	final Class<M> motiveClass;
	private WorkingMemoryAddress robotBeliefAddr=null;

	/**
	 * 
	 */
	protected AbstractBeliefMotiveGenerator(String beliefType,
			Class<M> motiveClass, Class<T> beliefClass) {
		beliefView = WMBeliefView.create(this, beliefClass, beliefType);
		this.motiveClass = motiveClass;
		beliefView.registerHandler(this);
	}
	
	protected WorkingMemoryAddress getRobotBeliefAddr() {
		if (robotBeliefAddr==null) {
			for (Entry<WorkingMemoryAddress, T> beliefEntry : beliefView.entrySet()) {
				if (beliefEntry.getValue().type.equals("Robot")) {
					robotBeliefAddr=beliefEntry.getKey();
					break;
				}
			}
			getLogger().warn("unable to find belief 'Robot'");
		}
		return robotBeliefAddr;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see motivation.components.generators.AbstractMotiveGenerator#start()
	 */
	@Override
	protected void start() {
		super.start();
		try {
			beliefView.start();
		} catch (UnknownSubarchitectureException e) {
			logException("could not start view", e);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * castutils.castextensions.WMView.ChangeHandler#entryChanged(java.util.Map,
	 * cast.cdl.WorkingMemoryChange, Ice.ObjectImpl, Ice.ObjectImpl)
	 */
	@Override
	public synchronized void entryChanged(Map<WorkingMemoryAddress, T> map,
			WorkingMemoryChange wmc, T newEntry, T oldEntry)
			throws CASTException {
		M motive = null;
		switch (wmc.operation) {
		case ADD: {
			motive = checkForAddition(wmc.address, newEntry);
			if (motive != null) {
				motive.thisEntry = new WorkingMemoryAddress(newDataID(),
						getSubarchitectureID());
				addToWorkingMemory(motive.thisEntry, motive);
				bel2motiveMap.put(wmc.address, motive.thisEntry);
			}
			break;
		}
		case OVERWRITE: {
			WorkingMemoryAddress correspondingWMA = bel2motiveMap
					.get(wmc.address);
			if (correspondingWMA == null) {
				motive = checkForAddition(wmc.address, newEntry);
			} else {
				try {
					lockEntry(correspondingWMA,
							WorkingMemoryPermissions.LOCKEDOD);
					motive = getMemoryEntry(correspondingWMA, motiveClass);
					motive = checkForUpdate(newEntry, motive);
					if (motive == null) {
						deleteFromWorkingMemory(correspondingWMA);
						bel2motiveMap.remove(wmc.address);
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
			WorkingMemoryAddress correspondingWMA = bel2motiveMap
					.get(wmc.address);
			if (correspondingWMA != null) {
				try {
					lockEntry(correspondingWMA,
							WorkingMemoryPermissions.LOCKEDOD);
					deleteFromWorkingMemory(correspondingWMA);
				} finally {
					unlockEntry(correspondingWMA);
				}
				bel2motiveMap.remove(wmc.address);
			}
			break;
		}
		}

	}

	protected <T2 extends Motive> T2 fillDefault(T2 result) {
		result.created = getCASTTime();
		result.correspondingUnion = "";
		result.maxExecutionTime = DEFAULT_MAX_EXECUTION_TIME;
		result.maxPlanningTime = DEFAULT_MAX_PLANNING_TIME;
		result.priority = MotivePriority.UNSURFACE;
		result.status = MotiveStatus.UNSURFACED;
		return result;
	}

	protected abstract M checkForUpdate(T newEntry, M motive);

	protected abstract M checkForAddition(WorkingMemoryAddress addr, T newEntry);

}
