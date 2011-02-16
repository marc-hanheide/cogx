/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.generators;

import java.util.Map;

import motivation.slice.Motive;
import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryPermissions;
import castutils.castextensions.WMView;
import castutils.castextensions.WMView.ChangeHandler;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.WMBeliefView;
import eu.cogx.planner.facade.PlannerFacade;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public abstract class AbstractBeliefMotiveGenerator<M extends Motive, T extends dBelief>
		extends AbstractEpistemicObjectMotiveGenerator<M, T> implements
		ChangeHandler<T> {

	final WMView<T> beliefView;

	/**
	 * 
	 */
	protected AbstractBeliefMotiveGenerator(String beliefType,
			Class<M> motiveClass, Class<T> beliefClass) {
		super(motiveClass, beliefClass);
		beliefView = WMBeliefView.create(this, beliefClass, beliefType);
		beliefView.registerHandler(this);
	}

	// protected WorkingMemoryAddress getRobotBeliefAddr() {
	// if (robotBeliefAddr==null) {
	// for (Entry<WorkingMemoryAddress, T> beliefEntry : beliefView.entrySet())
	// {
	// if (beliefEntry.getValue().type.equals("Robot")) {
	// robotBeliefAddr=beliefEntry.getKey();
	// break;
	// }
	// }
	// getLogger().warn("unable to find belief 'Robot'");
	// }
	// return robotBeliefAddr;
	// }

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
//			// if goal is achieved already we should remove it!
//			if (motive != null
//					&& PlannerFacade.get(this).isGoalAchieved(
//							motive.goal.goalString)) {
//				motive = null;
//			}

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
//					// if goal is achieved already we should remove it!
//					if (motive != null
//							&& PlannerFacade.get(this).isGoalAchieved(
//									motive.goal.goalString)) {
//						motive = null;
//					}
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
				} catch (CASTException e) {
					logException(e);
					unlockEntry(correspondingWMA);
					throw (e);
				}
				bel2motiveMap.remove(wmc.address);
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
	protected void start() {
		super.start();
		try {
			beliefView.start();
		} catch (UnknownSubarchitectureException e) {
			logException("could not start view", e);
		}
	}

}
