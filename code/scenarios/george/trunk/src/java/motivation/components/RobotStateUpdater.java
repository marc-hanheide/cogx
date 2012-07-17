package motivation.components;

import java.util.HashMap;
import java.util.Map;

import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.util.WMMotiveView;
import motivation.util.WMMotiveView.MotiveStateTransition;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import castutils.castextensions.WMView.ChangeHandler;
import execution.slice.Robot;

public class RobotStateUpdater extends ManagedComponent {

	private static final boolean ROBOT_INITIATED_MODE = true;

	private final HashMap<WorkingMemoryAddress, Class<? extends Motive>> m_activeMotives;
	private final WMMotiveView m_motiveView;

	public RobotStateUpdater() {
		m_activeMotives = new HashMap<WorkingMemoryAddress, Class<? extends Motive>>();
		m_motiveView = WMMotiveView.create(this);
	}

	@Override
	protected void start() {

		try {

			addChangeFilter(
					ChangeFilterFactory.createGlobalTypeFilter(Robot.class),
					new WorkingMemoryChangeReceiver() {

						@Override
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) throws CASTException {
							m_robotAddress = _wmc.address;
							removeChangeFilter(this);
						}
					});

			m_motiveView.setStateChangeHandler(new MotiveStateTransition(
					MotiveStatus.WILDCARD, MotiveStatus.SURFACED),
					new ChangeHandler<Motive>() {

						@Override
						public void entryChanged(
								Map<WorkingMemoryAddress, Motive> map,
								WorkingMemoryChange wmc, Motive newEntry,
								Motive oldEntry) throws CASTException {
							motiveSurfaced(wmc, newEntry);
						}
					});

			ChangeHandler<Motive> inactiveHandler = new ChangeHandler<Motive>() {
				@Override
				public void entryChanged(Map<WorkingMemoryAddress, Motive> map,
						WorkingMemoryChange wmc, Motive newEntry,
						Motive oldEntry) throws CASTException {
					motiveUnsurfaced(wmc, newEntry);
				}
			};

			m_motiveView.setStateChangeHandler(new MotiveStateTransition(
					MotiveStatus.SURFACED, MotiveStatus.UNSURFACED),
					inactiveHandler);

			m_motiveView.setStateChangeHandler(new MotiveStateTransition(
					MotiveStatus.WILDCARD, MotiveStatus.COMPLETED),
					inactiveHandler);

			m_motiveView.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}
	}

	private boolean m_isInRobotInitiatedMode = false;

	private boolean inRobotInitiatedMode() {
		return m_isInRobotInitiatedMode;
	}

	private boolean activeMotivesContainRobotInitiated() {
		
		return m_activeMotives.values().contains(
				LearnObjectFeatureMotive.class);
	}

	protected void motiveUnsurfaced(WorkingMemoryChange wmc, Motive newEntry)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {

		Class<? extends Motive> removed = m_activeMotives.remove(wmc.address);
		if (removed == null) {
			getLogger().error(
					"Unsurfaced motive was not seen before: "
							+ CASTUtils.toString(wmc), getLogAdditions());
		}

		if (inRobotInitiatedMode() && !activeMotivesContainRobotInitiated()) {
			switchMode(!ROBOT_INITIATED_MODE);
		}
	}

	private WorkingMemoryAddress m_robotAddress;

	private void switchMode(boolean robotInitiatedMode)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException,
			ConsistencyException, PermissionException {
		
		if(robotInitiatedMode) {
			println("switching to robot initiated mode");
		}
		else {
			println("switching OUT of robot initiated mode");
		}
		
		assert (m_robotAddress != null);
		lockEntry(m_robotAddress, WorkingMemoryPermissions.LOCKEDODR);
		Robot rbt = getMemoryEntry(m_robotAddress, Robot.class);
		rbt.isInRobotInitiatedMode = robotInitiatedMode;
		overwriteWorkingMemory(m_robotAddress, rbt);
		unlockEntry(m_robotAddress);
		m_isInRobotInitiatedMode = robotInitiatedMode;
	}

	protected void motiveSurfaced(WorkingMemoryChange wmc, Motive newEntry)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
		println("motiveSurfaced: " + CASTUtils.toString(wmc.address) + " "
				+ newEntry.getClass());
		m_activeMotives.put(wmc.address, newEntry.getClass());
		if (!inRobotInitiatedMode() && activeMotivesContainRobotInitiated()) {
			switchMode(ROBOT_INITIATED_MODE);
		}
	}

}
