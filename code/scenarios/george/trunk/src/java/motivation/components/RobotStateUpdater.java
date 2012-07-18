package motivation.components;

import java.util.HashMap;

import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
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
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import execution.slice.Robot;

public class RobotStateUpdater extends ManagedComponent implements
		WorkingMemoryChangeReceiver {

	private static final boolean ROBOT_INITIATED_MODE = true;

	private static final String ROBOT_INITIATED_TYPE = CASTUtils
			.typeName(LearnObjectFeatureMotive.class);

	private final HashMap<WorkingMemoryAddress, String> m_activeMotives;

	public RobotStateUpdater() {
		m_activeMotives = new HashMap<WorkingMemoryAddress, String>();
	}

	@Override
	protected void start() {

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(Robot.class),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						m_robotAddress = _wmc.address;
						removeChangeFilter(this);
					}
				});

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(Motive.class), this);

	}

	private boolean m_isInRobotInitiatedMode = false;

	private boolean inRobotInitiatedMode() {
		return m_isInRobotInitiatedMode;
	}

	private boolean activeMotivesContainRobotInitiated() {

		return m_activeMotives.values().contains(ROBOT_INITIATED_TYPE);
	}

	protected void motiveUnsurfaced(WorkingMemoryAddress wmc, String motiveType)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {

		String removed = m_activeMotives.remove(wmc);
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

		if (robotInitiatedMode) {
			println("switching to robot initiated mode");
		} else {
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

	protected void motiveSurfaced(WorkingMemoryAddress wmc, String newEntry)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
		String put = m_activeMotives.put(wmc, newEntry);
		if (put == null) {
			println("motiveSurfaced: " + CASTUtils.toString(wmc) + " "
					+ newEntry);
			if (!inRobotInitiatedMode() && activeMotivesContainRobotInitiated()) {
				switchMode(ROBOT_INITIATED_MODE);
			}
		}
	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {

		if (_wmc.operation != WorkingMemoryOperation.DELETE) {

			try {
				Motive mtv = getMemoryEntry(_wmc.address, Motive.class);
				if (mtv.status == MotiveStatus.ACTIVE
						|| mtv.status == MotiveStatus.SURFACED) {
					motiveSurfaced(_wmc.address, _wmc.type);
				} else if (_wmc.operation != WorkingMemoryOperation.ADD) {
					motiveUnsurfaced(_wmc.address, _wmc.type);
				}
			} catch (DoesNotExistOnWMException e) {
				motiveUnsurfaced(_wmc.address, _wmc.type);
			}
		} else {
			motiveUnsurfaced(_wmc.address, _wmc.type);
		}

	}
}
