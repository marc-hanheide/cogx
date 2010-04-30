package spatial.motivation;

import java.util.Stack;
import java.lang.Thread;

import org.apache.log4j.Logger;

import VisionData.DetectionCommand;
import VisionData.PeopleDetectionCommand;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.util.NonBlockingActionExecutor;
import cast.AlreadyExistsOnWMException;
import execution.slice.actions.LookForObjectsAndPeople;


public class LookForObjectsAndPeopleExecutor extends NonBlockingActionExecutor {

	private String[] m_labels;

	private final int m_detections;
	protected final ManagedComponent m_component;
	private final Stack<NavCommand> m_remainingCommands;
	private final WorkingMemoryChangeReceiver m_afterObjectDetect;
	private final WorkingMemoryChangeReceiver m_afterPeopleDetect;

	private final WorkingMemoryChangeReceiver m_afterTurn;
	private String m_navCmdID;

	static Logger logger = Logger.getLogger(LookForObjectsAndPeople.class);

	public LookForObjectsAndPeopleExecutor(ManagedComponent _component,
			final int _detections) {
		m_component = _component;
		m_detections = _detections;
		m_remainingCommands = new Stack<NavCommand>();

		logger.info("new LookForObjectsAndPeople for " + m_detections
				+ " detections.");

		double increment = (2 * Math.PI) / m_detections;
		for (int i = 0; i < m_detections; i++) {
			NavCommand cmd = SpatialActionInterface.newNavCommand();
			cmd.cmd = CommandType.TURN;
			cmd.angle = new double[] { increment };
                        cmd.tolerance = new double[] { 15.0 * Math.PI / 180 };
			m_remainingCommands.push(cmd);
		}

		m_afterObjectDetect = new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				m_component.removeChangeFilter(this);
				logger.debug("afterObjectDetectListener triggered: "
						+ CASTUtils.toString(_wmc));
				triggerPeopleDetection();

			}
		};

		m_afterPeopleDetect = new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				m_component.removeChangeFilter(this);
				logger.debug("afterPeopleDetectListener triggered: "
						+ CASTUtils.toString(_wmc));

				if (!m_remainingCommands.empty()) {
					m_navCmdID = m_component.newDataID();
					m_component.addChangeFilter(ChangeFilterFactory
							.createIDFilter(m_navCmdID,
									WorkingMemoryOperation.OVERWRITE),
							m_afterTurn);
					m_component.addToWorkingMemory(m_navCmdID,
							m_remainingCommands.pop());
				} else {
					executionComplete(TriBool.TRITRUE);
				}
			}
		};

		m_afterTurn = new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {

				logger.debug("afterTurnListener triggered: "
						+ CASTUtils.toString(_wmc));

				// read in the nav cmd
				m_component.lockEntry(_wmc.address,
						WorkingMemoryPermissions.LOCKEDODR);

				NavCommand cmd = m_component.getMemoryEntry(_wmc.address,
						NavCommand.class);
				logger.debug("nav command status: " + cmd.comp.name());

				// if this command failed, fail the whole thing
				if (cmd.comp == Completion.COMMANDFAILED) {
					m_component.removeChangeFilter(this);
					m_component.deleteFromWorkingMemory(_wmc.address);
					m_remainingCommands.clear();
					executionComplete(TriBool.TRIFALSE);

				} else if (cmd.comp == Completion.COMMANDSUCCEEDED) {
					logger.debug("time to detect now, triggerDetection()");
					m_navCmdID = null;
					m_component.removeChangeFilter(this);
					m_component.deleteFromWorkingMemory(_wmc.address);

					// now we've moved, trigger a detection
					triggerObjectDetection();
				} else {
					// m_component.log("command in progress: " + cmd.comp);
					m_component.unlockEntry(_wmc.address);
				}
			}
		};

	}

	protected WorkingMemoryChangeReceiver getAfterPeopleDetectionReceiver() {
		return m_afterPeopleDetect;
	}

	protected WorkingMemoryChangeReceiver getAfterObjectDetectionReceiver() {
		return m_afterObjectDetect;
	}

	@Override
	public void executeAction() {
		logger.debug("execute action!");

		triggerObjectDetection();
	}

	@Override
	public boolean accept(Action _action) {
		m_labels = ((LookForObjectsAndPeople) _action).labels;
		return true;
	}

	protected void triggerObjectDetection() {
		m_component.log("object detection triggered");

		// Fire off a detection command
		DetectionCommand detect = new DetectionCommand(m_labels);
		String id = m_component.newDataID();
		try {
			Thread.sleep(1000);
			m_component
					.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
							WorkingMemoryOperation.DELETE),
							getAfterObjectDetectionReceiver());
			m_component.addToWorkingMemory(id, detect);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	protected void triggerPeopleDetection() {
		m_component.log("people detection triggered");
		
		// Fire off a detection command
		PeopleDetectionCommand detect = new PeopleDetectionCommand();
		String id = m_component.newDataID();
		try {
			m_component
					.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
							WorkingMemoryOperation.DELETE),
							getAfterPeopleDetectionReceiver());
			m_component.addToWorkingMemory(id, detect);
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}


	@Override
	public void stopExecution() {
		// remove overwrite receiver

		try {
			logger.debug("aborting execution if not already stopped");

			m_remainingCommands.clear();
			m_component.removeChangeFilter(m_afterTurn);

			// if a nav command is happening:
			if (m_navCmdID != null) {

				// reread
				m_component.lockEntry(m_navCmdID,
						WorkingMemoryPermissions.LOCKEDODR);
				NavCommand navCmd = m_component.getMemoryEntry(m_navCmdID,
						NavCommand.class);
				navCmd.comp = Completion.COMMANDABORTED;
				m_component.overwriteWorkingMemory(m_navCmdID, navCmd);
				m_component.unlockEntry(m_navCmdID);
				m_navCmdID = null;
			}

		} catch (SubarchitectureComponentException e) {
			m_component.logException(e);
		}

	}

}
