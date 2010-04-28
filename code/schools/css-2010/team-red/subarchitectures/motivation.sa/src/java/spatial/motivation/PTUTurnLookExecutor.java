package spatial.motivation;

import java.util.Stack;

import org.apache.log4j.Logger;

import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import VisionData.DetectionCommand;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.AlreadyExistsOnWMException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.util.NonBlockingActionExecutor;
import execution.slice.actions.PTULookForObjects;

public class PTUTurnLookExecutor extends NonBlockingActionExecutor {

	private final int m_detections;
	protected final ManagedComponent m_component;
	private final Stack<NavCommand> m_remainingCommands;
	private final WorkingMemoryChangeReceiver m_afterDetect;

	private final WorkingMemoryChangeReceiver m_afterTurn;
	private String m_navCmdID;

	private String[] m_labels;

	static Logger logger = Logger.getLogger(PTUTurnLookExecutor.class);

	public PTUTurnLookExecutor(ManagedComponent _component,
			final int _detections) {
		m_detections = _detections;
		m_component = _component;
		m_remainingCommands = new Stack<NavCommand>();

		logger.info("new PTUTurnLookExecutor for " + m_detections
				+ " detections.");

		NavCommand cmd = SpatialActionInterface.newNavCommand();
		cmd.cmd = CommandType.TURN;
		cmd.angle = new double[] { Math.PI };
		m_remainingCommands.push(cmd);

		m_afterDetect = new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				m_component.removeChangeFilter(this);
				logger.info("afterDetectListener triggered: "
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

				logger.info("afterTurnListener triggered: "
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
					triggerDetection();
				} else {
					// m_component.log("command in progress: " + cmd.comp);
					m_component.unlockEntry(_wmc.address);
				}
			}
		};

	}

	protected WorkingMemoryChangeReceiver getAfterDetectionReceiver() {
		return m_afterDetect;
	}

	@Override
	public void executeAction() {
		logger.info("execute action!");

		triggerDetection();
	}

	/**
	 * 
	 */
	protected void triggerDetection() {
		m_component.log("detection triggered");
		logger.info("detection triggered");
		// Fire off a detection command
		DetectionCommand detect = new DetectionCommand(m_labels);
		String id = m_component.newDataID();
		try {
			m_component.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
							WorkingMemoryOperation.DELETE),
							getAfterDetectionReceiver());
			m_component.addToWorkingMemory(id, detect);
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}

	public boolean accept(Action _action) {
		m_labels = ((PTULookForObjects) _action).labels;
		return true;
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
