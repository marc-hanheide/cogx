package spatial.motivation;

import java.util.Stack;

import org.apache.log4j.Logger;

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

public abstract class TurnAndLookExecutor extends NonBlockingActionExecutor {

	private final int m_detections;
	protected final ManagedComponent m_component;
	protected final Stack<NavCommand> m_remainingCommands;
	protected final WorkingMemoryChangeReceiver m_afterDetect;

	protected final WorkingMemoryChangeReceiver m_afterTurn;
	protected String m_navCmdID;

	static Logger logger = Logger.getLogger(TurnAndLookExecutor.class);

	public TurnAndLookExecutor(ManagedComponent _component,
			final int _detections) {
		m_component = _component;
		m_detections = _detections;
		m_remainingCommands = new Stack<NavCommand>();

		logger.info("new TurnAndLookExecutor for " + m_detections
				+ " detections.");

		double increment = (2 * Math.PI) / m_detections;
		for (int i = 0; i < m_detections; i++) {
			NavCommand cmd = SpatialActionInterface.newNavCommand();
			cmd.cmd = CommandType.TURN;
			cmd.angle = new double[] { increment };
			m_remainingCommands.push(cmd);
		}

		m_afterDetect = new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				m_component.removeChangeFilter(this);
				logger.debug("afterDetectListener triggered: "
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
		logger.debug("execute action!");

		triggerDetection();
	}

	/**
	 * 
	 */
	protected abstract void triggerDetection();

	@Override
	public boolean accept(Action _action) {
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
