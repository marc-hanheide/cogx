package spatial.execution;

import java.util.Stack;

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

public abstract class TurnAndLookExecutor<ActionType extends Action> extends NonBlockingActionExecutor<ActionType> {

	private final int m_detections;

	private final Stack<NavCommand> m_remainingCommands;
	private final WorkingMemoryChangeReceiver m_afterDetect;

	private final WorkingMemoryChangeReceiver m_afterTurn;
	private String m_navCmdID;

	public TurnAndLookExecutor(ManagedComponent _component, Class<ActionType> _actCls,
			final int _detections) {
		
		super(_component, _actCls);
		
		m_detections = _detections;
		m_remainingCommands = new Stack<NavCommand>();

		
		
		getComponent().log("new TurnAndLookExecutor for " + m_detections
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
				getComponent().removeChangeFilter(this);
				getComponent().log("afterDetectListener triggered: "
						+ CASTUtils.toString(_wmc));

				if (!m_remainingCommands.empty()) {
					m_navCmdID = getComponent().newDataID();
					getComponent().addChangeFilter(ChangeFilterFactory
							.createIDFilter(m_navCmdID,
									WorkingMemoryOperation.OVERWRITE),
							m_afterTurn);
					getComponent().addToWorkingMemory(m_navCmdID,
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

				getComponent().log("afterTurnListener triggered: "
						+ CASTUtils.toString(_wmc));

				// read in the nav cmd
				getComponent().lockEntry(_wmc.address,
						WorkingMemoryPermissions.LOCKEDODR);

				NavCommand cmd = getComponent().getMemoryEntry(_wmc.address,
						NavCommand.class);
				getComponent().log("nav command status: " + cmd.comp.name());

				// if this command failed, fail the whole thing
				if (cmd.comp == Completion.COMMANDFAILED) {
					getComponent().removeChangeFilter(this);
					getComponent().deleteFromWorkingMemory(_wmc.address);
					m_remainingCommands.clear();
					executionComplete(TriBool.TRIFALSE);

				} else if (cmd.comp == Completion.COMMANDSUCCEEDED) {
					getComponent().log("time to detect now, triggerDetection()");
					m_navCmdID = null;
					getComponent().removeChangeFilter(this);
					getComponent().deleteFromWorkingMemory(_wmc.address);

					// now we've moved, trigger a detection
					triggerDetection();
				} else {
					// getComponent().log("command in progress: " + cmd.comp);
					getComponent().unlockEntry(_wmc.address);
				}
			}
		};

	}

	protected WorkingMemoryChangeReceiver getAfterDetectionReceiver() {
		return m_afterDetect;
	}

	@Override
	public void executeAction() {
		getComponent().log("execute action!");

		triggerDetection();
	}

	/**
	 * 
	 */
	protected abstract void triggerDetection();

	
	@Override
	public void stopExecution() {
		// remove overwrite receiver

		try {
			getComponent().log("aborting execution if not already stopped");

			m_remainingCommands.clear();
			getComponent().removeChangeFilter(m_afterTurn);

			// if a nav command is happening:
			if (m_navCmdID != null) {

				// reread
				getComponent().lockEntry(m_navCmdID,
						WorkingMemoryPermissions.LOCKEDODR);
				NavCommand navCmd = getComponent().getMemoryEntry(m_navCmdID,
						NavCommand.class);
				navCmd.comp = Completion.COMMANDABORTED;
				getComponent().overwriteWorkingMemory(m_navCmdID, navCmd);
				getComponent().unlockEntry(m_navCmdID);
				m_navCmdID = null;
			}

		} catch (SubarchitectureComponentException e) {
			getComponent().logException(e);
		}

	}

}
