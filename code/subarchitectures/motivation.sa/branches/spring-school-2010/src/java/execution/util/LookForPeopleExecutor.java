package execution.util;

import java.util.Stack;

import spatial.motivation.SpatialActionInterface;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import VisionData.PeopleDetectionCommand;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import execution.slice.Action;
import execution.slice.TriBool;

/**
 * Executor which
 * 
 * @author nah
 * 
 */
public class LookForPeopleExecutor extends NonBlockingActionExecutor {

	private int m_detections;
	private final ManagedComponent m_component;
	private final Stack<NavCommand> m_remainingCommands;
	private final WorkingMemoryChangeReceiver m_afterDetect;

	private final WorkingMemoryChangeReceiver m_afterTurn;
	private String m_navCmdID;

	public LookForPeopleExecutor(ManagedComponent _component, int _detections) {
		m_component = _component;
		m_detections = _detections;
		m_remainingCommands = new Stack<NavCommand>();

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
				// read in the nav cmd
				m_component.lockEntry(_wmc.address,
						WorkingMemoryPermissions.LOCKEDODR);

				NavCommand cmd = m_component.getMemoryEntry(_wmc.address,
						NavCommand.class);

				// if this command failed, fail the whole thing
				if (cmd.comp == Completion.COMMANDFAILED) {
					m_component.removeChangeFilter(this);
					m_component.deleteFromWorkingMemory(_wmc.address);
					m_remainingCommands.clear();
					executionComplete(TriBool.TRIFALSE);

				} else if (cmd.comp == Completion.COMMANDSUCCEEDED) {
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

	@Override
	public void executeAction() {
		triggerDetection();
	}

	/**
	 * 
	 */
	private void triggerDetection() {
		// Fire off a detection command
		PeopleDetectionCommand detect = new PeopleDetectionCommand();
		String id = m_component.newDataID();
		try {
			m_component.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
					WorkingMemoryOperation.DELETE), m_afterDetect);
			m_component.addToWorkingMemory(id, detect);
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}

	@Override
	public boolean accept(Action _action) {

		return true;
	}

	@Override
	public void stopExecution() {
		// remove overwrite receiver

		try {
			m_component.log("aborting execution");
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
