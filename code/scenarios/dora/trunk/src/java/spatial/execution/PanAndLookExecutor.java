package spatial.execution;

import java.util.Stack;

import ptz.PTZCompletion;
import ptz.PTZPose;
import ptz.SetPTZPoseCommand;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.castextensions.WMEventQueue;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.util.NonBlockingActionExecutor;

public abstract class PanAndLookExecutor<ActionType extends Action> extends
		NonBlockingActionExecutor<ActionType> {

	private static final double FIXED_TILT = -40 * Math.PI / 180.0;

	private final int m_detections;

	private final Stack<SetPTZPoseCommand> m_remainingCommands;
	private final WorkingMemoryChangeReceiver m_afterDetect;

	private final WorkingMemoryChangeReceiver m_afterTurn;
	private String m_PanCmd;

	public PanAndLookExecutor(ManagedComponent _component,
			Class<ActionType> _actCls, final int _detections) {

		super(_component, _actCls);

		m_detections = _detections;
		m_remainingCommands = new Stack<SetPTZPoseCommand>();

		getComponent().log(
				"new PanAndLookExecutor for " + m_detections + " detections.");

		double increment = (Math.PI) / m_detections;
		double pan = -Math.PI / 2.0;
		for (int i = 0; i < m_detections; i++) {
			SetPTZPoseCommand cmd = new SetPTZPoseCommand(
					new PTZPose(pan, 0, 1), PTZCompletion.COMPINIT);
			pan += increment;
			m_remainingCommands.push(cmd);
		}

		m_afterDetect = new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				getComponent().removeChangeFilter(this);
				getComponent().log(
						"afterDetectListener triggered: "
								+ CASTUtils.toString(_wmc));

				if (!m_remainingCommands.empty()) {
					getComponent().log("more commands exists");
					m_PanCmd = getComponent().newDataID();
					getComponent().addChangeFilter(
							ChangeFilterFactory.createIDFilter(m_PanCmd,
									WorkingMemoryOperation.OVERWRITE),
							m_afterTurn);
					getComponent().addToWorkingMemory(m_PanCmd,
							m_remainingCommands.pop());
				} else {
					publishActionOutcome();
					executionComplete(TriBool.TRITRUE);
				}
			}
		};

		m_afterTurn = new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {

				getComponent().log(
						"afterTurnListener triggered: "
								+ CASTUtils.toString(_wmc));
				SetPTZPoseCommand cmd = getComponent().getMemoryEntry(
						_wmc.address, SetPTZPoseCommand.class);
				getComponent().log("ptz command status: " + cmd.comp.name());

				// if this command failed, fail the whole thing
				if (cmd.comp == PTZCompletion.FAILED) {
					getComponent().removeChangeFilter(this);
					getComponent().deleteFromWorkingMemory(_wmc.address);
					m_remainingCommands.clear();
					executionComplete(TriBool.TRIFALSE);

				} else if (cmd.comp == PTZCompletion.SUCCEEDED) {
					getComponent()
							.log("time to detect now, triggerDetection()");
					m_PanCmd = null;
					getComponent().removeChangeFilter(this);
					getComponent().deleteFromWorkingMemory(_wmc.address);

					// now we've moved, trigger a detection
					triggerDetection();
				} else {
					// getComponent().log("command in progress: " + cmd.comp);

				}
			}
		};

	}

	abstract protected void publishActionOutcome();

	@Override
	protected void executionComplete(TriBool success) {
		SetPTZPoseCommand ptzCommand = new SetPTZPoseCommand(new PTZPose(0,
				FIXED_TILT, 1), PTZCompletion.COMPINIT);
		String id = getComponent().newDataID();
		WMEventQueue queue = new WMEventQueue();
		getComponent().addChangeFilter(
				ChangeFilterFactory.createIDFilter(id,
						WorkingMemoryOperation.OVERWRITE), queue);
		// wait for the command to overwritten
		try {
			getComponent().addToWorkingMemory(id, ptzCommand);
			queue.take();
			getComponent().deleteFromWorkingMemory(id);
		} catch (InterruptedException e) {
			logException(e);
		} catch (CASTException e) {
			logException(e);
		}
		super.executionComplete(success);
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

		} catch (SubarchitectureComponentException e) {
			getComponent().logException(e);
		}

	}

}
