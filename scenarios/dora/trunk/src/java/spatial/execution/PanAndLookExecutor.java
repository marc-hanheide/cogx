package spatial.execution;

import java.util.Stack;

import ptz.PTZCompletion;
import ptz.PTZPose;
import ptz.SetPTZPoseCommand;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.util.NonBlockingActionExecutor;
import facades.SpatialFacade;

public abstract class PanAndLookExecutor<ActionType extends Action> extends
		NonBlockingActionExecutor<ActionType> {

	// the angles that are put on the stack of command to look at (remember,
	// it's a stack, so the order is somewhat reversed (last position is first)
	private static final float[] FIXED_ANGLES = new float[] { 0, 45, 90, -45,
			-90 };

	// private static final double FIXED_TILT = -40 * Math.PI / 180.0;

	private final Stack<SetPTZPoseCommand> m_remainingCommands;
	private final WorkingMemoryChangeReceiver m_afterDetect;

	private final WorkingMemoryChangeReceiver m_afterTurn;
	private String m_PanCmd;

	public PanAndLookExecutor(ManagedComponent _component,
			Class<ActionType> _actCls, final int _detections) {

		super(_component, _actCls);

		m_remainingCommands = new Stack<SetPTZPoseCommand>();

		for (float angle_deg : FIXED_ANGLES) {
			SetPTZPoseCommand cmd = new SetPTZPoseCommand(new PTZPose(angle_deg
					* Math.PI / 180.0, 0, 1), PTZCompletion.COMPINIT);

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

	// @Override
	// protected void executionComplete(TriBool success) {
	// SetPTZPoseCommand ptzCommand = new SetPTZPoseCommand(new PTZPose(0,
	// FIXED_TILT, 1), PTZCompletion.COMPINIT);
	// String id = getComponent().newDataID();
	// WMEventQueue queue = new WMEventQueue();
	// getComponent().addChangeFilter(
	// ChangeFilterFactory.createIDFilter(id,
	// WorkingMemoryOperation.OVERWRITE), queue);
	// // wait for the command to overwritten
	// try {
	// getComponent().addToWorkingMemory(id, ptzCommand);
	// println("going back to original pose");
	// queue.take();
	// println("pose command overwritten");
	// getComponent().deleteFromWorkingMemory(id);
	// } catch (InterruptedException e) {
	// logException(e);
	// } catch (CASTException e) {
	// logException(e);
	// }
	// super.executionComplete(success);
	// }

	protected WorkingMemoryChangeReceiver getAfterDetectionReceiver() {
		return m_afterDetect;
	}

	@Override
	public void executeAction() {
		getComponent().log("execute action!");

		randomTurn();

		
	}

	private void randomTurn() {
		try {
			String spatialSA = SpatialFacade.get(getComponent()).getSpatialSA();
			String id = getComponent().newDataID();
			final WorkingMemoryChangeReceiver chgFilter = new WorkingMemoryChangeReceiver() {

				@Override
				public void workingMemoryChanged(WorkingMemoryChange arg0)
						throws CASTException {
					try {
						NavCommand nc = getComponent().getMemoryEntry(
								arg0.address, NavCommand.class);
						if (nc.comp == Completion.COMMANDABORTED
								|| nc.comp == Completion.COMMANDFAILED
								|| nc.comp == Completion.COMMANDSUCCEEDED) {
							println("random turn completed");
							triggerDetection();
							// getComponent().removeChangeFilter(chgFilter);
						}
					} catch (CASTException e) {
						logException(e);
					}
				}
			};
			getComponent().addChangeFilter(
					ChangeFilterFactory.createAddressFilter(id, spatialSA,
							WorkingMemoryOperation.OVERWRITE), chgFilter);
			NavCommand cmd = SpatialActionInterface.newNavCommand();
			cmd.cmd = CommandType.TURN;
			cmd.destId = new long[] {};
			cmd.angle = new double[] { Math.random() * Math.PI - Math.PI / 2 };
			cmd.distance = new double[] {};
			cmd.pose = new double[] {};
			cmd.tolerance = new double[] { 0.1, 0.1, Math.PI * 10.0 / 180.0 };
			getComponent().addToWorkingMemory(
					new WorkingMemoryAddress(id, spatialSA), cmd);
		} catch (CASTException e) {
			logException(e);
		}
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
