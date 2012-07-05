package manipulation.execution.george;

import manipulation.execution.slice.ArmMovementTask;
import manipulation.execution.slice.ManipulationTaskStatus;
import manipulation.execution.slice.ManipulationTaskType;
import VisionData.VisualObject;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.beliefs.utils.BeliefUtils;
import execution.components.AbstractActionInterface;
import execution.slice.Robot;
import execution.slice.TriBool;
import execution.slice.actions.ArmToHomePos;
import execution.slice.actions.PointToObject;
import execution.util.ComponentActionFactory;
import execution.util.LocalActionStateManager;
import execution.util.NonBlockingCompleteFromStatusExecutor;

public class ManipulationActionInterface extends
		AbstractActionInterface<MergedBelief> {

	private static final boolean ARM_IS_RESTING = true;

	public ManipulationActionInterface() {
		super(MergedBelief.class);
	}

	public static class PointToObjectExecutor
			extends
			NonBlockingCompleteFromStatusExecutor<PointToObject, ArmMovementTask> {

		public PointToObjectExecutor(ManagedComponent _component) {
			super(_component, PointToObject.class, ArmMovementTask.class);
		}

		@Override
		protected TriBool executionResult(ArmMovementTask _cmd) {
			if (_cmd.status == ManipulationTaskStatus.MCSUCCEEDED) {
				
				//Update object timestamp, ticket 528
				try {
					WorkingMemoryAddress voAddr = _cmd.objPointerSeq[0].address;
					VisualObject visualObject = getComponent().getMemoryEntry(voAddr, VisualObject.class);
					CASTTime tm = getComponent().getTimeServer().getCASTTime();
					visualObject.salience = tm.s + 1e-6 * tm.us;
					getComponent().overwriteWorkingMemory(voAddr, visualObject);
					log("updated salience of visual object");
				} catch (DoesNotExistOnWMException e) {
					logException(e);
				} catch (UnknownSubarchitectureException e) {
					logException(e);
				} catch (ConsistencyException e) {
					logException(e);
				} catch (PermissionException e) {
					logException(e);
				}
				
				return TriBool.TRITRUE;
			} else {
				return TriBool.TRIFALSE;
			}
		}

		@Override
		protected void actionComplete() {
			try {
				

				
				((ManipulationActionInterface) getComponent())
						.updateArmRestingState(!ManipulationActionInterface.ARM_IS_RESTING);
			} catch (SubarchitectureComponentException e) {
				logException("Problem updating robot state", e);
			}
		}

		@Override
		public void executeAction() {
			try {

				WorkingMemoryPointer visObjPtr = BeliefUtils
						.recurseAncestorsForType(getComponent(),
								getAction().beliefAddress,
								CASTUtils.typeName(VisualObject.class));

				if (visObjPtr == null) {
					getComponent()
							.getLogger()
							.warn("Action failed because VisualObject pointer was null",
									getComponent().getLogAdditions());
					executionComplete(TriBool.TRIFALSE);
				} else {
					log("belief addr "
							+ CASTUtils.toString(getAction().beliefAddress)
							+ " yielded VO addr "
							+ CASTUtils.toString(visObjPtr.address));
					ArmMovementTask cmd = new ArmMovementTask();
					cmd.objPointerSeq = new WorkingMemoryPointer[] { visObjPtr };
					cmd.taskType = ManipulationTaskType.POINTOBJ0;
					cmd.status = ManipulationTaskStatus.MCREQUESTED;
					addThenCompleteOnOverwrite(cmd);
				}
			} catch (Exception e) {
				logException(e);
				executionComplete(TriBool.TRIFALSE);
			}

		}
	}

	public static class ArmToHomePosExecutor
			extends
			NonBlockingCompleteFromStatusExecutor<ArmToHomePos, ArmMovementTask> {

		public ArmToHomePosExecutor(ManagedComponent _component) {
			super(_component, ArmToHomePos.class, ArmMovementTask.class);
		}

		@Override
		protected TriBool executionResult(ArmMovementTask _cmd) {
			if (_cmd.status == ManipulationTaskStatus.MCSUCCEEDED) {
				return TriBool.TRITRUE;
			} else {
				return TriBool.TRIFALSE;
			}
		}

		@Override
		public void executeAction() {
			ArmMovementTask cmd = new ArmMovementTask();
			cmd.taskType = ManipulationTaskType.RETRACTARM;
			cmd.status = ManipulationTaskStatus.MCREQUESTED;
			cmd.objPointerSeq = new WorkingMemoryPointer[0];
			addThenCompleteOnOverwrite(cmd);
		}

		@Override
		protected void actionComplete() {
			try {
				((ManipulationActionInterface) getComponent())
						.updateArmRestingState(ManipulationActionInterface.ARM_IS_RESTING);
			} catch (SubarchitectureComponentException e) {
				logException("Problem updating robot state", e);
			}
		}
	}

	private WorkingMemoryAddress m_robotStateAddr;

	private void updateArmRestingState(boolean _armIsResting)
			throws SubarchitectureComponentException {
		if (m_robotStateAddr == null) {
			getLogger().error("Robot state address is missing",
					getLogAdditions());
		} else {
			lockEntry(m_robotStateAddr, WorkingMemoryPermissions.LOCKEDODR);
			Robot rbt = getMemoryEntry(m_robotStateAddr, Robot.class);
			rbt.armIsResting = _armIsResting;
			overwriteWorkingMemory(m_robotStateAddr, rbt);
			unlockEntry(m_robotStateAddr);
		}
	}

	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);
		m_actionStateManager
				.registerActionType(
						PointToObject.class,
						new ComponentActionFactory<PointToObject, PointToObjectExecutor>(
								this, PointToObjectExecutor.class));
		m_actionStateManager.registerActionType(ArmToHomePos.class,
				new ComponentActionFactory<ArmToHomePos, ArmToHomePosExecutor>(
						this, ArmToHomePosExecutor.class));

		// collect address of robot belief
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Robot.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				m_robotStateAddr = _wmc.address;
				removeChangeFilter(this);
			}
		});

	}
}
