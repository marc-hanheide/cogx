package manipulation.execution.george;

import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import eu.cogx.beliefs.slice.MergedBelief;
import execution.components.AbstractActionInterface;
import execution.slice.Robot;
import execution.slice.TriBool;
import execution.slice.actions.ArmToHomePos;
import execution.slice.actions.PointToObject;
import execution.util.BlockingActionExecutor;
import execution.util.ComponentActionFactory;
import execution.util.LocalActionStateManager;

/**
 * 
 * TODO Refactor so that it shares code with real interface.
 * 
 * @author nah
 * 
 */
public class FakeManipulationActionInterface extends
		AbstractActionInterface<MergedBelief> {

	private static final boolean ARM_IS_RESTING = true;

	public FakeManipulationActionInterface() {
		super(MergedBelief.class);
	}

	public static class PointToObjectExecutor extends
			BlockingActionExecutor<PointToObject> {

		public PointToObjectExecutor(ManagedComponent _component) {
			super(_component, PointToObject.class);
		}

		protected void actionComplete() {
			try {
				((FakeManipulationActionInterface) getComponent())
						.updateArmRestingState(!FakeManipulationActionInterface.ARM_IS_RESTING);
			} catch (SubarchitectureComponentException e) {
				logException("Problem updating robot state", e);
			}
		}

		@Override
		public TriBool execute() {

			try {

				WorkingMemoryPointer visObjPtr = ((FakeManipulationActionInterface) getComponent())
						.getFirstAncestorOfBelief(getAction().beliefAddress);

				if (visObjPtr == null) {
					getComponent()
							.getLogger()
							.warn("Action failed because VisualObject pointer was null",
									getComponent().getLogAdditions());
					return TriBool.TRIFALSE;
				} else {
					log("belief addr "
							+ CASTUtils.toString(getAction().beliefAddress)
							+ " yielded VO addr "
							+ CASTUtils.toString(visObjPtr.address));

					println("FAKE ARM STUFF: sleeping, updating robot state, then returning true");
					Thread.sleep(3000);
					actionComplete();
					return TriBool.TRITRUE;

				}
			} catch (Exception e) {
				logException(e);
				return TriBool.TRIFALSE;
			}

		}
	}

	public static class ArmToHomePosExecutor extends
			BlockingActionExecutor<ArmToHomePos> {

		public ArmToHomePosExecutor(ManagedComponent _component) {
			super(_component, ArmToHomePos.class);
		}

		@Override
		public TriBool execute() {
			println("FAKE ARM STUFF: sleeping, updating robot state, then returning true");
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				logException(e);
			}
			actionComplete();
			return TriBool.TRITRUE;

		}

		protected void actionComplete() {
			try {
				((FakeManipulationActionInterface) getComponent())
						.updateArmRestingState(FakeManipulationActionInterface.ARM_IS_RESTING);
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
