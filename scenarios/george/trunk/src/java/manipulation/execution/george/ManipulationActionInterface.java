package manipulation.execution.george;

import manipulation.execution.slice.ArmMovementTask;
import manipulation.execution.slice.ManipulationTaskStatus;
import manipulation.execution.slice.ManipulationTaskType;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import execution.components.AbstractActionInterface;
import execution.slice.TriBool;
import execution.slice.actions.PointToObject;
import execution.util.ComponentActionFactory;
import execution.util.LocalActionStateManager;
import execution.util.NonBlockingCompleteFromStatusExecutor;

public class ManipulationActionInterface extends AbstractActionInterface {

	public static class PointToObjectExecutor
			extends
			NonBlockingCompleteFromStatusExecutor<PointToObject, ArmMovementTask> {

		public PointToObjectExecutor(ManagedComponent _component) {
			super(_component, PointToObject.class, ArmMovementTask.class);
		}
		
		@Override
		protected TriBool executionResult(ArmMovementTask _cmd) {
			if(_cmd.status == ManipulationTaskStatus.MCSUCCEEDED) {
				return TriBool.TRITRUE;
			}
			else {
				return TriBool.TRIFALSE;
			}
		}

		@Override
		public void executeAction() {
			try {

				WorkingMemoryPointer visObjPtr = ((ManipulationActionInterface) getComponent())
						.getFirstAncestorOfBelief(getAction().beliefAddress);

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
					cmd.objPointerSeq = new WorkingMemoryPointer[] {visObjPtr};
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

	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);
		m_actionStateManager.registerActionType(PointToObject.class,
				new ComponentActionFactory<PointToObject,PointToObjectExecutor>(this,
						PointToObjectExecutor.class));
	}

}
