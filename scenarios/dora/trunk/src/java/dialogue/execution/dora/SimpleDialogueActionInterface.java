package dialogue.execution.dora;

import cast.architecture.ManagedComponent;
import dialogue.execution.AbstractDialogueActionInterface;
import eu.cogx.beliefs.slice.GroundedBelief;
import execution.slice.TriBool;
import execution.slice.actions.ReportPosition;
import execution.util.BlockingActionExecutor;
import execution.util.ComponentActionFactory;

/**
 * Receives actions from the execution system and interfaces with the rest of
 * the dialogue system.
 * 
 * @author nah
 * 
 */
public class SimpleDialogueActionInterface extends
		AbstractDialogueActionInterface<GroundedBelief> {

	
	public SimpleDialogueActionInterface() {
		super(GroundedBelief.class);
	}
	
	public static class ReportPositionDialogue extends
			BlockingActionExecutor<ReportPosition> {

		public ReportPositionDialogue(ManagedComponent _component) {
			super(_component, ReportPosition.class);
		}

		@Override
		public TriBool execute() {
			TriBool result = TriBool.TRITRUE;

			return result;

		}

	}

	public static class DirectReportPosition extends
			BlockingActionExecutor<ReportPosition> {

		public DirectReportPosition(ManagedComponent _component) {
			super(_component, ReportPosition.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRITRUE;
			println("DirectReportPosition doesn't do anything!");
			return result;

		}
	}

	@Override
	protected void start() {
		super.start();

		if (m_fakeIt) {
			m_actionStateManager.registerActionType(ReportPosition.class,
					new ComponentActionFactory<ReportPosition,DirectReportPosition>(this,
							DirectReportPosition.class));

		} else {
			m_actionStateManager.registerActionType(ReportPosition.class,
					new ComponentActionFactory<ReportPosition,ReportPositionDialogue>(this,
							ReportPositionDialogue.class));
		}
	}

}
