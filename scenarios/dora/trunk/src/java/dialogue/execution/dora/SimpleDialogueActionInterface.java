package dialogue.execution.dora;

import java.util.ArrayList;

import javax.swing.JOptionPane;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.components.RoomMembershipMediator;
import eu.cogx.perceptmediator.dora.VisualObjectTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
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
		dialogue.execution.DialogueActionInterface {

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
					new ComponentActionFactory<DirectReportPosition>(this,
							DirectReportPosition.class));

		} else {
			m_actionStateManager.registerActionType(ReportPosition.class,
					new ComponentActionFactory<ReportPositionDialogue>(this,
							ReportPositionDialogue.class));
		}
	}

}
