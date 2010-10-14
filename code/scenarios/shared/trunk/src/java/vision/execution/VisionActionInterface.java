/**
 * 
 */
package vision.execution;

import java.util.Hashtable;
import java.util.Map;
import java.util.Set;

import VisionData.DetectionCommand;
import VisionData.ForegroundedModel;
import VisionData.PeopleDetectionCommand;
import VisionData.VisualLearningTask;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.VisualObjectTransferFunction;
import execution.slice.TriBool;
import execution.slice.actions.BackgroundModels;
import execution.slice.actions.BeliefPlusStringAction;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.slice.actions.ForegroundModels;
import execution.slice.actions.LearnColour;
import execution.slice.actions.LearnIdentity;
import execution.slice.actions.LearnShape;
import execution.slice.actions.RecogniseForegroundedModels;
import execution.slice.actions.UnlearnColour;
import execution.slice.actions.UnlearnIdentity;
import execution.slice.actions.UnlearnShape;
import execution.util.BlockingActionExecutor;
import execution.util.ComponentActionFactory;
import execution.util.LocalActionStateManager;
import execution.util.NonBlockingCompleteOnOperationExecutor;

/**
 * Component to listen to planner actions the trigger the vision sa as
 * appropriate. Must be run from the vision sa.
 * 
 * @author nah
 * 
 */
public class VisionActionInterface extends ManagedComponent {

	private LocalActionStateManager m_actionStateManager;

	/**
	 * An action executor to handle object detection.
	 * 
	 * @author nah
	 */
	public static class DetectObjectsActionExecutor extends
			NonBlockingCompleteOnOperationExecutor<DetectObjects> implements
			WorkingMemoryChangeReceiver {

		private String[] m_labels;

		public DetectObjectsActionExecutor(ManagedComponent _component) {
			super(_component, DetectObjects.class);
		}

		@Override
		public boolean acceptAction(DetectObjects _action) {
			DetectObjects action = _action;
			m_labels = action.labels;
			return true;
		}

		@Override
		public void executeAction() {
			DetectionCommand cmd = new DetectionCommand(m_labels);
			addThenCompleteOnDelete(cmd);
		}

	}

	/**
	 * An action executor to handle people detection.
	 * 
	 * @author nah
	 */
	public static class DetectPeopleActionExecutor extends
			NonBlockingCompleteOnOperationExecutor<DetectPeople> implements
			WorkingMemoryChangeReceiver {

		public DetectPeopleActionExecutor(ManagedComponent _component) {
			super(_component, DetectPeople.class);
		}

		@Override
		public boolean acceptAction(DetectPeople _action) {
			return true;
		}

		@Override
		public void executeAction() {
			PeopleDetectionCommand cmd = new PeopleDetectionCommand();
			addThenCompleteOnDelete(cmd);
		}

	}

	public static class ForegroundModelExecutor extends
			BlockingActionExecutor<ForegroundModels> {

		public ForegroundModelExecutor(ManagedComponent _component) {
			super(_component, ForegroundModels.class);
		}

		@Override
		protected VisionActionInterface getComponent() {
			return (VisionActionInterface) super.getComponent();
		}

		@Override
		public TriBool execute() {

			String[] models = getAction().models;
			for (String model : models) {
				if (!getComponent().m_foregroundedModels.contains(model)) {
					ForegroundedModel foreground = new ForegroundedModel(model);
					WorkingMemoryAddress addr = new WorkingMemoryAddress(
							getComponent().newDataID(), getComponent()
									.getSubarchitectureID());
					try {
						getComponent().addToWorkingMemory(addr, foreground);
						getComponent().m_foregroundedModels.put(model, addr);
						getComponent().log("Foregrounded model: " + model);

					} catch (CASTException e) {
						getComponent().logException(e);
						return TriBool.TRIFALSE;
					}
				} else {
					getComponent().log(
							"Not foregrounding already foregrounded model: "
									+ model);
				}
			}

			return TriBool.TRITRUE;
		}
	}

	public static class BackgroundModelExecutor extends
			BlockingActionExecutor<BackgroundModels> {

		public BackgroundModelExecutor(ManagedComponent _component) {
			super(_component, BackgroundModels.class);
		}

		@Override
		protected VisionActionInterface getComponent() {
			return (VisionActionInterface) super.getComponent();
		}

		@Override
		public TriBool execute() {

			String[] models = getAction().models;
			for (String model : models) {
				WorkingMemoryAddress addr = getComponent().m_foregroundedModels
						.remove(model);
				if (addr != null) {
					try {
						getComponent().deleteFromWorkingMemory(addr);
						getComponent().log("Backgrounded model: " + model);
					} catch (CASTException e) {
						getComponent().logException(e);
						return TriBool.TRIFALSE;
					}
				} else {
					getComponent().log(
							"Not backgrounding non-foregrounded model: "
									+ model);
				}
			}

			return TriBool.TRITRUE;
		}
	}

	/**
	 * An action executor to handle object detection.
	 * 
	 * @author nah
	 */
	public static class RecogniseForegroundedModelsExecutor extends
			NonBlockingCompleteOnOperationExecutor<RecogniseForegroundedModels>
			implements WorkingMemoryChangeReceiver {

		public RecogniseForegroundedModelsExecutor(ManagedComponent _component) {
			super(_component, RecogniseForegroundedModels.class);
		}

		@Override
		protected VisionActionInterface getComponent() {
			return (VisionActionInterface) super.getComponent();
		}

		@Override
		public boolean acceptAction(RecogniseForegroundedModels _action) {
			return true;
		}

		@Override
		public void executeAction() {
			Set<String> foregroundedModels = getComponent().m_foregroundedModels
					.keySet();
			DetectionCommand cmd = new DetectionCommand(
					new String[foregroundedModels.size()]);
			cmd.labels = foregroundedModels.toArray(cmd.labels);
			addThenCompleteOnDelete(cmd);
		}

	}

	public static abstract class LearnInstructionExecutor<ActionType extends BeliefPlusStringAction>
			extends NonBlockingCompleteOnOperationExecutor<ActionType> {

		private final String m_concept;
		private final String m_featurePostfix;
		private final double m_weight;

		public LearnInstructionExecutor(ManagedComponent _component,
				Class<ActionType> _actCls, String _concept, double _weight, String _featurePostfix) {
			super(_component, _actCls);
			m_concept = _concept;
			m_weight = _weight;
			m_featurePostfix = _featurePostfix;

		}

		protected boolean acceptAction(ActionType _action) {
			return true;
		}

		@Override
		protected VisionActionInterface getComponent() {
			return (VisionActionInterface) super.getComponent();
		}

		@Override
		public void executeAction() {
			try {
				WorkingMemoryAddress beliefID = getAction().beliefAddress;

				VisualLearningTask cmd;

				cmd = new VisualLearningTask(getComponent().getVisualObjectID(
						beliefID), beliefID.id, m_concept,
						new String[] { getAction().value },
						new double[] { m_weight });

				getComponent().log(
						"got the vis obj id: "
								+ getComponent().getVisualObjectID(beliefID));

				addThenCompleteOnOverwrite(cmd);

			} catch (CASTException e) {
				getComponent().logException(e);
			}
		}

		@Override
		protected void actionComplete() {
			try {
				getComponent().addBooleanFeature(getAction().beliefAddress,
						m_concept + m_featurePostfix, true);
			} catch (CASTException e) {
				logException(e);
			}
		}
	}

	public static class LearnColourExecutor extends
			LearnInstructionExecutor<LearnColour> {

		public LearnColourExecutor(ManagedComponent _component) {
			super(_component, LearnColour.class, "color", 1, "-learned");
		}
	}

	public static class LearnShapeExecutor extends
			LearnInstructionExecutor<LearnShape> {

		public LearnShapeExecutor(ManagedComponent _component) {
			super(_component, LearnShape.class, "shape", 1, "-learned");
		}

	}

	public static class LearnIdentityExecutor extends
			LearnInstructionExecutor<LearnIdentity> {

		public LearnIdentityExecutor(ManagedComponent _component) {
			super(_component, LearnIdentity.class, "ident", 1, "-learned");
		}
	}

	public static class UnlearnColourExecutor extends
			LearnInstructionExecutor<UnlearnColour> {

		public UnlearnColourExecutor(ManagedComponent _component) {
			super(_component, UnlearnColour.class, "color", -1, "-unlearned");
		}
	}

	public static class UnlearnShapeExecutor extends
			LearnInstructionExecutor<UnlearnShape> {

		public UnlearnShapeExecutor(ManagedComponent _component) {
			super(_component, UnlearnShape.class, "shape", -1, "-unlearned");
		}

	}

	public static class UnlearnIdentityExecutor extends
			LearnInstructionExecutor<UnlearnIdentity> {

		public UnlearnIdentityExecutor(ManagedComponent _component) {
			super(_component, UnlearnIdentity.class, "ident", -1,  "-unlearned");
		}
	}

	@Override
	protected void configure(Map<String, String> _config) {
	}

	private String getVisualObjectID(WorkingMemoryAddress _beliefID)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

		GroundedBelief belief = getMemoryEntry(_beliefID, GroundedBelief.class);
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> pb = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, belief);
		return pb.getContent()
				.get(VisualObjectTransferFunction.VISUAL_OBJECT_ID)
				.getDistribution().getMostLikely().getProposition();
	}

	private void addBooleanFeature(WorkingMemoryAddress _beliefAddress,
			String _feature, boolean _value) throws DoesNotExistOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {

		GroundedBelief belief = getMemoryEntry(_beliefAddress,
				GroundedBelief.class);
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> pb = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, belief);

		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(_value, 1);

		pb.getContent().put(_feature, fd);
		overwriteWorkingMemory(_beliefAddress, pb.get());
	}

	private final Hashtable<String, WorkingMemoryAddress> m_foregroundedModels;

	public VisionActionInterface() {
		m_foregroundedModels = new Hashtable<String, WorkingMemoryAddress>();
	}

	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);

		// direct dections

		m_actionStateManager.registerActionType(DetectObjects.class,
				new ComponentActionFactory<DetectObjectsActionExecutor>(this,
						DetectObjectsActionExecutor.class));

		m_actionStateManager.registerActionType(DetectPeople.class,
				new ComponentActionFactory<DetectPeopleActionExecutor>(this,
						DetectPeopleActionExecutor.class));

		// model loading

		m_actionStateManager.registerActionType(ForegroundModels.class,
				new ComponentActionFactory<ForegroundModelExecutor>(this,
						ForegroundModelExecutor.class));

		m_actionStateManager.registerActionType(BackgroundModels.class,
				new ComponentActionFactory<BackgroundModelExecutor>(this,
						BackgroundModelExecutor.class));

		// actions with loaded models

		m_actionStateManager
				.registerActionType(
						RecogniseForegroundedModels.class,
						new ComponentActionFactory<RecogniseForegroundedModelsExecutor>(
								this, RecogniseForegroundedModelsExecutor.class));

		// learning executors

		m_actionStateManager.registerActionType(LearnColour.class,
				new ComponentActionFactory<LearnColourExecutor>(this,
						LearnColourExecutor.class));
		m_actionStateManager.registerActionType(LearnShape.class,
				new ComponentActionFactory<LearnShapeExecutor>(this,
						LearnShapeExecutor.class));
		m_actionStateManager.registerActionType(LearnIdentity.class,
				new ComponentActionFactory<LearnIdentityExecutor>(this,
						LearnIdentityExecutor.class));

		m_actionStateManager.registerActionType(UnlearnColour.class,
				new ComponentActionFactory<UnlearnColourExecutor>(this,
						UnlearnColourExecutor.class));
		m_actionStateManager.registerActionType(UnlearnShape.class,
				new ComponentActionFactory<UnlearnShapeExecutor>(this,
						UnlearnShapeExecutor.class));
		m_actionStateManager.registerActionType(UnlearnIdentity.class,
				new ComponentActionFactory<UnlearnIdentityExecutor>(this,
						UnlearnIdentityExecutor.class));

	}

}
