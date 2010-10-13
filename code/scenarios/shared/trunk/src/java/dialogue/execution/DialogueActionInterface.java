package dialogue.execution;

import java.util.ArrayList;
import java.util.Map;

import javax.swing.JOptionPane;

import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.slice.GroundedToSharedBeliefMap;
import de.dfki.lt.tr.beliefs.data.CASTFrame;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.SharedBelief;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.slice.actions.AskForColour;
import execution.slice.actions.AskForIdentity;
import execution.slice.actions.AskForObjectWithFeatureValue;
import execution.slice.actions.AskForShape;
import execution.slice.actions.AskPolarColour;
import execution.slice.actions.AskPolarIdentity;
import execution.slice.actions.AskPolarShape;
import execution.slice.actions.BeliefPlusStringAction;
import execution.slice.actions.SingleBeliefAction;
import execution.util.BlockingActionExecutor;
import execution.util.ComponentActionFactory;
import execution.util.LocalActionStateManager;
import execution.util.NonBlockingCompleteOnOperationExecutor;

/**
 * Receives actions from the execution system and interfaces with the rest of
 * the dialogue system.
 * 
 * @author nah
 * 
 */
public class DialogueActionInterface extends ManagedComponent {
	protected LocalActionStateManager m_actionStateManager;
	private GroundedToSharedBeliefMap m_groundedToShared;

	protected boolean m_fakeIt;

	public static class DirectColourAnswer extends
			BlockingActionExecutor<AskForColour> {

		public DirectColourAnswer(ManagedComponent _component) {
			super(_component, AskForColour.class);
		}

		@Override
		public TriBool execute() {
			TriBool result = TriBool.TRIFALSE;
			try {
				((DialogueActionInterface) getComponent())
						.askForFeatureThenSetDirect("color", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;
		}

	}

	public abstract static class IntentionDialogueAction<T extends Action>
			extends NonBlockingCompleteOnOperationExecutor<T> {

		public IntentionDialogueAction(ManagedComponent _component,
				Class<T> _cls) {
			super(_component, _cls);
		}

		/**
		 * List of postconditions, just containing (ModalFormula "agent"
		 * (ElementaryFormula "human"))
		 * 
		 * @param _groundedBeliefAddress
		 * @param _sharedBeliefAddress
		 * @return
		 */
		ArrayList<dFormula> getPostconditions() {
			ArrayList<dFormula> postconditions = new ArrayList<dFormula>();
			// postconditions.add(new ModalFormula(-1, "agent",
			// PropositionFormula
			// .create("human").get()));
			return postconditions;
		}

		/**
		 * Empty list of preconditions.
		 * 
		 * @param _groundedBeliefAddress
		 * @param _sharedBeliefAddress
		 * @return
		 */
		ArrayList<dFormula> getPreconditions() {
			ArrayList<dFormula> preconditions = new ArrayList<dFormula>();
			return preconditions;
		}

		/**
		 * Get the intention to be the action. If null is returned, failure is
		 * reported to the planner.
		 * 
		 * @return
		 */
		protected Intention getIntention() {
			Intention intention = createEmptyIntention();
			((ComplexFormula) intention.content.get(0).preconditions).forms = getPreconditions();

			ModalFormula state = new ModalFormula(-1, "state",
					new ComplexFormula(-1, getPostconditions(), BinaryOp.conj));
			ArrayList<dFormula> states = new ArrayList<dFormula>();
			states.add(state);

			((ComplexFormula) intention.content.get(0).postconditions).forms = states;

			// ((ComplexFormula) intention.content.get(0).postconditions).forms
			// = getPostconditions();
			return intention;
		}

		/**
		 * Creates and empty intention with ComplexFormluas for precondition and
		 * postcondition but with null valued formula lists.
		 * 
		 * @return
		 */
		protected Intention createEmptyIntention() {
			// TODO sweet-talk marc into writing proxies for this too
			Intention robotIntention = new Intention();
			robotIntention.frame = CASTFrame.create().get();
			robotIntention.estatus = new PrivateEpistemicStatus(
					org.cognitivesystems.binder.thisAgent.value);

			// actual content
			IntentionalContent content = new IntentionalContent();
			content.probValue = 1f;
			content.agents = new ArrayList<String>();
			content.agents.add(org.cognitivesystems.binder.thisAgent.value);

			// precondition
			ComplexFormula preconditions = new ComplexFormula(-1, null,
					BinaryOp.conj);
			content.preconditions = preconditions;

			// postcondition
			ComplexFormula postconditions = new ComplexFormula(-1, null,
					BinaryOp.conj);

			ModalFormula state = new ModalFormula(-1, "state", postconditions);

			ComplexFormula states = new ComplexFormula(-1,
					new ArrayList<dFormula>(1), BinaryOp.conj);
			states.forms.add(state);

			content.postconditions = states;

			robotIntention.content = new ArrayList<IntentionalContent>(1);
			robotIntention.content.add(content);
			return robotIntention;
		}

		@Override
		public void executeAction() {
			Intention robotIntention = getIntention();
			if (robotIntention != null) {
				assert(robotIntention.id == null);
				robotIntention.id = getComponent().newDataID();
				addThenCompleteOnDelete(robotIntention.id,robotIntention);
			} else {
				
				println("returning fail due to null intention");
				executionComplete(TriBool.TRIFALSE);
			}
		}

	}

	public abstract static class BeliefIntentionDialogueAction<T extends SingleBeliefAction>
			extends IntentionDialogueAction<T> {

		public BeliefIntentionDialogueAction(ManagedComponent _component,
				Class<T> _cls) {
			super(_component, _cls);
		}

		/**
		 * List of postconditions, just containing (ModalFormula "agent"
		 * (ElementaryFormula "human"))
		 * 
		 * @param _groundedBeliefAddress
		 * @param _sharedBeliefAddress
		 * @return
		 */
		protected ArrayList<dFormula> getPostconditions(
				WorkingMemoryAddress _groundedBeliefAddress,
				WorkingMemoryAddress _sharedBeliefAddress) {
			return super.getPostconditions();
		}

		/**
		 * Adds the precondition that the shared belief exists.
		 * 
		 * @param _groundedBeliefAddress
		 * @param _sharedBeliefAddress
		 * @return
		 */
		protected ArrayList<dFormula> getPreconditions(
				WorkingMemoryAddress _groundedBeliefAddress,
				WorkingMemoryAddress _sharedBeliefAddress) {

			ArrayList<dFormula> preconditions = super.getPreconditions();

			preconditions.add(new ModalFormula(-1, "belief", WMPointer.create(
					_sharedBeliefAddress,
					CASTUtils.typeName(SharedBelief.class)).get()));
			return preconditions;

		}

		protected Intention getIntention() {

			WorkingMemoryAddress groundedBeliefAddress = getAction().beliefAddress;
			WorkingMemoryAddress sharedBeliefAddress = ((DialogueActionInterface) getComponent())
					.getSharedBeliefAddress(groundedBeliefAddress);

			if (sharedBeliefAddress == null) {
				println("Giving up because no shared belief yet for: "
						+ groundedBeliefAddress);
				return null;
			}

			Intention intention = createEmptyIntention();
			((ComplexFormula) intention.content.get(0).preconditions).forms = getPreconditions(
					groundedBeliefAddress, sharedBeliefAddress);

			ModalFormula state = new ModalFormula(-1, "state",
					new ComplexFormula(-1, getPostconditions(
							groundedBeliefAddress, sharedBeliefAddress),
							BinaryOp.conj));
			ArrayList<dFormula> states = new ArrayList<dFormula>();
			states.add(state);

			((ComplexFormula) intention.content.get(0).postconditions).forms = states;

			// ((ComplexFormula) intention.content.get(0).postconditions).forms
			// = getPostconditions(
			// groundedBeliefAddress, sharedBeliefAddress);
			return intention;
		}

	}

	public abstract static class FeatureRequestDialogueAction<T extends SingleBeliefAction>
			extends BeliefIntentionDialogueAction<T> {

		private final String m_feature;

		public FeatureRequestDialogueAction(ManagedComponent _component,
				Class<T> _cls, String _feature) {
			super(_component, _cls);
			m_feature = _feature;
		}

		protected ArrayList<dFormula> getPostconditions(
				WorkingMemoryAddress _groundedBeliefAddress,
				WorkingMemoryAddress _sharedBeliefAddress) {

			ArrayList<dFormula> postconditions = super.getPostconditions(
					_groundedBeliefAddress, _sharedBeliefAddress);

			postconditions.add(PropositionFormula.create("question-answered")
					.get());

			postconditions
					.add(new ModalFormula(
							-1,
							"agent",
							PropositionFormula
									.create(org.cognitivesystems.binder.humanAgent.value)
									.get()));

			postconditions.add(new ModalFormula(-1, "about", WMPointer.create(
					_groundedBeliefAddress,
					CASTUtils.typeName(GroundedBelief.class)).get()));
			postconditions.add(new ModalFormula(-1, "feature",
					PropositionFormula.create(m_feature).get()));

			return postconditions;
		}

	}

	public abstract static class AskForPolarDialogue<T extends BeliefPlusStringAction>
			extends FeatureRequestDialogueAction<T> {

		public AskForPolarDialogue(ManagedComponent _component, Class<T> _cls,
				String _feature) {
			super(_component, _cls, _feature);
		}

		@Override
		protected ArrayList<dFormula> getPostconditions(
				WorkingMemoryAddress _groundedBeliefAddress,
				WorkingMemoryAddress _sharedBeliefAddress) {
			ArrayList<dFormula> postconditions = super.getPostconditions(
					_groundedBeliefAddress, _sharedBeliefAddress);
			postconditions.add(new ModalFormula(-1, "hypo", PropositionFormula
					.create(getAction().value).get()));
			return postconditions;
		}

	}

	public static class AskForObjectWithFeatureValueDialogue extends
			IntentionDialogueAction<AskForObjectWithFeatureValue> {

		public AskForObjectWithFeatureValueDialogue(ManagedComponent _component) {
			super(_component, AskForObjectWithFeatureValue.class);
		}

		@Override
		ArrayList<dFormula> getPostconditions() {
			ArrayList<dFormula> postconditions = super.getPostconditions();
			postconditions.add(PropositionFormula.create("object-shown").get());
			postconditions.add(new ModalFormula(-1, "feature",
					PropositionFormula.create(getAction().feature).get()));
			postconditions.add(new ModalFormula(-1, "hypo", PropositionFormula
					.create(getAction().value).get()));
			return postconditions;
		}

	}

	public static class AskForColourValueDialogue extends
			FeatureRequestDialogueAction<AskForColour> {
		public AskForColourValueDialogue(ManagedComponent _component) {
			super(_component, AskForColour.class, "color");
		}
	}

	public static class AskForShapeValueDialogue extends
			FeatureRequestDialogueAction<AskForShape> {
		public AskForShapeValueDialogue(ManagedComponent _component) {
			super(_component, AskForShape.class, "shape");
		}
	}

	public static class AskForIdentityValueDialogue extends
			FeatureRequestDialogueAction<AskForIdentity> {
		public AskForIdentityValueDialogue(ManagedComponent _component) {
			super(_component, AskForIdentity.class, "identity");
		}
	}

	public static class AskForColourPolarDialogue extends
			AskForPolarDialogue<AskPolarColour> {
		public AskForColourPolarDialogue(ManagedComponent _component) {
			super(_component, AskPolarColour.class, "color");
		}
	}

	public static class AskForShapePolarDialogue extends
			AskForPolarDialogue<AskPolarShape> {
		public AskForShapePolarDialogue(ManagedComponent _component) {
			super(_component, AskPolarShape.class, "shape");
		}
	}

	public static class AskForIdentityPolarDialogue extends
			AskForPolarDialogue<AskPolarIdentity> {
		public AskForIdentityPolarDialogue(ManagedComponent _component) {
			super(_component, AskPolarIdentity.class, "identity");
		}
	}

	public static class DirectShapeAnswer extends
			BlockingActionExecutor<AskForShape> {

		public DirectShapeAnswer(ManagedComponent _component) {
			super(_component, AskForShape.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRIFALSE;
			try {
				((DialogueActionInterface) getComponent())
						.askForFeatureThenSetDirect("shape", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}

	}

	public static class DirectIdentityAnswer extends
			BlockingActionExecutor<AskForIdentity> {

		public DirectIdentityAnswer(ManagedComponent _component) {
			super(_component, AskForIdentity.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRIFALSE;
			try {
				((DialogueActionInterface) getComponent())
						.askForFeatureThenSetDirect("ident", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}

	}

	public static class DirectPolarShapeAnswer extends
			BlockingActionExecutor<AskPolarShape> {

		public DirectPolarShapeAnswer(ManagedComponent _component) {
			super(_component, AskPolarShape.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRIFALSE;
			try {

				((DialogueActionInterface) getComponent())
						.askPolarFeatureThenSetDirect("shape", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}
	}

	public static class DirectPolarColourAnswer extends
			BlockingActionExecutor<AskPolarColour> {

		public DirectPolarColourAnswer(ManagedComponent _component) {
			super(_component, AskPolarColour.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRIFALSE;
			try {

				((DialogueActionInterface) getComponent())
						.askPolarFeatureThenSetDirect("color", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}
	}

	public static class DirectPolarIdentityAnswer extends
			BlockingActionExecutor<AskPolarIdentity> {

		public DirectPolarIdentityAnswer(ManagedComponent _component) {
			super(_component, AskPolarIdentity.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRIFALSE;
			try {

				((DialogueActionInterface) getComponent())
						.askPolarFeatureThenSetDirect("identity", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}
	}

	public static class DirectAskForObject extends
			BlockingActionExecutor<AskForObjectWithFeatureValue> {
		public DirectAskForObject(ManagedComponent _component) {
			super(_component, AskForObjectWithFeatureValue.class);
		}

		@Override
		public TriBool execute() {
			JOptionPane.showMessageDialog(null,
					"OI YOU! Add a new object that looks " + getAction().value
							+ " DO AS I SAY! .... please xxx");
			return TriBool.TRITRUE;
		}
	}

	

	

	private void addFeatureDirectly(SingleBeliefAction _action,
			String _feature, String _value, double _prob)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
		GroundedBelief belief = getMemoryEntry(_action.beliefAddress,
				GroundedBelief.class);
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> pb = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, belief);

		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(_value, _prob);
		pb.getContent().put(_feature, fd);
		overwriteWorkingMemory(_action.beliefAddress, pb.get());
	}

	private static String askForFeatureValue(String _feature) {
		return (String) JOptionPane.showInputDialog(null, "What " + _feature
				+ " is the object?");
	}

	private static boolean askPolarFeatureValue(String _feature, String _value) {
		String answer = (String) JOptionPane.showInputDialog(null, "Is the "
				+ _feature + " " + _value + "?");
		if (answer.toLowerCase().startsWith("y")) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 * @throws UnknownSubarchitectureException
	 */
	private void askForFeatureThenSetDirect(String _feature,
			SingleBeliefAction _action) throws DoesNotExistOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {
		addFeatureDirectly(_action, _feature, askForFeatureValue(_feature), 1d);
	}

	/**
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 * @throws UnknownSubarchitectureException
	 */
	private boolean askPolarFeatureThenSetDirect(String _feature,
			BeliefPlusStringAction _action) throws DoesNotExistOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {
		if (askPolarFeatureValue(_feature, _action.value)) {
			addFeatureDirectly(_action, _feature, _action.value, 1d);
			return true;
		} else {
			addFeatureDirectly(_action, _feature, _action.value, 0d);
		}
		return false;
	}

	public void addBooleanFeature(WorkingMemoryAddress _beliefAddress,
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

	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);

		// DoNothingActionExecutorFactory derelictFactory = new
		// DoNothingActionExecutorFactory(
		// this);

		if (m_fakeIt) {
			m_actionStateManager.registerActionType(AskForColour.class,
					new ComponentActionFactory<DirectColourAnswer>(this,
							DirectColourAnswer.class));

			m_actionStateManager.registerActionType(AskForShape.class,
					new ComponentActionFactory<DirectShapeAnswer>(this,
							DirectShapeAnswer.class));

			m_actionStateManager.registerActionType(AskForIdentity.class,
					new ComponentActionFactory<DirectIdentityAnswer>(this,
							DirectIdentityAnswer.class));

			m_actionStateManager.registerActionType(AskPolarColour.class,
					new ComponentActionFactory<DirectPolarColourAnswer>(this,
							DirectPolarColourAnswer.class));

			m_actionStateManager.registerActionType(AskPolarShape.class,
					new ComponentActionFactory<DirectPolarShapeAnswer>(this,
							DirectPolarShapeAnswer.class));

			m_actionStateManager.registerActionType(AskPolarIdentity.class,
					new ComponentActionFactory<DirectPolarIdentityAnswer>(this,
							DirectPolarIdentityAnswer.class));

			m_actionStateManager.registerActionType(
					AskForObjectWithFeatureValue.class,
					new ComponentActionFactory<DirectAskForObject>(this,
							DirectAskForObject.class));

			} else {

			m_actionStateManager.registerActionType(AskForColour.class,
					new ComponentActionFactory<AskForColourValueDialogue>(this,
							AskForColourValueDialogue.class));

			m_actionStateManager.registerActionType(AskForShape.class,
					new ComponentActionFactory<AskForShapeValueDialogue>(this,
							AskForShapeValueDialogue.class));

			m_actionStateManager.registerActionType(AskForIdentity.class,
					new ComponentActionFactory<AskForIdentityValueDialogue>(
							this, AskForIdentityValueDialogue.class));

			m_actionStateManager.registerActionType(AskPolarColour.class,
					new ComponentActionFactory<AskForColourPolarDialogue>(this,
							AskForColourPolarDialogue.class));

			m_actionStateManager.registerActionType(AskPolarShape.class,
					new ComponentActionFactory<AskForShapePolarDialogue>(this,
							AskForShapePolarDialogue.class));

			m_actionStateManager.registerActionType(AskPolarIdentity.class,
					new ComponentActionFactory<AskForIdentityPolarDialogue>(
							this, AskForIdentityPolarDialogue.class));

			m_actionStateManager
					.registerActionType(
							AskForObjectWithFeatureValue.class,
							new ComponentActionFactory<AskForObjectWithFeatureValueDialogue>(
									this,
									AskForObjectWithFeatureValueDialogue.class));

			

			// TODO replace this with one of Marc's magic thingys
			WorkingMemoryChangeReceiver updater = new WorkingMemoryChangeReceiver() {

				@Override
				public void workingMemoryChanged(WorkingMemoryChange _wmc)
						throws CASTException {
					m_groundedToShared = getMemoryEntry(_wmc.address,
							GroundedToSharedBeliefMap.class);
				}
			};

			// look for the map between grounded and shared beliefs
			addChangeFilter(
					ChangeFilterFactory.createGlobalTypeFilter(
							GroundedToSharedBeliefMap.class,
							WorkingMemoryOperation.ADD), updater);
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					GroundedToSharedBeliefMap.class,
					WorkingMemoryOperation.OVERWRITE), updater);

		}

	}

	@Override
	protected void configure(Map<String, String> _config) {
		m_fakeIt = _config.containsKey("--fake-it");
	}

	private WorkingMemoryAddress getSharedBeliefAddress(
			WorkingMemoryAddress _groundedBeliefAddress) {
		WorkingMemoryAddress addr = null;
		if (m_groundedToShared != null) {
			addr = m_groundedToShared.map.get(_groundedBeliefAddress);
		} else {
			println("trying to get shared belief address before map has been generated");
		}
		return addr;
	}

}
