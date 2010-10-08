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
	private LocalActionStateManager m_actionStateManager;
	private GroundedToSharedBeliefMap m_groundedToShared;

	private boolean m_fakeIt;

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

	public static class AskForShapeDialogue extends
			NonBlockingCompleteOnOperationExecutor<AskForShape> {

		public AskForShapeDialogue(ManagedComponent _component) {
			super(_component, AskForShape.class);
		}

		@Override
		protected boolean acceptAction(AskForShape _action) {
			return true;
		}

		@Override
		public void executeAction() {

			WorkingMemoryAddress groundedBeliefAddress = getAction().beliefAddress;
			WorkingMemoryAddress sharedBeliefAddress = ((DialogueActionInterface) getComponent())
					.getSharedBeliefAddress(groundedBeliefAddress);

			if (sharedBeliefAddress == null) {
				println("Giving up because no shared belief yet for: "
						+ groundedBeliefAddress);
				executionComplete(TriBool.TRIFALSE);
			} else {

				// as copied from
				// http://codex.cs.bham.ac.uk/trac/cogx/wiki/documents/scenarios/beliefs/lingproduction

				// TODO sweet-talk marc into writing proxies for this too
				Intention robotIntention = new Intention();
				robotIntention.frame = CASTFrame.create().get();
				robotIntention.estatus = new PrivateEpistemicStatus("robot");

				// actual content
				IntentionalContent content = new IntentionalContent();
				content.probValue = 1f;
				content.agents = new ArrayList<String>();
				content.agents.add("human");

				// precondition
				ComplexFormula preconditions = new ComplexFormula(-1,
						new ArrayList<dFormula>(1), BinaryOp.conj);
				preconditions.forms.add(new ModalFormula(-1, "belief",
						WMPointer.create(sharedBeliefAddress).get()));
				content.preconditions = preconditions;

				// postcondition
				ComplexFormula postconditions = new ComplexFormula(-1,
						new ArrayList<dFormula>(4), BinaryOp.conj);
				postconditions.forms.add(PropositionFormula.create(
						"question-answered").get());
				postconditions.forms.add(new ModalFormula(-1, "agent",
						PropositionFormula.create("human").get()));
				postconditions.forms.add(new ModalFormula(-1, "about",
						WMPointer.create(groundedBeliefAddress).get()));
				postconditions.forms.add(new ModalFormula(-1, "feature",
						PropositionFormula.create("color").get()));
				content.postconditions = postconditions;

				robotIntention.content = new ArrayList<IntentionalContent>(1);
				robotIntention.content.add(content);

				addThenCompleteOnDelete(robotIntention);
			}

		}

	}

	public abstract static class AskForFeatureValueDialogue<T extends SingleBeliefAction>
			extends NonBlockingCompleteOnOperationExecutor<T> {

		private final String m_feature;

		public AskForFeatureValueDialogue(ManagedComponent _component,
				Class<T> _cls, String _feature) {
			super(_component, _cls);
			m_feature = _feature;
		}

		protected Intention getIntention(
				WorkingMemoryAddress _groundedBeliefAddress,
				WorkingMemoryAddress _sharedBeliefAddress) {
			// TODO sweet-talk marc into writing proxies for this too
			Intention robotIntention = new Intention();
			robotIntention.frame = CASTFrame.create().get();
			robotIntention.estatus = new PrivateEpistemicStatus("robot");

			// actual content
			IntentionalContent content = new IntentionalContent();
			content.probValue = 1f;
			content.agents = new ArrayList<String>();
			content.agents.add("robot");

			// precondition
			ComplexFormula preconditions = new ComplexFormula(-1,
					new ArrayList<dFormula>(1), BinaryOp.conj);
			preconditions.forms.add(new ModalFormula(-1, "belief", WMPointer
					.create(_sharedBeliefAddress).get()));
			content.preconditions = preconditions;

			// postcondition
			ComplexFormula postconditions = new ComplexFormula(-1,
					new ArrayList<dFormula>(4), BinaryOp.conj);
			postconditions.forms.add(PropositionFormula.create(
					"question-answered").get());
			postconditions.forms.add(new ModalFormula(-1, "agent",
					PropositionFormula.create("human").get()));
			postconditions.forms.add(new ModalFormula(-1, "about", WMPointer
					.create(_groundedBeliefAddress).get()));
			postconditions.forms.add(new ModalFormula(-1, "feature",
					PropositionFormula.create(m_feature).get()));

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

			WorkingMemoryAddress groundedBeliefAddress = getAction().beliefAddress;
			WorkingMemoryAddress sharedBeliefAddress = ((DialogueActionInterface) getComponent())
					.getSharedBeliefAddress(groundedBeliefAddress);

			if (sharedBeliefAddress == null) {
				println("Giving up because no shared belief yet for: "
						+ groundedBeliefAddress);
				executionComplete(TriBool.TRIFALSE);
			} else {

				// as copied from
				// http://codex.cs.bham.ac.uk/trac/cogx/wiki/documents/scenarios/beliefs/lingproduction

				Intention robotIntention = getIntention(groundedBeliefAddress,
						sharedBeliefAddress);
				addThenCompleteOnOverwrite(robotIntention);
			}

		}

	}

	public abstract static class AskForPolarDialogue<T extends BeliefPlusStringAction>
			extends AskForFeatureValueDialogue<T> {

		public AskForPolarDialogue(ManagedComponent _component, Class<T> _cls,
				String _feature) {
			super(_component, _cls, _feature);
		}

		@Override
		protected Intention getIntention(
				WorkingMemoryAddress _groundedBeliefAddress,
				WorkingMemoryAddress _sharedBeliefAddress) {
			Intention intention = super.getIntention(_groundedBeliefAddress,
					_sharedBeliefAddress);
			ComplexFormula postconditions = (ComplexFormula) intention.content
					.get(0).postconditions;
			postconditions.forms.add(new ModalFormula(-1, "hypo",
					PropositionFormula.create(getAction().value).get()));
			return intention;
		}

	}

	public static class AskForColourValueDialogue extends
			AskForFeatureValueDialogue<AskForColour> {
		public AskForColourValueDialogue(ManagedComponent _component) {
			super(_component, AskForColour.class, "color");
		}
	}

	public static class AskForShapeValueDialogue extends
			AskForFeatureValueDialogue<AskForShape> {
		public AskForShapeValueDialogue(ManagedComponent _component) {
			super(_component, AskForShape.class, "shape");
		}
	}

	public static class AskForIdentityValueDialogue extends
			AskForFeatureValueDialogue<AskForIdentity> {
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
			JOptionPane.showMessageDialog(null, "OI YOU! Add a new object that looks " +getAction().value + " DO AS I SAY! .... please xxx");
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
			
			m_actionStateManager.registerActionType(AskForObjectWithFeatureValue.class,
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
