/**
 * 
 */
package vision.execution.george;

import VisionData.MoveToViewConeCommand;
import VisionData.VisionCommandStatus;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import eu.cogx.beliefs.slice.GroundedBelief;
import execution.slice.Robot;
import execution.slice.TriBool;
import execution.slice.actions.george.yr3.AnalyzeProtoObject;
import execution.slice.actions.george.yr3.MoveToViewCone;
import execution.util.ComponentActionFactory;
import execution.util.DoNothingActionExecutorFactory;
import execution.util.LocalActionStateManager;
import execution.util.NonBlockingCompleteFromStatusExecutor;

/**
 * Component to listen to planner actions the trigger the vision sa as
 * appropriate. Must be run from the vision sa.
 * 
 * @author nah
 * 
 */
public class VisionActionInterface extends ManagedComponent {

	private LocalActionStateManager m_actionStateManager;

	private WorkingMemoryAddress m_viewStateAddress;

	public static class MoveToViewConeExecutor
			extends
			NonBlockingCompleteFromStatusExecutor<MoveToViewCone, MoveToViewConeCommand> {

		public MoveToViewConeExecutor(ManagedComponent _component) {
			super(_component, MoveToViewCone.class, MoveToViewConeCommand.class);
		}

		public WorkingMemoryAddress getViewStateAddress() {
			return ((VisionActionInterface) getComponent()).m_viewStateAddress;
		}

		public void setViewStateAddress(WorkingMemoryAddress _vsa) {
			((VisionActionInterface) getComponent()).m_viewStateAddress = _vsa;
		}

		@Override
		public void executeAction() {

			try {
				WorkingMemoryPointer viewConePtr = ((VisionActionInterface) getComponent())
						.getFirstAncestorOfBelief(getAction().beliefAddress);
				if (viewConePtr == null) {
					getComponent().getLogger().warn(
							"Action failed because ViewCone pointer was null",
							getComponent().getLogAdditions());
					executionComplete(TriBool.TRIFALSE);
				} else {
					MoveToViewConeCommand cmd = new MoveToViewConeCommand();
					cmd.target = viewConePtr;
					cmd.status = VisionCommandStatus.VCREQUESTED;
					addThenCompleteOnOverwrite(cmd);
				}

			} catch (Exception e) {
				getComponent().logException(e);
				executionComplete(TriBool.TRIFALSE);
			}
		}

		@Override
		protected TriBool executionResult(MoveToViewConeCommand _cmd) {

			try {
				if (_cmd.status == VisionCommandStatus.VCSUCCEEDED) {
					((VisionActionInterface) getComponent())
							.recordCurrentViewCone(_cmd.target);
					return TriBool.TRITRUE;
				} else {
					((VisionActionInterface) getComponent())
							.recordCurrentViewCone(null);
					return TriBool.TRIFALSE;
				}
			} catch (Exception e) {
				logException(e);
				return TriBool.TRIFALSE;
			}
		}

	}

	/**
	 * @param _cmd
	 * @throws AlreadyExistsOnWMException
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 */
	private void recordCurrentViewCone(WorkingMemoryPointer _viewconePtr)
			throws AlreadyExistsOnWMException, DoesNotExistOnWMException,
			UnknownSubarchitectureException, ConsistencyException,
			PermissionException {
		Robot rbt = new Robot(_viewconePtr);
		if (m_viewStateAddress == null) {
			m_viewStateAddress = new WorkingMemoryAddress(newDataID(),
					getSubarchitectureID());
			addToWorkingMemory(m_viewStateAddress, rbt);
		} else {
			overwriteWorkingMemory(m_viewStateAddress, rbt);
		}
	}

	private WorkingMemoryPointer getFirstAncestorOfBelief(
			WorkingMemoryAddress _beliefAddress)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

		GroundedBelief belief = getMemoryEntry(_beliefAddress,
				GroundedBelief.class);

		CASTBeliefHistory hist = (CASTBeliefHistory) belief.hist;
		return hist.ancestors.get(0);
	}

	// private String getPropositionFromBelief(
	// WorkingMemoryAddress _beliefAddress, String _index)
	// throws DoesNotExistOnWMException, UnknownSubarchitectureException {
	//
	// GroundedBelief belief = getMemoryEntry(_beliefAddress,
	// GroundedBelief.class);
	// CASTIndependentFormulaDistributionsBelief<GroundedBelief> pb =
	// CASTIndependentFormulaDistributionsBelief
	// .create(GroundedBelief.class, belief);
	// return pb.getContent().get(_index).getDistribution().getMostLikely()
	// .getProposition();
	// }

	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);

		// direct dections

		m_actionStateManager.registerActionType(MoveToViewCone.class,
				new ComponentActionFactory<MoveToViewConeExecutor>(this,
						MoveToViewConeExecutor.class));

		m_actionStateManager.registerActionType(AnalyzeProtoObject.class,
				new DoNothingActionExecutorFactory(this));

	}

	@Override
	protected void runComponent() {
		lockComponent();
		try {
			recordCurrentViewCone(null);
		} catch (CASTException e) {
			logException(e);
		}
		unlockComponent();
	}

}
