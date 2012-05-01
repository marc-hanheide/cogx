/**
 * 
 */
package vision.execution.george;

import java.util.Map;

import ptz.PTZInterface;
import ptz.PTZInterfacePrx;
import ptz.PTZReading;
import NavData.RobotPose2d;
import VisionData.AnalyzeProtoObjectCommand;
import VisionData.MoveToViewConeCommand;
import VisionData.ProtoObject;
import VisionData.ViewCone;
import VisionData.VisionCommandStatus;
import VisionData.VisualLearningTask;
import cast.AlreadyExistsOnWMException;
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
import cast.cdl.WorkingMemoryPermissions;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cogx.Math.Vector3;
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.beliefs.utils.BeliefUtils;
import execution.components.AbstractActionInterface;
import execution.slice.Robot;
import execution.slice.TriBool;
import execution.slice.actions.BeliefPlusStringAction;
import execution.slice.actions.LearnColour;
import execution.slice.actions.LearnIdentity;
import execution.slice.actions.LearnShape;
import execution.slice.actions.UnlearnColour;
import execution.slice.actions.UnlearnIdentity;
import execution.slice.actions.UnlearnShape;
import execution.slice.actions.george.yr3.AnalyzeProtoObject;
import execution.slice.actions.george.yr3.MoveToViewCone;
import execution.util.ComponentActionFactory;
import execution.util.LocalActionStateManager;
import execution.util.NonBlockingCompleteFromStatusExecutor;

/**
 * Component to listen to planner actions the trigger the vision sa as
 * appropriate. Must be run from the vision sa.
 * 
 * @author nah
 * 
 */
public class VisionActionInterface extends
		AbstractActionInterface<MergedBelief> {

	public static final String LEARNED_FEATURE_POSTFIX = "-learned";
	public static final String UNLEARNED_FEATURE_POSTFIX = "-unlearned";

	private WorkingMemoryAddress m_viewStateAddress;

	private String m_ptzServerComponent;

	// determine whether to fake the robot pose
	private boolean m_fakeRobotPose;
	// is the arm resting at system startup?
	private boolean m_armIsRestingInitially;

	public VisionActionInterface() {
		super(MergedBelief.class);
	}

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

				WorkingMemoryPointer viewConePtr = BeliefUtils
						.recurseAncestorsForType(getComponent(),
								getAction().beliefAddress,
								CASTUtils.typeName(ViewCone.class));

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

					// tell the planner that we're now looking at this
					((VisionActionInterface) getComponent())
							.recordCurrentViewCone(_cmd.target);

					// record that we have looked at it
					((VisionActionInterface) getComponent()).addFeature(
							getAction().beliefAddress, "looked-at", true);

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

	public static abstract class LearnInstructionExecutor<ActionType extends BeliefPlusStringAction>
			extends
			NonBlockingCompleteFromStatusExecutor<ActionType, VisualLearningTask> {

		private final String m_concept;
		private final String m_featurePostfix;
		private final double m_weight;

		public LearnInstructionExecutor(ManagedComponent _component,
				Class<ActionType> _actCls, String _concept, double _weight,
				String _featurePostfix) {
			super(_component, _actCls, VisualLearningTask.class);
			m_concept = _concept;
			m_weight = _weight;
			m_featurePostfix = _featurePostfix;

		}

		protected boolean acceptAction(ActionType _action) {
			return true;
		}

		@Override
		protected TriBool executionResult(VisualLearningTask _cmd) {
			if (_cmd.status == VisionCommandStatus.VCSUCCEEDED) {
				return TriBool.TRITRUE;
			} else {
				return TriBool.TRIFALSE;
			}
		}

		@Override
		protected VisionActionInterface getComponent() {
			return (VisionActionInterface) super.getComponent();
		}

		@Override
		public void executeAction() {
			try {
				WorkingMemoryAddress beliefID = getAction().beliefAddress;

				VisualLearningTask cmd = null;

				WorkingMemoryPointer visualObectPtr = ((VisionActionInterface) getComponent())
						.getFirstAncestorOfBelief(getAction().beliefAddress);

				cmd = new VisualLearningTask(visualObectPtr, beliefID.id,
						m_concept, new String[] { getAction().value },
						new double[] { m_weight },
						VisionCommandStatus.VCREQUESTED);

				// Hack by Alen - is this still needed in year 3?
				// getComponent().addBooleanFeature(getAction().beliefAddress,
				// m_concept + m_featurePostfix, true);

				addThenCompleteOnOverwrite(cmd);

				// getComponent().sleepComponent(10000);

			} catch (CASTException e) {
				getComponent().logException(e);
			}
		}

		@Override
		protected void actionComplete() {
			try {
				getComponent().addFeature(getAction().beliefAddress,
						m_concept + m_featurePostfix, true);

				// Hack by Alen
				// getComponent().sleepComponent(10000);
			} catch (CASTException e) {
				logException(e);
			}

		}
	}

	public static class LearnColourExecutor extends
			LearnInstructionExecutor<LearnColour> {

		public LearnColourExecutor(ManagedComponent _component) {
			super(_component, LearnColour.class, "color", 1,
					LEARNED_FEATURE_POSTFIX);
		}
	}

	public static class LearnShapeExecutor extends
			LearnInstructionExecutor<LearnShape> {

		public LearnShapeExecutor(ManagedComponent _component) {
			super(_component, LearnShape.class, "shape", 1,
					LEARNED_FEATURE_POSTFIX);
		}

	}

	public static class LearnIdentityExecutor extends
			LearnInstructionExecutor<LearnIdentity> {

		public LearnIdentityExecutor(ManagedComponent _component) {
			super(_component, LearnIdentity.class, "ident", 1,
					LEARNED_FEATURE_POSTFIX);
		}
	}

	public static class UnlearnColourExecutor extends
			LearnInstructionExecutor<UnlearnColour> {

		public UnlearnColourExecutor(ManagedComponent _component) {
			super(_component, UnlearnColour.class, "color", -1,
					UNLEARNED_FEATURE_POSTFIX);
		}
	}

	public static class UnlearnShapeExecutor extends
			LearnInstructionExecutor<UnlearnShape> {

		public UnlearnShapeExecutor(ManagedComponent _component) {
			super(_component, UnlearnShape.class, "shape", -1,
					UNLEARNED_FEATURE_POSTFIX);
		}

	}

	public static class UnlearnIdentityExecutor extends
			LearnInstructionExecutor<UnlearnIdentity> {

		public UnlearnIdentityExecutor(ManagedComponent _component) {
			super(_component, UnlearnIdentity.class, "ident", -1, "-unlearned");
		}
	}

	public static class AnalyzeProtoObjectExecutor
			extends
			NonBlockingCompleteFromStatusExecutor<AnalyzeProtoObject, AnalyzeProtoObjectCommand> {

		public AnalyzeProtoObjectExecutor(ManagedComponent _component) {
			super(_component, AnalyzeProtoObject.class,
					AnalyzeProtoObjectCommand.class);
		}

		@Override
		protected TriBool executionResult(AnalyzeProtoObjectCommand _cmd) {
			try {
				if (_cmd.status == VisionCommandStatus.VCSUCCEEDED) {
					return TriBool.TRITRUE;
				} else {
					return TriBool.TRIFALSE;
				}
			} catch (Exception e) {
				logException(e);
				return TriBool.TRIFALSE;
			}
		}

		@Override
		public void executeAction() {
			try {

				WorkingMemoryPointer protoObjPtr = BeliefUtils
						.recurseAncestorsForType(getComponent(),
								getAction().beliefAddress,
								CASTUtils.typeName(ProtoObject.class));

				if (protoObjPtr == null) {
					getComponent()
							.getLogger()
							.warn("Action failed because ProtoObject pointer was null",
									getComponent().getLogAdditions());
					executionComplete(TriBool.TRIFALSE);
				} else {
					log("belief addr "
							+ CASTUtils.toString(getAction().beliefAddress)
							+ " yielded PO addr "
							+ CASTUtils.toString(protoObjPtr.address));
					AnalyzeProtoObjectCommand cmd = new AnalyzeProtoObjectCommand();
					cmd.protoObjectAddr = protoObjPtr.address;
					cmd.status = VisionCommandStatus.VCREQUESTED;
					addThenCompleteOnOverwrite(cmd);
				}
			} catch (Exception e) {
				logException(e);
				executionComplete(TriBool.TRIFALSE);
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

		// This component is in charge of adding the inital robot struct
		if (m_viewStateAddress == null) {
			m_viewStateAddress = new WorkingMemoryAddress(newDataID(),
					getSubarchitectureID());
			Robot rbt = new Robot(_viewconePtr, m_armIsRestingInitially, false,
					false);
			addToWorkingMemory(m_viewStateAddress, rbt);
		} else {
			lockEntry(m_viewStateAddress, WorkingMemoryPermissions.LOCKEDODR);
			Robot rbt = getMemoryEntry(m_viewStateAddress, Robot.class);
			rbt.currentViewCone = _viewconePtr;
			overwriteWorkingMemory(m_viewStateAddress, rbt);
			unlockEntry(m_viewStateAddress);
		}
	}

	@Override
	protected void configure(Map<String, String> _config) {
		m_ptzServerComponent = _config.get("--ptzserver");
		if (m_ptzServerComponent == null) {
			m_ptzServerComponent = "ptz.server";
		}

		m_fakeRobotPose = _config.containsKey("--fake-pose");

		// TODO allow for configuration, but default should be true
		m_armIsRestingInitially = true;
	}

	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);

		m_actionStateManager
				.registerActionType(
						MoveToViewCone.class,
						new ComponentActionFactory<MoveToViewCone, MoveToViewConeExecutor>(
								this, MoveToViewConeExecutor.class));

		m_actionStateManager
				.registerActionType(
						AnalyzeProtoObject.class,
						new ComponentActionFactory<AnalyzeProtoObject, AnalyzeProtoObjectExecutor>(
								this, AnalyzeProtoObjectExecutor.class));

		m_actionStateManager.registerActionType(LearnColour.class,
				new ComponentActionFactory<LearnColour, LearnColourExecutor>(
						this, LearnColourExecutor.class));
		m_actionStateManager.registerActionType(LearnShape.class,
				new ComponentActionFactory<LearnShape, LearnShapeExecutor>(
						this, LearnShapeExecutor.class));
		m_actionStateManager
				.registerActionType(
						LearnIdentity.class,
						new ComponentActionFactory<LearnIdentity, LearnIdentityExecutor>(
								this, LearnIdentityExecutor.class));

		m_actionStateManager
				.registerActionType(
						UnlearnColour.class,
						new ComponentActionFactory<UnlearnColour, UnlearnColourExecutor>(
								this, UnlearnColourExecutor.class));
		m_actionStateManager.registerActionType(UnlearnShape.class,
				new ComponentActionFactory<UnlearnShape, UnlearnShapeExecutor>(
						this, UnlearnShapeExecutor.class));
		m_actionStateManager
				.registerActionType(
						UnlearnIdentity.class,
						new ComponentActionFactory<UnlearnIdentity, UnlearnIdentityExecutor>(
								this, UnlearnIdentityExecutor.class));

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(RobotPose2d.class),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {

						boolean success = createInitialView(_wmc.address);
						if (success) {
							removeChangeFilter(this);
						}
					}
				});

	}

	private static ViewCone createViewConeFromPosition(RobotPose2d _pose,
			PTZReading _ptz) {
		ViewCone vc = new ViewCone();
		vc.anchor = new Vector3(_pose.x, _pose.y, _pose.theta);
		vc.x = 0;
		vc.y = 0;
		vc.viewDirection = _pose.theta + _ptz.pose.pan;
		vc.tilt = _ptz.pose.tilt;
		vc.target = null;
		return vc;
	}

	/**
	 * Creates a ViewCone from the robot's current position and uses records it
	 * as the current active viewcone.
	 * 
	 * @param _poseAddr
	 * @return
	 * @throws CASTException
	 */
	private boolean createInitialView(WorkingMemoryAddress _poseAddr)
			throws CASTException {
		PTZInterfacePrx ptz = getIceServer(m_ptzServerComponent,
				PTZInterface.class, PTZInterfacePrx.class);
		if (ptz != null) {
			PTZReading currentPTZPose = ptz.getPose();
			RobotPose2d currentRobotPose = getMemoryEntry(_poseAddr,
					RobotPose2d.class);
			ViewCone vc = createViewConeFromPosition(currentRobotPose,
					currentPTZPose);
			WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(),
					getSubarchitectureID());
			addToWorkingMemory(wma, vc);
			recordCurrentViewCone(new WorkingMemoryPointer(wma,
					CASTUtils.typeName(vc)));
		}
		return false;
	}

	@Override
	protected void runComponent() {
		if (m_fakeRobotPose) {
			getLogger()
					.warn("Faking robot pose. Don't use when real robot poses are being generated");
			RobotPose2d pose = new RobotPose2d(getCASTTime(), 0, 0, 0, null);
			try {
				addToWorkingMemory(newDataID(), pose);
			} catch (AlreadyExistsOnWMException e) {
				logException(e);
			}
		}
	}

}
