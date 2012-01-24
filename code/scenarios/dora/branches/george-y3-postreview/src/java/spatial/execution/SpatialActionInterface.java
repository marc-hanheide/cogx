/**
 * 
 */
package spatial.execution;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;

import SpatialData.AVSStatus;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Place;
import SpatialData.Priority;
import SpatialData.ProcessConeGroup;
import SpatialData.ProcessViewPointCommand;
import SpatialData.RelationalViewPointGenerationCommand;
import SpatialData.SpatialRelation;
import SpatialData.StatusError;
import SpatialData.ViewPoint;
import SpatialData.ViewPointGenerationCommand;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import execution.slice.TriBool;
import execution.slice.actions.CreateConesForModel;
import execution.slice.actions.CreateRelationalConesForModel;
import execution.slice.actions.ExplorePlace;
import execution.slice.actions.GoToPlace;
import execution.slice.actions.LookForObjects;
import execution.slice.actions.LookForPeople;
import execution.slice.actions.ProcessCone;
import execution.slice.actions.ProcessConeGroupAction;
import execution.slice.actions.ProcessConesAtPlace;
import execution.slice.actions.TurnToHuman;
import execution.util.ActionExecutor;
import execution.util.ActionExecutorFactory;
import execution.util.BlockingActionExecutor;
import execution.util.ComponentActionFactory;
import execution.util.LocalActionStateManager;
import execution.util.NonBlockingCompleteOnOperationExecutor;

/**
 * Component to listen to planner actions the trigger the spatial sa as
 * appropriate.
 * 
 * @author nah
 * 
 */
public class SpatialActionInterface extends ManagedComponent {

	public class AlwaysSucceedsExecutor extends
			BlockingActionExecutor<ExplorePlace> {

		public AlwaysSucceedsExecutor(ManagedComponent _component) {
			super(_component, ExplorePlace.class);
		}

		@Override
		public TriBool execute() {
			return TriBool.TRITRUE;
		}

	}

	public class AlwaysSucceedsExecutorFactory extends
			ComponentActionFactory<ExplorePlace, AlwaysSucceedsExecutor> {

		public AlwaysSucceedsExecutorFactory(ManagedComponent _component) {
			super(_component, AlwaysSucceedsExecutor.class);
		}

	}

	public static class ViewConeGenerationExecutor extends
			NonBlockingCompleteOnOperationExecutor<CreateConesForModel> {

		public ViewConeGenerationExecutor(ManagedComponent _component) {
			super(_component, CreateConesForModel.class);
		}

		@Override
		protected boolean acceptAction(CreateConesForModel _action) {
			return true;
		}

		@Override
		public void executeAction() {
			// first delete all cones from previous object call
			((SpatialActionInterface) getComponent())
					.removeObjectViewPoints(getAction().model);

			// the generate new ones
			ViewPointGenerationCommand cmd = new ViewPointGenerationCommand(
					getAction().model, getAction().placeIDs,
					AVSStatus.INPROGRESS);
			addThenCompleteOnOverwrite(cmd);
		}
	}

	public static class RelationalViewConeGenerationExecutor
			extends
			NonBlockingCompleteOnOperationExecutor<CreateRelationalConesForModel> {

		public RelationalViewConeGenerationExecutor(ManagedComponent _component) {
			super(_component, CreateRelationalConesForModel.class);
		}

		@Override
		protected boolean acceptAction(CreateRelationalConesForModel _action) {
			return true;
		}

		@Override
		public void executeAction() {
			// first delete all cones from previous object call
			// ((SpatialActionInterface) getComponent())
			// .removeObjectViewPoints(getAction().model);
			SpatialRelation rel = null;
			if ("inroom".equals(getAction().relation)) {
				rel = SpatialRelation.INROOM;
			} else if ("in".equals(getAction().relation)) {
				rel = SpatialRelation.INOBJECT;
			} else if ("on".equals(getAction().relation)) {
				rel = SpatialRelation.ON;
			} else {
				assert false : "unknown relation: " + getAction().relation;
			}

			// the generate new ones
			RelationalViewPointGenerationCommand cmd = new RelationalViewPointGenerationCommand(
					getAction().model, rel, getAction().supportObject,
					getAction().supportObjectCategory, getAction().roomID,
					AVSStatus.INPROGRESS);
			addThenCompleteOnOverwrite(cmd);
		}
	}

	public static class ViewConeProcessExecutor extends
			NonBlockingCompleteOnOperationExecutor<ProcessCone> {

		private ViewPoint m_vp;

		public ViewConeProcessExecutor(ManagedComponent _component) {
			super(_component, ProcessCone.class);
		}

		@Override
		protected boolean acceptAction(ProcessCone _action) {
			try {
				m_vp = getComponent().getMemoryEntry(_action.coneAddress,
						ViewPoint.class);
			} catch (CASTException e) {
				logException(e);
				return false;
			}
			return true;
		}

		@Override
		public void executeAction() {
			ProcessViewPointCommand cmd = new ProcessViewPointCommand(
					AVSStatus.INPROGRESS, m_vp, new String[] { m_vp.label });
			addThenCompleteOnOverwrite(cmd);
		}
	}

	public static class ConeGroupProcessExecutor extends
			NonBlockingCompleteOnOperationExecutor<ProcessConeGroupAction> {

		private long coneId;

		public ConeGroupProcessExecutor(ManagedComponent _component) {
			super(_component, ProcessConeGroupAction.class);
		}

		@Override
		protected boolean acceptAction(ProcessConeGroupAction _action) {
			// try {
			coneId = _action.coneGroupID;
			// } catch (CASTException e) {
			// logException(e);
			// return false;
			// }
			return true;
		}

		@Override
		public void executeAction() {
			ProcessConeGroup cmd = new ProcessConeGroup(AVSStatus.INPROGRESS,
					coneId);
			addThenCompleteOnOverwrite(cmd);
		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {

			// read in the nav cmd
			getComponent().lockEntry(_wmc.address,
					WorkingMemoryPermissions.LOCKEDODR);
			ProcessConeGroup cmd = getComponent().getMemoryEntry(_wmc.address,
					ProcessConeGroup.class);
			if (cmd.status == AVSStatus.FAILED) {
				log("command failed by the looks of this: " + cmd.status);
				getComponent().unlockEntry(_wmc.address);
				actionComplete();
				executionComplete(TriBool.TRIFALSE);
				getComponent().removeChangeFilter(this);
			} else if (cmd.status == AVSStatus.SUCCESS) {
				log("command completed by the looks of this: " + cmd.status);
				getComponent().unlockEntry(_wmc.address);
				actionComplete();
				executionComplete(TriBool.TRITRUE);
				getComponent().removeChangeFilter(this);
			} else {
				log("command in progress: " + cmd.status);
				getComponent().unlockEntry(_wmc.address);
			}
		}

	}

	public static class ProcessAllViewConesAtPlaceExecutor extends
			NonBlockingCompleteOnOperationExecutor<ProcessConesAtPlace> {

		private List<ViewPoint> m_vps;

		public ProcessAllViewConesAtPlaceExecutor(ManagedComponent _component) {
			super(_component, ProcessConesAtPlace.class);
		}

		@Override
		public void executeAction() {

			try {
				m_vps = ((SpatialActionInterface) getComponent())
						.getViewPointsForPlaceAndObject(
								(int) getAction().placeID, getAction().model);

				// for (ViewPoint vp : m_vps) {
				// log("cone to process: " + vp.label + " " + vp.probability);
				// }

				processNextCone();

			} catch (CASTException e) {
				logException(e);
				executionComplete(TriBool.TRIFALSE);
			}

		}

		private void processNextCone() throws AlreadyExistsOnWMException,
				DoesNotExistOnWMException, UnknownSubarchitectureException {
			if (m_vps.isEmpty()) {
				executionComplete(TriBool.TRITRUE);
				return;
			}
			ViewPoint vp = m_vps.remove(0);
			ProcessViewPointCommand cmd = new ProcessViewPointCommand(
					AVSStatus.INPROGRESS, vp, new String[] { vp.label });

			WorkingMemoryAddress cmdAddr = newWorkingMemoryAddress();

			getComponent().addChangeFilter(
					ChangeFilterFactory.createAddressFilter(cmdAddr,
							WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {

						@Override
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) throws CASTException {
							// delete previous command
							try {
								getComponent().removeChangeFilter(this);
								getComponent().deleteFromWorkingMemory(
										_wmc.address);
								processNextCone();
							} catch (CASTException e) {
								logException(
										"Exception during all cone execution, failing action",
										e);
								executionComplete(TriBool.TRIFALSE);
							}
						}
					});
			getComponent().addToWorkingMemory(cmdAddr, cmd);

		}
	}

	/**
	 * 
	 * @param _placeID
	 * @param _model
	 * @return
	 * @throws UnknownSubarchitectureException
	 * @throws DoesNotExistOnWMException
	 */
	private List<ViewPoint> getViewPointsForPlaceAndObject(int _placeID,
			String _model) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		List<ViewPoint> vps = Collections.emptyList();

		if (m_viewPointsForObject != null) {

			HashSet<WorkingMemoryAddress> addressSet = m_viewPointsForObject
					.get(_model);

			if (addressSet != null) {
				vps = new ArrayList<ViewPoint>(addressSet.size());
				for (WorkingMemoryAddress vpAddr : addressSet) {
					ViewPoint vp = getMemoryEntry(vpAddr, ViewPoint.class);
					if (vp.closestPlaceId == _placeID) {
						vps.add(vp);
					}
				}

				// sort to start with most likely place to find an object
				Collections.sort(vps,
						ViewPointProbabilityComparator.getInstance());
			}
		}
		return vps;
	}

	// not used in year 2
	// private class AVSExecutor extends Thread implements ActionExecutor,
	// WorkingMemoryChangeReceiver {
	//
	// private WorkingMemoryAddress m_avsAddr;
	// private AVSCommand m_avsCmd;
	// private long[] m_avsPlaceIDs;
	// private ExecutionCompletionCallback m_callback;
	// private boolean m_isStopped;
	//
	// public AVSExecutor() {
	// m_isStopped = false;
	// }
	//
	// @Override
	// public boolean accept(Action _action) {
	// m_avsPlaceIDs = ((ActiveVisualSearch) _action).placeIDs;
	// return true;
	// }
	//
	// @Override
	// public TriBool execute() {
	// return null;
	// }
	//
	// @Override
	// public void execute(ExecutionCompletionCallback _callback) {
	//
	// log("running AVS on places: " + Arrays.toString(m_avsPlaceIDs));
	//
	// // avs never returns, so just just keep listening
	// m_avsCmd = new AVSCommand(m_avsPlaceIDs, AVSAction.PLAN);
	// m_avsAddr = new WorkingMemoryAddress(newDataID(),
	// getSubarchitectureID());
	// try {
	// addChangeFilter(ChangeFilterFactory.createAddressFilter(
	// m_avsAddr, WorkingMemoryOperation.DELETE), this);
	// addToWorkingMemory(m_avsAddr, m_avsCmd);
	// m_callback = _callback;
	// if (useAVSTimeout()) {
	// start();
	// }
	// } catch (CASTException e) {
	// println(e.message);
	// e.printStackTrace();
	// _callback.executionComplete(TriBool.TRIFALSE);
	// }
	// }
	//
	// @Override
	// public boolean isBlockingAction() {
	// return false;
	// }
	//
	// @Override
	// public void run() {
	// // sleep for the timeout
	// if (useAVSTimeout()) {
	// try {
	// Thread.sleep(m_avsTimeoutMillis);
	// if (!m_isStopped) {
	// log("halting AVS after timeout");
	// // stop avs
	// m_avsCmd.cmd = AVSAction.STOPAVS;
	// overwriteWorkingMemory(m_avsAddr, m_avsCmd);
	// removeChangeFilter(this);
	// m_callback.executionComplete(TriBool.TRITRUE);
	// }
	// } catch (InterruptedException e) {
	// e.printStackTrace();
	// } catch (DoesNotExistOnWMException e) {
	// e.printStackTrace();
	// } catch (ConsistencyException e) {
	// e.printStackTrace();
	// } catch (PermissionException e) {
	// e.printStackTrace();
	// } catch (UnknownSubarchitectureException e) {
	// e.printStackTrace();
	// } catch (SubarchitectureComponentException e) {
	// // TODO Auto-generated catch block
	// e.printStackTrace();
	// }
	// }
	// }
	//
	// @Override
	// public void stopExecution() {
	// if (!m_isStopped) {
	// try {
	// m_isStopped = true;
	// m_avsCmd.cmd = AVSAction.STOPAVS;
	// overwriteWorkingMemory(m_avsAddr, m_avsCmd);
	// removeChangeFilter(this);
	//
	// } catch (DoesNotExistOnWMException e) {
	// e.printStackTrace();
	// } catch (ConsistencyException e) {
	// e.printStackTrace();
	// } catch (PermissionException e) {
	// e.printStackTrace();
	// } catch (UnknownSubarchitectureException e) {
	// e.printStackTrace();
	// } catch (SubarchitectureComponentException e) {
	// e.printStackTrace();
	// }
	// }
	// }
	//
	// @Override
	// public void workingMemoryChanged(WorkingMemoryChange _arg0)
	// throws CASTException {
	// // only called when command has been deleted, i.e. avs has finished
	// // for some reason
	// if (!m_isStopped) {
	// println("AVS finished on its own");
	// m_isStopped = true;
	// m_callback.executionComplete(TriBool.TRITRUE);
	// try {
	// removeChangeFilter(this);
	// } catch (SubarchitectureComponentException e) {
	// e.printStackTrace();
	// }
	// }
	// }
	//
	// }

	// not used in year 2
	// private class AVSExecutor extends Thread implements ActionExecutor,
	// WorkingMemoryChangeReceiver {
	//
	// private WorkingMemoryAddress m_avsAddr;
	// private AVSCommand m_avsCmd;
	// private long[] m_avsPlaceIDs;
	// private ExecutionCompletionCallback m_callback;
	// private boolean m_isStopped;
	//
	// public AVSExecutor() {
	// m_isStopped = false;
	// }
	//
	// @Override
	// public boolean accept(Action _action) {
	// m_avsPlaceIDs = ((ActiveVisualSearch) _action).placeIDs;
	// return true;
	// }
	//
	// @Override
	// public TriBool execute() {
	// return null;
	// }
	//
	// @Override
	// public void execute(ExecutionCompletionCallback _callback) {
	//
	// log("running AVS on places: " + Arrays.toString(m_avsPlaceIDs));
	//
	// // avs never returns, so just just keep listening
	// m_avsCmd = new AVSCommand(m_avsPlaceIDs, AVSAction.PLAN);
	// m_avsAddr = new WorkingMemoryAddress(newDataID(),
	// getSubarchitectureID());
	// try {
	// addChangeFilter(ChangeFilterFactory.createAddressFilter(
	// m_avsAddr, WorkingMemoryOperation.DELETE), this);
	// addToWorkingMemory(m_avsAddr, m_avsCmd);
	// m_callback = _callback;
	// if (useAVSTimeout()) {
	// start();
	// }
	// } catch (CASTException e) {
	// println(e.message);
	// e.printStackTrace();
	// _callback.executionComplete(TriBool.TRIFALSE);
	// }
	// }
	//
	// @Override
	// public boolean isBlockingAction() {
	// return false;
	// }
	//
	// @Override
	// public void run() {
	// // sleep for the timeout
	// if (useAVSTimeout()) {
	// try {
	// Thread.sleep(m_avsTimeoutMillis);
	// if (!m_isStopped) {
	// log("halting AVS after timeout");
	// // stop avs
	// m_avsCmd.cmd = AVSAction.STOPAVS;
	// overwriteWorkingMemory(m_avsAddr, m_avsCmd);
	// removeChangeFilter(this);
	// m_callback.executionComplete(TriBool.TRITRUE);
	// }
	// } catch (InterruptedException e) {
	// e.printStackTrace();
	// } catch (DoesNotExistOnWMException e) {
	// e.printStackTrace();
	// } catch (ConsistencyException e) {
	// e.printStackTrace();
	// } catch (PermissionException e) {
	// e.printStackTrace();
	// } catch (UnknownSubarchitectureException e) {
	// e.printStackTrace();
	// } catch (SubarchitectureComponentException e) {
	// // TODO Auto-generated catch block
	// e.printStackTrace();
	// }
	// }
	// }
	//
	// @Override
	// public void stopExecution() {
	// if (!m_isStopped) {
	// try {
	// m_isStopped = true;
	// m_avsCmd.cmd = AVSAction.STOPAVS;
	// overwriteWorkingMemory(m_avsAddr, m_avsCmd);
	// removeChangeFilter(this);
	//
	// } catch (DoesNotExistOnWMException e) {
	// e.printStackTrace();
	// } catch (ConsistencyException e) {
	// e.printStackTrace();
	// } catch (PermissionException e) {
	// e.printStackTrace();
	// } catch (UnknownSubarchitectureException e) {
	// e.printStackTrace();
	// } catch (SubarchitectureComponentException e) {
	// e.printStackTrace();
	// }
	// }
	// }
	//
	// @Override
	// public void workingMemoryChanged(WorkingMemoryChange _arg0)
	// throws CASTException {
	// // only called when command has been deleted, i.e. avs has finished
	// // for some reason
	// if (!m_isStopped) {
	// println("AVS finished on its own");
	// m_isStopped = true;
	// m_callback.executionComplete(TriBool.TRITRUE);
	// try {
	// removeChangeFilter(this);
	// } catch (SubarchitectureComponentException e) {
	// e.printStackTrace();
	// }
	// }
	// }
	//
	// }

	private class GoToPlaceExecutor implements ActionExecutor<GoToPlace>,
			WorkingMemoryChangeReceiver {

		private ExecutionCompletionCallback m_callback;
		private boolean m_isComplete = false;
		private WorkingMemoryAddress m_navCmdAddr;
		private long m_placeID;

		@Override
		public Class<GoToPlace> getActionClass() {
			return GoToPlace.class;
		}

		public boolean accept(GoToPlace _action) {
			m_placeID = _action.placeID;
			return true;
		}

		public TriBool execute() {
			return null;
		}

		public void execute(ExecutionCompletionCallback _callback) {
			// if we haven't seen this place, then fail
			if (!m_placeIDs.contains(m_placeID)) {
				_callback.executionComplete(TriBool.TRIFALSE);
				return;
			}

			// else create the command and send it off, ignoring path transition
			// probs for now
			NavCommand cmd = newNavCommand();
			cmd.cmd = CommandType.GOTOPLACE;
			cmd.destId = new long[1];
			cmd.destId[0] = m_placeID;

			// going to add locally for the time being, but using wma to allow
			// this to be changed later
			m_navCmdAddr = new WorkingMemoryAddress(newDataID(),
					getSubarchitectureID());
			addChangeFilter(ChangeFilterFactory.createAddressFilter(
					m_navCmdAddr, WorkingMemoryOperation.OVERWRITE), this);
			m_callback = _callback;

			try {
				addToWorkingMemory(m_navCmdAddr, cmd);
			} catch (CASTException e) {
				logException(e);
				_callback.executionComplete(TriBool.TRIFALSE);
			}
		}

		public boolean isBlockingAction() {
			return false;
		}

		@Override
		public void stopExecution() {
			// remove overwrite receiver
			if (!m_isComplete) {
				try {
					log("aborting execution");
					removeChangeFilter(this);
					// reread
					lockEntry(m_navCmdAddr, WorkingMemoryPermissions.LOCKEDODR);
					NavCommand navCmd = getMemoryEntry(m_navCmdAddr,
							NavCommand.class);
					navCmd.comp = Completion.COMMANDABORTED;
					overwriteWorkingMemory(m_navCmdAddr, navCmd);
					unlockEntry(m_navCmdAddr);

				} catch (DoesNotExistOnWMException e) {
					//
					// e.printStackTrace();
				} catch (SubarchitectureComponentException e) {
					e.printStackTrace();
				}
			}
		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {

			// read in the nav cmd
			lockEntry(_wmc.address, WorkingMemoryPermissions.LOCKEDODR);
			NavCommand cmd = getMemoryEntry(_wmc.address, NavCommand.class);
			if (cmd.comp == Completion.COMMANDFAILED) {
				log("command failed by the looks of this: " + cmd.comp);
				m_isComplete = true;
				m_callback.executionComplete(TriBool.TRIFALSE);
				deleteFromWorkingMemory(_wmc.address);
				removeChangeFilter(this);
			} else if (cmd.comp == Completion.COMMANDSUCCEEDED) {
				log("command completed by the looks of this: " + cmd.comp);
				m_isComplete = true;
				m_callback.executionComplete(TriBool.TRITRUE);
				deleteFromWorkingMemory(_wmc.address);
				removeChangeFilter(this);
			} else if (cmd.comp == Completion.COMMANDABORTED) {
				log("command aborted by the looks of this: " + cmd.comp);
				m_isComplete = true;
				// FIXME: We should really check the cmd's status for
				// success/failure,
				// But the planner performs better if we always signal TRITRUE
				// here.
				m_callback.executionComplete(TriBool.TRITRUE);
				deleteFromWorkingMemory(_wmc.address);
				removeChangeFilter(this);
			} else {
				log("command in progress: " + cmd.comp);
				unlockEntry(_wmc.address);
			}
		}
	}

	public class LookForPeopleExecutorFactory implements
			ActionExecutorFactory<LookForPeople> {

		private final ManagedComponent m_component;

		public LookForPeopleExecutorFactory(ManagedComponent _component) {
			m_component = _component;
		}

		@Override
		public ActionExecutor<LookForPeople> getActionExecutor() {
			return new LookForPeopleExecutor(m_component, m_detections);
		}

	}

	public class TurnToPersonExecutorFactory implements
			ActionExecutorFactory<TurnToHuman> {

		private final ManagedComponent m_component;

		public TurnToPersonExecutorFactory(ManagedComponent _component) {
			m_component = _component;
		}

		@Override
		public ActionExecutor<TurnToHuman> getActionExecutor() {
			return new TurnToPersonExecutor(m_component);
		}

	}

	public class LookForObjectsExecutorFactory implements
			ActionExecutorFactory<LookForObjects> {

		private final ManagedComponent m_component;

		public LookForObjectsExecutorFactory(ManagedComponent _component) {
			m_component = _component;
		}

		@Override
		public ActionExecutor<LookForObjects> getActionExecutor() {
			return new LookForObjectsExecutor(m_component, m_detections);
		}

	}

	/**
	 * Sets all values necessary to prevent exceptions later on
	 */
	public static NavCommand newNavCommand() {
		return new NavCommand(CommandType.STOP, Priority.NORMAL, null, null,
				null, null, null, StatusError.NONE, Completion.COMMANDPENDING);
	}

	private LocalActionStateManager m_actionStateManager;

	/**
	 * AVS timeout. If less than or equal to 0 there is no timeout. Default to
	 * 0.
	 * 
	 */
	private long m_avsTimeoutMillis = 0;

	private HashSet<Long> m_placeIDs;

	private int m_detections;

	private HashMap<String, HashSet<WorkingMemoryAddress>> m_viewPointsForObject;

	// private HashMap<Integer, HashSet<WorkingMemoryAddress>>
	// m_viewPointsForPlace;

	public SpatialActionInterface() {
		m_detections = 4;
	}

	@Override
	protected void configure(Map<String, String> _config) {
		String avsTimeoutString = _config.get("--avs-timeout");
		if (avsTimeoutString != null) {
			m_avsTimeoutMillis = Long.parseLong(avsTimeoutString);
		}

		if (useAVSTimeout()) {
			log("using AVS timeout of: " + m_avsTimeoutMillis + "ms");
		} else {
			log("no AVS timeout");
		}

		String numDetections = _config.get("--detections");
		if (numDetections != null) {
			m_detections = Integer.parseInt(numDetections);
		}
		log("when looking for people I will run " + m_detections
				+ " detections");

	}

	@Override
	protected void start() {
		m_actionStateManager = new LocalActionStateManager(this);

		m_actionStateManager.registerActionType(GoToPlace.class,
				new ActionExecutorFactory<GoToPlace>() {
					@Override
					public ActionExecutor<GoToPlace> getActionExecutor() {
						return new GoToPlaceExecutor();
					}
				});

		m_actionStateManager.registerActionType(TurnToHuman.class,
				new TurnToPersonExecutorFactory(this));

		final AlwaysSucceedsExecutorFactory alwaysExplore = new AlwaysSucceedsExecutorFactory(
				this);

		m_actionStateManager.registerActionType(ExplorePlace.class,
				alwaysExplore);

		m_actionStateManager.registerActionType(LookForObjects.class,
				new LookForObjectsExecutorFactory(this));
		m_actionStateManager.registerActionType(LookForPeople.class,
				new LookForPeopleExecutorFactory(this));

		m_actionStateManager
				.registerActionType(
						CreateConesForModel.class,
						new ComponentActionFactory<CreateConesForModel, ViewConeGenerationExecutor>(
								this, ViewConeGenerationExecutor.class));

		m_actionStateManager
				.registerActionType(
						CreateRelationalConesForModel.class,
						new ComponentActionFactory<CreateRelationalConesForModel, RelationalViewConeGenerationExecutor>(
								this,
								RelationalViewConeGenerationExecutor.class));

		m_actionStateManager
				.registerActionType(
						ProcessConeGroupAction.class,
						new ComponentActionFactory<ProcessConeGroupAction, ConeGroupProcessExecutor>(
								this, ConeGroupProcessExecutor.class));

		m_actionStateManager
				.registerActionType(
						ProcessCone.class,
						new ComponentActionFactory<ProcessCone, ViewConeProcessExecutor>(
								this, ViewConeProcessExecutor.class));

		m_actionStateManager
				.registerActionType(
						ProcessConesAtPlace.class,
						new ComponentActionFactory<ProcessConesAtPlace, ProcessAllViewConesAtPlaceExecutor>(
								this, ProcessAllViewConesAtPlaceExecutor.class));

		// add a listener to check for place ids, for checking purposes
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			/**
			 * @param _place
			 */
			private void addPlace(Place _place) {
				// lazy creation
				if (m_placeIDs == null) {
					m_placeIDs = new HashSet<Long>();
				}

				m_placeIDs.add(_place.id);
				// log("stored id: " + _place.id);
			}

			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				Place place = getMemoryEntry(_wmc.address, Place.class);

				// store id
				addPlace(place);
			}
		});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				ViewPoint.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						addViewPoint(_wmc.address);
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				ViewPoint.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						removeViewPoint(_wmc.address);
					}
				});
	}

	private void removeViewPoint(WorkingMemoryAddress _address) {
		if (m_viewPointsForObject != null) {

			Set<Entry<String, HashSet<WorkingMemoryAddress>>> entrySet = m_viewPointsForObject
					.entrySet();
			for (Entry<String, HashSet<WorkingMemoryAddress>> entry : entrySet) {
				if (entry.getValue().remove(_address)) {
					return;
				}
			}
		}

	}

	private void addViewPoint(WorkingMemoryAddress _address) {
		try {
			ViewPoint vp = getMemoryEntry(_address, ViewPoint.class);
			storeObjectViewPointMapping(_address, vp);
			// storePlaceIDViewPointMapping(_address, vp);
		} catch (CASTException e) {
			logException(e);
		}
	}

	// /**
	// * @param _address
	// * @param vp
	// */
	// private void storePlaceIDViewPointMapping(WorkingMemoryAddress _address,
	// ViewPoint vp) {
	//
	// if (m_viewPointsForPlace == null) {
	// m_viewPointsForPlace = new HashMap<Integer,
	// HashSet<WorkingMemoryAddress>>();
	// }
	//
	// int placeID = vp.closestPlaceId;
	//
	// HashSet<WorkingMemoryAddress> objectSet = m_viewPointsForPlace
	// .get(placeID);
	// if (objectSet == null) {
	// objectSet = new HashSet<WorkingMemoryAddress>();
	// m_viewPointsForPlace.put(placeID, objectSet);
	// }
	// objectSet.add(_address);
	// }

	/**
	 * @param _address
	 * @param vp
	 */
	private void storeObjectViewPointMapping(WorkingMemoryAddress _address,
			ViewPoint vp) {

		if (m_viewPointsForObject == null) {
			m_viewPointsForObject = new HashMap<String, HashSet<WorkingMemoryAddress>>();
		}

		String objectLabel = vp.label;

		HashSet<WorkingMemoryAddress> objectSet = m_viewPointsForObject
				.get(objectLabel);
		if (objectSet == null) {
			objectSet = new HashSet<WorkingMemoryAddress>();
			m_viewPointsForObject.put(objectLabel, objectSet);
		}
		objectSet.add(_address);
	}

	private void removeObjectViewPoints(String _label) {
		assert (_label != null);
		// needed because the non-block action is run in a thread
		lockComponent();

		if (m_viewPointsForObject != null) {
			HashSet<WorkingMemoryAddress> viewPointAddressess = m_viewPointsForObject
					.remove(_label);
			if (viewPointAddressess == null) {
				println("no stored cones for object label: " + _label);
			} else {
				for (WorkingMemoryAddress addr : viewPointAddressess) {

					try {
						deleteFromWorkingMemory(addr);
						// removeStoredPlaceIDViewPointMapping(addr);
					} catch (CASTException e) {
						logException(e);
					}
				}

			}
		} else {
			println("no stored cones for object label: " + _label);
		}

		unlockComponent();

	}

	// private void removeStoredPlaceIDViewPointMapping(WorkingMemoryAddress
	// _addr) {
	// for (HashSet<WorkingMemoryAddress> hashSet : m_viewPointsForPlace
	// .values()) {
	// if (hashSet.remove(_addr)) {
	// return;
	// }
	// }
	// println("OHAI, we didn't have a viewpoint stored for that address");
	// }

	/**
	 * @return
	 */
	private boolean useAVSTimeout() {
		return m_avsTimeoutMillis > 0;
	}

}
