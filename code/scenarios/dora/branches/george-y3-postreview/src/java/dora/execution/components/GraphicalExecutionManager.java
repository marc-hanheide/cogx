/**
 * 
 */
package dora.execution.components;

import java.util.Map;

import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import dora.execution.util.ActionInterfaceFrame;
import eu.cogx.beliefs.slice.GroundedBelief;
import execution.components.AbstractExecutionManager;
import execution.slice.Action;
import execution.slice.actions.BackgroundModels;
import execution.slice.actions.BeliefPlusStringAction;
import execution.slice.actions.CreateConesForModel;
import execution.slice.actions.CreateRelationalConesForModel;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.slice.actions.EngageWithHuman;
import execution.slice.actions.ForegroundModels;
import execution.slice.actions.GoToPlace;
import execution.slice.actions.LookForObjects;
import execution.slice.actions.LookForPeople;
import execution.slice.actions.ProcessConeGroupAction;
import execution.slice.actions.ProcessConesAtPlace;
import execution.slice.actions.RecogniseForegroundedModels;
import execution.slice.actions.ReportPosition;
import execution.slice.actions.SingleBeliefAction;
import execution.util.ActionMonitor;

/**
 * 
 * Config options:
 * 
 * --labels=label1,label2,label3 etc. labels that are used to detect objects
 * 
 * @author nah
 * 
 */
public class GraphicalExecutionManager extends AbstractExecutionManager {

	private static final String[] DEFAULT_LABELS = { "record1", "record2",
			"record3", "record4" };

	private final ActionInterfaceFrame m_gui;

	private WorkingMemoryAddress m_currentActionAddress;

	public WorkingMemoryAddress getCurrentActionAddress() {
		return m_currentActionAddress;
	}

	/**
	 * 
	 */
	public GraphicalExecutionManager() {
		m_gui = new ActionInterfaceFrame(this);
		m_gui.pack();
		m_gui.setSize(500, 500);
		m_gui.setVisible(true);

	}

	@Override
	protected void configure(Map<String, String> _config) {
		String labels = _config.get("--labels");
		String[] objectLabels = DEFAULT_LABELS;
		if (labels != null) {
			objectLabels = labels.split(",");
		}

		log("using object labels: " + objectLabels);
		m_gui.setObjectModels(objectLabels);
	}

	@Override
	protected void start() {

		// use these to harvest beliefs
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				GroundedBelief.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						try {
							addStableBelief(_wmc.address, getMemoryEntry(
									_wmc.address, dBelief.class));
						} catch (CASTException e) {
							logException("Carry on regardless", e);
						}
					}
				});

		// use these to harvest beliefs
		// addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
		// GroundedBelief.class, WorkingMemoryOperation.OVERWRITE),
		// new WorkingMemoryChangeReceiver() {
		// @Override
		// public void workingMemoryChanged(WorkingMemoryChange _wmc)
		// throws CASTException {
		// try {
		// // FIXME: horribly inefficient I guess
		// removeStableBelief(_wmc.address);
		// addStableBelief(_wmc.address,
		// getMemoryEntry(_wmc.address, dBelief.class));
		// } catch (CASTException e) {
		// logException("Carry on regardless", e);
		// }
		// }
		// });

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				GroundedBelief.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						removeStableBelief(_wmc.address);
					}
				});

	}

	private void removeStableBelief(WorkingMemoryAddress _address) {
		m_gui.removeBelief(_address);
	}

	private void addStableBelief(WorkingMemoryAddress _address, dBelief _belief)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		IndependentFormulaDistributionsBelief<dBelief> b = IndependentFormulaDistributionsBelief
				.create(dBelief.class, _belief);
		m_gui.addBelief(_address, b);
	}

	// private void addPlace(WorkingMemoryAddress _address, Place _memoryEntry)
	// {
	// m_gui.addPlace(_address, _memoryEntry.id, _memoryEntry.status);
	// }
	//
	// private void updatePlace(WorkingMemoryAddress _address, Place
	// _memoryEntry) {
	// m_gui.updatePlace(_address, _memoryEntry.id, _memoryEntry.status);
	// }
	//
	//
	// private void removePlace(WorkingMemoryAddress _address) {
	// m_gui.removePlace(_address);
	// }

	public WorkingMemoryAddress triggerGoToAction(long _placeID,
			ActionMonitor _monitor) throws CASTException {
		GoToPlace act = newActionInstance(GoToPlace.class);
		act.placeID = _placeID;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerProcessConesAtPlace(long _placeID,
			String _model, ActionMonitor _monitor) throws CASTException {
		ProcessConesAtPlace act = newActionInstance(ProcessConesAtPlace.class);
		act.placeID = _placeID;
		act.model = _model;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	@Override
	public WorkingMemoryAddress triggerExecution(Action action,
			ActionMonitor monitor) throws AlreadyExistsOnWMException,
			DoesNotExistOnWMException, UnknownSubarchitectureException {

		WorkingMemoryAddress adr = super.triggerExecution(action, monitor);
		m_gui.setStatus(action);
		return adr;
	}

	public WorkingMemoryAddress triggerConeGeneration(String _model,
			int _roomId, ActionMonitor _monitor) throws CASTException {
		CreateRelationalConesForModel act = newActionInstance(CreateRelationalConesForModel.class);
		act.model = _model;
		act.relation = "inroom";
		act.roomID = _roomId;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerConeGenerationOnObject(String _model,
			String _supportModel, WorkingMemoryAddress _supportObject,
			int _roomId, ActionMonitor _monitor) throws CASTException {
		CreateRelationalConesForModel act = newActionInstance(CreateRelationalConesForModel.class);
		act.model = _model;
		act.supportObjectCategory = _supportModel;
		act.relation = "on";
		act.supportObject = _supportObject.id;
		act.roomID = _roomId;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerProccesCone(
			WorkingMemoryAddress _coneAddr, int _coneID, ActionMonitor _monitor)
			throws CASTException {
		ProcessConeGroupAction act = newActionInstance(ProcessConeGroupAction.class);
		act.coneGroupID = _coneID;
		act.coneGroupBeliefID = _coneAddr;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerDetectObjects(String[] _models,
			ActionMonitor _monitor) throws CASTException {
		DetectObjects act = newActionInstance(DetectObjects.class);
		act.labels = _models;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerDetectPeople(ActionMonitor _monitor)
			throws CASTException {
		DetectPeople act = newActionInstance(DetectPeople.class);
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerLookForObjects(String[] _models,
			ActionMonitor _monitor) throws CASTException {
		LookForObjects act = newActionInstance(LookForObjects.class);
		act.labels = _models;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerLookForPeople(ActionMonitor _monitor)
			throws CASTException {
		LookForPeople act = newActionInstance(LookForPeople.class);
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public boolean stopCurrentAction() throws PermissionException,
			UnknownSubarchitectureException {
		if (m_currentActionAddress == null) {
			return false;
		}
		try {
			deleteFromWorkingMemory(m_currentActionAddress);
			return true;
		} catch (DoesNotExistOnWMException e) {
			return false;
		}

	}

	public WorkingMemoryAddress backgroundModels(String[] _models,
			ActionMonitor _monitor) throws CASTException {
		BackgroundModels act = newActionInstance(BackgroundModels.class);
		act.models = _models;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress recogniseForegroundedModels(
			ActionMonitor _monitor) throws CASTException {
		RecogniseForegroundedModels act = newActionInstance(RecogniseForegroundedModels.class);
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress foregroundModels(String[] _models,
			ActionMonitor _monitor) throws CASTException {
		ForegroundModels act = newActionInstance(ForegroundModels.class);
		act.models = _models;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerReportPositionAction(
			WorkingMemoryAddress _beliefAddress, ActionMonitor _monitor)
			throws CASTException {
		ReportPosition act = newActionInstance(ReportPosition.class);
		act.beliefAddress = _beliefAddress;
		println("sending report position action");
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;

	}

	public WorkingMemoryAddress triggerEngagementAction(
			WorkingMemoryAddress _beliefAddress, boolean _engage,
			ActionMonitor _monitor) throws CASTException {
		EngageWithHuman act = newActionInstance(EngageWithHuman.class);
		act.beliefAddress = _beliefAddress;
		act.disengage = !_engage;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress executeSingleBeliefAction(
			WorkingMemoryAddress _beliefID, ActionMonitor _monitor,
			Class<? extends SingleBeliefAction> _actionCls)
			throws CASTException {
		SingleBeliefAction act = newActionInstance(_actionCls);
		act.beliefAddress = _beliefID;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress executeSingleBeliefPlusStringAction(
			WorkingMemoryAddress _beliefID, String _string,
			ActionMonitor _monitor,
			Class<? extends BeliefPlusStringAction> _actionCls)
			throws CASTException {
		BeliefPlusStringAction act = newActionInstance(_actionCls);
		act.beliefAddress = _beliefID;
		act.value = _string;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	@Override
	public boolean isPaused() {
		return false;
	}

	// public WorkingMemoryAddress triggerFeatureValueTest(String _beliefID,
	// String _featureLabel,
	// FeatureValue _fv, ActionMonitor _monitor) throws CASTException {
	// ComsysTestFeatureValue act =
	// newActionInstance(ComsysTestFeatureValue.class);
	// act.beliefID = _beliefID;
	// act.featureType = _featureLabel;
	// act.featureValue = _fv;
	// m_currentActionAddress = triggerExecution(act, _monitor);
	// return m_currentActionAddress;
	// }

}
