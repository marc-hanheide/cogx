/**
 * 
 */
package george.execution.components;

import java.util.Map;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import eu.cogx.beliefs.slice.GroundedBelief;
import execution.components.AbstractExecutionManager;
import execution.slice.actions.AskForColour;
import execution.slice.actions.AskForIdentity;
import execution.slice.actions.AskForObjectWithFeatureValue;
import execution.slice.actions.AskForShape;
import execution.slice.actions.AskPolarColour;
import execution.slice.actions.AskPolarIdentity;
import execution.slice.actions.AskPolarShape;
import execution.slice.actions.BackgroundModels;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.slice.actions.ForegroundModels;
import execution.slice.actions.LearnColour;
import execution.slice.actions.LearnIdentity;
import execution.slice.actions.LearnShape;
import execution.slice.actions.LookForObjects;
import execution.slice.actions.LookForPeople;
import execution.slice.actions.RecogniseForegroundedModels;
import execution.slice.actions.SingleBeliefAction;
import execution.slice.actions.UnlearnColour;
import execution.slice.actions.UnlearnIdentity;
import execution.slice.actions.UnlearnShape;
import execution.slice.actions.george.yr3.MoveToViewCone;
import execution.util.ActionMonitor;
import george.execution.util.ActionInterfaceFrame;

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

//	private static final String[] DEFAULT_LABELS = { "record1", "record2",
//			"record3", "record4" };

	private final ActionInterfaceFrame m_gui;

	private WorkingMemoryAddress m_currentActionAddress;

	/**
	 * 
	 */
	public GraphicalExecutionManager() {
		m_gui = new ActionInterfaceFrame(this);
		m_gui.pack();
		m_gui.setVisible(true);

	}

	@Override
	protected void configure(Map<String, String> _config) {
//		String labels = _config.get("--labels");
//		String[] objectLabels = DEFAULT_LABELS;
//		if (labels != null) {
//			objectLabels = labels.split(",");
//		}

//		log("using object labels: " + objectLabels);
//		m_gui.setObjectModels(objectLabels);
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
							addGroundedBelief(
									_wmc.address,
									getMemoryEntry(_wmc.address,
											GroundedBelief.class));
						} catch (CASTException e) {
							logException("Carry on regardless", e);
						}
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				GroundedBelief.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {

						removeGroundedBelief(_wmc.address);
						
					}
				});

	}

	private void removeGroundedBelief(WorkingMemoryAddress _address) {
		m_gui.removeBelief(_address);
	}

	private void addGroundedBelief(WorkingMemoryAddress _address,
			GroundedBelief _belief) {
		m_gui.addBelief(_address, _belief);
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

	// public WorkingMemoryAddress triggerAskForFeatureAction(String _beliefID,
	// String _featureType, ActionMonitor _monitor) throws CASTException {
	// AskForFeatureValue act = newActionInstance(AskForFeatureValue.class);
	// act.beliefID = _beliefID;
	// act.featureID = _featureType;
	// m_currentActionAddress = triggerExecution(act, _monitor);
	// return m_currentActionAddress;
	// }

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

	public WorkingMemoryAddress learnColour(WorkingMemoryAddress _beliefID,
			String _colour, ActionMonitor _monitor) throws CASTException {
		LearnColour act = newActionInstance(LearnColour.class);
		act.beliefAddress = _beliefID;
		act.value = _colour;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress learnShape(WorkingMemoryAddress _beliefID,
			String _shape, ActionMonitor _monitor) throws CASTException {
		LearnShape act = newActionInstance(LearnShape.class);
		act.beliefAddress = _beliefID;
		act.value = _shape;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress learnIdentity(WorkingMemoryAddress _beliefID,
			String _identity, ActionMonitor _monitor) throws CASTException {
		LearnIdentity act = newActionInstance(LearnIdentity.class);
		act.beliefAddress = _beliefID;
		act.value = _identity;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress unlearnColour(WorkingMemoryAddress _beliefID,
			String _colour, ActionMonitor _monitor) throws CASTException {
		UnlearnColour act = newActionInstance(UnlearnColour.class);
		act.beliefAddress = _beliefID;
		act.value = _colour;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress unlearnShape(WorkingMemoryAddress _beliefID,
			String _shape, ActionMonitor _monitor) throws CASTException {
		UnlearnShape act = newActionInstance(UnlearnShape.class);
		act.beliefAddress = _beliefID;
		act.value = _shape;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress unlearnIdentity(WorkingMemoryAddress _beliefID,
			String _identity, ActionMonitor _monitor) throws CASTException {
		UnlearnIdentity act = newActionInstance(UnlearnIdentity.class);
		act.beliefAddress = _beliefID;
		act.value = _identity;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}
	
	public WorkingMemoryAddress askForColour(WorkingMemoryAddress _beliefID,
			ActionMonitor _monitor) throws CASTException {
		AskForColour act = newActionInstance(AskForColour.class);
		act.beliefAddress = _beliefID;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	
	public WorkingMemoryAddress executeSingleBeliefAction(WorkingMemoryAddress _beliefID,
			ActionMonitor _monitor, Class<? extends SingleBeliefAction> _actionCls) throws CASTException {
		SingleBeliefAction act = newActionInstance(_actionCls);
		act.beliefAddress = _beliefID;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}
	

	
	public WorkingMemoryAddress askForShape(WorkingMemoryAddress _beliefID,
			ActionMonitor _monitor) throws CASTException {
		AskForShape act = newActionInstance(AskForShape.class);
		act.beliefAddress = _beliefID;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress askForIdentity(WorkingMemoryAddress _beliefID,
			ActionMonitor _monitor) throws CASTException {
		AskForIdentity act = newActionInstance(AskForIdentity.class);
		act.beliefAddress = _beliefID;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress askPolarColour(WorkingMemoryAddress _beliefID,
			String _value, ActionMonitor _monitor) throws CASTException {
		AskPolarColour act = newActionInstance(AskPolarColour.class);
		act.beliefAddress = _beliefID;
		act.value = _value;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress askPolarShape(WorkingMemoryAddress _beliefID,
			String _value, ActionMonitor _monitor) throws CASTException {
		AskPolarShape act = newActionInstance(AskPolarShape.class);
		act.beliefAddress = _beliefID;
		act.value = _value;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress askPolarIdentity(
			WorkingMemoryAddress _beliefID, String _value,
			ActionMonitor _monitor) throws CASTException {
		AskPolarIdentity act = newActionInstance(AskPolarIdentity.class);
		act.beliefAddress = _beliefID;
		act.value = _value;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress askForObject(String _feature, String _value,
			ActionMonitor _monitor) throws CASTException {
		AskForObjectWithFeatureValue act = newActionInstance(AskForObjectWithFeatureValue.class);
		act.feature = _feature;
		act.value = _value;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	@Override
	public boolean isPaused() {
		// TODO Auto-generated method stub
		return false;
	}

}
