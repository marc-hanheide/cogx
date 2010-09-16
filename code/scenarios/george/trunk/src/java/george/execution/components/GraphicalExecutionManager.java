/**
 * 
 */
package george.execution.components;

import java.util.Map;

import VisionData.VisualLearnerLearningTask;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import eu.cogx.beliefs.slice.PerceptBelief;
import execution.components.AbstractExecutionManager;
import execution.slice.actions.BackgroundModels;
import execution.slice.actions.ComsysQueryFeature;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.slice.actions.ForegroundModels;
import execution.slice.actions.LearnColour;
import execution.slice.actions.LearnIdentity;
import execution.slice.actions.LearnShape;
import execution.slice.actions.LookForObjects;
import execution.slice.actions.LookForPeople;
import execution.slice.actions.RecogniseForegroundedModels;
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

	private static final String[] DEFAULT_LABELS = { "record1", "record2",
			"record3", "record4" };

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
				PerceptBelief.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						addPerceptBelief(_wmc.address, getMemoryEntry(
								_wmc.address, PerceptBelief.class));
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				PerceptBelief.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {

						removePerceptBelief(_wmc.address);
					}
				});

	}

	private void removePerceptBelief(WorkingMemoryAddress _address) {
		m_gui.removeBelief(_address);
	}

	private void addPerceptBelief(WorkingMemoryAddress _address,
			PerceptBelief _belief) {
		m_gui.addBelief(_address, _belief);
	}


	public WorkingMemoryAddress triggerDetectObjects(String[] _models, ActionMonitor _monitor)
			throws CASTException {
		DetectObjects act = newActionInstance(DetectObjects.class);
		act.labels =_models;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerDetectPeople(ActionMonitor _monitor)
			throws CASTException {
		DetectPeople act = newActionInstance(DetectPeople.class);
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerLookForObjects(String[] _models, ActionMonitor _monitor)
			throws CASTException {
		LookForObjects act = newActionInstance(LookForObjects.class);
		act.labels =_models;
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

	public WorkingMemoryAddress triggerAskForFeatureAction(String _beliefID,
			String _featureType, ActionMonitor _monitor) throws CASTException {
		ComsysQueryFeature act = newActionInstance(ComsysQueryFeature.class);
		act.beliefID = _beliefID;
		act.featureID = _featureType;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
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

	public WorkingMemoryAddress learnColour(String _beliefID, String _colour, ActionMonitor _monitor) throws CASTException {
		LearnColour act = newActionInstance(LearnColour.class);
		act.beliefId = _beliefID;
		act.value = _colour;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}
	public WorkingMemoryAddress learnShape(String _beliefID, String _shape, ActionMonitor _monitor) throws CASTException {
		LearnShape act = newActionInstance(LearnShape.class);
		act.beliefId = _beliefID;
		act.value = _shape;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}
	public WorkingMemoryAddress learnIdentity(String _beliefID, String _identity, ActionMonitor _monitor) throws CASTException {
		LearnIdentity act = newActionInstance(LearnIdentity.class);
		act.beliefId = _beliefID;
		act.value = _identity;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

//	public WorkingMemoryAddress triggerFeatureValueTest(String _beliefID, String _featureLabel,
//			FeatureValue _fv, ActionMonitor _monitor) throws CASTException {
//		ComsysTestFeatureValue act = newActionInstance(ComsysTestFeatureValue.class);
//		act.beliefID = _beliefID;
//		act.featureType = _featureLabel;
//		act.featureValue = _fv;
//		m_currentActionAddress = triggerExecution(act, _monitor);
//		return m_currentActionAddress;		
//	}


}
