/**
 * 
 */
package dora.execution.components;

import java.util.Map;

import SpatialData.Place;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import execution.components.AbstractExecutionManager;
import execution.slice.actions.ActiveVisualSearch;
import execution.slice.actions.BackgroundModels;
import execution.slice.actions.ComsysQueryFeature;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.slice.actions.ForegroundModels;
import execution.slice.actions.GoToPlace;
import execution.slice.actions.LookForObjects;
import execution.slice.actions.LookForPeople;
import execution.slice.actions.RecogniseForegroundedModels;
import execution.util.ActionInterfaceFrame;
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
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _arg0)
					throws CASTException {
				addPlace(_arg0.address, getMemoryEntry(_arg0.address,
						Place.class));
			}

		});

		// use these to harvest beliefs
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				dBelief.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						addStableBelief(_wmc.address, getMemoryEntry(
								_wmc.address, dBelief.class));
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				dBelief.class, WorkingMemoryOperation.DELETE),
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

	private void addStableBelief(WorkingMemoryAddress _address,
			dBelief _belief) {
		m_gui.addBelief(_address, _belief);
	}

	private void addPlace(WorkingMemoryAddress _address, Place _memoryEntry) {
		m_gui.addPlace(_memoryEntry.id);
	}

	public WorkingMemoryAddress triggerGoToAction(long _placeID,
			ActionMonitor _monitor) throws CASTException {
		GoToPlace act = newActionInstance(GoToPlace.class);
		act.placeID = _placeID;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerAVSAction(long[] _placeIDs,
			ActionMonitor _monitor) throws CASTException {
		ActiveVisualSearch act = newActionInstance(ActiveVisualSearch.class);
		act.placeIDs = _placeIDs;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
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
