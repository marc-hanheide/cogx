/**
 * 
 */
package execution.components;

import java.util.Map;

import SpatialData.Place;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.actions.ActiveVisualSearch;
import execution.slice.actions.ComsysQueryFeature;
import execution.slice.actions.ComsysTestFeatureValue;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.slice.actions.GoToPlace;
import execution.slice.actions.GoToPlaceRough;
import execution.slice.actions.LookForObjects;
import execution.slice.actions.PTULookForObjects;
import execution.slice.actions.LookForPeople;
import execution.util.ActionInterfaceFrame;
import execution.util.ActionMonitor;

/**
 * 
 * Config options:
 * 
 * --labels="label1,label2,label3" etc. labels that are used to detect objects
 * --tolerance <tol> tolerance for the gotoplacerough
 * 
 * @author nah
 * 
 */
public class GraphicalExecutionManager extends AbstractExecutionManager {

	private static final String[] DEFAULT_LABELS = { "record1", "record2","record3", "record4" };

	private final ActionInterfaceFrame m_gui;
	private String[] m_objectLabels;

	private WorkingMemoryAddress m_currentActionAddress;

	/**
	 * 
	 */
	public GraphicalExecutionManager() {
		m_gui = new ActionInterfaceFrame(this);
		m_gui.pack();
		m_gui.setVisible(true);
		m_objectLabels = DEFAULT_LABELS;
	}

	@Override
	protected void configure(Map<String, String> _config) {
		//System.exit(0);
		String labels = _config.get("--labels");
		if (labels != null) {
			m_objectLabels = labels.split(",");
		}
		log("using object labels: " + m_objectLabels);
		double tol = 0.5;
		String tolstring = _config.get("--tolerance");
		if (tolstring != null) {
			tol = Double.parseDouble(tolstring);
		}
		m_gui.setTolerance(tol);
		log("Using tolerance " + tol + "m");

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
				StableBelief.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						addStableBelief(_wmc.address, getMemoryEntry(
								_wmc.address, StableBelief.class));
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				StableBelief.class, WorkingMemoryOperation.DELETE),
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
			StableBelief _belief) {
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

	public WorkingMemoryAddress triggerGoToRoughAction(long _placeID, double tol,
			ActionMonitor _monitor) throws CASTException {
		GoToPlaceRough act = newActionInstance(GoToPlaceRough.class);
		act.placeID = _placeID;
		act.tol = new double[] { tol };
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

	public WorkingMemoryAddress triggerDetectObjects(ActionMonitor _monitor)
			throws CASTException {
		DetectObjects act = newActionInstance(DetectObjects.class);
		act.labels = m_objectLabels;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerDetectPeople(ActionMonitor _monitor)
			throws CASTException {
		DetectPeople act = newActionInstance(DetectPeople.class);
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerLookForObjects(ActionMonitor _monitor)
			throws CASTException {
		LookForObjects act = newActionInstance(LookForObjects.class);
		act.labels = m_objectLabels;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;
	}

	public WorkingMemoryAddress triggerPTULookForObjects(ActionMonitor _monitor)
			throws CASTException {
		PTULookForObjects act = newActionInstance(PTULookForObjects.class);
		act.labels = m_objectLabels;
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

	public WorkingMemoryAddress triggerFeatureValueTest(String _beliefID, String _featureLabel,
			FeatureValue _fv, ActionMonitor _monitor) throws CASTException {
		ComsysTestFeatureValue act = newActionInstance(ComsysTestFeatureValue.class);
		act.beliefID = _beliefID;
		act.featureType = _featureLabel;
		act.featureValue = _fv;
		m_currentActionAddress = triggerExecution(act, _monitor);
		return m_currentActionAddress;		
	}


}
