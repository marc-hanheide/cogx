/**
 * 
 */
package execution.components;

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
import execution.slice.actions.ActiveVisualSearch;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.slice.actions.GoToPlace;
import execution.slice.actions.LookForPeople;
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
		String labels = _config.get("--labels");
		if (labels != null) {
			m_objectLabels = labels.split(",");
		}
		log("using object labels: " + m_objectLabels);
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

}
