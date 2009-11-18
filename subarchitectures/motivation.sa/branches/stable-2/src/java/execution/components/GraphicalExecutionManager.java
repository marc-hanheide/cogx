/**
 * 
 */
package execution.components;

import SpatialData.Place;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.actions.ActiveVisualSearch;
import execution.slice.actions.GoToPlace;
import execution.util.ActionInterfaceFrame;
import execution.util.ActionMonitor;

/**
 * @author nah
 *
 */
public class GraphicalExecutionManager extends AbstractExecutionManager {

	private final ActionInterfaceFrame m_gui;
	
	/**
	 * 
	 */
	public GraphicalExecutionManager() {
		m_gui = new ActionInterfaceFrame(this);
		m_gui.pack();
		m_gui.setVisible(true);
	}
	
	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class, WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _arg0)
					throws CASTException {
				addPlace(_arg0.address, getMemoryEntry(_arg0.address, Place.class));
			}

		});
	}

	private void addPlace(WorkingMemoryAddress _address,
			Place _memoryEntry) {
		m_gui.addPlace(_memoryEntry.id);
	}

	public void triggerGoToAction(long _placeID, ActionMonitor _monitor) throws CASTException {
		GoToPlace act = newActionInstance(GoToPlace.class);
		act.placeID = _placeID;
		triggerExecution(act, _monitor);
	}
	
	
	public void triggerAVSAction(long[] _placeIDs, ActionMonitor _monitor) throws CASTException {
		ActiveVisualSearch act = newActionInstance(ActiveVisualSearch.class);
		act.placeIDs = _placeIDs;
		triggerExecution(act, _monitor);
	}
}
