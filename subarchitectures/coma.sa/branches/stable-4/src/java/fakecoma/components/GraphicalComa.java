/**
 * 
 */
package fakecoma.components;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Map;
import java.util.Vector;

import SpatialData.Place;

import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import fakecoma.gui.ComaJFrame;

/**
 * @author cogx
 * 
 */
public class GraphicalComa extends ManagedComponent implements ActionListener {
	ComaJFrame frame;

	public GraphicalComa() {
		super();
		frame = new ComaJFrame(this);

	}

	@Override
	protected void configure(Map<String, String> config) {

	}

	@Override
	protected void start() {
		frame.pack();
		frame.setVisible(true);
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
		frame.addPlace(_memoryEntry.id);
	}

	@Override
	protected void stop() {
		frame.setVisible(false);
		frame.dispose();
	}

	@Override
	public void actionPerformed(ActionEvent arg0) {
		Vector<Long> places = frame.getPlaces();
		
	}
}
