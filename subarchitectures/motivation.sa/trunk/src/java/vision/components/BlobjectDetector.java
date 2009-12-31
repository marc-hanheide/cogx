package vision.components;

import java.util.Map;

import blobfinder.BlobFinderInterface;
import blobfinder.BlobFinderInterfacePrx;

import vision.VisionUtils;
import VisionData.DetectionCommand;
import VisionData.VisualObject;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * A fake object detector that uses the player blobfinder model to find objects.
 * Each object must have a unique colour.
 * 
 * 
 * 
 * @author nah
 * 
 */
public class BlobjectDetector extends ManagedComponent implements
		WorkingMemoryChangeReceiver {

	private BlobFinderInterfacePrx m_blobFinder;

	public BlobjectDetector() {
		m_blobFinder = null;
	}

	@Override
	protected void start() {

		try {
			m_blobFinder = getIceServer("blob.server",
					BlobFinderInterface.class, BlobFinderInterfacePrx.class);
		} catch (CASTException e) {
			e.printStackTrace();
		}

		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				DetectionCommand.class, WorkingMemoryOperation.ADD), this);

	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange _arg0)
			throws CASTException {

		// load command
		DetectionCommand dc = getMemoryEntry(_arg0.address,
				DetectionCommand.class);

		// because vision is never this quick...
		sleepComponent(2000);

		for (String label : dc.labels) {
			// for the time being just fail
			VisualObject obj = VisionUtils.newVisualObject();
			obj.detectionConfidence = 0f;
			obj.label = label;
			addToWorkingMemory(newDataID(), obj);
		}

	}

	@Override
	protected void configure(Map<String, String> _arg0) {
	}

	@Override
	protected void runComponent() {
		if (m_blobFinder != null) {
			while (isRunning()) {
				println("blobs: " + m_blobFinder.getBlobCount());
				sleepComponent(1000);
			}
		}
	}

}
