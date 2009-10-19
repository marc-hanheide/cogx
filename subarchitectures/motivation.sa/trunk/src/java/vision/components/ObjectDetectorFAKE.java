package vision.components;

import cogx.Math.Matrix33;
import cogx.Math.Vector3;
import VisionData.DetectionCommand;
import VisionData.Face;
import VisionData.GeometryModel;
import VisionData.Vertex;
import VisionData.VisualObject;
import VisionData.VisualObjectView;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class ObjectDetectorFAKE extends ManagedComponent implements
		WorkingMemoryChangeReceiver {

	public ObjectDetectorFAKE() {
	}

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				DetectionCommand.class, WorkingMemoryOperation.ADD), this);

	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange _arg0)
			throws CASTException {

		// load command
		DetectionCommand dc = getMemoryEntry(_arg0.address,
				DetectionCommand.class);

		// just 'cos
		sleepComponent(500);

		for (String label : dc.labels) {
			// for the time being just fail
			VisualObject obj = newVisualObject();
			
			//over 0.5 is a detection for AVS
			obj.detectionConfidence = 0.6d;
			obj.label = label;
			addToWorkingMemory(newDataID(), obj);
		}

	}

	/**
	 * @return
	 */
	private VisualObject newVisualObject() {
		VisualObject obj = new VisualObject(new cogx.Math.Pose3(newVector3(), new Matrix33(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d)),
				new String[0], 0d, new cogx.Math.Sphere3(newVector3(), 0d), getCASTTime(),
				new VisualObjectView[0], new GeometryModel(new Vertex[0], new Face[0]), "", 0d, 0d);
		return obj;
	}

	/**
	 * @return
	 */
	private Vector3 newVector3() {
		return new Vector3(0d,
				0d, 0d);
	}

}
