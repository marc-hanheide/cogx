/**
 * 
 */
package vision.components;

import cogx.Math.Pose3;
import cogx.Math.Vector3;
import mathlib.Functions;
import VisionData.DetectionCommand;
import VisionData.Post3DObject;
import VisionData.Recognizer3DCommand;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author cogx
 * 
 */
public class BlobjectDetector extends AbstractBlobjectDetector {

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * vision.components.AbstractBlobjectDetector#submitDetectionResult(java
	 * .lang.String, boolean)
	 */
	@Override
	protected void submitDetectionResult(String label, boolean b) {
		try {
			submitVisualObjectToWM(label, Functions.pose3FromEuler(new Vector3(
					1, 0, 0), 0, 0, 0), b);
		} catch (AlreadyExistsOnWMException e) {
			logException(e);
		}

	}

	private void submitVisualObjectToWM(String label, Pose3 objPose,
			boolean positiveDetection) throws AlreadyExistsOnWMException {
		Post3DObject postCmd = new Post3DObject(label, objPose,
				positiveDetection);
		addToWorkingMemory(newDataID(), postCmd);
	}

	@Override
	protected void registerChangeFilters() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				DetectionCommand.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						newDetectionCommand(_arg0);
					}
				});

		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				Recognizer3DCommand.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						newRecognizer3DCommand(_arg0);
					}
				});

	}

	protected void newRecognizer3DCommand(WorkingMemoryChange _wmc)
			throws CASTException {
		Recognizer3DCommand cmd = getMemoryEntry(_wmc.address,
				Recognizer3DCommand.class);

		// do detection
		detectObjectsForLabels(new String[] { cmd.label });

		// executed the command, results (if any) are on working memory,
		// now delete command as not needed anymore
		getMemoryEntry(_wmc.address, Recognizer3DCommand.class);
		overwriteWorkingMemory(_wmc.address, cmd);
	}

	protected void newDetectionCommand(WorkingMemoryChange _wmc)
			throws CASTException {

		// load command
		DetectionCommand dc = getMemoryEntry(_wmc.address,
				DetectionCommand.class);

		// do detection
		detectObjectsForLabels(dc.labels);

		// executed the command, results (if any) are on working memory,
		// now delete command as not needed anymore
		deleteFromWorkingMemory(_wmc.address);

	}

}
