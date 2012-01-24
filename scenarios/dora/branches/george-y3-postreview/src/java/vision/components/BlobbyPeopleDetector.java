/**
 * 
 */
package vision.components;

import java.util.Set;

import VisionData.PeopleDetectionCommand;
import VisionData.Person;
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
public class BlobbyPeopleDetector extends AbstractBlobjectDetector {

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * vision.components.AbstractBlobjectDetector#submitDetectionResult(java
	 * .lang.String, boolean)
	 */
	@Override
	protected void submitDetectionResult(String label, boolean b) {
		println("submitting result for " + label + "=" + b);
		try {
			submitVisualObjectToWM(label, b);
		} catch (AlreadyExistsOnWMException e) {
			logException(e);
		}

	}

	private void submitVisualObjectToWM(String label, boolean positiveDetection)
			throws AlreadyExistsOnWMException {
		double prob = positiveDetection ? 1.0 : 0.0;
		Person person = new Person(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, prob);
		addToWorkingMemory(newDataID(), person);
	}

	@Override
	protected void registerChangeFilters() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				PeopleDetectionCommand.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _arg0)
							throws CASTException {
						newDetectionCommand(_arg0);
					}
				});
	}

	protected void newDetectionCommand(WorkingMemoryChange _wmc)
			throws CASTException {

		// load command
		getMemoryEntry(_wmc.address, PeopleDetectionCommand.class);

		// do detection
		Set<String> labelSet = m_label2colour.keySet();
		String[] labels = labelSet.toArray(new String[1]);
		detectObjectsForLabels(labels);

		// executed the command, results (if any) are on working memory,
		// now delete command as not needed anymore
		deleteFromWorkingMemory(_wmc.address);

	}

}
