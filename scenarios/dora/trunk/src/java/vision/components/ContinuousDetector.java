/**
 * 
 */
package vision.components;

import java.util.Arrays;
import java.util.Map;
import java.util.Vector;
import java.util.Map.Entry;

import VisionData.DetectionCommand;
import cast.CASTException;
import cast.architecture.ManagedComponent;

/**
 * write DetectionCommands to working memory at a frequency of 1Hz to trigger
 * object detection. A list of label has to be given as configuration, i.e.
 * --label-record1 --label-mug, etc. to register for specific object detections.
 * 
 * @author marc
 * 
 */
public class ContinuousDetector extends ManagedComponent {

	private static final String LABEL_CONFIG_PREFIX = "--label-";
	private static final int LABEL_CONFIG_PREFIX_LENGTH = LABEL_CONFIG_PREFIX
			.length();

	private Vector<String> labels;

	/**
	 * 
	 */
	public ContinuousDetector() {
		super();
		labels = new Vector<String>();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		sleepComponent(5000); // sleep some time before start cracking
		while (isRunning()) {
			String id = newDataID();
			String[] labelsArr = new String[labels.size()];
			labels.copyInto(labelsArr);
			println("ask to detected " + Arrays.deepToString(labelsArr));
			DetectionCommand dc = new DetectionCommand(labelsArr);
			try {
				addToWorkingMemory(id, dc);
				sleepComponent(1000);
				// remove it again
				//deleteFromWorkingMemory(id);
			} catch (CASTException e) {
				logException(e);
			}
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		for (Entry<String, String> entry : config.entrySet()) {
			String key = entry.getKey();

			if (key.startsWith(LABEL_CONFIG_PREFIX)) {
				// parse out confidence
				String confString = key.substring(LABEL_CONFIG_PREFIX_LENGTH);
				labels.add(confString);
			}

		}
	}

}
