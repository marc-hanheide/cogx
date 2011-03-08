package vision.components;

import java.util.HashMap;
import java.util.Map;

import vision.VisionUtils;
import VisionData.DetectionCommand;
import VisionData.Recognizer3DCommand;
import VisionData.VisualObject;
import blobfinder.BlobFinderInterface;
import blobfinder.BlobFinderInterfacePrx;
import blobfinder.BlobInfo;
import blobfinder.ColorRGB;
import cast.AlreadyExistsOnWMException;
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
 * Configure option: multiple entries of the form --label-X=R,G,B where X is the
 * label to be compared against a recognition command and RBG describes the
 * colour to treat as that label. e.g. --label-teabox=255,0,0
 * --label-journal=0,255,0
 * 
 * 
 * @author nah
 * 
 */
public class BlobjectDetector extends ManagedComponent {

	private BlobFinderInterfacePrx m_blobFinder;
	private static final String LABEL_CONFIG_PREFIX = "--label-";
	private static final int LABEL_CONFIG_PREFIX_LENGTH = LABEL_CONFIG_PREFIX
			.length();

	private final HashMap<String, ColorRGB> m_label2colour;

	public BlobjectDetector() {
		m_blobFinder = null;
		m_label2colour = new HashMap<String, ColorRGB>();
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

	public void newRecognizer3DCommand(WorkingMemoryChange _wmc)
			throws CASTException {
		Recognizer3DCommand cmd = getMemoryEntry(_wmc.address,
				Recognizer3DCommand.class);

		// do detection
		detectObjectsForLabels(new String[] { cmd.label });

		// executed the command, results (if any) are on working memory,
		// now delete command as not needed anymore
		overwriteWorkingMemory(_wmc.address, cmd);
	}

	public void newDetectionCommand(WorkingMemoryChange _wmc)
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

	/**
	 * @param dc
	 * @throws AlreadyExistsOnWMException
	 */
	private void detectObjectsForLabels(String[] _labels)
			throws AlreadyExistsOnWMException {
		// because vision is never this quick...
		sleepComponent(500);

		BlobInfo[] blobs = m_blobFinder.getBlobs();

		// if no blobs
		if (blobs.length == 0) {
			log("see no blobs around here");
			// we don't see anything
			for (String label : _labels) {
				// for the time being just fail
				VisualObject obj = VisionUtils.newVisualObject();
				obj.identLabels = new String[1];
				obj.identLabels[0] = label;

				obj.identDistrib = new double[1];
				obj.identDistrib[0] = 0;

				obj.detectionConfidence = 0f;
				addToWorkingMemory(newDataID(), obj);
			}
		} else {
			log("there are some blobs around");
			for (String label : _labels) {
				log("  is it a " + label + "?");
				VisualObject obj = VisionUtils.newVisualObject();
				obj.identLabels = new String[1];
				obj.identLabels[0] = label;
				obj.identDistrib = new double[1];
				obj.identDistrib[0] = 0;

				// default to not seen (i.e. 0)
				obj.detectionConfidence = 0f;

				// get expected rgb for label
				ColorRGB rgb = m_label2colour.get(label);

				// if the colour code is not known continue with next label
				if (rgb != null) {
					// now see if we have this one in our blobs
					for (BlobInfo blob : blobs) {
						println("is " + VisionUtils.toString(blob.colour)
								+ " equal to " + VisionUtils.toString(rgb)
								+ "?");
						if (blob.colour.equals(rgb)) {
							println("  YES, found an object");
							obj.detectionConfidence = 1f;
							obj.identDistrib[0] = 1;
							break;
						}
					}
				}
				addToWorkingMemory(newDataID(), obj);
			}
		}
	}

	@Override
	protected void configure(Map<String, String> _arg0) {
		for (String key : _arg0.keySet()) {
			if (key.startsWith(LABEL_CONFIG_PREFIX)) {
				// parse out label
				String label = key.substring(LABEL_CONFIG_PREFIX_LENGTH);

				String rgbJoined = _arg0.get(key);
				String[] labels = rgbJoined.split(",");

				assert (labels.length == 3);
				int r = Integer.parseInt(labels[0]);
				assert (r >= 0 && r <= 255);
				int g = Integer.parseInt(labels[1]);
				assert (g >= 0 && g <= 255);
				int b = Integer.parseInt(labels[2]);
				assert (b >= 0 && b <= 255);
				log("stored: " + label + " " + r + " " + g + " " + b);
				m_label2colour.put(label, new ColorRGB(r, g, b));
			}
		}

	}

	@Override
	protected void runComponent() {

		// for testing. not all values look sane!

		// if (m_blobFinder != null) {
		// while (isRunning()) {
		// lockComponent();
		// BlobInfo[] blobs = m_blobFinder.getBlobs();
		// for (BlobInfo blob : blobs) {
		// println(blob.id);
		// println(blob.area);
		// println(blob.boundingBox.pos.x);
		// println(blob.boundingBox.pos.y);
		// println(blob.boundingBox.width);
		// println(blob.boundingBox.height);
		// println(blob.colour.r);
		// println(blob.colour.g);
		// println(blob.colour.b);
		// println(blob.range);
		// }
		// unlockComponent();
		// sleepComponent(1000);
		// }
		// }
	}

}
