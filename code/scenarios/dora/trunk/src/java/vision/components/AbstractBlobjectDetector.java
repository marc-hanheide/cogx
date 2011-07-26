package vision.components;

import java.util.HashMap;
import java.util.Map;

import vision.VisionUtils;
import blobfinder.BlobFinderInterface;
import blobfinder.BlobFinderInterfacePrx;
import blobfinder.BlobInfo;
import blobfinder.ColorRGB;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.architecture.ManagedComponent;

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
public abstract class AbstractBlobjectDetector extends ManagedComponent {

	private BlobFinderInterfacePrx m_blobFinder;
	private static final String LABEL_CONFIG_PREFIX = "--label-";
	private static final int LABEL_CONFIG_PREFIX_LENGTH = LABEL_CONFIG_PREFIX
			.length();

	protected final HashMap<String, ColorRGB> m_label2colour;

	public AbstractBlobjectDetector() {
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

		registerChangeFilters();

	}

	protected abstract void registerChangeFilters();

	/**
	 * @param dc
	 * @throws AlreadyExistsOnWMException
	 */
	protected void detectObjectsForLabels(String[] _labels)
			throws AlreadyExistsOnWMException {
		// because vision is never this quick...
		sleepComponent(500);

		BlobInfo[] blobs = m_blobFinder.getBlobs();

		// if no blobs
		if (blobs.length == 0) {
			log("see no blobs around here");
			// we don't see anything
			for (String label : _labels) {
				if (m_label2colour.containsKey(label)) {
					log("was looking for object " + label
							+ " but couldn't see any blobs at all");
					submitDetectionResult(label, false);
				} else {
					log("cannot detect this object... ignore it.");
				}
			}
		} else {
			log("there are some blobs around");
			for (String label : _labels) {
				log("  is it a " + label + "?");
				ColorRGB rgb = m_label2colour.get(label);
				boolean seenIt = false;
				// if the colour code is not known continue with next label
				if (rgb != null) {
					// now see if we have this one in our blobs
					for (BlobInfo blob : blobs) {
						println("is " + VisionUtils.toString(blob.colour)
								+ " equal to " + VisionUtils.toString(rgb)
								+ "?");
						if (blob.colour.equals(rgb)) {
							println("  YES, found an object");
							seenIt = true;
							break;
						}
					}
					submitDetectionResult(label, seenIt);
				}
			}
		}
	}

	protected abstract void submitDetectionResult(String label, boolean b);

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
				println("will recognise: " + label + " as blobs of color " + r
						+ ", " + g + ", " + b);
				m_label2colour.put(label, new ColorRGB(r, g, b));
			}
		}

	}

}
