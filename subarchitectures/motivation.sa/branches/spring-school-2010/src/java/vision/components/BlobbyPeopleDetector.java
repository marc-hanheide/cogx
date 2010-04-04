package vision.components;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import VisionData.Person;
import blobfinder.BlobFinderInterface;
import blobfinder.BlobFinderInterfacePrx;
import blobfinder.BlobInfo;
import blobfinder.ColorRGB;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;

/**
 * A fake people detector that uses the player blobfinder model to find people.
 * 
 * Configure options: one entry. of the form --person-colour=R,G,B which
 * describes the blob colour to treat as a person (0 ... 255 for colour
 * channels). Default is yellow (255,255,0) --image-width (in pixels) specifies
 * the assumed width of the image used by the blob finder (default in stage is
 * 80 pixels). --person-width assumed width of a person in metres (default 0.4).
 * 
 * Assumes an instance of the BlobFinderInterface is running somewhere with the
 * component name blob.server.
 * 
 * @author nah
 * 
 */
public class BlobbyPeopleDetector extends ManagedComponent {

	private BlobFinderInterfacePrx m_blobFinder;
	private static final String PERSON_COLOUR_KEY = "--person-colour";
	private static final String IMAGE_WIDTH_KEY = "--image-width";
	private static final String PERSON_WIDTH_KEY = "--person-width";

	private ColorRGB m_personColour;
	private double m_personWidth;
	private int m_imageXCentre;
	private HashMap<Integer, WorkingMemoryAddress> m_blobID2People;

	public BlobbyPeopleDetector() {
		m_blobFinder = null;
		m_personColour = new ColorRGB(255, 255, 0);
		m_personWidth = 0.4d;
		m_imageXCentre = 40;
	}

	@Override
	protected void start() {

		try {
			m_blobFinder = getIceServer("blob.server",
					BlobFinderInterface.class, BlobFinderInterfacePrx.class);
		} catch (CASTException e) {
			throw new RuntimeException(e);
		}

	}

	public void detectPeople() {

		BlobInfo[] blobs = m_blobFinder.getBlobs();

		// empty previous set if blob map is null
		Set<Integer> previousBlobs = m_blobID2People != null ? new HashSet<Integer>(
				m_blobID2People.keySet())
				: new HashSet<Integer>();

		for (BlobInfo blobInfo : blobs) {
			// if we have a blob that looks like a human
			if (blobInfo.colour.equals(m_personColour)) {

				// lazy creation
				if (m_blobID2People == null) {
					m_blobID2People = new HashMap<Integer, WorkingMemoryAddress>();
				}

				// println(blobInfo.colour.r);
				// println(blobInfo.colour.g);
				// println(blobInfo.colour.b);

				Person person = createPersonFromBlob(blobInfo);
				println(toString(person));

			}
		}

	}

	private static String toString(Person _person) {
		return CASTUtils.concatenate("Person ", _person.angle, " degrees ",
				_person.distance, " distance");
	}

	/**
	 * Generates a new person struct. Does the maths by assuming that the whole
	 * person is in view and the ratio between person width in metres and pixels
	 * is the same for image width.
	 * 
	 * @param _blobInfo
	 * @return
	 */
	private Person createPersonFromBlob(BlobInfo _blobInfo) {

		// get range in metre
		double rangeMetres = _blobInfo.range / 1000;

		// convert image coords to real world info
		double pixelsPerMetre = _blobInfo.boundingBox.width / m_personWidth;

//		println("p/m " + pixelsPerMetre);

		// offset of bounding box, assumes camera is in the middle of the robot
		double pixelOffsetOfPerson = _blobInfo.boundingBox.pos.x
				- m_imageXCentre;

//		println("offset " + pixelOffsetOfPerson);

		double deltaXMetres = pixelOffsetOfPerson / pixelsPerMetre;

//		println("deltaX " + deltaXMetres);

		double angleRadians = Math.asin(deltaXMetres / rangeMetres);

		println("rads " + angleRadians);

		double angleDegrees = Math.toDegrees(angleRadians);

		return new Person(angleDegrees, rangeMetres, deltaXMetres, rangeMetres
				* Math.cos(angleRadians), 0, 0);
	}

	/**
	 * Returns the integer associated with _key in the _config map. Returns null
	 * if the key does not exist.
	 * 
	 * @param _key
	 * @param _config
	 * @return
	 */
	private final static Integer extractConfigInt(String _key,
			Map<String, String> _config) {
		assert (_key != null);
		assert (_config != null);

		String val = _config.get(_key);
		if (val == null) {
			return null;
		} else {
			return Integer.parseInt(val);
		}
	}

	/**
	 * Returns the double associated with _key in the _config map. Returns null
	 * if the key does not exist.
	 * 
	 * @param _key
	 * @param _config
	 * @return
	 */
	private final static Double extractConfigDouble(String _key,
			Map<String, String> _config) {
		assert (_key != null);
		assert (_config != null);

		String val = _config.get(_key);
		if (val == null) {
			return null;
		} else {
			return Double.parseDouble(val);
		}
	}

	@Override
	protected void configure(Map<String, String> _config) {

		Integer width = extractConfigInt(IMAGE_WIDTH_KEY, _config);
		m_imageXCentre = width != null ? width / 2 : m_imageXCentre;

		Double person = extractConfigDouble(PERSON_WIDTH_KEY, _config);
		m_personWidth = person != null ? person : m_personWidth;

		String personColourString = _config.get(PERSON_COLOUR_KEY);
		if (personColourString != null) {
			String[] labels = personColourString.split(",");
			assert (labels.length == 3);
			int r = Integer.parseInt(labels[0]);
			assert (r >= 0 && r <= 255);
			int g = Integer.parseInt(labels[1]);
			assert (g >= 0 && g <= 255);
			int b = Integer.parseInt(labels[2]);
			assert (b >= 0 && b <= 255);
			m_personColour = new ColorRGB(r, g, b);
		}

		log("image width: " + m_imageXCentre * 2);
		log("person width: " + m_personWidth);
		log(CASTUtils.concatenate("person colour: ", m_personColour.r, " ",
				m_personColour.g, " " + m_personColour.b));
	}

	@Override
	protected void runComponent() {

		// for testing

		if (m_blobFinder != null) {
			while (isRunning()) {
				lockComponent();
				detectPeople();
				unlockComponent();
				sleepComponent(1000);
			}
		}
	}

}
