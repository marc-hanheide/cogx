package vision.components;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import VisionData.PeopleDetectionCommand;
import VisionData.Person;
import blobfinder.BlobFinderInterface;
import blobfinder.BlobFinderInterfacePrx;
import blobfinder.BlobInfo;
import blobfinder.ColorRGB;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.WMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.castextensions.Condition;
import castutils.castextensions.Converter;
import castutils.castextensions.HashCoder;
import castutils.castextensions.WMTypeAlignment;
import castutils.meta.OperationPerformer;
import castutils.slice.WMOperation;

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
	private OperationPerformer m_performer;
	private WMTypeAlignment<BlobInfo, Person> m_aligner;
	static int blobCounter=0;

	private final class BlobInfo2PersonConverter implements
			Converter<BlobInfo, Person> {

		/**
		 * Generates a new person struct. Does the maths by assuming that the
		 * whole person is in view and the ratio between person width in metres
		 * and pixels is the same for image width.
		 * 
		 * @param _blobInfo
		 * @return
		 */
		public Person convert(BlobInfo _blobInfo) {
			// get range in metre
			double rangeMetres = _blobInfo.range / 1000;

			// convert image coords to real world info
			double pixelsPerMetre = _blobInfo.boundingBox.width / m_personWidth;

			// println("p/m " + pixelsPerMetre);

			// offset of bounding box, assumes camera is in the middle of the
			// robot
			double pixelOffsetOfPerson = _blobInfo.boundingBox.pos.x
					- m_imageXCentre;

			// println("offset " + pixelOffsetOfPerson);

			double deltaXMetres = pixelOffsetOfPerson / pixelsPerMetre;

			// println("deltaX " + deltaXMetres);

			double angleRadians = Math.asin(deltaXMetres / rangeMetres);

			// println("rads " + angleRadians);

			double angleDegrees = Math.toDegrees(angleRadians);

			return new Person(angleDegrees, rangeMetres, deltaXMetres,
					rangeMetres * Math.cos(angleRadians), 0, 0);
		}
	}

	private final class BlobInfoHasher implements HashCoder<BlobInfo> {
		public int hashCode(BlobInfo _t) {
			// each blob is a new blob for us...
			return blobCounter++;
			//			return _t.id;
		}
	}

	private final class PersonColourTester implements Condition<BlobInfo> {
		public boolean test(BlobInfo _t) {
			return _t.colour.equals(m_personColour);
		}
	}

	public BlobbyPeopleDetector() {
		m_blobFinder = null;
		m_personColour = new ColorRGB(255, 255, 0);
		m_personWidth = 0.4d;
		m_imageXCentre = 40;

		m_performer = new OperationPerformer(this);
		m_aligner = new WMTypeAlignment<BlobInfo, Person>(this,
				new BlobInfo2PersonConverter(), new PersonColourTester(),
				new BlobInfoHasher());
	}

	@Override
	protected void start() {

		try {
			m_blobFinder = getIceServer("blob.server",
					BlobFinderInterface.class, BlobFinderInterfacePrx.class);

			addChangeFilter(ChangeFilterFactory.createTypeFilter(
					PeopleDetectionCommand.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						@Override
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) throws CASTException {
							handleDetectionCommand(_wmc.address);
						}
					});

		} catch (CASTException e) {
			throw new RuntimeException(e);
		}

	}

	private void handleDetectionCommand(WorkingMemoryAddress _address)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException,
			PermissionException {
		// PeopleDetectionCommand cmd = getMemoryEntry(_address,
		// PeopleDetectionCommand.class);
		detectPeople();
		deleteFromWorkingMemory(_address);
	}

	public void detectPeople() {
		try {
			BlobInfo[] blobs = m_blobFinder.getBlobs();
			log(blobs.length + " blobs");
			List<WMOperation> operations = m_aligner.sync(Arrays.asList(blobs));
			log(operations.size() + " operations");
			m_performer.performOperations(operations);
		} catch (WMException e) {
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		}
	}

	private static String toString(Person _person) {
		return CASTUtils.concatenate("Person ", _person.angle, " degrees ",
				_person.distance, " distance");
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
		//
		// if (m_blobFinder != null) {
		// while (isRunning()) {
		// lockComponent();
		// detectPeople();
		// unlockComponent();
		// sleepComponent(1000);
		// }
		// }
	}

}
