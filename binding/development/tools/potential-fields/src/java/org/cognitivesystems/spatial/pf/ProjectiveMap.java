package org.cognitivesystems.spatial.pf;

/**
 * 
 */

/**
 * Input into this class is in metres, so 1.0 is one metre. The underlying map
 * representation uses 1 cm grid squares, so objects specified in greater detail
 * will be abstracted over.
 * 
 * @author nah
 */
public abstract class ProjectiveMap extends SpatialMap {

	// need to make sure we have set values to pass to JNI
	protected static enum ProjectiveDirection {
		BACK(3), FRONT(2), LEFT(0), RIGHT(1);

		private int m_val;

		ProjectiveDirection(int _val) {
			m_val = _val;
		}

		public int value() {
			return m_val;
		}
	}

	static {
		try {
			System.loadLibrary("PotentialFieldsJNI");
		}
		catch (UnsatisfiedLinkError e) {
			System.err.println("Cannot load or link PotentialFieldsJNI");
			System.err.println("Exception msg: " + e.getMessage());
			System.err.println("Library path: "
					+ System.getProperty("java.library.path"));
			e.printStackTrace();
			System.exit(1);
		}
	}

	private ProjectiveDirection m_dir;

	/**
	 * The constructor needs to specify the bounding box of the scene.
	 */
	ProjectiveMap(float _blx, float _bly, float _trx, float _try,
			ProjectiveDirection _dir) {
		super(_blx, _bly, _trx, _try);
		int width = (int) metres2Centimetres(m_trx - m_blx);
		int height = (int) metres2Centimetres(m_try - m_bly);
		m_dir = _dir;

		createNativeMap(width, height, _dir.value());
	}

	/**
	 * Add a new object to the proximity map
	 * 
	 * @param _id
	 * @param _x
	 * @param _y
	 * @param _z
	 * @throws SpatialMapException 
	 */
	public void addObject(String _id, float _x, float _y, float _z) throws SpatialMapException {
		if (isInMap(_x, _y)) {
			// add into the underlying map representation
			addObjectNative(_id, metres2Centimetres(translateXToMap(_x)),
					metres2Centimetres(translateYToMap(_y)), 0, m_dir.value());
		}
		else {
			throw new SpatialMapException("Point beyond map boundaries: (" + _x
					+ "," + _y + ")");
		}
	}

	public PotentialFieldPoint nextSweetSpot() {
		PotentialFieldPoint pfp = nextSweetSpotNative(m_dir.value());
		// System.out.println(pfp.getX());
		pfp.setX(translateXFromMap(centimetres2Metres(pfp.getX())));
		// System.out.println(pfp.getX());
		pfp.setY(translateYFromMap(centimetres2Metres(pfp.getY())));
		return pfp;
	}

	public void projectiveMap(String _landmark) {
		projectiveMapNative(_landmark, m_dir.value());
	}

	public float projectiveValue(String _target, String _landmark) {
		return projectiveValueNative(_target, _landmark, m_dir.value());
	}

	/**
	 * Set the position of the view of the map... current assumes they're
	 * looking directly at the scene!
	 * 
	 * @param _x
	 * @param _y
	 * @param _z
	 */
	public void setViewerPosition(float _x, float _y, float _z) {
		setViewerPositionNative(metres2Centimetres(translateXToMap(_x)),
				metres2Centimetres(translateYToMap(_y)), 0, m_dir.value());
	}

	private static native void addObjectNative(String _id, float _x, float _y,
			float _z, int _dir);

	private static native void createNativeMap(int _width, int _height, int _dir);

	private static native PotentialFieldPoint nextSweetSpotNative(int _dir);

	private static native void projectiveMapNative(String _landmark, int _dir);

	private static native float projectiveValueNative(String _target,
			String _landmark, int _dir);

	private static native void setViewerPositionNative(float _x, float _y,
			float _z, int _dir);

	/**
	 * @param args
	 * @throws SpatialMapException 
	 */
	public static void main(String[] args) throws SpatialMapException {

		ProjectiveMap projMap = new LeftProjectiveMap(-0.5f, -0.5f, 0.5f, 0.5f);

		projMap.setViewerPosition(0.0f, -1f, 0);
		projMap.addObject("obj0", 0.05f, -0.05f, 0);
		projMap.addObject("obj1", -0.05f, -0.05f, 0);
		projMap.addObject("obj2", -0.05f, 0.05f, 0);
		projMap.addObject("obj3", -0.05f, -0.02f, 0);

		projMap.projectiveMap("obj2");

		//
		// for (int i = 0; i < 10; i++) {
		// PotentialFieldPoint pfp = proxMap.nextSweetSpot();
		// System.out.println("sweet spot: " + pfp.getX() + ","
		// + pfp.getY() + " " + pfp.getValue());
		// }

		System.out.println(projMap.projectiveValue("obj3", "obj2"));
		System.out.println(projMap.projectiveValue("obj3", "obj1"));
		System.out.println(projMap.projectiveValue("obj3", "obj0"));

		System.out.println(projMap.projectiveValue("obj2", "obj3"));
		System.out.println(projMap.projectiveValue("obj2", "obj1"));
		System.out.println(projMap.projectiveValue("obj2", "obj0"));

		System.out.println(projMap.projectiveValue("obj1", "obj3"));
		System.out.println(projMap.projectiveValue("obj1", "obj2"));
		System.out.println(projMap.projectiveValue("obj1", "obj0"));

		System.out.println(projMap.projectiveValue("obj0", "obj3"));
		System.out.println(projMap.projectiveValue("obj0", "obj2"));
		System.out.println(projMap.projectiveValue("obj0", "obj1"));

	}

}
