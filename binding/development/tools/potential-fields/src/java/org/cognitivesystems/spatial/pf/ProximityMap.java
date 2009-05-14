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
public class ProximityMap extends SpatialMap {

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

	/**
	 * The constructor needs to specify the bounding box of the scene.
	 */
	public ProximityMap(float _blx, float _bly, float _trx, float _try) {
		super(_blx, _bly, _trx, _try);

		int width = (int) metres2Centimetres(m_trx - m_blx);
		int height = (int) metres2Centimetres(m_try - m_bly);

		createNativeMap(width, height);
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
					metres2Centimetres(translateYToMap(_y)), 0);
		}
		else {
			throw new SpatialMapException("Point beyond map boundaries: (" + _x + "," + _y + ")");
		}
	}

	public float proximityValue(String _target, String _landmark) {
		return proximityValueNative(_target, _landmark);
	}

	public void proximityMap(String _landmark) {
		proximityMapNative(_landmark);
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
				metres2Centimetres(translateYToMap(_y)), 0);
	}

	private static native void addObjectNative(String _id, float _x, float _y,
			float _z);

	private static native void createNativeMap(int _width, int _height);

	private static native float proximityValueNative(String _target,
			String _landmark);

	private static native void proximityMapNative(String _landmark);

	private static native void setViewerPositionNative(float _x, float _y,
			float _z);

	private static native PotentialFieldPoint nextSweetSpotNative();

	public PotentialFieldPoint nextSweetSpot() {
		PotentialFieldPoint pfp = nextSweetSpotNative();
		// System.out.println(pfp.getX());
		pfp.setX(translateXFromMap(centimetres2Metres(pfp.getX())));
		// System.out.println(pfp.getX());
		pfp.setY(translateYFromMap(centimetres2Metres(pfp.getY())));
		return pfp;
	}

	/**
	 * @param args
	 * @throws SpatialMapException 
	 */
	public static void main(String[] args) throws SpatialMapException {

		ProximityMap proxMap = new ProximityMap(-0.5f, -0.5f, 0.5f, 0.5f);

		proxMap.setViewerPosition(0.0f, -1f, 0);
		proxMap.addObject("obj0", 0.05f, -0.05f, 0);
		proxMap.addObject("obj1", -0.05f, -0.05f, 0);
		proxMap.addObject("obj2", -0.05f, 0.05f, 0);
		proxMap.addObject("obj3", -0.05f, -0.02f, 0);

		// proxMap.proximityMap("obj2");
		//
		// for (int i = 0; i < 10; i++) {
		// PotentialFieldPoint pfp = proxMap.nextSweetSpot();
		// System.out.println("sweet spot: " + pfp.getX() + ","
		// + pfp.getY() + " " + pfp.getValue());
		// }

		System.out.println(proxMap.proximityValue("obj3", "obj2"));
		System.out.println(proxMap.proximityValue("obj3", "obj1"));
		System.out.println(proxMap.proximityValue("obj3", "obj0"));

		System.out.println(proxMap.proximityValue("obj2", "obj3"));
		System.out.println(proxMap.proximityValue("obj2", "obj1"));
		System.out.println(proxMap.proximityValue("obj2", "obj0"));

		System.out.println(proxMap.proximityValue("obj1", "obj3"));
		System.out.println(proxMap.proximityValue("obj1", "obj2"));
		System.out.println(proxMap.proximityValue("obj1", "obj0"));

		System.out.println(proxMap.proximityValue("obj0", "obj3"));
		System.out.println(proxMap.proximityValue("obj0", "obj2"));
		System.out.println(proxMap.proximityValue("obj0", "obj1"));

	}

}
