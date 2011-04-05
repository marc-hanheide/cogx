package manipulation.core.share.types;

import manipulation.core.share.armConnector.ArmConnector;

/**
 * represents the given configuration of the program
 * 
 * @author ttoenige
 * 
 */
public class Configuration {
	double reachingDistance;
	double distanceInFrontDesk;
	String configPath;
	double maxDistanceForViewPoints;
	double nearPoints;
	ArmConnector.ArmName armName;
	double fixedtiltAngle;

	/**
	 * Constructor of the configuration representation
	 * 
	 * @param reachingDistance
	 *            distance between border points of a region in meter
	 * @param distanceInFrontDesk
	 *            distance of the viewpoints in front of a desk to a border
	 *            point in meter
	 * @param configPath
	 *            path to the calibration file
	 * @param maxDistanceForViewPoints
	 *            how far can the camera looks
	 * @param nearPoints
	 *            which viewpoints should be deleted when the robot does not
	 *            found the object
	 * @param armName
	 *            name of the used arm
	 * @param fixedTiltAngle
	 *            angle to look down
	 */
	public Configuration(double reachingDistance, double distanceInFrontDesk,
			String configPath, double maxDistanceForViewPoints,
			double nearPoints, ArmConnector.ArmName armName,
			double fixedTiltAngle) {
		this.reachingDistance = reachingDistance;
		this.distanceInFrontDesk = distanceInFrontDesk;
		this.configPath = configPath;
		this.maxDistanceForViewPoints = maxDistanceForViewPoints;
		this.armName = armName;
		this.nearPoints = nearPoints;
		this.fixedtiltAngle = fixedTiltAngle;

	}

	/**
	 * gets the distance between border points of a region in meter
	 * 
	 * @return distance between border points of a region in meter
	 */
	public double getReachingDistance() {
		return reachingDistance;
	}

	/**
	 * sets distance between border points of a region in meter
	 * 
	 * @param reachingDistance
	 *            new distance value in meter
	 */
	public void setReachingDistance(double reachingDistance) {
		this.reachingDistance = reachingDistance;
	}

	/**
	 * gets the distance of the viewpoints in front of a desk to a border point
	 * in meter
	 * 
	 * @return distance of the viewpoints in front of a desk to a border point
	 *         in meter
	 */
	public double getDistanceInFrontDesk() {
		return distanceInFrontDesk;
	}

	/**
	 * sets distance of the viewpoints in front of a desk to a border point in
	 * meter
	 * 
	 * @param distanceInFrontDesk
	 *            relevant distance value in meter
	 */
	public void setDistanceInFrontDesk(double distanceInFrontDesk) {
		this.distanceInFrontDesk = distanceInFrontDesk;
	}

	/**
	 * gets the path to the calibration file
	 * 
	 * @return path to the calibration file
	 */
	public String getConfigPath() {
		return configPath;
	}

	/**
	 * sets path to the calibration file
	 * 
	 * @param configPath
	 *            relevant path
	 */
	public void setConfigPath(String configPath) {
		this.configPath = configPath;
	}

	/**
	 * @return the armName
	 */
	public ArmConnector.ArmName getArmName() {
		return armName;
	}

	/**
	 * @param armName
	 *            the armName to set
	 */
	public void setArmName(ArmConnector.ArmName armName) {
		this.armName = armName;
	}

	/**
	 * @return the nearPoints
	 */
	public double getNearPoints() {
		return nearPoints;
	}

	/**
	 * @param nearPoints
	 *            the nearPoints to set
	 */
	public void setNearPoints(double nearPoints) {
		this.nearPoints = nearPoints;
	}

	public double getMaxDistanceForViewPoints() {
		return maxDistanceForViewPoints;
	}

	public void setMaxDistanceForViewPoints(double maxDistanceForViewPoints) {
		this.maxDistanceForViewPoints = maxDistanceForViewPoints;
	}

	/**
	 * @return the fixedtiltAngle
	 */
	public double getFixedtiltAngle() {
		return fixedtiltAngle;
	}

	/**
	 * @param fixedtiltAngle
	 *            the fixedtiltAngle to set
	 */
	public void setFixedtiltAngle(double fixedtiltAngle) {
		this.fixedtiltAngle = fixedtiltAngle;
	}

}
