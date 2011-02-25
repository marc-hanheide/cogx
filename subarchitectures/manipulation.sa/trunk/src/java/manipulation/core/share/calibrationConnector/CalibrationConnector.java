package manipulation.core.share.calibrationConnector;

import manipulation.core.share.exceptions.CalibrationException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;

import org.apache.commons.math.linear.Array2DRowRealMatrix;

/**
 * represent a connection to a calibration algorithm
 * 
 * @author ttoenige
 * 
 */
public interface CalibrationConnector {
	/**
	 * calculates the camera to robot calibration
	 * 
	 * @throws CalibrationException
	 */
	public void calculateCamToRobCalibration() throws CalibrationException;

	/**
	 * read the calibration from a configuration file (path defined while
	 * starting the program)
	 * 
	 * @throws CalibrationException
	 */
	public void readCamToRobCalibration() throws CalibrationException;

	/**
	 * gets the camera to robot translation
	 * 
	 * @return camera to robot translation
	 * @throws CalibrationException
	 */
	public Vector3D getCamToRobTranslation() throws CalibrationException;

	/**
	 * gets the camera to robot rotation
	 * 
	 * @return camera to robot rotation
	 * @throws CalibrationException
	 */
	public Matrix getCamToRobRotation() throws CalibrationException;

	/**
	 * save the camera to robot calibration in a configuration file (path
	 * defined while starting the program)
	 * 
	 * @param R
	 *            rotation matrix to save
	 * @param t
	 *            translation vector to save
	 * @throws CalibrationException
	 */
	public void saveCamToRobCalibration(Array2DRowRealMatrix R,
			Array2DRowRealMatrix t) throws CalibrationException;

	/**
	 * add a pair of points for a later calculation of the calibration
	 * 
	 * @param sourcePoint
	 *            point in camera coordinates
	 * @param targetPoint
	 *            the same point in arm coordinates
	 */
	public void addCamToRobPair(Vector3D sourcePoint, Vector3D targetPoint);

	/**
	 * reset all pairs of points
	 */
	public void resetCamToRobAllPairs();

	/**
	 * delete the last safe pair
	 */
	public void resetCamToRobLastPair();

	/**
	 * function to test the calibration algorithm
	 */
	public void testCamToRobAdd();

	/**
	 * gets the item rotation (defined in camera coordinates) in robot
	 * coordinates
	 * 
	 * @param camRotation
	 *            item rotation in camera coordinates
	 * @return item rotation in robot coordinates
	 */
	public Matrix getCamRotationInRob(Matrix camRotation);

	/**
	 * gets the item translation (defined in camera coordiantes) in robot
	 * coordinates
	 * 
	 * @param camPoint
	 *            item position in camera coordinates
	 * @return item position in robot coordinates
	 */
	public Vector3D getCamPointInRob(Vector3D camPoint);
}
