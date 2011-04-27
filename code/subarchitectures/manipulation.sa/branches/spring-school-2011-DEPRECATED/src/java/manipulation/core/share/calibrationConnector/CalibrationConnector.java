package manipulation.core.share.calibrationConnector;

import manipulation.core.share.exceptions.CalibrationException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;

/**
 * represent a connection to a calibration algorithm
 * 
 * @author Torben Toeniges
 * 
 */
public interface CalibrationConnector {
	/**
	 * read the calibration from a configuration file (path defined while
	 * starting the program)
	 * 
	 * @throws CalibrationException
	 */
	public void readCalibrationFile() throws CalibrationException;

	/**
	 * gets the camera to robot translation (note: the camera system is
	 * providing coordinates in the robot system)
	 * 
	 * @return translation vector
	 * @throws CalibrationException
	 */
	public Vector3D getCamToRobTranslation() throws CalibrationException;

	/**
	 * gets the camera to robot rotation (note: the camera system is providing
	 * coordinates in the robot system)
	 * 
	 * @return rotation matrix
	 * @throws CalibrationException
	 */
	public Matrix getCamToRobRotation() throws CalibrationException;

	/**
	 * gets the robot to arm rotation
	 * 
	 * @return robot to arm rotation
	 * @throws CalibrationException
	 */
	public Matrix getRobToArmRotation() throws CalibrationException;

	/**
	 * gets the robot to arm translation
	 * 
	 * @return robot to arm translation
	 * @throws CalibrationException
	 */
	public Vector3D getRobToArmTranslation() throws CalibrationException;

	/**
	 * gets the item rotation (defined in camera coordinates) in robot
	 * coordinates (note: the camera system is providing coordinates in the
	 * robot system)
	 * 
	 * @param camRotation
	 *            item rotation in camera coordinates
	 * @return item rotation in robot coordinates (no changes to the input)
	 */
	public Matrix getCamRotationInRob(Matrix camRotation);

	/**
	 * gets the item translation (defined in camera coordinates) in robot
	 * coordinates (note: the camera system is providing coordinates in the
	 * robot system)
	 * 
	 * @param camPoint
	 *            item position in camera coordinates
	 * @return item position in robot coordinates (no changes to the input)
	 */
	public Vector3D getCamPointInRob(Vector3D camPoint);
}
