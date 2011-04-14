package manipulation.core.cogx.calibrationConnector;

import java.util.LinkedList;
import java.util.List;

import manipulation.core.share.Manipulator;
import manipulation.core.share.calibrationConnector.CalibrationConnector;
import manipulation.core.share.exceptions.CalibrationException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import nu.xom.Builder;
import nu.xom.Document;
import nu.xom.Element;

import org.apache.commons.math.linear.Array2DRowRealMatrix;
import org.apache.log4j.Logger;

/**
 * calibration connector
 * 
 * @author ttoenige
 * 
 */
public class CogXCalibConnector implements CalibrationConnector {
	private Logger logger = Logger.getLogger(this.getClass());

	private Manipulator manipulator;

	private Matrix rotation = new Matrix();
	private Vector3D translation = new Vector3D(0, 0, 0);
	private boolean calibrationRead = false;

	/**
	 * constructor of the homographical transformation
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public CogXCalibConnector(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	private String getFirstElem(Element parent, String name) {
		return parent.getFirstChildElement(name).getValue();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void calculateCamToRobCalibration() throws CalibrationException {
		logger.error("No need for an implementation for the spring school cogx system!");
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void readCalibrationFile() throws CalibrationException {
		Builder builder = new Builder();
		try {
			Document calibrationDoc = builder.build(manipulator
					.getConfiguration().getConfigPath());

			Element root = calibrationDoc.getRootElement();
			Element rotationElem = root.getFirstChildElement("rmat");

			String rotationalDataRaw = getFirstElem(rotationElem, "data");
			String[] splittedRotationalData = rotationalDataRaw.split(" ");

			List<Double> rotationalDataList = new LinkedList<Double>();
			for (int i = 0; i < splittedRotationalData.length; i++) {
				if (!splittedRotationalData[i].isEmpty()) {
					try {
						rotationalDataList.add(Double.valueOf(
								splittedRotationalData[i]).doubleValue());
					} catch (Exception e) {
					}
				}
			}

			rotation.setM00(rotationalDataList.get(0));
			rotation.setM01(rotationalDataList.get(1));
			rotation.setM02(rotationalDataList.get(2));

			rotation.setM10(rotationalDataList.get(3));
			rotation.setM11(rotationalDataList.get(4));
			rotation.setM12(rotationalDataList.get(5));

			rotation.setM20(rotationalDataList.get(6));
			rotation.setM21(rotationalDataList.get(7));
			rotation.setM22(rotationalDataList.get(8));
			
			Element translationElem = root.getFirstChildElement("tvec");

			String transloationDataRaw = getFirstElem(translationElem, "data");
			String[] splittedTranslationalData = transloationDataRaw.split(" ");

			List<Double> translationalDataList = new LinkedList<Double>();
			for (int i = 0; i < splittedTranslationalData.length; i++) {
				if (!splittedTranslationalData[i].isEmpty()) {
					try {
						translationalDataList.add(Double.valueOf(
								splittedTranslationalData[i]).doubleValue());
					} catch (Exception e) {
					}
				}
			}

			translation.setX(translationalDataList.get(0));
			translation.setY(translationalDataList.get(1));
			translation.setZ(translationalDataList.get(2));
			
		} catch (Exception e) {
			throw new CalibrationException(
					"Error while reading cablibration file");
		}

		calibrationRead = true;

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vector3D getCamToRobTranslation() throws CalibrationException {
		logger.info("Cam system is providing coordinates in robot coordinates");
		return new Vector3D(0, 0, 0);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Matrix getCamToRobRotation() throws CalibrationException {
		logger.info("Cam system is providing coordinates in robot coordinates");
		return new Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void saveCamToRobCalibration(Array2DRowRealMatrix R,
			Array2DRowRealMatrix t) throws CalibrationException {
		logger.error("No need for an implementation for the spring school cogx system!");
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void resetCamToRobAllPairs() {
		logger.error("No need for an implementation for the spring school cogx system!");
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void resetCamToRobLastPair() {
		logger.error("No need for an implementation for the spring school cogx system!");
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void addCamToRobPair(Vector3D sourcePoint, Vector3D targetPoint) {
		logger.error("No need for an implementation for the spring school cogx system!");
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void testCamToRobAdd() {
		logger.error("No need for an implementation for the spring school cogx system!");
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vector3D getCamPointInRob(Vector3D camPoint) {
		logger.info("Cam system is providing coordinates in robot coordinates");
		return camPoint;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Matrix getCamRotationInRob(Matrix camRotation) {
		logger.info("Cam system is providing coordinates in robot coordinates");
		return camRotation;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Matrix getRobToArmRotation() throws CalibrationException {
		if (!calibrationRead)
			readCalibrationFile();

		return rotation;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vector3D getRobToArmTranslation() throws CalibrationException {
		if (!calibrationRead)
			readCalibrationFile();

		return translation;
	}
}
