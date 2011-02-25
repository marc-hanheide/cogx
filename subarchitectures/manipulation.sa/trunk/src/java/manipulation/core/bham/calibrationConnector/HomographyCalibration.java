package manipulation.core.bham.calibrationConnector;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Vector;

import manipulation.core.bham.converter.BhamConverter;
import manipulation.core.share.Manipulator;
import manipulation.core.share.calibrationConnector.CalibrationConnector;
import manipulation.core.share.exceptions.CalibrationException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import manipulation.math.MathOperation;
import nu.xom.Builder;
import nu.xom.Document;
import nu.xom.Element;

import org.apache.commons.math.linear.Array2DRowRealMatrix;
import org.apache.commons.math.linear.SingularValueDecomposition;
import org.apache.commons.math.linear.SingularValueDecompositionImpl;
import org.apache.log4j.Logger;

/**
 * calibration algorithm which is using a homographical transformation
 * 
 * @author ttoenige
 * 
 */
public class HomographyCalibration implements CalibrationConnector {
	private Logger logger = Logger.getLogger(this.getClass());
	
	private Manipulator manipulator;

	private Matrix rotation = new Matrix();
	private Vector3D translation = new Vector3D(0, 0, 0);
	private boolean calibrationRead = false;

	private Vector<Vector3D> sourcePoints = new Vector<Vector3D>();
	private Vector<Vector3D> targetPoints = new Vector<Vector3D>();

	/**
	 * constructo of the homographical transformation
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public HomographyCalibration(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	private String getFirstElem(Element parent, String name) {
		return parent.getFirstChildElement(name).getValue();
	}

	private void addElem(Element parent, String name, String value) {
		Element e = new Element(name);
		e.appendChild(value);
		parent.appendChild(e);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void calculateCamToRobCalibration() throws CalibrationException {
		if (sourcePoints.size() < 3)
			throw new CalibrationException("too few point pairs");

		logger.info("using points: ");
		for (int i = 0; i < sourcePoints.size(); i++) {
			logger.info(sourcePoints.get(i) + " --> " + targetPoints.get(i));
		}

		// calculate mean vectors
		Array2DRowRealMatrix sourcePointsMean = BhamConverter
				.convertVecToRowMatrix(MathOperation.getMean(sourcePoints));
		Array2DRowRealMatrix targetPointsMean = BhamConverter
				.convertVecToRowMatrix(MathOperation.getMean(targetPoints));

		logger.info("source points mean: " + sourcePointsMean);
		logger.info("target points mean: " + targetPointsMean);
		logger.info("diff: " + sourcePointsMean.subtract(targetPointsMean));

		// calculate covariance matrix
		Array2DRowRealMatrix K = new Array2DRowRealMatrix(3, 3);
		for (int i = 0; i < sourcePoints.size(); i++) {
			Array2DRowRealMatrix ai = BhamConverter
					.convertVecToRowMatrix(sourcePoints.get(i));
			Array2DRowRealMatrix bj = BhamConverter
					.convertVecToRowMatrix(targetPoints.get(i));

			logger.info("Calculating covariance... ");
			K = (Array2DRowRealMatrix) K.add(bj.subtract(targetPointsMean)
					.multiply(ai.subtract(sourcePointsMean).transpose()));

			logger.info("done");
		}

		logger.info(K);

		// calculate svd
		SingularValueDecomposition svd = new SingularValueDecompositionImpl(K);

		logger.info("U: " + svd.getU());
		logger.info("VT: " + svd.getVT());

		logger.info("calculating rotation... ");

		Array2DRowRealMatrix R = (Array2DRowRealMatrix) svd.getU().multiply(
				svd.getVT());

		logger.info(R);

		logger.info("done");

		// TODO wenn determinante nicht ca. 1 dann abbrechen

		logger.info("calculationg transformation... ");
		Array2DRowRealMatrix t = (Array2DRowRealMatrix) targetPointsMean
				.subtract(R.multiply(sourcePointsMean));

		logger.info(t);

		logger.info("done");

		logger.info("test: ");
		double normSum0 = 0;
		for (int i = 0; i < sourcePoints.size(); i++) {
			Array2DRowRealMatrix test = new Array2DRowRealMatrix(3, 1);
			test.setEntry(0, 0, sourcePoints.get(i).getX());
			test.setEntry(1, 0, sourcePoints.get(i).getY());
			test.setEntry(2, 0, sourcePoints.get(i).getZ());

			Array2DRowRealMatrix newPoint = (R.multiply(test)).add(t);

			Array2DRowRealMatrix real = new Array2DRowRealMatrix(3, 1);
			real.setEntry(0, 0, targetPoints.get(i).getX());
			real.setEntry(1, 0, targetPoints.get(i).getY());
			real.setEntry(2, 0, targetPoints.get(i).getZ());

			Array2DRowRealMatrix diff = new Array2DRowRealMatrix(3, 1);
			diff = test.subtract(real);

			double norm0 = diff.getNorm() * diff.getNorm();
			normSum0 += norm0;

			logger.info("(" + test + " -> " + real + " calc: " + newPoint
					+ " | error: " + norm0);

		}

		normSum0 = normSum0 / sourcePoints.size();

		logger.info("Mean square error: " + normSum0);

		Array2DRowRealMatrix outMean = (R.multiply(sourcePointsMean)).add(t);
		Array2DRowRealMatrix diffMean = outMean.subtract(targetPointsMean);

		logger.info("out mean: " + outMean + " --> diff mean " + diffMean);

		saveCamToRobCalibration(R, t);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void readCamToRobCalibration() throws CalibrationException {
		Builder builder = new Builder();
		try {
			Document calibrationDoc = builder.build(manipulator
					.getConfiguration().getConfigPath());

			Element root = calibrationDoc.getRootElement();
			Element rotationElem = root.getFirstChildElement("rotation");

			rotation.setM00(Double
					.parseDouble(getFirstElem(rotationElem, "r00")));
			rotation.setM10(Double
					.parseDouble(getFirstElem(rotationElem, "r10")));
			rotation.setM20(Double
					.parseDouble(getFirstElem(rotationElem, "r20")));

			rotation.setM01(Double
					.parseDouble(getFirstElem(rotationElem, "r01")));
			rotation.setM11(Double
					.parseDouble(getFirstElem(rotationElem, "r11")));
			rotation.setM21(Double
					.parseDouble(getFirstElem(rotationElem, "r21")));

			rotation.setM02(Double
					.parseDouble(getFirstElem(rotationElem, "r02")));
			rotation.setM12(Double
					.parseDouble(getFirstElem(rotationElem, "r12")));
			rotation.setM22(Double
					.parseDouble(getFirstElem(rotationElem, "r22")));

			Element translationElem = root.getFirstChildElement("translation");

			translation.setX(Double.parseDouble(getFirstElem(translationElem,
					"x")));
			translation.setY(Double.parseDouble(getFirstElem(translationElem,
					"y")));
			translation.setZ(Double.parseDouble(getFirstElem(translationElem,
					"z")));

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
		if (!calibrationRead)
			readCamToRobCalibration();

		return translation;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Matrix getCamToRobRotation() throws CalibrationException {
		if (!calibrationRead)
			readCamToRobCalibration();

		return rotation;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void saveCamToRobCalibration(Array2DRowRealMatrix R,
			Array2DRowRealMatrix t) throws CalibrationException {
		Element root = new Element("armCalibration");

		Element rotation = new Element("rotation");

		addElem(rotation, "r00", Double.toString(R.getEntry(0, 0)));
		addElem(rotation, "r10", Double.toString(R.getEntry(1, 0)));
		addElem(rotation, "r20", Double.toString(R.getEntry(2, 0)));

		addElem(rotation, "r01", Double.toString(R.getEntry(0, 1)));
		addElem(rotation, "r11", Double.toString(R.getEntry(1, 1)));
		addElem(rotation, "r21", Double.toString(R.getEntry(2, 1)));

		addElem(rotation, "r02", Double.toString(R.getEntry(0, 2)));
		addElem(rotation, "r12", Double.toString(R.getEntry(1, 2)));
		addElem(rotation, "r22", Double.toString(R.getEntry(2, 2)));

		root.appendChild(rotation);

		Element translation = new Element("translation");

		addElem(translation, "x", Double.toString(t.getEntry(0, 0)));
		addElem(translation, "y", Double.toString(t.getEntry(1, 0)));
		addElem(translation, "z", Double.toString(t.getEntry(2, 0)));

		root.appendChild(translation);

		Document doc = new Document(root);

		try {
			FileWriter fw = new FileWriter(manipulator.getConfiguration()
					.getConfigPath());
			fw.write(doc.toXML());
			fw.close();
		} catch (IOException e) {
			throw new CalibrationException("Cannot write file");
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void resetCamToRobAllPairs() {
		sourcePoints.clear();
		targetPoints.clear();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void resetCamToRobLastPair() {
		sourcePoints.remove(sourcePoints.lastElement());
		targetPoints.remove(targetPoints.lastElement());
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void addCamToRobPair(Vector3D sourcePoint, Vector3D targetPoint) {
		logger.debug("Vision Point: " + sourcePoint + "Arm Point "
				+ targetPoint);

		sourcePoints.add(sourcePoint);
		targetPoints.add(targetPoint);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void testCamToRobAdd() {
		addCamToRobPair(new Vector3D(0, 0, 0), new Vector3D(0, 0, 0));
		addCamToRobPair(new Vector3D(1, 1, 1), new Vector3D(1, 1, 1));
		addCamToRobPair(new Vector3D(2, 2, 2), new Vector3D(2, 2, 2));
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vector3D getCamPointInRob(Vector3D camPoint) {
		Vector3D returnValue = null;
		try {
			returnValue = MathOperation.getVectorAddition(MathOperation
					.getMatrixVectorMultiplication(getCamToRobRotation(),
							camPoint), getCamToRobTranslation());
		} catch (CalibrationException e) {
			logger.error(e);
		}

		return returnValue;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Matrix getCamRotationInRob(Matrix camRotation) {
		Matrix returnValue = null;

		try {
			returnValue = MathOperation.getMatrixMatrixMultiplication(
					getCamToRobRotation(), camRotation);
		} catch (CalibrationException e) {
			logger.error(e);
		}

		return returnValue;
	}
}
