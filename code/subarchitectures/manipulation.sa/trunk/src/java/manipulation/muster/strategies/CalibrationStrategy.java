package manipulation.muster.strategies;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.strategies.parts.StrategyPart.PartName;
import manipulation.muster.strategies.parts.calibration.CalibrationPart;

/**
 * defines a global strategy (state machine) to calibrate the arm to the camera
 * 
 * @author ttoenige
 * 
 */
public class CalibrationStrategy extends Strategy {
	/**
	 * constructor of the global strategy
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public CalibrationStrategy(Manipulator manipulator) {
		setManipulator(manipulator);
		initParts();
		setFirstPart(getPart(PartName.CALIBRATION));
		setName(Name.CALIBRATION);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void initParts() {
		addToPartList(new CalibrationPart(getManipulator(), this));
	}

}
