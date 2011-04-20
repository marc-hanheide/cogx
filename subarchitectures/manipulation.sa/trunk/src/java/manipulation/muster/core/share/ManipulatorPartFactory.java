package manipulation.muster.core.share;

import manipulation.muster.core.share.armConnector.ArmConnector;
import manipulation.muster.core.share.baseConnector.BaseConnector;
import manipulation.muster.core.share.baseConnector.MapConnector;
import manipulation.muster.core.share.calibrationConnector.CalibrationConnector;
import manipulation.muster.core.share.camConnector.CamConnector;
import manipulation.muster.core.share.panTiltConnector.PanTiltConnector;
import manipulation.muster.core.share.virtualSceneConnector.VirtualSceneConnector;

/**
 * factory to build the manipulator
 * 
 * @author ttoenige
 * 
 */
public interface ManipulatorPartFactory {
	/**
	 * creates an arm hardware connector
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @return created arm hardware connector
	 */
	public ArmConnector createArmConnector(Manipulator manipulator);

	/**
	 * creates a base hardware connector
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @return created base hardware connector
	 */
	public BaseConnector createBaseConnector(Manipulator manipulator);

	/**
	 * creates a camera connector
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @return created camera connector
	 */
	public CamConnector createCamConnector(Manipulator manipulator);

	/**
	 * creates a map connector to navigate on
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @return created map connector
	 */
	public MapConnector createMapConnector(Manipulator manipulator);

	/**
	 * creates a virtual scene connector
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @return created virtual scene connector
	 */
	public VirtualSceneConnector createVirtualSceneConnector(Manipulator manipulator);

	/**
	 * creates a calibration connector
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @return created calibration connector
	 */
	public CalibrationConnector createCalibrationConnector(
			Manipulator manipulator);

	public PanTiltConnector createPanTiltConnector(Manipulator manipulator);

}
