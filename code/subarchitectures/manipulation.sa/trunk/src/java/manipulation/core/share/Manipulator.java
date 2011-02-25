package manipulation.core.share;

import manipulation.core.share.armConnector.ArmConnector;
import manipulation.core.share.baseConnector.BaseConnector;
import manipulation.core.share.baseConnector.MapAlgorithms;
import manipulation.core.share.baseConnector.MapConnector;
import manipulation.core.share.calibrationConnector.CalibrationConnector;
import manipulation.core.share.camConnector.CamConnector;
import manipulation.core.share.panTiltConnector.PanTiltConnector;
import manipulation.core.share.simulationConnector.SimulationConnector;
import manipulation.core.share.types.Configuration;
import manipulation.itemMemory.ItemMemory;
import manipulation.runner.share.Runner;

import org.apache.log4j.Logger;

/**
 * abstract representation of a manipulator
 * 
 * @author ttoenige
 * 
 */
public abstract class Manipulator {
	private Logger logger = Logger.getLogger(this.getClass());
	
	/**
	 * represent different names of possible manipulators
	 * 
	 * @author ttoenige
	 * 
	 */
	public enum ManipulatorName {
		/**
		 * manipulator name for an intelligent grasping task
		 */
		INTELLIGENT_GRASPING
	}

	private ManipulatorName name;
	private ArmConnector armConnector;
	private BaseConnector baseConnector;
	private CamConnector camConnector;
	private MapConnector mapConnector;
	private SimulationConnector simulationConnector;
	private CalibrationConnector calibrationConnector;
	private PanTiltConnector panTiltConnector;

	private MapAlgorithms mapAlgorithms;
	private Configuration configuration;
	private ItemMemory itemMemory;
	private Runner runner;

	/**
	 * gets the name of the manipulator
	 * 
	 * @return name of the manipulator
	 */
	public ManipulatorName getName() {
		return name;
	}

	/**
	 * sets the name of the manipulator
	 * 
	 * @param name
	 *            new name value
	 */
	public void setName(ManipulatorName name) {
		this.name = name;
	}

	/**
	 * gets the arm-connector of the current manipulator
	 * 
	 * @return arm-connector of the current manipulator
	 */
	public ArmConnector getArmConnector() {
		return armConnector;
	}

	/**
	 * sets the arm-connector of the current manipulator
	 * 
	 * @param armConnector
	 *            new arm-connector
	 */
	public void setArmConnector(ArmConnector armConnector) {
		this.armConnector = armConnector;
	}

	/**
	 * gets the base-connector of the current manipulator
	 * 
	 * @return base-connector of the current manipulator
	 */
	public BaseConnector getBaseConnector() {
		return baseConnector;
	}

	/**
	 * sets the base-connector of the current manipulator
	 * 
	 * @param baseConnector
	 *            new base-connector
	 */
	public void setBaseConnector(BaseConnector baseConnector) {
		this.baseConnector = baseConnector;
	}

	/**
	 * gets the camera-connector of the current manipulator
	 * 
	 * @return camera-connector of the current manipulator
	 */
	public CamConnector getCamConnector() {
		return camConnector;
	}

	/**
	 * sets the camera-connector of the current manipulator
	 * 
	 * @param camConnector
	 *            new camera-connector
	 */
	public void setCamConnector(CamConnector camConnector) {
		this.camConnector = camConnector;
	}

	/**
	 * gets the map-connector of the current manipulator
	 * 
	 * @return map-connector of the current manipulator
	 */
	public MapConnector getMapConnector() {
		return mapConnector;
	}

	/**
	 * sets the map-connector of the current manipulator
	 * 
	 * @param mapConnector
	 *            new map-connector
	 */
	public void setMapConnector(MapConnector mapConnector) {
		this.mapConnector = mapConnector;
	}

	/**
	 * gets the virtual-scene-connector of the current manipulator
	 * 
	 * @return virtual-scene-connector of the current manipulator
	 */
	public SimulationConnector getSimulationConnector() {
		return simulationConnector;
	}

	/**
	 * sets the virtual-scene-connector of the current manipulator
	 * 
	 * @param simulationConnector
	 *            new virtual-scene-connector
	 */
	public void setSimulationConnector(SimulationConnector simulationConnector) {
		this.simulationConnector = simulationConnector;
	}

	/**
	 * gets the calibration-connector of the current manipulator
	 * 
	 * @return calibration-connector of the current manipulator
	 */
	public CalibrationConnector getCalibrationConnector() {
		return calibrationConnector;
	}

	/**
	 * sets the calibration-connector of the current manipulator
	 * 
	 * @param calibrationConnector
	 *            new calibration-connector
	 */
	public void setCalibrationConnector(
			CalibrationConnector calibrationConnector) {
		this.calibrationConnector = calibrationConnector;
	}

	/**
	 * @return the panTiltConnector
	 */
	public PanTiltConnector getPanTiltConnector() {
		return panTiltConnector;
	}

	/**
	 * @param panTiltConnector
	 *            the panTiltConnector to set
	 */
	public void setPanTiltConnector(PanTiltConnector panTiltConnector) {
		this.panTiltConnector = panTiltConnector;
	}

	/**
	 * gets the map-algorithms
	 * 
	 * @return map-algorithms of the current manipulator
	 */
	public MapAlgorithms getMapAlgorithms() {
		// TODO was ist wenn nicht da?
		return mapAlgorithms;
	}

	/**
	 * generate the map-algorithms
	 */
	public void generateMapAlgorithms() {
		this.mapAlgorithms = new MapAlgorithms(this);
	}

	/**
	 * gets the configuration of the current manipulator
	 * 
	 * @return configuration of the current manipulator
	 */
	public Configuration getConfiguration() {
		return configuration;
	}

	/**
	 * sets the configuration of the current manipulator
	 * 
	 * @param configuration
	 *            new configuration of the current manipulator
	 */
	public void setConfiguration(Configuration configuration) {
		this.configuration = configuration;
	}

	/**
	 * gets the item-memory of the current manipulator
	 * 
	 * @return item-memory of the current manipulator
	 */
	public ItemMemory getItemMemory() {
		return itemMemory;
	}

	/**
	 * sets the item-memory of the current manipulator
	 * 
	 * @param itemMemory
	 *            new item-memory of the current manipulator
	 */
	public void setItemMemory(ItemMemory itemMemory) {
		this.itemMemory = itemMemory;
	}

	/**
	 * gets the runner of the current manipulator
	 * 
	 * @return runner of the current manipulator
	 */
	public Runner getRunner() {
		return runner;
	}

	/**
	 * sets the runner of the current manipulator
	 * 
	 * @param runner
	 *            new runner of the current manipulator
	 */
	public void setRunner(Runner runner) {
		this.runner = runner;
	}

	/**
	 * prepare the manipulator (instantiate all parts of the manipulator)
	 */
	protected abstract void prepare();
}
