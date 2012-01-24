package manipulation.core.share;

import manipulation.commandWatcher.CommandWatcher;
import manipulation.core.share.armConnector.ArmConnector;
import manipulation.core.share.baseConnector.BaseConnector;
import manipulation.core.share.calibrationConnector.CalibrationConnector;
import manipulation.core.share.camConnector.CamConnector;
import manipulation.core.share.types.Configuration;
import manipulation.core.share.virtualSceneConnector.VirtualSceneConnector;
import manipulation.itemMemory.ItemMemory;
import manipulation.runner.share.Runner;

import org.apache.log4j.Logger;

/**
 * abstract representation of a manipulator
 * 
 * @author Torben Toeniges
 * 
 */
public abstract class Manipulator {
	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * represents different names of possible manipulators
	 * 
	 * @author Torben Toeniges
	 * 
	 */
	public enum ManipulatorName {
		COGX_MOBILE_MANIPULATOR
	}

	private ManipulatorName name;
	private ArmConnector armConnector;
	private BaseConnector baseConnector;
	private CamConnector camConnector;
	private VirtualSceneConnector virtualSceneConnector;
	private CalibrationConnector calibrationConnector;

	private Configuration configuration;
	private ItemMemory itemMemory;
	private CommandWatcher watcher;
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
	 * gets the virtual-scene-connector of the current manipulator
	 * 
	 * @return virtual-scene-connector of the current manipulator
	 */
	public VirtualSceneConnector getVirtualSceneConnector() {
		return virtualSceneConnector;
	}

	/**
	 * sets the virtual-scene-connector of the current manipulator
	 * 
	 * @param virtualSceneConnector
	 *            new virtual-scene-connector
	 */
	public void setVirtualSceneConnector(
			VirtualSceneConnector virtualSceneConnector) {
		this.virtualSceneConnector = virtualSceneConnector;
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
	 * @return the watcher
	 */
	public CommandWatcher getWatcher() {
		return watcher;
	}

	/**
	 * @param watcher
	 *            the watcher to set
	 */
	public void setWatcher(CommandWatcher watcher) {
		this.watcher = watcher;
	}

	/**
	 * prepare the manipulator (instantiate all parts of the manipulator)
	 */
	protected abstract void prepare();
}
