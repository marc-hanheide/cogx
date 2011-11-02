package manipulation.core.share.types;

import manipulation.core.share.armConnector.ArmConnector;

/**
 * represents the given configuration of the program
 * 
 * @author Torben Toeniges
 * 
 */
public class Configuration {
	String configPath;
	ArmConnector.ArmName armName;
	boolean gui;
  boolean enabledTableObstacleHack;

	public Configuration(String configPath, ArmConnector.ArmName armName,
			boolean gui, boolean enabledTableObstacleHack) {
		this.configPath = configPath;
		this.armName = armName;
		this.gui = gui;
    this.enabledTableObstacleHack = enabledTableObstacleHack;
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
	 * @return the gui
	 */
	public boolean isGui() {
		return gui;
	}

	/**
	 * @param gui
	 *            the gui to set
	 */
	public void setGui(boolean gui) {
		this.gui = gui;
	}

	/**
	 * @return enabledTableObstacleHack
	 */
	public boolean isEnabledTableObstacleHack() {
		return enabledTableObstacleHack;
	}

	/**
	 * @param enabledTableObstacleHack
	 */
	public void setTableObstacleHack(boolean set) {
		this.enabledTableObstacleHack = set;
	}
}
