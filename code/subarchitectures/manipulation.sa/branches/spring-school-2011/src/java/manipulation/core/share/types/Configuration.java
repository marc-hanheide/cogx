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

	public Configuration(String configPath, ArmConnector.ArmName armName) {
		this.configPath = configPath;
		this.armName = armName;
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
}
