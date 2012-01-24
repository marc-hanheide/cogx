package manipulation.core.cogx.virtualSceneConnector.initObjects;

import golem.tinyice.ArmDesc;
import golem.tinyice.Mat33;
import golem.tinyice.Mat34;
import golem.tinyice.Vec3;

/**
 * Initialization of the ArmDesc ICE class
 * 
 * @author Torben Toeniges
 * 
 */
public class ArmDescI extends ArmDesc {

	private static final long serialVersionUID = 454730042521968537L;

	/**
	 * Constructor of the ArmDesc initialization
	 */
	public ArmDescI() {
		globalPose = new Mat34(new Mat33(1, 0, 0, 0, 1, 0, 0, 0, 1), new Vec3(
				0, 0, 0));
		path = "";
	}
}
