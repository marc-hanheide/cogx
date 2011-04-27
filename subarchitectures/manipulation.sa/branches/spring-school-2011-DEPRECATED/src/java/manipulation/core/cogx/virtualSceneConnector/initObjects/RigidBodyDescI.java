package manipulation.core.cogx.virtualSceneConnector.initObjects;

import golem.tinyice.Mat33;
import golem.tinyice.Mat34;
import golem.tinyice.RigidBodyDesc;
import golem.tinyice.ShapeDesc;
import golem.tinyice.Vec3;

/**
 * Initialization of the RigidBodyDesc ICE class
 * 
 * @author Torben Toeniges
 * 
 */
public class RigidBodyDescI extends RigidBodyDesc {
	private static final long serialVersionUID = 1806815465299420447L;

	/**
	 * Constructor of the RigidBodyDesc initialization
	 */
	public RigidBodyDescI() {
		shapes = new ShapeDesc[1];
		kinematic = false;

		globalPose = new Mat34(new Mat33(1, 0, 0, 0, 1, 0, 0, 0, 1), new Vec3(
				0, 0, 0));
	}
}
