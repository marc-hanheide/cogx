package manipulation.core.cogx.virtualSceneConnector.initObjects;

import golem.tinyice.Mat33;
import golem.tinyice.Mat34;
import golem.tinyice.RGBA;
import golem.tinyice.SphereShapeDesc;
import golem.tinyice.Vec3;

/**
 * Initialization of the SphereShapeDesc ICE class
 * 
 * @author ttoenige
 * 
 */
public class SphereShapeDescI extends SphereShapeDesc {
	private static final long serialVersionUID = -2051952772990880622L;


	/**
	 * Constructor of the SphereShapeDesc initialization
	 */
	public SphereShapeDescI() {
		radius = 0.1;
		density = 1;
		group = 1;
		localPose = new Mat34(new Mat33(1, 0, 0, 0, 1, 0, 0, 0, 1), new Vec3(0,
				0, 0));
		color = new RGBA(1, 1, 1, 1);
	}
}
