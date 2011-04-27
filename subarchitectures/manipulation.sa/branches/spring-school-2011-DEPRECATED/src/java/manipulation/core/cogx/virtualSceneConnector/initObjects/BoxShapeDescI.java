package manipulation.core.cogx.virtualSceneConnector.initObjects;

import golem.tinyice.BoxShapeDesc;
import golem.tinyice.Mat33;
import golem.tinyice.Mat34;
import golem.tinyice.RGBA;
import golem.tinyice.Vec3;

/**
 * Initialization of the BoxShapeDesc ICE class
 * 
 * @author Torben Toeniges
 * 
 */
public class BoxShapeDescI extends BoxShapeDesc {

	private static final long serialVersionUID = 3826111681446112514L;

	/**
	 * Constructor of the BoxShapeDesc initialization
	 */
	public BoxShapeDescI() {
		dimensions = new Vec3(0.1, 0.1, 0.1);
		density = 1;
		group = 1;
		localPose = new Mat34(new Mat33(1, 0, 0, 0, 1, 0, 0, 0, 1), new Vec3(0,
				0, 0));
		color = new RGBA(1, 1, 1, 1);
	}
	
}
