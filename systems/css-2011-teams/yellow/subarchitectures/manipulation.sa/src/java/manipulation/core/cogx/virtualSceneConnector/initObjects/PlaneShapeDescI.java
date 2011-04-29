package manipulation.core.cogx.virtualSceneConnector.initObjects;

import golem.tinyice.Mat33;
import golem.tinyice.Mat34;
import golem.tinyice.PlaneShapeDesc;
import golem.tinyice.RGBA;
import golem.tinyice.Vec3;

/**
 * Initialization of the PlaneShapeDesc ICE class
 * 
 * @author Torben Toeniges
 * 
 */
public class PlaneShapeDescI extends PlaneShapeDesc {
	
	private static final long serialVersionUID = -2372664238934726967L;

	/**
	 * Constructor of the PlaneShapeDesc initialization
	 */
	public PlaneShapeDescI() {

		normal = new Vec3(0, 0, 1);
		distance = 0;
		localPose = new Mat34(new Mat33(1, 0, 0, 0, 1, 0, 0, 0, 1), new Vec3(0,
				0, 0));

		density = 1;
		group = 1;

		color = new RGBA(1, 1, 1, 1);
	}

}
