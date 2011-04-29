package manipulation.core.cogx.virtualSceneConnector.initObjects;

import golem.tinyice.KatanaArmDesc;
import golem.tinyice.Mat33;
import golem.tinyice.Mat34;
import golem.tinyice.Vec3;

/**
 * Initialization of the KatanaArmDesc ICE class
 * 
 * @author Torben Toeniges
 * 
 */
public class KatanaArmDescI extends KatanaArmDesc {

	private static final long serialVersionUID = 454730042521968537L;

	/**
	 * Constructor of the KatanaArmDesc initialization
	 */
	public KatanaArmDescI() {
		bGripper = true;
		
		sensorIndexSet = new int[16];
		
		for (int i = 0; i < sensorIndexSet.length; i++) {
			sensorIndexSet[i] = i;
		}
		
		globalPose = new Mat34(new Mat33(1, 0, 0, 0, 1, 0, 0, 0, 1), new Vec3(
				0, 0, 0));
		path = "";
	}
}
