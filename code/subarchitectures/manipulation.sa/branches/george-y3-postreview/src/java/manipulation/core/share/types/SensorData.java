package manipulation.core.share.types;

/**
 * represents the sensor data of the gripper
 * 
 * @author ttoenige
 * 
 */
public class SensorData {
	/**
	 * represents the corresponding sensor position on the gripper
	 * 
	 * @author ttoenige
	 * 
	 */
	public enum SensorPosition {
		/**
		 * force sensor located on the right finger close to the joint
		 */
		FORCE_RIGHT_NEAR,
		/**
		 * force sensor located on the right finger far from the joint
		 */
		FORCE_RIGHT_FAR,
		/**
		 * force sensor located on the left finger close to the joint
		 */
		FORCE_LEFT_NEAR,
		/**
		 * force sensor located on the left finger far from the joint
		 */
		FORCE_LEFT_FAR,
		/**
		 * infrared sensor located on the right finger on the outside
		 */
		INFRARED_RIGHT_OUTSIDE,
		/**
		 * infrared sensor located on the right finger on the front
		 */
		INFRARED_RIGHT_FRONT,
		/**
		 * infrared sensor located on the right finger on the inside close to
		 * the joint
		 */
		INFRARED_RIGHT_INSIDE_NEAR,
		/**
		 * infrared sensor located on the right finger on the inside far from
		 * the joint
		 */
		INFRARED_RIGHT_INSIDE_FAR,
		/**
		 * infrared sensor located on the left finger on the outside
		 */
		INFRARED_LEFT_OUTSIDE,
		/**
		 * infrared sensor located on the left finger on the front
		 */
		INFRARED_LEFT_FRONT,
		/**
		 * infrared sensor located on the left finger on the inside close to the
		 * joint
		 */
		INFRARED_LEFT_INSIDE_NEAR,
		/**
		 * infrared sensor located on the left finger on the inside far from the
		 * joint
		 */
		INFRARED_LEFT_INSIDE_FAR,
		/**
		 * infrared sensor located in the middle between both fingers
		 */
		INFRARED_MIDDLE,
		/**
		 * undefined sensor
		 */
		UNDEFINED
	}

	/**
	 * converts the sensor indices to {@see SensorPosition}
	 * 
	 * @param index
	 *            corresponding index
	 * @return converted SensorPosition
	 */
	public static SensorPosition convertIndexToSensorPosition(int index) {
		switch (index) {
		case 0:
			return SensorPosition.INFRARED_RIGHT_INSIDE_NEAR;
		case 1:
			return SensorPosition.INFRARED_RIGHT_INSIDE_FAR;
		case 2:
			return SensorPosition.UNDEFINED;
		case 3:
			return SensorPosition.UNDEFINED;
		case 4:
			return SensorPosition.INFRARED_RIGHT_OUTSIDE;
		case 5:
			return SensorPosition.INFRARED_RIGHT_FRONT;
		case 6:
			return SensorPosition.FORCE_RIGHT_NEAR;
		case 7:
			return SensorPosition.FORCE_RIGHT_FAR;
		case 8:
			return SensorPosition.INFRARED_LEFT_INSIDE_NEAR;
		case 9:
			return SensorPosition.INFRARED_LEFT_INSIDE_FAR;
		case 10:
			return SensorPosition.UNDEFINED;
		case 11:
			return SensorPosition.INFRARED_MIDDLE;
		case 12:
			return SensorPosition.INFRARED_LEFT_OUTSIDE;
		case 13:
			return SensorPosition.INFRARED_LEFT_FRONT;
		case 14:
			return SensorPosition.FORCE_LEFT_NEAR;
		case 15:
			return SensorPosition.FORCE_LEFT_FAR;
		default:
			return SensorPosition.UNDEFINED;
		}
	}

	/**
	 * converts the {@see SensorPosition} to sensor indices
	 * 
	 * @param sensorPosition
	 *            corresponding sensor position
	 * @return converted index value
	 */
	public static int convertSensorPositionToIndex(SensorPosition sensorPosition) {
		switch (sensorPosition) {
		case INFRARED_RIGHT_INSIDE_NEAR:
			return 0;
		case INFRARED_RIGHT_INSIDE_FAR:
			return 1;
		case INFRARED_RIGHT_OUTSIDE:
			return 4;
		case INFRARED_RIGHT_FRONT:
			return 5;
		case FORCE_RIGHT_NEAR:
			return 6;
		case FORCE_RIGHT_FAR:
			return 7;
		case INFRARED_LEFT_INSIDE_NEAR:
			return 8;
		case INFRARED_LEFT_INSIDE_FAR:
			return 9;
		case INFRARED_MIDDLE:
			return 11;
		case INFRARED_LEFT_OUTSIDE:
			return 12;
		case INFRARED_LEFT_FRONT:
			return 13;
		case FORCE_LEFT_NEAR:
			return 14;
		case FORCE_LEFT_FAR:
			return 15;
		default:
			return -1;
		}
	}

}