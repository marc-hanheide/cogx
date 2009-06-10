/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celm.util;

//import NavData.RobotPose;


public class TimedPosition {

	//TODO: unset this
	private final long milliseconds = 0;

	// private final RobotPose rp;

//	public TimedPosition(RobotPose rp) {
//
//		this.milliseconds = CASTTimeConverter
//				.toMillisecondsSinceEpochTime(rp.m_time);
////		this.rp = rp;
//	}

//	public TimedPosition(RobotPose rp, long milliseconds) {
//
//		this.milliseconds = milliseconds;
////		this.rp = rp;
//	}

	public long getTime() {
		return milliseconds;
	}

//	public RobotPose getRobotPose() {
//		return rp;
//	}

	public String toString() {
//		return "TimedPosition (t: " + milliseconds + ", x: " + rp.x + ", y: "
//				+ rp.y + ")";
		return "TimedPosition (t: " + milliseconds + ")";
	}

}
