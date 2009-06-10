/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celm.util;

import celm.conversion.CASTTimeConverter;
import dummy.RobotPose;

//import NavData.RobotPose;

public class TimedPosition {

	// TODO: unset this
	private final long milliseconds;

	private final RobotPose rp;

	public TimedPosition(RobotPose rp) {

		this.milliseconds = CASTTimeConverter
				.toMillisecondsSinceEpochTime(rp.time);
		this.rp = rp;
	}

	public TimedPosition(RobotPose rp, long milliseconds) {

		this.milliseconds = milliseconds;
		this.rp = rp;
	}

	public long getTime() {
		return milliseconds;
	}

	public RobotPose getRobotPose() {
		return rp;
	}

	public String toString() {
		return "TimedPosition (t: " + milliseconds + ", x: " + rp.x + ", y: "
				+ rp.y + ")";
	}

}
