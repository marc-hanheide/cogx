/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celm.util;

import NavData.RobotPose2d;
import celm.conversion.CASTTimeConverter;

public class TimedPosition {

	// TODO: unset this
	private final long milliseconds;

	private final RobotPose2d rp;
	//private final Object rp;

	public TimedPosition(RobotPose2d rp) {

		this.milliseconds = CASTTimeConverter.toMillisecondsSinceEpochTime(rp.time);
		this.rp = rp;
	}

	public TimedPosition(RobotPose2d rp, long milliseconds) {

		this.milliseconds = milliseconds;
		this.rp = rp;
	}

	public long getTime() {
		return milliseconds;
	}

	public Object getRobotPose() {
		return rp;
	}

	public String toString() {
		return "TimedPosition (t: " + milliseconds + ", x: " + rp.x + ", y: " + rp.y + ")";
		
	}

}
