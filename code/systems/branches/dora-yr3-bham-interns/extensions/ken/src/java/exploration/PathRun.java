package exploration;

import java.io.Serializable;
import java.util.Date;

/**
 * class for storing a single run in a network
 * 
 * @author ken
 * 
 */
public class PathRun implements Serializable {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Date timeStart;
	private long timeTaken;

	public PathRun(Date timeStart) {
		this.timeStart = timeStart;

	}

	public void setTimeTaken(Date currentDate) {
		timeTaken = currentDate.getTime() - timeStart.getTime();
	}

	/**
	 * will return 0 if the run was a failiure
	 * 
	 * @return
	 */
	public long timeTaken() {

		return timeTaken;
	}

	public Date timeStarted() {
		return timeStart;
	}

	@Override
	public String toString() {
		if (timeTaken != 0) {
			return "Run beginning at " + timeStart + " took " + timeTaken;
		} else {
			return "Run beginning at" + timeStart + " has failed";

		}
	}
}
