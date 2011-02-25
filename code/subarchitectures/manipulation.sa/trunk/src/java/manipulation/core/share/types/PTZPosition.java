package manipulation.core.share.types;

public class PTZPosition {
	private double pan;
	private double tilt;

	/**
	 * @return the pan
	 */
	public double getPan() {
		return pan;
	}

	/**
	 * @param pan
	 *            the pan to set
	 */
	public void setPan(double pan) {
		this.pan = pan;
	}

	/**
	 * @return the tilt
	 */
	public double getTilt() {
		return tilt;
	}

	/**
	 * @param tilt
	 *            the tilt to set
	 */
	public void setTilt(double tilt) {
		this.tilt = tilt;
	}

	public PTZPosition(double pan, double tilt) {
		this.pan = pan;
		this.tilt = tilt;
	}

	
}
