package manipulation.core.share.types;

public class ArmError {
	private Vector3D poseError;
	private double angleError;
	private boolean success;

	/**
	 * @return the poseError
	 */
	public Vector3D getPoseError() {
		return poseError;
	}

	/**
	 * @param poseError
	 *            the poseError to set
	 */
	public void setPoseError(Vector3D poseError) {
		this.poseError = poseError;
	}

	/**
	 * @return the angleError
	 */
	public double getAngleError() {
		return angleError;
	}

	/**
	 * @param angleError
	 *            the angleError to set
	 */
	public void setAngleError(double angleError) {
		this.angleError = angleError;
	}

	/**
	 * @return the success
	 */
	public boolean isSuccess() {
		return success;
	}

	/**
	 * @param success
	 *            the success to set
	 */
	public void setSuccess(boolean success) {
		this.success = success;
	}

	public ArmError(Vector3D poseError, double angleError, boolean success) {
		this.poseError = poseError;
		this.angleError = angleError;
		this.success = success;
	}
	
	public ArmError(Vector3D poseError, double angleError) {
		this.poseError = poseError;
		this.angleError = angleError;
		this.success = true;
	}


	public ArmError() {
	}

}
