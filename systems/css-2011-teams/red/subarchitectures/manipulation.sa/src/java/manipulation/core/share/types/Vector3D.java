package manipulation.core.share.types;

/**
 * represents a 3D vector
 * 
 * @author ttoenige
 * 
 */
public class Vector3D {

	private double x;
	private double y;
	private double z;

	/**
	 * constructor of the vector
	 * 
	 * @param x
	 *            new x-value
	 * @param y
	 *            new y-value
	 * @param z
	 *            new z-value
	 */
	public Vector3D(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	/**
	 * gets the x-value of the vector
	 * 
	 * @return x-value of the vector
	 */
	public double getX() {
		return x;
	}

	/**
	 * sets the x-value of the vector
	 * 
	 * @param x
	 *            new value
	 */
	public void setX(double x) {
		this.x = x;
	}

	/**
	 * gets the y-value of the vector
	 * 
	 * @return y-value of the vector
	 */
	public double getY() {
		return y;
	}

	/**
	 * sets the y-value of the vector
	 * 
	 * @param y
	 *            new value
	 */
	public void setY(double y) {
		this.y = y;
	}

	/**
	 * gets the z-value of the vector
	 * 
	 * @return z-value of the vector
	 */
	public double getZ() {
		return z;
	}

	/**
	 * sets the z-value of the vector
	 * 
	 * @param z
	 *            new value
	 */
	public void setZ(double z) {
		this.z = z;
	}

	/**
	 * gets a 2D vector without the z-value of this vector
	 * 
	 * @return 2D vector without the z-value of this vector
	 */
	public Vector2D forgetThirdDimension() {
		return new Vector2D(getX(), getY());
	}

	/**
	 * normalizes the vector
	 * 
	 * @return normalized vector
	 */
	public Vector3D norm() {
		return new Vector3D(getX() / getLength(), getY() / getLength(), getZ()
				/ getLength());
	}

	/**
	 * gets the length of the vector
	 * 
	 * @return length of the vector
	 */
	public double getLength() {
		return Math.sqrt(Math.pow(getX(), 2) + Math.pow(getY(), 2)
				+ Math.pow(getZ(), 2));
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public String toString() {
		return "{VECTOR3D: x:" + getX() + " y: " + getY() + " z: " + getZ()
				+ "}";
	}

	@Override
	public boolean equals(Object arg0) {
		Vector3D input = ((Vector3D) arg0);

		if (this.getX() == input.getX() && this.getY() == input.getY()
				&& this.getZ() == input.getZ()) {
			return true;
		} else {
			return false;
		}

	}

}
