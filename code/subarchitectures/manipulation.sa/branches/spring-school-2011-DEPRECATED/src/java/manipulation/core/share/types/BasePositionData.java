package manipulation.core.share.types;

/**
 * represents a base position of the robot
 * 
 * @author Torben Toeniges
 * 
 */
public class BasePositionData {

	private Vector2D point;
	private double angle;

	/**
	 * copy constructor for a robot a base position
	 * 
	 * @param data
	 *            to copy position data
	 */
	public BasePositionData(BasePositionData data) {
		this.point = data.point;
		this.angle = data.angle;
	}

	/**
	 * constructor for a base position
	 * 
	 * @param x
	 *            x-coordinate of the base position in meter
	 * @param y
	 *            y-coordinate of the base position in meter
	 * @param angle
	 *            angle of the base position in radiant
	 */
	public BasePositionData(double x, double y, double angle) {
		point = new Vector2D(x, y);
		this.angle = angle;
	}

	/**
	 * constructor for a base position
	 * 
	 * @param point
	 *            relevant base position point in meter
	 * @param angle
	 *            angle of the base position in radiant
	 */
	public BasePositionData(Vector2D point, double angle) {
		this.point = point;
		this.angle = angle;
	}

	/**
	 * gets the (x,y)-position of the base position in meter
	 * 
	 * @return (x,y)-position of the base position in meter
	 */
	public Vector2D getPoint() {
		return point;
	}

	/**
	 * sets the (x,y)-position of the base position in meter
	 * 
	 * @param point
	 *            relevant new point
	 */
	public void setPoint(Vector2D point) {
		this.point = point;
	}

	/**
	 * gets the angle of the base in radiant
	 * 
	 * @return angle of the base in radiant
	 */
	public double getAngle() {
		return angle;
	}

	/**
	 * sets the corresponding angle of the base
	 * 
	 * @param angle
	 *            input angle value in radiant
	 */
	public void setAngle(double angle) {
		this.angle = angle;
	}

	/**
	 * converts the base position in an array ([0] = x-value , [1] = y-value,
	 * [2] = angle)
	 * 
	 * @return converted array data ([0] = x-value , [1] = y-value, [2] = angle)
	 */
	public double[] toArray() {
		return new double[] { point.getX(), point.getY(), angle };
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public String toString() {
		return "{BASENAVIGATIONDATA: point: " + getPoint() + " angle: "
				+ getAngle() + "}";
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean equals(Object data) {
		if (getPoint().equals(((BasePositionData) data).point)
				&& getAngle() == ((BasePositionData) data).getAngle()) {
			return true;
		} else {
			return false;
		}
	}

}
