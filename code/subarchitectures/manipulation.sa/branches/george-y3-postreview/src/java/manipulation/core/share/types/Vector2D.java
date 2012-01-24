package manipulation.core.share.types;

/**
 * represents a 2D-Vector
 * 
 * @author ttoenige
 * 
 */
public class Vector2D {
	private double x;
	private double y;

	/**
	 * default constructor
	 */
	public Vector2D() {

	}

	/**
	 * constructor using (x,y) values
	 * 
	 * @param x
	 *            x-value of the vector
	 * @param y
	 *            y-value of the vector
	 */
	public Vector2D(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * copy constructor
	 * 
	 * @param vector
	 *            new vector value
	 */
	public Vector2D(Vector2D vector) {
		this.x = vector.getX();
		this.y = vector.getY();
	}

	/**
	 * gets the x-value
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
	 * gets the y-value
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
	 * calculates the distance between this vector and another vector
	 * 
	 * @param point
	 *            target vector for the distance calculation
	 * @return distance between the vectors
	 */
	public double calculateDistance(Vector2D point) {
		return Math.sqrt(Math.pow(Math.abs(this.getX() - point.getX()), 2)
				+ Math.pow(Math.abs(this.getY() - point.getY()), 2));
	}

	/**
	 * adds another vector to this vector
	 * 
	 * @param point
	 *            vector to add
	 */
	public void add(Vector2D point) {
		setX(getX() + point.getX());
		setY(getY() + point.getY());
	}

	/**
	 * gets the length of the vector
	 * 
	 * @return length of the vector
	 */
	public double getLength() {
		return Math.sqrt(Math.pow(getX(), 2) + Math.pow(getY(), 2));
	}

	/**
	 * gets the normalized vector
	 * 
	 * @return normalized vector
	 */
	public Vector2D norm() {
		return new Vector2D(getX() / getLength(), getY() / getLength());
	}

	/**
	 * converts the vector to an array ([0] = x-value, [1] = y-value)
	 * 
	 * @return array representation
	 */
	public double[] toArray() {
		return new double[] { x, y };
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean equals(Object obj) {
		if (this.getX() == ((Vector2D) obj).getX()
				&& this.getY() == ((Vector2D) obj).getY())
			return true;
		else
			return false;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public String toString() {
		return "{VECTOR2D: x:" + getX() + " y: " + getY() + "}";
	}
}
