package manipulation.core.share.types;


/**
 * represents a border point of a region
 * 
 * @author ttoenige
 * 
 */
public class BorderPoint {
	private Vector2D point;

	/**
	 * constructor of a border point
	 * 
	 * @param point
	 *            position value of the point (in meter)
	 */
	public BorderPoint(Vector2D point) {
		this.point = point;
	}

	/**
	 * gets the relevant border point
	 * 
	 * @return relevant border point
	 */
	public Vector2D getPoint() {
		return point;
	}

	/**
	 * sets the border point
	 * 
	 * @param point
	 *            new point value
	 */
	public void setPoint(Vector2D point) {
		this.point = point;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public String toString() {
		return "{BORDERPOINT: point: " + getPoint() + "}";
	}

}
