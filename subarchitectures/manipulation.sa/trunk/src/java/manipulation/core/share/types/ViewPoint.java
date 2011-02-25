package manipulation.core.share.types;

/**
 * represents a viewpoint of an item
 * 
 * @author ttoenige
 * 
 */
public class ViewPoint {
	private double error;
	private BasePositionData position;

	/**
	 * constructor of a viewpoint
	 * 
	 * @param error
	 *            error-value of a viewpoint (related to the position to the
	 *            item)
	 * @param position
	 *            position of the viewpoint
	 */
	public ViewPoint(double error, BasePositionData position) {
		super();
		this.error = error;
		this.position = position;
	}

	/**
	 * gets the error-value of a viewpoint (related to the position to the item)
	 * 
	 * @return error-value of a viewpoint (related to the position to the item)
	 */
	public double getError() {
		return error;
	}

	/**
	 * sets error-value of a viewpoint (related to the position to the item)
	 * 
	 * @param error
	 */
	public void setError(double error) {
		this.error = error;
	}

	/**
	 * gets the position of the viewpoint
	 * 
	 * @return position of the viewpoint
	 */
	public BasePositionData getPosition() {
		return position;
	}

	/**
	 * sets the position of the viewpoint
	 * 
	 * @param position
	 *            new position
	 */
	public void setPosition(BasePositionData position) {
		this.position = position;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public String toString() {
		return "{VIEWPOINT: error: " + getError() + "position: "
				+ getPosition() + "}";
	}
}
