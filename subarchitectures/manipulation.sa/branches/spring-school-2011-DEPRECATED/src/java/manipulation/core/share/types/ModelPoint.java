package manipulation.core.share.types;

/**
 * represents a point of the visual model an item
 * 
 * @author ttoenige
 * 
 */
public class ModelPoint {
	private Vector3D position;
	private Vector3D normal;

	/**
	 * constructor of a point of the visual model of an item
	 * 
	 * @param position
	 *            position value of the point
	 * @param normal
	 *            normal of the point
	 */
	public ModelPoint(Vector3D position, Vector3D normal) {
		this.position = position;
		this.normal = normal;
	}

	/**
	 * gets the position of the model point
	 * 
	 * @return position of the model point
	 */
	public Vector3D getPosition() {
		return position;
	}

	/**
	 * sets the position of the model point
	 * 
	 * @param position
	 *            new position value
	 */
	public void setPosition(Vector3D position) {
		this.position = position;
	}

	/**
	 * gets the normal of the point
	 * 
	 * @return normal of the point
	 */
	public Vector3D getNormal() {
		return normal;
	}

	/**
	 * sets the normal of the model-point
	 * 
	 * @param normal
	 */
	public void setNormal(Vector3D normal) {
		this.normal = normal;
	}
}
