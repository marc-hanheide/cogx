package manipulation.core.share.types;

public class Pose {
	private Matrix rotation;
	private Vector3D translation;

	public Pose(Matrix rotation, Vector3D translation) {
		this.rotation = rotation;
		this.translation = translation;
	}

	/**
	 * @return the rotation
	 */
	public Matrix getRotation() {
		return rotation;
	}

	/**
	 * @param rotation
	 *            the rotation to set
	 */
	public void setRotation(Matrix rotation) {
		this.rotation = rotation;
	}

	/**
	 * @return the translation
	 */
	public Vector3D getTranslation() {
		return translation;
	}

	/**
	 * @param translation
	 *            the translation to set
	 */
	public void setTranslation(Vector3D translation) {
		this.translation = translation;
	}

}
