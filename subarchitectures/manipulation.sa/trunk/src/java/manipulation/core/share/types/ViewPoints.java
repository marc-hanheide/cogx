package manipulation.core.share.types;

import java.util.List;

public class ViewPoints {

	List<ViewPoint> points;

	/**
	 * @return the points
	 */
	public List<ViewPoint> getPoints() {
		return points;
	}

	/**
	 * @param points
	 *            the points to set
	 */
	public void setPoints(List<ViewPoint> points) {
		this.points = points;
	}

	public ViewPoints(List<ViewPoint> points) {
		this.points = points;
	}

}
