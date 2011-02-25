package manipulation.core.share.types;

import java.util.List;

import manipulation.core.share.exceptions.ItemException;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.PropertyName;

/**
 * represents a region around an item.
 * 
 * @author ttoenige
 * 
 */
public class Region {

	List<Vector2D> innerPoints;
	List<BorderPoint> borderPoints;

	/**
	 * constructor of a region for an item
	 * 
	 * @param innerPoints
	 *            a list of the point inside the region
	 * @param borderPoints
	 *            a list of the border points of the region
	 */
	public Region(List<Vector2D> innerPoints, List<BorderPoint> borderPoints) {
		this.innerPoints = innerPoints;
		this.borderPoints = borderPoints;

	}

	/**
	 * gets the inner points
	 * 
	 * @return the included point of the region
	 */
	public List<Vector2D> getInnerPoints() {
		return innerPoints;
	}

	/**
	 * sets the included points of the region
	 * 
	 * @param innerPoints
	 */
	public void setInnerPoints(List<Vector2D> innerPoints) {
		this.innerPoints = innerPoints;
	}

	/**
	 * gets the border points
	 * 
	 * @return all border points of the region
	 */
	public List<BorderPoint> getBorderPoints() {
		return borderPoints;
	}

	/**
	 * sets the border points of the region
	 * 
	 * @param borderViewPoints
	 */
	public void setBorderPoints(List<BorderPoint> borderViewPoints) {
		this.borderPoints = borderViewPoints;
	}

	/**
	 * updates the best border point of this region
	 * 
	 * @param item
	 *            item in world coordinates
	 * @return best border point of this region
	 * @throws ItemException
	 */
	public BorderPoint updateBestBorderPoint(Item item) throws ItemException {

		Vector3D objPoint = ((Vector3D) item
				.getAttribute(PropertyName.WORLD_POSITION));
		BorderPoint nearestPoint = null;
		double shortestDist = Double.MAX_VALUE;
		for (BorderPoint borderPoint : borderPoints) {
			double currentDist = borderPoint.getPoint().calculateDistance(
					objPoint.forgetThirdDimension());
			if (currentDist < shortestDist) {
				nearestPoint = borderPoint;
				shortestDist = currentDist;
			}
		}
		return nearestPoint;
	}

	/**
	 * checks if the given point is inside the region
	 * 
	 * @param objPoint
	 *            world position of the object
	 * @return <code>true</code> if point is inside the region,
	 *         <code>false</code> if the point is not inside the region
	 */
	public boolean isInside(Vector3D objPoint) {
		if (innerPoints.contains(objPoint.forgetThirdDimension()))
			return true;
		else
			return false;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public String toString() {
		return "{REGION: Innerpoints:" + innerPoints.toString()
				+ " BorderViewPoint: " + borderPoints.toString() + "}";
	}

}
