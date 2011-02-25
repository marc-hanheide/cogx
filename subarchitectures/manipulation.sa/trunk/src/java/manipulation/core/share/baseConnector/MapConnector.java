package manipulation.core.share.baseConnector;

import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.GlobalMap;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;

/**
 * represents a connector to a map
 * 
 * @author ttoenige
 * 
 */
public interface MapConnector {
	/**
	 * gets the map value of world coordinates (3D)
	 * 
	 * @param data
	 *            world coordinates (z-value will be deleted)
	 * @return map value of the world coordinates
	 */
	public double getDataFromWorldCoordinates(Vector3D data);

	/**
	 * gets the map value of world coordinates (2D)
	 * 
	 * @param data
	 *            world coordinates
	 * @return map value of the world coordinates
	 */
	public double getDataFromWorldCoordinates(Vector2D data);

	/**
	 * get the index value of given world coordinates (3D)
	 * 
	 * @param data
	 *            given world coordinates (z-value will be deleted)
	 * @return index value of the world coordinates
	 */
	public Vector2D getIndexFromWorldCoordinates(Vector3D data);

	/**
	 * get the index value of given world coordinates (2D)
	 * 
	 * @param data
	 *            given world coordinates
	 * @return index value of the world coordinates
	 */
	public Vector2D getIndexFromWorldCoordinates(Vector2D data);

	/**
	 * get the index value of a given base position
	 * 
	 * @param data
	 *            given base position of the base
	 * @return index value of the world coordinates
	 */
	public Vector2D getIndexFromWorldCoordinates(BasePositionData data);

	/**
	 * gets the data value of the map from given index
	 * 
	 * @param index
	 *            given index
	 * @return data value of the map
	 */
	public double getDataFromIndex(Vector2D index);

	/**
	 * gets world coordinates from given index value
	 * 
	 * @param point
	 *            given index value
	 * @return world coordinates of the given index value
	 */
	public Vector2D getWorldFromIndex(Vector2D point);

	/**
	 * update the map
	 * 
	 * @throws ExternalMemoryException
	 */
	public void updateMap() throws ExternalMemoryException;

	/**
	 * update the map in a constant interval
	 * 
	 * 
	 */
	public void updateMapConstant();

	/**
	 * stops updating the map in a constant interval
	 * 
	 * 
	 */
	public void stopupdateMapConstant();

	/**
	 * gets the last updated map (last time updateMap is used)
	 * 
	 * @return last updated map (last time updateMap is used)
	 */
	public GlobalMap getMap();

	/**
	 * checks weather the point is free
	 * 
	 * @param point
	 *            world coordinates of the point to check
	 * @return <code>true</code> if the point is free <code>false</code> if the
	 *         point is not free
	 */
	public boolean isPointFree(Vector2D point);

	/**
	 * checks weather the point is unknown
	 * 
	 * @param point
	 *            world coordinates of the point to check
	 * @return <code>true</code> if the point is unknown <code>false</code> if
	 *         the point is not unknown
	 */
	public boolean isPointUnknown(Vector2D point);

	/**
	 * checks weather the point is an obstacle
	 * 
	 * @param point
	 *            world coordinates of the point to check
	 * @return <code>true</code> if the point is an obstacle <code>false</code>
	 *         if the point is not an obstacle
	 */
	public boolean isPointObstacle(Vector2D point);
}
