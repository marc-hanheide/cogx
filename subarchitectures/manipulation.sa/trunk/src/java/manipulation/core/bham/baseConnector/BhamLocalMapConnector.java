package manipulation.core.bham.baseConnector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.baseConnector.MapConnector;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.GlobalMap;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;
import manipulation.runner.bham.BhamRunner;

import org.apache.log4j.Logger;

import FrontierInterface.LocalGridMap;
import FrontierInterface.LocalMapInterface;
import FrontierInterface.LocalMapInterfacePrx;
import SpatialData.Place;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.core.CASTData;

/**
 * represents a connector to the local maps of the Birmingham / CogX environment
 * 
 * @author ttoenige
 * 
 */
public class BhamLocalMapConnector implements MapConnector {

	private GlobalMap map = new GlobalMap();

	private LocalMapInterfacePrx localMapInterface;
	// TODO das ist doof

	private Manipulator manipulator;

	private Logger logger = Logger.getLogger(this.getClass());

	private Thread t;

	/**
	 * constructor of the connection to the local map of the Birmingham / CogX
	 * environment
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public BhamLocalMapConnector(Manipulator manipulator) {
		this.manipulator = manipulator;

		try {
			localMapInterface = ((BhamRunner) manipulator.getRunner())
					.getIceServer("map.manager", LocalMapInterface.class,
							LocalMapInterfacePrx.class);
		} catch (CASTException e) {
			logger.error(e);
		}

		t = new Thread(new MapUpdaterRunnable(manipulator));

		updateMapConstant();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public double getDataFromWorldCoordinates(Vector2D data) {
		return getDataFromIndex(getIndexFromWorldCoordinates(data));
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public double getDataFromWorldCoordinates(Vector3D data) {
		return getDataFromIndex(getIndexFromWorldCoordinates(data));
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public double getDataFromIndex(Vector2D index) {
		// TODO 1 ist Wand, 2 ist weiß nicht, 0 ist frei

		return Double.parseDouble(Character.toString(map.getData()[(int) index
				.getY()
				+ (int) (index.getX()) * map.getySize()]));
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vector2D getIndexFromWorldCoordinates(Vector3D data) {
		return getIndexFromWorldCoordinates(new Vector2D(data.getX(), data
				.getY()));
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vector2D getIndexFromWorldCoordinates(BasePositionData data) {
		return getIndexFromWorldCoordinates(data.getPoint());
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vector2D getWorldFromIndex(Vector2D point) {

		return new Vector2D(map.getCenter().getX()
				+ (point.getX() * map.getCellSize()) - (map.getxSize() / 2)
				* map.getCellSize(), map.getCenter().getY()
				+ (point.getY() * map.getCellSize()) - (map.getySize() / 2)
				* map.getCellSize());

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vector2D getIndexFromWorldCoordinates(Vector2D data) {
		int i = ((int) (Math.floor((data.getX() - map.getCenter().getX())
				/ (map.getCellSize() / 2.0)) + (data.getX() >= map.getCenter()
				.getX() ? 1 : -1)) / 2)
				+ map.getySize() / 2;
		int j = ((int) (Math.floor((data.getY() - map.getCenter().getY())
				/ (map.getCellSize() / 2.0)) + (data.getY() >= map.getCenter()
				.getY() ? 1 : -1)) / 2)
				+ map.getxSize() / 2;

		return new Vector2D(i, j);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void updateMap() throws ExternalMemoryException {
		try {
			CASTData<Place> allPlaces[];
			allPlaces = ((BhamRunner) manipulator.getRunner())
					.getWorkingMemoryEntries(Place.class);

			long[] placesIDs = new long[allPlaces.length];
			for (int i = 0; i < allPlaces.length; i++) {
				placesIDs[i] = allPlaces[i].getData().id;
			}

			LocalGridMap localGridMap = localMapInterface
					.getCombinedGridMap(placesIDs);

			byte[] dataArray = localGridMap.data;

			map.setData(new String(dataArray).toCharArray());

			map.setCellSize(localGridMap.cellSize);
			map.setSize(localGridMap.size);
			map.setCenter(new Vector2D(localGridMap.xCenter,
					localGridMap.yCenter));

		} catch (SubarchitectureComponentException e) {
			throw new ExternalMemoryException("Cannot get a map");
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public GlobalMap getMap() {
		return map;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean isPointFree(Vector2D point) {
		// TODO eigene ENUMS fuer freien raum etc
		if (getDataFromWorldCoordinates(point) == 0) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean isPointUnknown(Vector2D point) {
		if (getDataFromWorldCoordinates(point) == 2) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean isPointObstacle(Vector2D point) {
		if (getDataFromWorldCoordinates(point) == 1) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void updateMapConstant() {
		if (!t.isAlive()) {
			t = new Thread(new MapUpdaterRunnable(manipulator));
			t.start();
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stopupdateMapConstant() {

		if (t.isAlive()) {
			t.interrupt();
		}

	}

}
