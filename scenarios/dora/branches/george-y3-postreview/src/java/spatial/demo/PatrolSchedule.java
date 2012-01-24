package spatial.demo;

import SpatialData.Place;

public interface PatrolSchedule extends Iterable<Place> {

	/**
	 * Adds a place. If schedule is active, this won't be considered until the
	 * next round.
	 * 
	 * @param _p
	 */
	public void addPlace(Place _p);

}
