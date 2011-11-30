package spatial.demo;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import SpatialData.Place;

/**
 * 
 * Visits places in the order they were added.
 * 
 * @author nah
 * 
 */
public class AdditionOrderPatrolSchedule implements PatrolSchedule {

	private final List<Place> m_allPlaces;
	private List<Place> m_toVisit;

	public AdditionOrderPatrolSchedule() {
		m_allPlaces = new ArrayList<Place>();

	}

	public void addPlace(Place _p) {
		m_allPlaces.add(_p);
	}

	@Override
	public Iterator<Place> iterator() {
		m_toVisit = new ArrayList<Place>(m_allPlaces);
		return m_toVisit.iterator();
	}

}
