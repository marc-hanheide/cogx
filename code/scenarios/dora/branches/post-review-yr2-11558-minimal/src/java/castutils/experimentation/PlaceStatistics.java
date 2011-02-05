/**
 * 
 */
package castutils.experimentation;

import java.util.Map;

import NavData.NavGraph;
import SpatialData.Place;
import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.logging.ComponentLogger;
import castutils.castextensions.WMView;
import castutils.castextensions.WMView.ChangeHandler;

/**
 * @author marc
 *
 */
public class PlaceStatistics extends ManagedComponent implements ChangeHandler<Place> {

	private ComponentLogger logger;
	WMView<NavGraph> navGraph;
	WMView<Place> places;

	/**
	 * @param motives
	 */
	public PlaceStatistics() {
		super();
		this.places = WMView.create(this, Place.class);
		this.navGraph = WMView.create(this, NavGraph.class);

	}
	
	/* (non-Javadoc)
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		
		super.start();
		logger = getLogger();
		try {
			places.registerHandler(this);
			places.start();
			navGraph.start();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public void entryChanged(Map<WorkingMemoryAddress, Place> map,
			WorkingMemoryChange wmc, Place newEntry, Place oldEntry)
			throws CASTException {
		int noTruePlaces = 0;
		int noPlaceHolders = 0;
		for (Place place : map.values()) {
			switch (place.status) {
			case PLACEHOLDER:
				noPlaceHolders++;
				break;
			case TRUEPLACE:
				noTruePlaces++;
				break;
			}
		}
		logger.info("TRUEPLACES: " + noTruePlaces);
		logger.info("PLACEHOLDERS: " + noPlaceHolders);
		for (NavGraph ng : navGraph.values()) {
			logger.info("NAVGRAPHEDGES: " + ng.aEdges.length);
		}

	}

}
