/**
 * 
 */
package spatial.motivation;

import java.util.HashMap;
import java.util.Map;

import motivation.components.generators.AbstractMotiveGenerator;
import motivation.factories.MotiveFactory;
import motivation.slice.Motive;
import motivation.slice.PatrolMotive;
import castutils.CASTTimeUtil;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import facades.SpatialFacade;

/**
 * @author marc
 * @deprecated
 */
public class DeprecatedPatrolPlaceGenerator extends AbstractMotiveGenerator {

	/**
	 * amount of milliseconds that corresponds to information gain of 0.5. If
	 * the place hasn't been explored for longer than this amount the gain is >
	 * 0.5 otherwise it's smaller using an exp reciproc model.
	 */
	private static final long EXP_NORM = 30000;
	private HashMap<Long, WorkingMemoryAddress> m_placeIDtoMotiveWMA;

	public DeprecatedPatrolPlaceGenerator() {
		super();
		m_placeIDtoMotiveWMA = new HashMap<Long, WorkingMemoryAddress>();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.generators.Generator#updateMotive(cast.cdl.WorkingMemoryAddress
	 * , cast.cdl.WorkingMemoryAddress)
	 */
	@Override
	protected boolean checkMotive(Motive motive) throws CASTException {
		try {
			Place source = getMemoryEntry(motive.referenceEntry, Place.class);

			// if it is a yet unexplored one...
			log("there is a place to be checked, id=" + source.id);

			if (source.status == PlaceStatus.TRUEPLACE) {
				log("  it's a TRUEPLACE, so it should be considered as a motive");

				Place currentPlace = SpatialFacade.get(this).getPlace();
				if (currentPlace != null) {
					log("we are currently at place " + currentPlace.id);
					// if we currently are at the place of this motive we update
					// the last visited time stamp
					CASTTime now = this.getCASTTime();
					if (currentPlace.id == ((PatrolMotive) motive).placeID) {
						((PatrolMotive) motive).lastVisisted = now;
						// reset the number of tries to zero, as we succeeded to
						// patrol here
						motive.tries = 0;
					}
					long timediff = CASTTimeUtil.diff(now,
							((PatrolMotive) motive).lastVisisted);
					motive.informationGain = 1 - (1. / Math
							.exp((timediff / EXP_NORM) * Math.log(2)));
					double costs = SpatialFacade.get(this).queryCosts(
							currentPlace.id, ((PatrolMotive) motive).placeID);
					if (costs < Double.MAX_VALUE)
						motive.costs = (float) SpatialFacade.get(this)
								.queryCosts(currentPlace.id,
										((PatrolMotive) motive).placeID);
					else {
						println("couldn't compute proper costs... leaving costs untouched");
					}
				}
				log("writing PatrolMotive to WM for place_id "
						+ ((PatrolMotive) motive).placeID);
				WorkingMemoryAddress motiveAddress = write(motive);
				// need to store mapping back to place struct to establish
				// relations between places and their properties
				if (!m_placeIDtoMotiveWMA.containsKey(source.id)) {
					m_placeIDtoMotiveWMA.put(source.id, motiveAddress);
				}

				return true;
			} else {
				log("  turn out this place is a placeholder, so, it should be no motive then");
				log("  getting rid of PatrolMotive for place_id "
						+ ((PatrolMotive) motive).placeID);
				remove(motive);
			}
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return false;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		try {
			SpatialFacade.get(this).registerPlaceChangedCallback(
					new SpatialFacade.PlaceChangedHandler() {

						@Override
						public synchronized void update(Place p) {
							for (Ice.ObjectImpl m : motives.getMapByType(
									PatrolMotive.class).values())
								scheduleCheckMotive((Motive) m);

						}
					});
		} catch (CASTException e1) {
			println("exception when registering placeChangedCallbacks");
			e1.printStackTrace();
		}
		WorkingMemoryChangeReceiver rec = new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				debug(CASTUtils.toString(_wmc));
				// create a new motive from this node...
				PatrolMotive newMotive = MotiveFactory
						.createPatrolMotive(_wmc.address);

				Place p;
				try {
					p = getMemoryEntry(_wmc.address, Place.class);
					newMotive.placeID = p.id;
					// start the place checker thread here

				} catch (CASTException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				scheduleCheckMotive(newMotive);
			}
		};
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), rec);
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.OVERWRITE), rec);

	}

	@Override
	protected void configure(Map<String, String> _config) {
	}

}
