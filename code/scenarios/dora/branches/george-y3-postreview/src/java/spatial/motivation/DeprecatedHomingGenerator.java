/**
 * 
 */
package spatial.motivation;

import motivation.components.generators.AbstractMotiveGenerator;
import motivation.factories.MotiveFactory;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import facades.SpatialFacade;


/**
 * @author marc
 * @deprecated
 */
public class DeprecatedHomingGenerator extends AbstractMotiveGenerator {
	/** add a really low gain to homing */
	private static final double HOMING_GAIN = 0.01;
	Place homeBase = null;

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.generators.Generator#updateMotive(cast.cdl.WorkingMemoryAddress
	 * , cast.cdl.WorkingMemoryAddress)
	 */
	@Override
	protected boolean checkMotive(Motive motive) {
		try {
			Place source = getMemoryEntry(motive.referenceEntry, Place.class);

			Place currentPlace = SpatialFacade.get(this).getPlace();
			if (currentPlace != null) {
				debug("we are currently at place " + currentPlace.id);
				if (currentPlace.id == ((HomingMotive) motive).homePlaceID) {
					motive.informationGain = 0.0;
					motive.tries = 0;
					motive.costs = (float) 0.0;
				} else {
					motive.informationGain = HOMING_GAIN;
					double costs = SpatialFacade.get(this).queryCosts(
							currentPlace.id, ((HomingMotive) motive).homePlaceID);
					if (costs < Double.MAX_VALUE)
						motive.costs = (float) costs;
					else {
						println("couldn't compute proper costs... leaving costs untouched");
					}

				}
			}

			if (source.status == PlaceStatus.TRUEPLACE) {
				log("  we have a first true place and take it as the home base");
				homeBase = source;
				write(motive);
				return true;
			}
		} catch (CASTException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return false;
	}

	// (= position(agent) position_id)

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
									HomingMotive.class).values())
								scheduleCheckMotive((Motive) m);
						}
					});
			SpatialFacade.get(this).registerPlaceChangedCallback(
					new SpatialFacade.PlaceChangedHandler() {

						@Override
						public synchronized void update(Place p) {
							for (Ice.ObjectImpl m : motives.getMapByType(
									HomingMotive.class).values())
								scheduleCheckMotive((Motive) m);
						}
					});
		} catch (CASTException e1) {
			println("exception when registering placeChangedCallbacks");
			e1.printStackTrace();
		}

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				debug(CASTUtils.toString(_wmc));
				// create a new motive from this node...
				if (homeBase == null) { // if home base is not set yet
					HomingMotive newMotive = MotiveFactory
							.createHomingMotive(_wmc.address);
					Place p;
					try {

						p = getMemoryEntry(_wmc.address, Place.class);
						newMotive.homePlaceID = p.id;
					} catch (CASTException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					scheduleCheckMotive(newMotive);
				} else { // if we have a home base already, remove the listener,
					// we are happy!
					try {
						removeChangeFilter(this);
					} catch (SubarchitectureComponentException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
		});
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#stop()
	 */
	@Override
	protected void stop() {
		// TODO Auto-generated method stub
		super.stop();
	}

}
