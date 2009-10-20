/**
 * 
 */
package spatial.motivation;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.Callable;

import motivation.components.generators.AbstractMotiveGenerator;
import motivation.factories.MotiveFactory;
import motivation.slice.ExploreMotive;
import motivation.slice.Motive;
import motivation.util.castextensions.WMEntrySet;
import motivation.util.facades.SpatialFacade;
import FrontierInterface.PlaceInterface;
import FrontierInterface.PlaceInterfacePrx;
import Ice.ObjectImpl;
import SpatialData.Place;
import SpatialData.PlaceHolder;
import SpatialData.PlacePrx;
import SpatialData.PlaceStatus;
import SpatialProperties.AssociatedBorderPlaceholderProperty;
import SpatialProperties.GatewayPlaceProperty;
import SpatialProperties.AssociatedSpacePlaceholderProperty;
import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.FloatValue;
import SpatialProperties.PlaceProperty;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * @author marc
 * 
 */
public class ExplorePlaceGenerator extends AbstractMotiveGenerator {

	protected class CurrentPlaceChecker implements Runnable {
		SpatialFacade spatialFacade;
		Place currentPlace;
		Callable<Place> placeChangedCallable;

		/**
		 * @param spatialFacade
		 * @param placeChangedCallable
		 */
		public CurrentPlaceChecker(SpatialFacade spatialFacade,
				Callable<Place> placeChangedCallable) {
			super();
			this.spatialFacade = spatialFacade;
			this.placeChangedCallable = placeChangedCallable;
		}

		synchronized void setPlace(Place place) {
			currentPlace = place;
		}

		synchronized Place getPlace() {
			return currentPlace;
		}

		@Override
		public void run() {
			while (isRunning()) {
				try {
					Place placeRightNow = spatialFacade.getCurrentPlace();
					if (currentPlace == null
							|| placeRightNow.id != currentPlace.id) {
						setPlace(placeRightNow);
						placeChangedCallable.call();
					}
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					break;
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}

		}

	}

	WMEntrySet borderProperties;
	WMEntrySet gatewayProperties;
	WMEntrySet spaceProperties;

	private HashMap<Long, WorkingMemoryAddress> m_placeIDtoBorderProperty;
	private HashMap<Long, WorkingMemoryAddress> m_placeIDtoGatewayProperty;
	private HashMap<Long, WorkingMemoryAddress> m_placeIDtoSpaceProperty;
	private HashMap<Long, WorkingMemoryAddress> m_placeIDtoMotiveWMA;

	private double m_spaceMeasureConstant;
	private double m_borderMeasureConstant;
	private double m_gatewayMeasureConstant;
	private PlaceInterfacePrx placeInterface;
	private SpatialFacade spatialFacade;
	private Thread currentPlaceCheckerThread;
	private CurrentPlaceChecker currentPlaceChecker;

	public ExplorePlaceGenerator() {
		super();
		m_placeIDtoBorderProperty = new HashMap<Long, WorkingMemoryAddress>();
		m_placeIDtoGatewayProperty = new HashMap<Long, WorkingMemoryAddress>();
		m_placeIDtoSpaceProperty = new HashMap<Long, WorkingMemoryAddress>();
		m_placeIDtoMotiveWMA = new HashMap<Long, WorkingMemoryAddress>();
		borderProperties = WMEntrySet.create(this,
				AssociatedBorderPlaceholderProperty.class);
		spaceProperties = WMEntrySet.create(this,
				AssociatedSpacePlaceholderProperty.class);
		gatewayProperties = WMEntrySet.create(this, GatewayPlaceProperty.class);

	}

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

			// if it is a yet unexplored one...
			log("there is a place to be checked, id=" + source.id);

			if (source.status == PlaceStatus.PLACEHOLDER) {
				log("  it's a placeholder, so it should be considered as a motive");

				Place currentPlace = currentPlaceChecker.getPlace();
				if (currentPlace != null) {
					log("we are currently at place " + currentPlace.id);
					motive.costs = (float) spatialFacade.queryCosts(
							currentPlace.id, ((ExploreMotive) motive).placeID);
				} else {
					motive.costs = Float.MAX_VALUE;
				}
				addHypothesisFeatures((ExploreMotive) motive);

				log("writing ExploreMotive to WM for place_id "
						+ ((ExploreMotive) motive).placeID);
				WorkingMemoryAddress motiveAddress = write(motive);
				// need to store mapping back to place struct to establish
				// relations between places and their properties
				if (!m_placeIDtoMotiveWMA.containsKey(source.id)) {
					log("this motive is not yet linked to properties... doing it");
					Set<WorkingMemoryAddress> ppaddrs = new HashSet<WorkingMemoryAddress>();
					ppaddrs.add(m_placeIDtoBorderProperty.get(source.id));
					ppaddrs.add(m_placeIDtoGatewayProperty.get(source.id));
					ppaddrs.add(m_placeIDtoSpaceProperty.get(source.id));
					for (WorkingMemoryAddress wma : ppaddrs)
						if (wma != null)
							addReceivers(motiveAddress, wma);
					m_placeIDtoMotiveWMA.put(source.id, motiveAddress);
					// schedule to check again!
					// scheduleCheckMotive(motive);
				}

				return true;
			} else {
				log("  turn out this place is not a placeholder, so, it should be no motive then");
				log("  getting rid of ExploreMotive for place_id "
						+ ((ExploreMotive) motive).placeID);
				remove(motive);
			}
		} catch (CASTException e) {
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
			placeInterface = this.getIceServer("place.manager",
					PlaceInterface.class, PlaceInterfacePrx.class);

		} catch (CASTException e1) {
			println("error setting up connection to PlaceManager");
			e1.printStackTrace();
		}

		spatialFacade = new SpatialFacade(this, placeInterface);

		currentPlaceChecker = new CurrentPlaceChecker(
				spatialFacade, new Callable<Place>() {

					@Override
					synchronized public Place call() throws Exception {
						log("place has changed... we have to check all ExploreMotives");
						for (ObjectImpl m : motives.getMapByType(
								ExploreMotive.class).values()) {
							scheduleCheckMotive((Motive) m);
						}
						return null;
					}
				});
		
		currentPlaceCheckerThread = new Thread(currentPlaceChecker);



		WMEntrySet.ChangeHandler propertyChangeHandler = new WMEntrySet.ChangeHandler() {

			@Override
			public void entryChanged(Map<WorkingMemoryAddress, ObjectImpl> map,
					WorkingMemoryChange wmc, ObjectImpl newEntry,
					ObjectImpl oldEntry) throws CASTException {
				if (wmc.operation != WorkingMemoryOperation.DELETE) {
					PlaceProperty placeProperty = (PlaceProperty) newEntry;

					// first time through, add the place to the source list for
					// the
					// place's motive
					if (wmc.operation == WorkingMemoryOperation.ADD) {
						WorkingMemoryAddress linkedMotive = m_placeIDtoMotiveWMA
								.get(placeProperty.placeId);
						if (linkedMotive != null) {
							log("found a motive that should be linked to the property");
							addReceivers(linkedMotive, wmc.address);
							Motive motive = (Motive) motives.get(linkedMotive);
							if (motive != null)
								scheduleCheckMotive(motive);
						}
					}
					if (placeProperty instanceof AssociatedBorderPlaceholderProperty)
						m_placeIDtoBorderProperty.put(placeProperty.placeId,
								wmc.address);
					else if (placeProperty instanceof AssociatedSpacePlaceholderProperty)
						m_placeIDtoSpaceProperty.put(placeProperty.placeId,
								wmc.address);
					else if (placeProperty instanceof GatewayPlaceProperty)
						m_placeIDtoGatewayProperty.put(placeProperty.placeId,
								wmc.address);

				}

			}
		};
		borderProperties.setHandler(propertyChangeHandler);
		borderProperties.start();

		gatewayProperties.setHandler(propertyChangeHandler);
		gatewayProperties.start();

		spaceProperties.setHandler(propertyChangeHandler);
		spaceProperties.start();

		// addChangeFilter(
		// ChangeFilterFactory
		// .createGlobalTypeFilter(AssociatedSpacePlaceholderProperty.class),
		// new WorkingMemoryChangeReceiver() {
		// @Override
		// public void workingMemoryChanged(WorkingMemoryChange _arg0)
		// throws CASTException {
		// associatedSpaceChange(_arg0);
		// }
		// });
		//
		// addChangeFilter(ChangeFilterFactory
		// .createGlobalTypeFilter(GatewayPlaceProperty.class),
		// new WorkingMemoryChangeReceiver() {
		// @Override
		// public void workingMemoryChanged(WorkingMemoryChange _arg0)
		// throws CASTException {
		// associatedGateway(_arg0);
		// }
		// });
		//
		// addChangeFilter(
		// ChangeFilterFactory
		// .createGlobalTypeFilter(AssociatedBorderPlaceholderProperty.class),
		// new WorkingMemoryChangeReceiver() {
		// @Override
		// public void workingMemoryChanged(WorkingMemoryChange _arg0)
		// throws CASTException {
		// associatedBorderChange(_arg0);
		// }
		// });

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				debug(CASTUtils.toString(_wmc));
				// create a new motive from this node...
				ExploreMotive newMotive = MotiveFactory
						.createExploreMotive(_wmc.address);

				Place p;
				try {
					p = getMemoryEntry(_wmc.address, Place.class);
					newMotive.placeID = p.id;
					// start the place checker thread here
					if (!currentPlaceCheckerThread.isAlive())
						currentPlaceCheckerThread.start();

				} catch (CASTException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				scheduleCheckMotive(newMotive);
			}
		});
		spatialFacade.start();
	}

	// protected void associatedBorderChange(WorkingMemoryChange _wmc)
	// throws DoesNotExistOnWMException, UnknownSubarchitectureException {
	//
	// if (_wmc.operation != WorkingMemoryOperation.DELETE) {
	// AssociatedBorderPlaceholderProperty borderProperty = getMemoryEntry(
	// _wmc.address, AssociatedBorderPlaceholderProperty.class);
	//
	// // first time through, add the place to the source list for the
	// // place's motive
	// if (_wmc.operation == WorkingMemoryOperation.ADD) {
	// assert m_placeIDtoMotiveWMA.containsKey(borderProperty.placeId) :
	// "place should be written before property";
	// addReceivers(m_placeIDtoMotiveWMA.get(borderProperty.placeId),
	// _wmc.address);
	// }
	//
	// m_placeIDtoBorderProperty
	// .put(
	// borderProperty.placeId,
	// getFirstPropertyValue((DiscreteProbabilityDistribution)
	// borderProperty.distribution));
	// }
	// }
	//
	// protected void associatedGateway(WorkingMemoryChange _wmc)
	// throws DoesNotExistOnWMException, UnknownSubarchitectureException {
	//
	// if (_wmc.operation != WorkingMemoryOperation.DELETE) {
	// GatewayPlaceProperty gatewayProperty = getMemoryEntry(_wmc.address,
	// GatewayPlaceProperty.class);
	// // first time through, add the place to the source list for the
	// // place's motive
	// if (_wmc.operation == WorkingMemoryOperation.ADD) {
	// if (m_placeIDtoMotiveWMA.containsKey(gatewayProperty.placeId))
	// addReceivers(m_placeIDtoMotiveWMA
	// .get(gatewayProperty.placeId), _wmc.address);
	// }
	// // TODO: This is a hacky way of doing it. We just assume that the
	// // existence of this property is taken as the 100% evidence of
	// // existence
	// m_placeIDtoGatewayProperty.put(gatewayProperty.placeId, 1.0);
	// // m_placeIDtoGatewayProperty
	// // .put(
	// // gatewayProperty.placeId,
	// // getFirstPropertyValue((DiscreteProbabilityDistribution)
	// // gatewayProperty.distribution));
	// println("associate gateway to " + gatewayProperty.placeId + ": "
	// + m_placeIDtoGatewayProperty.get(gatewayProperty.placeId));
	// }
	// }
	//
	// protected void associatedSpaceChange(WorkingMemoryChange _wmc)
	// throws DoesNotExistOnWMException, UnknownSubarchitectureException {
	// if (_wmc.operation != WorkingMemoryOperation.DELETE) {
	// AssociatedSpacePlaceholderProperty spaceProperty = getMemoryEntry(
	// _wmc.address, AssociatedSpacePlaceholderProperty.class);
	//
	// // first time through, add the place to the source list for the
	// // place's motive
	// if (_wmc.operation == WorkingMemoryOperation.ADD) {
	// assert m_placeIDtoMotiveWMA.containsKey(spaceProperty.placeId) :
	// "place should be written before property";
	// addReceivers(m_placeIDtoMotiveWMA.get(spaceProperty.placeId),
	// _wmc.address);
	// }
	//
	// m_placeIDtoSpaceProperty
	// .put(
	// spaceProperty.placeId,
	// getFirstPropertyValue((DiscreteProbabilityDistribution)
	// spaceProperty.distribution));
	// }
	// }

	private double getFirstPropertyValue(
			DiscreteProbabilityDistribution _probabilityDistribution) {
		if (_probabilityDistribution == null)
			return 0.0;
		else
			return ((FloatValue) _probabilityDistribution.data[0].value).value;
	}

	private double getFirstPropertyValue(PlaceProperty _property) {
		if (_property == null)
			return 0.0;
		else
			return getFirstPropertyValue((DiscreteProbabilityDistribution) _property.distribution);
	}

	/**
	 * 
	 * Assign an information gain value to the motive by using the two frontier
	 * measures.
	 * 
	 * @param _em
	 */
	private void addHypothesisFeatures(ExploreMotive _em) {
		long placeID = _em.placeID;

		// A measure of how many new nav nodes could be placed in the free space
		// beyond the frontoer

		PlaceProperty pp;
		WorkingMemoryAddress ppaddr;
		double spaceMeasure = 0.0;
		double borderMeasure = 0.0;
		double gatewayMeasure = 0.0;

		ppaddr = m_placeIDtoSpaceProperty.get(placeID);
		if (ppaddr != null) {
			pp = (PlaceProperty) spaceProperties.get(ppaddr);
			spaceMeasure = getFirstPropertyValue(pp);
		}
		ppaddr = m_placeIDtoBorderProperty.get(placeID);
		if (ppaddr != null) {
			pp = (PlaceProperty) borderProperties.get(ppaddr);
			borderMeasure = getFirstPropertyValue(pp);
		}
		ppaddr = m_placeIDtoGatewayProperty.get(placeID);
		if (ppaddr != null) {
			pp = (PlaceProperty) borderProperties.get(ppaddr);
			gatewayMeasure = getFirstPropertyValue(pp);
		}
		//		
		// Double gatewayMeasureBoxed = m_placeIDtoGatewayProperty.get(placeID);
		// double gatewayMeasure = gatewayMeasureBoxed == null ? 0
		// : gatewayMeasureBoxed;
		//
		// // A measure of how much of the frontier borders on unknown space
		// Double borderMeasureBoxed = m_placeIDtoBorderProperty.get(placeID);
		// double borderMeasure = borderMeasureBoxed == null ? 0
		// : borderMeasureBoxed;
		_em.informationGain = (spaceMeasure * m_spaceMeasureConstant)
				+ (borderMeasure * m_borderMeasureConstant)
				+ (gatewayMeasure * m_gatewayMeasureConstant);

		log(CASTUtils.concatenate(_em.informationGain, " = space(",
				spaceMeasure, " * ", m_spaceMeasureConstant, ") + border(",
				borderMeasure, " * ", m_borderMeasureConstant, ") + gateway(",
				gatewayMeasure, " * ", m_gatewayMeasureConstant, ")"));
	}

	@Override
	protected void configure(Map<String, String> _config) {
		m_spaceMeasureConstant = 1;
		m_borderMeasureConstant = 1;

	}

}
