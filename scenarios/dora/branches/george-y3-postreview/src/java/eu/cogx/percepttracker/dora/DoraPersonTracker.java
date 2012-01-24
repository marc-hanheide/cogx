/**
 * 
 */
package eu.cogx.percepttracker.dora;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;
import java.util.Map.Entry;

import SpatialData.Place;
import SpatialProperties.IntegerValue;
import SpatialProperties.PlaceContainmentAgentProperty;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import castutils.castextensions.BeliefWaiter;
import castutils.castextensions.WMEventQueue;
import castutils.castextensions.WMView;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import eu.cogx.beliefs.WMBeliefView;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.components.RoomMembershipMediator;
import eu.cogx.perceptmediator.dora.PersonTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;

/**
 * This person tracker localises persons at places and integrates several
 * observations (PerceptBelief) in a Bayesian way, taking the predefined
 * observation models into account. Currently observations are determinstic,
 * while the integration into the GroundedBelief is done in a predicate "exists"
 * which "true"-value is a probability derived from the Bayesian inference about
 * all observations.
 * 
 * @author hanheidm
 * 
 */
public class DoraPersonTracker extends ManagedComponent implements
		WorkingMemoryChangeReceiver {

	public static final String ASSOCIATED_WITH = "associated-with";
	WMEventQueue evQueue = new WMEventQueue();
	public static final String UPDATE_ON_MOVE = "--update-on-move";
	// create a view of all places and room beliefs
	WMView<GroundedBelief> spatialBeliefs = WMBeliefView.create(this,
			GroundedBelief.class, new String[] {
					SimpleDiscreteTransferFunction
							.getBeliefTypeFromCastType(Place.class),
					SimpleDiscreteTransferFunction
							.getBeliefTypeFromCastType(ComaRoom.class) });
	BeliefWaiter<GroundedBelief> waitingBeliefReader = new BeliefWaiter<GroundedBelief>(
			spatialBeliefs);

	Map<WorkingMemoryAddress, WorkingMemoryAddress> room2GroundedBeliefMap = Collections
			.synchronizedMap(new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>());
	Map<String, Collection<Boolean>> placeObservations = Collections
			.synchronizedMap(new HashMap<String, Collection<Boolean>>());

	PersonReasoningEngine reasoningEngine = new PersonReasoningEngine();
	private boolean updateOnPlaceChange = false;

	private void addObservations(PointerFormula placePtr, Boolean probExist) {
		Collection<Boolean> obs = placeObservations.get(placePtr.pointer.id);
		if (obs == null) {
			obs = new ArrayList<Boolean>();
			obs.add(probExist);
			placeObservations.put(placePtr.pointer.id, obs);
		} else {
			obs.add(probExist);
		}
	}

	private FormulaDistribution computeExistDistribution(
			Map<String, Vector<Double>> marginals) {

		FormulaDistribution placeDistribution = FormulaDistribution.create();
		Vector<Double> marginalExist = marginals
				.get(PersonReasoningEngine.NODE_PERSON_EXISTS_IN_ROOM);
		assert (marginalExist != null);
		placeDistribution.add(true, marginalExist.get(0));

		return placeDistribution;
	}

	private FormulaDistribution computePlaceDistribution(
			Map<WorkingMemoryAddress, GroundedBelief> placesInRoomAdr,
			Map<String, Vector<Double>> marginals) {

		FormulaDistribution placeDistribution = FormulaDistribution.create();
		int i=0;
		for (WorkingMemoryAddress wmaPlaceInRoom : placesInRoomAdr.keySet()) {
//			log("looking up marginals for place " + wmaPlaceInRoom.id);
//			Vector<Double> marginalForPlace = marginals
//					.get(PersonReasoningEngine.NODE_PERSON_EXISTS_IN_PLACE_PREFIX
//							+ wmaPlaceInRoom.id);
//			assert (marginalForPlace != null);
			log("looking up marginals for place " + wmaPlaceInRoom.id);
			Vector<Double> marginalForPlace = marginals
					.get(PersonReasoningEngine.LOCALISED);
			Vector<Double> marginalForExist = marginals
			.get(PersonReasoningEngine.NODE_PERSON_EXISTS_IN_ROOM);
			assert (marginalForPlace != null);

			placeDistribution.add(WMPointer.create(
					wmaPlaceInRoom,
					SimpleDiscreteTransferFunction
							.getBeliefTypeFromCastType(Place.class)).get(),
					marginalForPlace.get(i++)*marginalForExist.get(0));
		}
		return placeDistribution;
	}

	private WorkingMemoryAddress findRoomForPlace(WorkingMemoryAddress placeAdr) {
		println("looking up placeAdr " + placeAdr.id
				+ " in spatialBeliefs of size " + spatialBeliefs.size());
		FormulaDistribution rp = null;
		while (isRunning()) {
			GroundedBelief placeBel;
			try {
				placeBel = getMemoryEntry(placeAdr, GroundedBelief.class);

				CASTIndependentFormulaDistributionsBelief<GroundedBelief> belPrx = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class, placeBel);
				rp = belPrx.getContent().get(
						RoomMembershipMediator.ROOM_PROPERTY);
				if (rp == null) {
					try {
						getLogger().warn(
								"have to wait for place being properly set");
						Thread.sleep(100);
					} catch (InterruptedException e) {
						logException(e);
					}
				} else {
					PointerFormula placePtr = (PointerFormula) belPrx
							.getContent().get(
									RoomMembershipMediator.ROOM_PROPERTY)
							.getDistribution().getMostLikely().get();
					return placePtr.pointer;
				}
			} catch (CASTException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}

		}
		return null;

	}

	private void generateBeliefNet(
			Map<WorkingMemoryAddress, GroundedBelief> placesInRoomAdr) {
		Collection<String> obsIdsForRoom = new Vector<String>();
		Map<String, Collection<Boolean>> allObs = new HashMap<String, Collection<Boolean>>();
		for (WorkingMemoryAddress wmaPlaceInRoom : placesInRoomAdr.keySet()) {
			Collection<Boolean> obsPerPlace = placeObservations
					.get(wmaPlaceInRoom.id);
			if (obsPerPlace == null)
				obsPerPlace = new Vector<Boolean>();
			allObs.put(wmaPlaceInRoom.id, obsPerPlace);
			obsIdsForRoom.add(wmaPlaceInRoom.id);
		}
		reasoningEngine.submit(allObs);
	}

	private Map<WorkingMemoryAddress, GroundedBelief> getPlaceBeliefsForRoom(
			WorkingMemoryAddress comaRoomAdr) {
		Map<WorkingMemoryAddress, GroundedBelief> places = new HashMap<WorkingMemoryAddress, GroundedBelief>();
		for (Entry<WorkingMemoryAddress, GroundedBelief> bel : spatialBeliefs
				.entrySet()) {
			// skip everything that is not a place!
			if (!bel.getValue().type.equals(SimpleDiscreteTransferFunction
					.getBeliefTypeFromCastType(Place.class)))
				continue;
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> belPrx = CASTIndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, bel.getValue());
			FormulaDistribution roomProp = belPrx.getContent().get(
					RoomMembershipMediator.ROOM_PROPERTY);
			if (roomProp == null)
				continue;
			PointerFormula roomPtr = (PointerFormula) roomProp
					.getDistribution().getMostLikely().get();
			if (roomPtr.pointer.equals(comaRoomAdr)) { // if it points to the
				// room we are looking
				// for}
				places.put(bel.getKey(), bel.getValue());
			}
		}
		return places;
	}

	private void handleFirstObservation(WorkingMemoryChange event,
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb,
			PointerFormula placePtr, WorkingMemoryAddress roomAdr,
			FormulaDistribution placeDistribution,
			FormulaDistribution existsDistribution)
			throws AlreadyExistsOnWMException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> newGB = newBelief(
				pb, placePtr.pointer, event);
		WorkingMemoryAddress wma = new WorkingMemoryAddress(newGB.getId(),
				getSubarchitectureID());
		room2GroundedBeliefMap.put(roomAdr, wma);
		newGB.getContent().put(PersonTransferFunction.IS_IN, placeDistribution);
		newGB.getContent().put(PersonTransferFunction.EXISTS,
				existsDistribution);
		addRoomReference(roomAdr, newGB);
		addToWorkingMemory(wma, newGB.get());
	}

	private void handleSuccessiveObservation(WorkingMemoryChange event,
			PerceptBelief from, WorkingMemoryAddress roomAdr,
			FormulaDistribution placeDistribution,
			WorkingMemoryAddress wmaGrounded,
			FormulaDistribution existsDistribution)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException,
			ConsistencyException, PermissionException {
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = getGroundedBelief(wmaGrounded);
		log("we have found a person in the same room, " + roomAdr.id);
		manageHistory(event, from, gb.get());
		addRoomReference(roomAdr, gb);
		updateGroundedBelief(placeDistribution, wmaGrounded,
				existsDistribution, gb);
	}

	/**
	 * @param placeDistribution
	 * @param wmaGrounded
	 * @param existsDistribution
	 * @param gb
	 * @param v
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 * @throws UnknownSubarchitectureException
	 */
	private void updateGroundedBelief(FormulaDistribution placeDistribution,
			WorkingMemoryAddress wmaGrounded,
			FormulaDistribution existsDistribution,
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
		gb.getContent().put(PersonTransferFunction.IS_IN, placeDistribution);
		gb.getContent().put(PersonTransferFunction.EXISTS, existsDistribution);
		overwriteWorkingMemory(wmaGrounded, gb.get());
	}

	/**
	 * @param wmaGrounded
	 * @return
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	private CASTIndependentFormulaDistributionsBelief<GroundedBelief> getGroundedBelief(
			WorkingMemoryAddress wmaGrounded) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, getMemoryEntry(wmaGrounded,
						GroundedBelief.class));
		return gb;
	}

	private void manageHistory(WorkingMemoryChange ev, PerceptBelief from,
			GroundedBelief to) {
		if (!(to.hist instanceof CASTBeliefHistory)) {
			to.hist = new CASTBeliefHistory(
					new ArrayList<WorkingMemoryPointer>(1),
					new ArrayList<WorkingMemoryPointer>(1));
		}
		CASTBeliefHistory bh = (CASTBeliefHistory) to.hist;
		WorkingMemoryPointer ancesterPointer = new WorkingMemoryPointer(
				ev.address, ev.type);
		if (!bh.ancestors.contains(ancesterPointer)) {
			bh.ancestors.add(ancesterPointer);
		}
	}

	private CASTIndependentFormulaDistributionsBelief<GroundedBelief> newBelief(
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb,
			WorkingMemoryAddress placeWMA, WorkingMemoryChange event)
			throws AlreadyExistsOnWMException {
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> newGB = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class);
		newGB.setType(pb.getType());
		newGB.setFrame(pb.getFrame());
		newGB.setPrivate(pb.getPrivate());
		newGB.getContent().putAll(pb.getContent());
		newGB.setId(newDataID());
		manageHistory(event, pb.get(), newGB.get());
		return newGB;
	}

	private void processObservation(WorkingMemoryChange event,
			PerceptBelief from,
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb)
			throws AlreadyExistsOnWMException, DoesNotExistOnWMException,
			UnknownSubarchitectureException, ConsistencyException,
			PermissionException {
		try {
			PointerFormula placePtr = (PointerFormula) (pb.getContent().get(
					PersonTransferFunction.IS_IN).getDistribution()
					.getMostLikely().get());
			Boolean probExist = pb.getContent().get(
					PersonTransferFunction.EXISTS).getDistribution().get().values
					.get(0).prob > 0.5;
			addObservations(placePtr, probExist);

			WorkingMemoryAddress roomAdr = findRoomForPlace(placePtr.pointer);
			Map<WorkingMemoryAddress, GroundedBelief> placesInRoomAdr = getPlaceBeliefsForRoom(roomAdr);

			Map<String, Vector<Double>> marginals = getMarginals(placesInRoomAdr);
			FormulaDistribution placeDistribution = computePlaceDistribution(
					placesInRoomAdr, marginals);
			FormulaDistribution existDistribution = computeExistDistribution(marginals);

			// find the corresponding belief:
			WorkingMemoryAddress wmaGrounded = room2GroundedBeliefMap
					.get(roomAdr);
			if (wmaGrounded == null) {
				log("we have found a new person in room, " + roomAdr.id);
				handleFirstObservation(event, pb, placePtr, roomAdr,
						placeDistribution, existDistribution);
			} else {
				handleSuccessiveObservation(event, from, roomAdr,
						placeDistribution, wmaGrounded, existDistribution);
			}
		} catch (NullPointerException e) {
			logException(e);
		}
	}

	/**
	 * @param placesInRoomAdr
	 * @return
	 */
	private Map<String, Vector<Double>> getMarginals(
			Map<WorkingMemoryAddress, GroundedBelief> placesInRoomAdr) {
		generateBeliefNet(placesInRoomAdr);
		Map<String, Vector<Double>> marginals = reasoningEngine
				.queryMarginals();
		return marginals;
	}

	@Override
	protected void runComponent() {
		while (isRunning()) {
			WorkingMemoryChange wmc;
			try {
				wmc = evQueue.take();
				sleepComponent(200);
				updatedRobotPosition(wmc.address);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				PerceptBelief.class, WorkingMemoryOperation.ADD), this);
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				PerceptBelief.class, WorkingMemoryOperation.OVERWRITE), this);
		if (updateOnPlaceChange) {
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					PlaceContainmentAgentProperty.class,
					WorkingMemoryOperation.OVERWRITE), evQueue);
		}
		try {
			spatialBeliefs.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		if (config.containsKey(UPDATE_ON_MOVE))
			updateOnPlaceChange = true;
		else
			updateOnPlaceChange = false;
	}

	protected void updatedRobotPosition(WorkingMemoryAddress wma) {
		PlaceContainmentAgentProperty pcap;
		try {
			pcap = getMemoryEntry(wma, PlaceContainmentAgentProperty.class);
			long placeID = ((IntegerValue) pcap.mapValue).value;
			log("robot moved to place " + placeID);
			Entry<WorkingMemoryAddress, GroundedBelief> placeBel = waitingBeliefReader
					.read(new PlaceMatchingFunction(placeID));
			log("got place belief address: "
					+ CASTUtils.toString(placeBel.getKey()));
			WorkingMemoryAddress roomAdr = findRoomForPlace(placeBel.getKey());
			log("got room belief address: " + CASTUtils.toString(roomAdr));

			// find the corresponding belief:
			WorkingMemoryAddress wmaGrounded = room2GroundedBeliefMap
					.get(roomAdr);
			if (wmaGrounded == null) {
				log("couldn't find GroundedBelief for person in this room... nothing to do");
				return;
			}

			Map<WorkingMemoryAddress, GroundedBelief> placesInRoomAdr = getPlaceBeliefsForRoom(roomAdr);
			try {
				lockComponent();
				log("compute marginals");
				Map<String, Vector<Double>> marginals = getMarginals(placesInRoomAdr);
				FormulaDistribution placeDistribution = computePlaceDistribution(
						placesInRoomAdr, marginals);
				FormulaDistribution existsDistribution = computeExistDistribution(marginals);

				CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = getGroundedBelief(wmaGrounded);
				log("we have to update the person in room " + roomAdr.id);

				addRoomReference(roomAdr, gb);

				updateGroundedBelief(placeDistribution, wmaGrounded,
						existsDistribution, gb);
			} finally {
				unlockComponent();
			}
		} catch (CASTException e) {
			logException(e);
		} catch (InterruptedException e) {
			logException(e);
		} catch (NullPointerException e) {
			logException(e);
		}
	}

	/**
	 * @param roomAdr
	 * @param gb
	 */
	private void addRoomReference(WorkingMemoryAddress roomAdr,
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb) {
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(WMPointer.create(roomAdr,
				CASTUtils.typeName(GroundedBelief.class)).get(), 1.0);
		gb.getContent().put(ASSOCIATED_WITH, fd);
	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange event)
			throws CASTException {

		PerceptBelief from = getMemoryEntry(event.address, PerceptBelief.class);
		CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb = CASTIndependentFormulaDistributionsBelief
				.create(PerceptBelief.class, from);

		if (pb.getContent().get(PersonTransferFunction.PERSON_ID) == null) {
			log("belief is empty, returning.");
			return;
		}

		processObservation(event, from, pb);

	}

}
