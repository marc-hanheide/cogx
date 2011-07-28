/**
 * 
 */
package eu.cogx.percepttracker.dora;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;
import java.util.Map.Entry;

import SpatialData.Place;
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
import castutils.castextensions.BeliefWaiter;
import castutils.castextensions.IceXMLSerializer;
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

	private static final String NODE_PERSON_EXISTS = "person_exists";
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

	private WorkingMemoryAddress findRoomForPlace(WorkingMemoryAddress placeAdr) {
		println("looking up placeAdr " + placeAdr.id
				+ " in spatialBeliefs of size " + spatialBeliefs.size());
		GroundedBelief placeBel = spatialBeliefs.get(placeAdr);
		assert (placeBel != null);
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belPrx = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, placeBel);
		PointerFormula placePtr = (PointerFormula) belPrx.getContent().get(
				RoomMembershipMediator.ROOM_PROPERTY).getDistribution()
				.getMostLikely().get();
		return placePtr.pointer;

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

	/**
	 * @param event
	 * @param pb
	 * @param placePtr
	 * @param probExist
	 * @throws AlreadyExistsOnWMException
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	private void processFirstObservation(WorkingMemoryChange event,
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb,
			PointerFormula placePtr, Boolean probExist)
			throws AlreadyExistsOnWMException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		log("this is an entirely new person with place " + placePtr.pointer.id
				+ "that we haven't seen before");
		List<Boolean> obs = new ArrayList<Boolean>();
		obs.add(probExist);
		placeObservations.put(placePtr.pointer.id, obs);
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> newGB = newBelief(
				pb, placePtr.pointer, event);
		updateProbability(newGB, obs);
		WorkingMemoryAddress wma = new WorkingMemoryAddress(newGB.getId(),
				getSubarchitectureID());
		room2GroundedBeliefMap.put(placePtr.pointer, wma);
		addToWorkingMemory(wma, newGB.get());
	}

	/**
	 * @param event
	 * @param from
	 * @param placePtr
	 * @param probExist
	 * @param wmaGrounded
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 */
	private void processObservation(WorkingMemoryChange event,
			PerceptBelief from, PointerFormula placePtr, Boolean probExist,
			WorkingMemoryAddress wmaGrounded) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException, ConsistencyException,
			PermissionException {
		log("we have seen a person of " + placePtr.pointer.id
				+ " before, so check if we have to update at all");

		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, getMemoryEntry(wmaGrounded,
						GroundedBelief.class));

		Collection<Boolean> obs = placeObservations.get(placePtr.pointer.id);
		obs.add(probExist);

		log("we have found a person in the same place, " + placePtr.pointer.id);
		manageHistory(event, from, gb.get());
		updateProbability(gb, obs);
		overwriteWorkingMemory(wmaGrounded, gb.get());
	}

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				PerceptBelief.class, WorkingMemoryOperation.ADD), this);
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				PerceptBelief.class, WorkingMemoryOperation.OVERWRITE), this);
		try {
			spatialBeliefs.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}
	}

	private void updateProbability(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb,
			Collection<Boolean> obs) {
		log("BNet created, running inference now ");
		// BeliefNode locationNode = bn.findNode(NODE_PERSON_EXISTS);
		// CPF res = ls.queryMarginal(locationNode);
		// log("queryMarginal for exist node: " + res.get(0).getExpr());

		FormulaDistribution df = FormulaDistribution.create();
		// df.add(true, ((ValueDouble) res.get(0)).getValue());

		gb.getContent().put(PersonTransferFunction.EXISTS, df);

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

		PointerFormula placePtr = (PointerFormula) (pb.getContent().get(
				PersonTransferFunction.IS_IN).getDistribution().getMostLikely()
				.get());
		Boolean probExist = pb.getContent().get(PersonTransferFunction.EXISTS)
				.getDistribution().get().values.get(0).prob > 0.5;
		addObservations(placePtr, probExist);

		WorkingMemoryAddress roomAdr = findRoomForPlace(placePtr.pointer);
		Map<WorkingMemoryAddress, GroundedBelief> placesInRoomAdr = getPlaceBeliefsForRoom(roomAdr);
		println("found N=" + placesInRoomAdr.size() + " in this room");

		generateBeliefNet(placesInRoomAdr);
		FormulaDistribution placeDistribution = computePlaceDistribution(placesInRoomAdr);

		// find the corresponding belief:
		WorkingMemoryAddress wmaGrounded = room2GroundedBeliefMap.get(roomAdr);
		if (wmaGrounded == null) {
			log("we have found a new person in room, " + roomAdr.id);
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> newGB = newBelief(
					pb, placePtr.pointer, event);
			WorkingMemoryAddress wma = new WorkingMemoryAddress(newGB.getId(),
					getSubarchitectureID());
			room2GroundedBeliefMap.put(placePtr.pointer, wma);
			newGB.getContent().put(PersonTransferFunction.IS_IN,
					placeDistribution);
			addToWorkingMemory(wma, newGB.get());

		} else {
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, getMemoryEntry(wmaGrounded,
							GroundedBelief.class));
			log("we have found a person in the same room, " + roomAdr.id);
			manageHistory(event, from, gb.get());
			gb.getContent()
					.put(PersonTransferFunction.IS_IN, placeDistribution);
			overwriteWorkingMemory(wmaGrounded, gb.get());
		}

	}

	private FormulaDistribution computePlaceDistribution(
			Map<WorkingMemoryAddress, GroundedBelief> placesInRoomAdr) {
		Map<String, Vector<Double>> marginals = reasoningEngine
				.queryMarginals();

		FormulaDistribution placeDistribution = FormulaDistribution.create();

		for (WorkingMemoryAddress wmaPlaceInRoom : placesInRoomAdr.keySet()) {
			log("looking up marginals for place " + wmaPlaceInRoom.id);
			Vector<Double> marginalForPlace = marginals.get(wmaPlaceInRoom.id);
			assert (marginalForPlace != null);
			placeDistribution.add(WMPointer.create(
					wmaPlaceInRoom,
					SimpleDiscreteTransferFunction
							.getBeliefTypeFromCastType(Place.class)).get(),
					marginalForPlace.get(0));
		}
		return placeDistribution;
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
			println("observations for " + wmaPlaceInRoom.id + ": "
					+ IceXMLSerializer.toXMLString(obsPerPlace));
			obsIdsForRoom.add(wmaPlaceInRoom.id);
		}
		reasoningEngine.submit(allObs);
	}

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

}
