/**
 * 
 */
package eu.cogx.percepttracker.dora;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.dora.PersonTransferFunction;

/**
 * @author cogx
 * 
 */
public class DoraPersonTracker extends ManagedComponent implements
		WorkingMemoryChangeReceiver {

	Map<String, Set<WorkingMemoryAddress>> label2AddrMap = Collections
			.synchronizedMap(new HashMap<String, Set<WorkingMemoryAddress>>());

	@Override
	protected void runComponent() {
		// TODO Auto-generated method stub
		super.runComponent();
	}

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				PerceptBelief.class, WorkingMemoryOperation.ADD), this);
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				PerceptBelief.class, WorkingMemoryOperation.OVERWRITE), this);
		super.start();
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

		Set<WorkingMemoryAddress> wmaGroundedSet = label2AddrMap
				.get(placePtr.pointer.id);

		if (wmaGroundedSet == null) {
			label2AddrMap.put(placePtr.pointer.id,
					new HashSet<WorkingMemoryAddress>());
			log("this is an entirely new person with place "
					+ placePtr.pointer.id + "that we haven't seen before");
			newBelief(pb, placePtr.pointer.id, event);
			return;
		} else {
			log("we have seen a person of " + placePtr.pointer.id
					+ " before, so check if we have to update at all");
			boolean newNeeded = true;
			for (WorkingMemoryAddress wmaGrounded : wmaGroundedSet) {
				CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class, getMemoryEntry(
								wmaGrounded, GroundedBelief.class));
				// get the place id referred to:
				PointerFormula gbPlace = (PointerFormula) gb.getContent().get(
						PersonTransferFunction.IS_IN).getDistribution()
						.getMostLikely().get();
				if (gbPlace.pointer.id.equals(placePtr.pointer.id)) {
					log("we have found a person in the same place, "
							+ gbPlace.pointer.id);
					newNeeded = false;
					manageHistory(event, from, gb.get());
					overwriteWorkingMemory(wmaGrounded, gb.get());
					break;
				}
			}
			// we found no belief that we can assign to, so we need a new one
			if (newNeeded) {
				newBelief(pb, placePtr.pointer.id, event);
			}
		}

	}

	private void newBelief(
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb,
			String perceptLabel, WorkingMemoryChange event)
			throws AlreadyExistsOnWMException {
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> newGB = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class);
		newGB.setType(pb.getType());
		newGB.setFrame(pb.getFrame());
		newGB.setPrivate(pb.getPrivate());
		newGB.getContent().putAll(pb.getContent());
		newGB.setId(newDataID());
		manageHistory(event, pb.get(), newGB.get());
		addToWorkingMemory(newGB.getId(), newGB.get());
		label2AddrMap.get(perceptLabel)
				.add(
						new WorkingMemoryAddress(newGB.getId(),
								getSubarchitectureID()));
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

}
