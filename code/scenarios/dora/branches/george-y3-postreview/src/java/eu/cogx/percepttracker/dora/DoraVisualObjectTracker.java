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

import VisionData.VisualObject;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.dora.VisualObjectTransferFunction;

/**
 * @author cogx
 * 
 */
public class DoraVisualObjectTracker extends ManagedComponent implements
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

		if (pb.getContent().get(VisualObjectTransferFunction.LABEL_ID) == null) {
			log("belief is empty, returning.");
			return;
		}

		String perceptLabel = pb.getContent().get(
				VisualObjectTransferFunction.LABEL_ID).getDistribution()
				.getMostLikely().getProposition();

		FormulaDistribution coneFormulaDistribution = pb.getContent().get(
				VisualObjectTransferFunction.CONE);
		if (coneFormulaDistribution==null) {
			getLogger().error("could not find fomulas for " + VisualObjectTransferFunction.CONE+"! From here on, it will not work as expected!");
			return;
		}
		WorkingMemoryAddress conePointer = ((PointerFormula) coneFormulaDistribution
				.getDistribution().get().values.get(0).val).pointer;

		Set<WorkingMemoryAddress> wmaGroundedSet = label2AddrMap
				.get(perceptLabel);

		if (wmaGroundedSet == null) {
			label2AddrMap
					.put(perceptLabel, new HashSet<WorkingMemoryAddress>());
			log("this is an entirely new object with label " + perceptLabel
					+ "that we haven't seen before");
			newBelief(pb, perceptLabel, event);
			return;
		} else {
			log("we have seen an object of " + perceptLabel
					+ " before, so check if we have to update at all");
			boolean newNeeded = true;
			for (WorkingMemoryAddress wmaGrounded : wmaGroundedSet) {
				CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class, getMemoryEntry(
								wmaGrounded, GroundedBelief.class));

				WorkingMemoryAddress gbConePointer = ((PointerFormula) gb
						.getContent().get(VisualObjectTransferFunction.CONE)
						.getDistribution().get().values.get(0).val).pointer;

				if (conePointer.equals(gbConePointer)) {
					log("we have found an object in the same viewcone, "
							+ conePointer.id);
					newNeeded = false;
					manageHistory(event, from, gb.get());
					overwriteWorkingMemory(wmaGrounded, gb.get());
					break;
				}

				// float existingProb = sumProb(gb.getContent().get(
				// VisualObjectTransferFunction.IS_IN));
				// log("the current existingProb of the GroundedBelief is "
				// + existingProb + ", the perceptIsInProb is "
				// + perceptIsInProb);
				// if (perceptIsInProb < 0.5) {
				// log("perceptIsInProb < 0.5: so we overwrite with new prob="+perceptIsInProb);
				// setIsInProb(gb.getContent().get(
				// VisualObjectTransferFunction.IS_IN),
				// perceptPointer, (float) perceptIsInProb);
				// manageHistory(event, from, gb.get());
				// overwriteWorkingMemory(wmaGrounded, gb.get());
				// newNeeded = false;
				// } else if (existingProb < 0.5) {
				// log("existingProb < 0.5: so we overwrite with new prob="+perceptIsInProb);
				// setIsInProb(gb.getContent().get(
				// VisualObjectTransferFunction.IS_IN),
				// perceptPointer, (float) perceptIsInProb);
				// manageHistory(event, from, gb.get());
				// overwriteWorkingMemory(wmaGrounded, gb.get());
				// newNeeded = false;
				// break;
				// }

			}
			// we found no belief that we can assign to, so we need a new one
			if (newNeeded) {
				newBelief(pb, perceptLabel, event);
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

	public boolean setIsInProb(FormulaDistribution fd,
			WorkingMemoryAddress pointer, float prob) {
		FormulaValues fdRaw = fd.getDistribution().get();
		float sum = 0.0f;
		boolean found = false;
		for (FormulaProbPair v : fdRaw.values) {
			PointerFormula pf = (PointerFormula) v.val;
			if (pf.pointer.equals(pointer)) {
				if (prob > v.prob)
					v.prob = prob;
				found = true;
			}
			sum += v.prob;
		}
		if (!found) {
			fdRaw.values.add(new FormulaProbPair(new PointerFormula(-1,
					pointer, CASTUtils.typeName(VisualObject.class)), prob));
			sum += prob;
		}
		// // normalize
		// if (sum > 0.0) {
		// for (FormulaProbPair v : fdRaw.values) {
		// v.prob = v.prob / sum;
		// }
		// }
		if (sum > 1.0)
			return true;
		else
			return false;
	}

	public float sumProb(FormulaDistribution fd) {
		FormulaValues fdRaw = fd.getDistribution().get();
		float sum = 0.0f;
		for (FormulaProbPair v : fdRaw.values) {
			sum += v.prob;
		}
		return sum;
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
