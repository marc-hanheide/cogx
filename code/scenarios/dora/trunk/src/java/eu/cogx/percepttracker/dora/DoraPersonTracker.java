/**
 * 
 */
package eu.cogx.percepttracker.dora;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.core.DiscreteEvidence;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.dora.PersonTransferFunction;

/**
 * @author cogx
 * 
 */
public class DoraPersonTracker extends ManagedComponent implements
		WorkingMemoryChangeReceiver {
	private static final String NODE_PERSON_EXISTS = "person_exists";

	private static final double OBS_MODEL_TRUE_POS_PROB = 0.6;
	private static final double OBS_MODEL_FALSE_POS_PROB = 0.1;

	Map<WorkingMemoryAddress, WorkingMemoryAddress> label2AddrMap = Collections
			.synchronizedMap(new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>());
	Map<WorkingMemoryAddress, List<Boolean>> placeObservations = Collections
			.synchronizedMap(new HashMap<WorkingMemoryAddress, List<Boolean>>());

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

		Boolean probExist = pb.getContent().get(PersonTransferFunction.EXISTS)
				.getDistribution().get().values.get(0).prob > 0.5;

		WorkingMemoryAddress wmaGrounded = label2AddrMap.get(placePtr.pointer);

		if (wmaGrounded == null) {
			log("this is an entirely new person with place "
					+ placePtr.pointer.id + "that we haven't seen before");
			List<Boolean> obs = new ArrayList<Boolean>();
			obs.add(probExist);
			placeObservations.put(placePtr.pointer, obs);
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> newGB = newBelief(
					pb, placePtr.pointer, event);
			updateProbability(newGB, obs);
			WorkingMemoryAddress wma = new WorkingMemoryAddress(newGB.getId(),
					getSubarchitectureID());
			label2AddrMap.put(placePtr.pointer, wma);
			addToWorkingMemory(wma, newGB.get());
		} else {
			log("we have seen a person of " + placePtr.pointer.id
					+ " before, so check if we have to update at all");

			CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, getMemoryEntry(wmaGrounded,
							GroundedBelief.class));
			
			List<Boolean> obs = placeObservations.get(placePtr.pointer);
			obs.add(probExist);
			
			// // get the place id referred to:
			// PointerFormula gbPlace = (PointerFormula) gb.getContent().get(
			// PersonTransferFunction.IS_IN).getDistribution()
			// .getMostLikely().get();
			// if (gbPlace.pointer.id.equals(placePtr.pointer.id)) {
			log("we have found a person in the same place, "
					+ placePtr.pointer.id);
			manageHistory(event, from, gb.get());
			updateProbability(gb, obs);
			overwriteWorkingMemory(wmaGrounded, gb.get());

			// }

		}

	}

	private void updateProbability(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb,
			List<Boolean> obs) {
		// double pbProb = pb.getContent().get(PersonTransferFunction.EXISTS)
		// .getDistribution().get().values.get(0).prob;
		// double gbProb = pb.getContent().get(PersonTransferFunction.EXISTS)
		// .getDistribution().get().values.get(0).prob;
		BeliefNetwork bn = createExistenceNodeWithObservations(obs);
		log("BNet created, running inference now ");
		edu.ksu.cis.bnj.ver3.inference.Inference ls = new edu.ksu.cis.bnj.ver3.inference.exact.LS();
		ls.run(bn);
		BeliefNode locationNode = bn.findNode(NODE_PERSON_EXISTS);
		CPF res = ls.queryMarginal(locationNode);
		log("queryMarginal for exist node: " + res.get(0).getExpr());

		FormulaDistribution df = FormulaDistribution.create();
		df.add(true, ((ValueDouble) res.get(0)).getValue());

		gb.getContent().put(PersonTransferFunction.EXISTS, df);

	}

	private BeliefNetwork createExistenceNodeWithObservations(
			List<Boolean> observations) {
		BeliefNetwork g = new BeliefNetwork("person");

		BeliefNode existNode = new BeliefNode(NODE_PERSON_EXISTS,
				new Discrete(new String[] { "t", "f" }));
		g.addBeliefNode(existNode);
		CPF locationNodeCFP = existNode.getCPF();
		// initialize with uniform distribution
		for (int i = 0; i < locationNodeCFP.size(); i++) {
			locationNodeCFP.put(i,
					new ValueDouble(1.0 / locationNodeCFP.size()));
		}

		// iterate all observations for this label
		int obsId = 0;
		for (boolean hasBeenSeen : observations) {

			obsId++;

			BeliefNode observationNode = new BeliefNode("obs", new Discrete(
					new String[] { "true", "false" }));
			g.addBeliefNode(observationNode);
			g.connect(existNode, observationNode);
			CPF cpfObs = observationNode.getCPF();
			cpfObs.put(new int[] { 0, 0 }, new ValueDouble(
					OBS_MODEL_TRUE_POS_PROB));
			cpfObs.put(new int[] { 0, 1 }, new ValueDouble(
					OBS_MODEL_FALSE_POS_PROB));
			cpfObs.put(new int[] { 1, 0 }, new ValueDouble(
					1 - OBS_MODEL_TRUE_POS_PROB));
			cpfObs.put(new int[] { 1, 1 }, new ValueDouble(
					1 - OBS_MODEL_FALSE_POS_PROB));
			if (hasBeenSeen) {
				observationNode.setEvidence(new DiscreteEvidence(0));
			} else {
				observationNode.setEvidence(new DiscreteEvidence(1));
			}
		}

		return g;
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
