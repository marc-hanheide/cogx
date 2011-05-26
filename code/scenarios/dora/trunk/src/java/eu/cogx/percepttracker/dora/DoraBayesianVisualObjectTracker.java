/**
 * 
 */
package eu.cogx.percepttracker.dora;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
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
import castutils.castextensions.IceXMLSerializer;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.Formulas;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
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
import eu.cogx.perceptmediator.dora.VisualObjectTransferFunction;

/**
 * @author cogx
 * 
 */
public class DoraBayesianVisualObjectTracker extends ManagedComponent implements
		WorkingMemoryChangeReceiver {

	private static final double LOCATION_MODEL_TRUE_POS = 0.99;
	private static final double LOCATION_MODEL_FALSE_POS = 0.01;

	public static final double OBS_MODEL_TRUE_POS_PROB = 0.8;
	public static final double OBS_MODEL_FALSE_POS_PROB = 0.1;

	Map<String, WorkingMemoryAddress> label2AddrMap = Collections
			.synchronizedMap(new HashMap<String, WorkingMemoryAddress>());

	Map<String, Map<String, List<Double>>> observations = new HashMap<String, Map<String, List<Double>>>();

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

	private void storeNewObservation(String PlaceId, String perceptLabel,
			double obsProb) {

		Map<String, List<Double>> currentMap = observations.get(perceptLabel);
		if (currentMap == null) {
			currentMap = new HashMap<String, List<Double>>();
			observations.put(perceptLabel, currentMap);
		}
		List<Double> observationSet = currentMap.get(PlaceId);
		if (observationSet == null) {
			observationSet = new ArrayList<Double>();
			currentMap.put(PlaceId, observationSet);
		}
		observationSet.add(new Double(obsProb));
	}

	private BeliefNetwork createExistenceNodeWithObservations(String label,
			List<String> placeIds) {
		BeliefNetwork g = new BeliefNetwork(label);

		int existNodeCounter = 0;

		BeliefNode locationNode = new BeliefNode("location_" + label,
				new Discrete(placeIds.toArray(new String[0])));
		g.addBeliefNode(locationNode);
		CPF locationNodeCFP = locationNode.getCPF();
		// initialize with uniform distribution
		for (int i = 0; i < locationNodeCFP.size(); i++) {
			locationNodeCFP.put(i,
					new ValueDouble(1.0 / locationNodeCFP.size()));
		}

		// iterate all observations for this label
		for (String placeId : placeIds) {

			List<Double> obs = observations.get(label).get(placeId);

			// create existence node for the place (with its id as name)
			BeliefNode existenceNode = new BeliefNode(placeId, new Discrete(
					new String[] { "true", "false" }));
			g.addBeliefNode(existenceNode);
			g.connect(locationNode, existenceNode);
			CPF exCFP = existenceNode.getCPF();
			for (int i = 0; i < exCFP.size() / 2; i++) {
				exCFP.put(new int[] { 0, i }, new ValueDouble(
						LOCATION_MODEL_FALSE_POS));
				exCFP.put(new int[] { 1, i }, new ValueDouble(
						1 - LOCATION_MODEL_FALSE_POS));
			}
			exCFP.put(new int[] { 0, existNodeCounter }, new ValueDouble(
					LOCATION_MODEL_TRUE_POS));
			exCFP.put(new int[] { 1, existNodeCounter }, new ValueDouble(
					1 - LOCATION_MODEL_TRUE_POS));
			existNodeCounter++;

			if (obs != null) {
				for (Double obsProbability : obs) {
					BeliefNode observationNode = new BeliefNode("obs",
							new Discrete(new String[] { "true", "false" }));
					g.addBeliefNode(observationNode);
					g.connect(existenceNode, observationNode);
					CPF cpfObs = observationNode.getCPF();
					cpfObs.put(new int[] { 0, 0 }, new ValueDouble(
							OBS_MODEL_TRUE_POS_PROB));
					cpfObs.put(new int[] { 0, 1 }, new ValueDouble(
							OBS_MODEL_FALSE_POS_PROB));
					cpfObs.put(new int[] { 1, 0 }, new ValueDouble(
							1 - OBS_MODEL_TRUE_POS_PROB));
					cpfObs.put(new int[] { 1, 1 }, new ValueDouble(
							1 - OBS_MODEL_FALSE_POS_PROB));
					if (obsProbability.doubleValue() > 0.5) {
						observationNode.setEvidence(new DiscreteEvidence(0));
					} else {
						observationNode.setEvidence(new DiscreteEvidence(1));
					}
				}
			}
		}
		return g;
	}

	public static void main(String[] argv) {

		DoraBayesianVisualObjectTracker obj = new DoraBayesianVisualObjectTracker();

		obj.storeNewObservation("place1", "obj1", 1.0);
		obj.storeNewObservation("place1", "obj1", 1.0);
		obj.storeNewObservation("place1", "obj1", 1.0);
		obj.storeNewObservation("place2", "obj1", 1.0);
		obj.storeNewObservation("place2", "obj1", 0.0);
		obj.storeNewObservation("place2", "obj1", 0.0);
		obj.storeNewObservation("place2", "obj1", 0.0);

		// BeliefNetwork bn = obj
		// .createExistenceNodeWithObservations("obj1", Arrays.asList(
		// "place1", "place2", "place3", "place4", "place5"));
		// // System.out.println(IceXMLSerializer.toXMLString(bn.getGraph()));
		//
		// edu.ksu.cis.bnj.ver3.inference.Inference ls = new
		// edu.ksu.cis.bnj.ver3.inference.exact.LS();
		// ls.run(bn);
		// BeliefNode locationNode = bn.findNode("location_obj1");
		// CPF res = ls.queryMarginal(locationNode);
		//
		// for (int i = 0; i < res.size(); i++) {
		// if (res.get(i) instanceof ValueDouble)
		// System.out.println("res(" + locationNode.getDomain().getName(i)
		// + ")=" + ((ValueDouble) res.get(i)).getValue());
		// else
		// System.out.println("res(" + i + ") is "
		// + res.get(i).getClass().getName());
		// }

		FormulaDistribution df = obj
				.inferProbabilitiesForIsIn("obj1", "hurga", Arrays.asList(
						"place1", "place2", "place3", "place4", "place5"));

		System.out.println("computed FormulaDistribution for object obj1: "
				+ IceXMLSerializer.toXMLString(df.get().values));

		// BeliefNetwork g = new BeliefNetwork("Test");
		// BeliefNode locationNode = new BeliefNode("location", new Discrete(
		// new String[] { "place1", "place2", "placeUNKNOWN" }));
		// g.addBeliefNode(locationNode);
		// CPF cpflpl = locationNode.getCPF();
		// cpflpl.put(0, new ValueDouble(0.33));
		// cpflpl.put(1, new ValueDouble(0.33));
		// cpflpl.put(2, new ValueDouble(0.33));
		//
		// BeliefNode existenceNodePlace1 = new BeliefNode("existence1",
		// new Discrete(new String[] { "true", "false" }));
		// g.addBeliefNode(existenceNodePlace1);
		//
		// g.connect(locationNode, existenceNodePlace1);
		// CPF cpflp1 = existenceNodePlace1.getCPF();
		// // for (int i=0;i<cpflp1.size();i++) {
		// //
		// System.out.println("addr "+i+" => "+cpflp1.realaddr2addr(i)[0]+", "+cpflp1.realaddr2addr(i)[1]);
		// // }
		//
		// cpflp1.put(0, new ValueDouble(LOCATION_MODEL_TRUE_POS));
		// cpflp1.put(1, new ValueDouble(0.01));
		// cpflp1.put(2, new ValueDouble(0.01));
		// cpflp1.put(3, new ValueDouble(0.01));
		// cpflp1.put(4, new ValueDouble(LOCATION_MODEL_TRUE_POS));
		// cpflp1.put(5, new ValueDouble(LOCATION_MODEL_TRUE_POS));
		//
		// BeliefNode existenceNodePlace2 = new BeliefNode("existence2",
		// new Discrete(new String[] { "true", "false" }));
		// g.addBeliefNode(existenceNodePlace2);
		// g.connect(locationNode, existenceNodePlace2);
		// CPF cpflp2 = existenceNodePlace2.getCPF();
		// // for (int i=0;i<cpflp1.size();i++) {
		// //
		// System.out.println("addr "+i+" => "+cpflp1.realaddr2addr(i)[0]+", "+cpflp1.realaddr2addr(i)[1]);
		// // }
		//
		// cpflp2.put(0, new ValueDouble(0.01));
		// cpflp2.put(1, new ValueDouble(LOCATION_MODEL_TRUE_POS));
		// cpflp2.put(2, new ValueDouble(0.01));
		// cpflp2.put(3, new ValueDouble(LOCATION_MODEL_TRUE_POS));
		// cpflp2.put(4, new ValueDouble(0.01));
		// cpflp2.put(5, new ValueDouble(LOCATION_MODEL_TRUE_POS));
		//
		// Set<BeliefNode> observations = new HashSet<BeliefNode>();
		//
		// observations.add(new BeliefNode("o1", new Discrete(new String[] {
		// "true", "false" })));
		// observations.add(new BeliefNode("o2", new Discrete(new String[] {
		// "true", "false" })));
		// observations.add(new BeliefNode("o3", new Discrete(new String[] {
		// "true", "false" })));
		// // observations.add(new BeliefNode("o4", new Discrete(new String[] {
		// // "true", "false" })));
		//
		// for (BeliefNode bn : observations) {
		// g.addBeliefNode(bn);
		// g.connect(existenceNodePlace1, bn);
		// CPF cpf0 = bn.getCPF();
		// cpf0.put(0, new ValueDouble(OBS_MODEL_TRUE_POS_PROB));
		// cpf0.put(1, new ValueDouble(0.2));
		// cpf0.put(2, new ValueDouble(0.2));
		// cpf0.put(3, new ValueDouble(OBS_MODEL_TRUE_POS_PROB));
		// int val = (int) Math.ceil(Math.random() - 0.5);
		// System.out.println("val=" + val);
		// bn.setEvidence(new DiscreteEvidence(val));
		//
		// }
		// // System.out.println(IceXMLSerializer.toXMLString(g));
		// edu.ksu.cis.bnj.ver3.inference.exact.LS ls = new
		// edu.ksu.cis.bnj.ver3.inference.exact.LS();
		// ls.run(g);
		// CPF res = ls.queryMarginal(locationNode);
		//
		// for (int i = 0; i < res.size(); i++) {
		// if (res.get(i) instanceof ValueDouble)
		// System.out.println("res(" + locationNode.getDomain().getName(i)
		// + ")=" + ((ValueDouble) res.get(i)).getValue());
		// else
		// System.out.println("res(" + i + ") is "
		// + res.get(i).getClass().getName());
		// }
	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange event)
			throws CASTException {
		PerceptBelief from = getMemoryEntry(event.address, PerceptBelief.class);
		processNewPercept(event, from);

		// Set<WorkingMemoryAddress> wmaGroundedSet = label2AddrMap
		// .get(perceptLabel);
		// if (wmaGroundedSet == null) {
		// label2AddrMap
		// .put(perceptLabel, new HashSet<WorkingMemoryAddress>());
		// log("this is an entirely new object with label " + perceptLabel
		// + "that we haven't seen before");
		// newBelief(pb, perceptLabel, event);
		// return;
		// } else {
		// log("we have seen an object of " + perceptLabel
		// + " before, so check if we have to update at all");
		// boolean newNeeded = true;
		// for (WorkingMemoryAddress wmaGrounded : wmaGroundedSet) {
		// CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb =
		// CASTIndependentFormulaDistributionsBelief
		// .create(GroundedBelief.class,
		// getMemoryEntry(wmaGrounded,
		// GroundedBelief.class));
		// float existingProb = sumProb(gb.getContent().get(
		// VisualObjectTransferFunction.IS_IN));
		// log("the current existingProb of the GroundedBelief is "
		// + existingProb + ", the perceptIsInProb is "
		// + perceptIsInProb);
		// if (perceptIsInProb < 0.5) {
		// log("perceptIsInProb < 0.5: so we overwrite with new prob="
		// + perceptIsInProb);
		// setIsInProb(
		// gb.getContent().get(
		// VisualObjectTransferFunction.IS_IN),
		// perceptPointer, (float) perceptIsInProb);
		// manageHistory(event, from, gb.get());
		// overwriteWorkingMemory(wmaGrounded, gb.get());
		// newNeeded = false;
		// } else if (existingProb < 0.5) {
		// log("existingProb < 0.5: so we overwrite with new prob="
		// + perceptIsInProb);
		// setIsInProb(
		// gb.getContent().get(
		// VisualObjectTransferFunction.IS_IN),
		// perceptPointer, (float) perceptIsInProb);
		// manageHistory(event, from, gb.get());
		// overwriteWorkingMemory(wmaGrounded, gb.get());
		// newNeeded = false;
		// break;
		// }
		//
		// }
		// // we found no belief that we can assign to, so we need a new one
		// if (newNeeded) {
		// newBelief(pb, perceptLabel, event);
		// }
		// }

	}

	/**
	 * @param event
	 * @param from
	 * @throws AlreadyExistsOnWMException
	 */
	private void processNewPercept(WorkingMemoryChange event, PerceptBelief from)
			throws AlreadyExistsOnWMException {
		CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb = CASTIndependentFormulaDistributionsBelief
				.create(PerceptBelief.class, from);

		String perceptLabel = getPerceptLabel(pb);
		WorkingMemoryAddress placePointer = getFirstPlacePointer(pb);
		double perceptIsInProb = getFirstPlaceProbability(pb);

		storeNewObservation(placePointer.id, perceptLabel, perceptIsInProb);

		FormulaDistribution df = inferProbabilitiesForIsIn(perceptLabel,
				placePointer.subarchitecture,
				Arrays.asList("place1", "place2", "place3", "place4", "place5"));

		if (getLogger().isDebugEnabled())
			getLogger().debug(
					"computed FormulaDistribution for object " + perceptLabel
							+ ": " + IceXMLSerializer.toXMLString(df));

		CASTIndependentFormulaDistributionsBelief<GroundedBelief> newGB = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class);
		newGB.setType(pb.getType());
		newGB.setFrame(pb.getFrame());
		newGB.setPrivate(pb.getPrivate());
		newGB.setId(newDataID());
		newGB.getContent().put(VisualObjectTransferFunction.IS_IN, df);

		WorkingMemoryAddress wmaGrounded = label2AddrMap.get(perceptLabel);
		if (wmaGrounded == null) {
			manageHistory(event, pb.get(), newGB.get());
			addToWorkingMemory(newGB.getId(), newGB.get());
			label2AddrMap.put(perceptLabel,
					new WorkingMemoryAddress(newGB.getId(),
							getSubarchitectureID()));
		}
	}

	/**
	 * @param pb
	 * @return
	 */
	private double getFirstPlaceProbability(
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb) {
		double perceptIsInProb = pb.getContent()
				.get(VisualObjectTransferFunction.IS_IN).getDistribution()
				.get().values.get(0).prob;
		return perceptIsInProb;
	}

	/**
	 * @param pb
	 * @return
	 */
	private WorkingMemoryAddress getFirstPlacePointer(
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb) {
		WorkingMemoryAddress perceptPointer = ((PointerFormula) pb.getContent()
				.get(VisualObjectTransferFunction.IS_IN).getDistribution()
				.get().values.get(0).val).pointer;
		return perceptPointer;
	}

	/**
	 * @param pb
	 * @return
	 */
	private String getPerceptLabel(
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb) {
		String perceptLabel = pb.getContent()
				.get(VisualObjectTransferFunction.LABEL_ID).getDistribution()
				.getMostLikely().getProposition();
		return perceptLabel;
	}

	/**
	 * @param perceptLabel
	 * @param subArchitectureId
	 * @return
	 */
	private FormulaDistribution inferProbabilitiesForIsIn(String perceptLabel,
			String subArchitectureId, List<String> placeIds) {
		// compute new distribution by doing the BNet inference
		BeliefNetwork bn = createExistenceNodeWithObservations(perceptLabel,
				placeIds);
		// System.out.println(IceXMLSerializer.toXMLString(bn.getGraph()));

		edu.ksu.cis.bnj.ver3.inference.Inference ls = new edu.ksu.cis.bnj.ver3.inference.exact.LS();
		ls.run(bn);
		BeliefNode locationNode = bn.findNode("location_" + perceptLabel);
		CPF res = ls.queryMarginal(locationNode);

		FormulaValues fdRaw = new FormulaValues(
				new ArrayList<FormulaProbPair>());
		for (int i = 0; i < res.size(); i++) {
			fdRaw.values.add(new FormulaProbPair(new PointerFormula(-1,
					new WorkingMemoryAddress(locationNode.getDomain()
							.getName(i), subArchitectureId), CASTUtils
							.typeName(VisualObject.class)),
					(float) ((ValueDouble) res.get(i)).getValue()));

		}
		FormulaDistribution df = FormulaDistribution.create();
		df.setDistribution(Formulas.create(fdRaw));
		return df;
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
