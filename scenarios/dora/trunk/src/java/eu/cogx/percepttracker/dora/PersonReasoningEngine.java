/**
 * 
 */
package eu.cogx.percepttracker.dora;

import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import org.apache.log4j.Logger;

import castutils.castextensions.IceXMLSerializer;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.core.DiscreteEvidence;
import edu.ksu.cis.bnj.ver3.core.Value;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
import edu.ksu.cis.bnj.ver3.core.values.ValueZero;

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
public class PersonReasoningEngine {
	public static final String LOCALISED = "localised";

	public static final String NODE_PERSON_EXISTS_IN_PLACE_PREFIX = "exists_";

	public static final String NODE_PERSON_EXISTS_IN_ROOM = "person_exists";

	private static final double OBS_MODEL_FALSE_POS_PROB = 0.001;
	private static final double OBS_MODEL_TRUE_POS_PROB = 0.7;

	private static final double EXISTS_AT_PLACE_TRUE = 0.999;

	private static final double PRIOR_PERSON_EXISTS_IN_ROOM = 0.9;

	public static void main(String[] argv) {
		Map<String, Collection<Boolean>> allObs = new HashMap<String, Collection<Boolean>>();
		Collection<Boolean> placeObs;
		placeObs = Arrays.asList();
		allObs.put("p1", placeObs);
		placeObs = Arrays.asList();
//		allObs.put("p2", placeObs);
//		placeObs = Arrays.asList();
//		allObs.put("p3", placeObs);
//		placeObs = Arrays.asList();
//		allObs.put("p4", placeObs);
//		placeObs = Arrays.asList();
//		allObs.put("p5", placeObs);
//		placeObs = Arrays.asList();
//		allObs.put("p6", placeObs);
//		placeObs = Arrays.asList();
//		allObs.put("p7", placeObs);

		PersonReasoningEngine pre = new PersonReasoningEngine();
		pre.submit(allObs);
		for (BeliefNode n : pre.getNetwork().getNodes()) {
			CPF res = pre.getInferenceEngine().queryMarginal(n);

			System.out.print("queryMarginal for " + n.getName() + ": ");
			for (int i=0; i < res.size(); i++) {
				System.out.print(res.get(i).getExpr()+" ");
			}
			System.out.println();	

		}

	}

	Logger logger = Logger.getLogger(PersonReasoningEngine.class);

	BeliefNetwork bn;

	edu.ksu.cis.bnj.ver3.inference.Inference ls = new edu.ksu.cis.bnj.ver3.inference.exact.LS();

	/**
	 * @param observations
	 * @param g
	 * @param existNode
	 */
	private void addExistNodes(BeliefNetwork g, BeliefNode existInRoomNode,
			Map<String, Collection<Boolean>> observations) {

		String[] values = observations.keySet().toArray(new String[0]);
		BeliefNode isInPlace = new BeliefNode(LOCALISED, new Discrete(values));
		for (int i = 0; i < isInPlace.getCPF().size(); i++) {
			isInPlace.getCPF().put(i,
					new ValueDouble(1.0 / isInPlace.getCPF().size()));
		}
		g.addBeliefNode(isInPlace);

		for (String val : values) {

			BeliefNode existsNode = new BeliefNode(
					NODE_PERSON_EXISTS_IN_PLACE_PREFIX + val, new Discrete(
							new String[] { "true", "false" }));
			g.addBeliefNode(existsNode);
			g.connect(isInPlace, existsNode);
			g.connect(existInRoomNode, existsNode);

			CPF cpfObs = existsNode.getCPF();

			for (int i = 0; i < values.length; i++) {
				String val2 = values[i];
				if (val.equals(val2)) {
					cpfObs.put(new int[] { 0, 0, i }, new ValueDouble(
							EXISTS_AT_PLACE_TRUE));
					cpfObs.put(new int[] { 0, 1, i }, new ValueDouble(
							1 - EXISTS_AT_PLACE_TRUE));
					cpfObs.put(new int[] { 1, 0, i }, new ValueDouble(
							1 - EXISTS_AT_PLACE_TRUE));
					cpfObs.put(new int[] { 1, 1, i }, new ValueDouble(
							EXISTS_AT_PLACE_TRUE));
				} else {
					cpfObs.put(new int[] { 0, 0, i }, new ValueDouble(
							1 - EXISTS_AT_PLACE_TRUE));
					cpfObs.put(new int[] { 0, 1, i }, new ValueDouble(
							1 - EXISTS_AT_PLACE_TRUE));
					cpfObs.put(new int[] { 1, 0, i }, new ValueDouble(
							EXISTS_AT_PLACE_TRUE));
					cpfObs.put(new int[] { 1, 1, i }, new ValueDouble(
							EXISTS_AT_PLACE_TRUE));
				}
			}

			addObservationNodes(observations.get(val), g, existsNode);
		}
	}

	/**
	 * @param observations
	 * @param g
	 * @param existNode
	 */
	private void addObservationNodes(Collection<Boolean> observations,
			BeliefNetwork g, BeliefNode existNode) {
		// iterate all observations for this label
		int obsId = 0;
		for (boolean hasBeenSeen : observations) {

			obsId++;

			BeliefNode observationNode = new BeliefNode("obs_"
					+ existNode.getName() + "_" + obsId, new Discrete(
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
	}

	private void generateBayesNet(Map<String, Collection<Boolean>> observations) {
		bn = new BeliefNetwork("person");

		BeliefNode existInRoomNode = new BeliefNode(NODE_PERSON_EXISTS_IN_ROOM,
				new Discrete(new String[] { "t", "f" }));
		bn.addBeliefNode(existInRoomNode);
		CPF existInRoomCFP = existInRoomNode.getCPF();
		existInRoomCFP.put(0, new ValueDouble(PRIOR_PERSON_EXISTS_IN_ROOM));
		existInRoomCFP.put(1, new ValueDouble(1 - PRIOR_PERSON_EXISTS_IN_ROOM));

		addExistNodes(bn, existInRoomNode, observations);
	}

	public edu.ksu.cis.bnj.ver3.inference.Inference getInferenceEngine() {
		return ls;
	}

	public BeliefNetwork getNetwork() {

		return bn;
	}

	public Map<String, Vector<Double>> queryMarginals() {
		Map<String, Vector<Double>> marginals = new HashMap<String, Vector<Double>>(
				bn.getNodes().length);
		for (BeliefNode node : bn.getNodes()) {
			CPF cpf = ls.queryMarginal(node);
			Vector<Double> marginalsNode = new Vector<Double>();
			for (int i = 0; i < cpf.size(); i++) {
				Value val = cpf.get(i);
				if (val instanceof ValueDouble)
					marginalsNode.add(((ValueDouble) cpf.get(i)).getValue());
				else if (val instanceof ValueZero)
					marginalsNode.add(0.0);
				else
					logger.warn("unsupported value type "
							+ val.getClass().getName());
			}
			logger.debug("queryMarginal for " + node.getName() + ": "
					+ IceXMLSerializer.toXMLString(marginalsNode));
			marginals.put(node.getName(), marginalsNode);
		}
		return marginals;
	}

	public BeliefNetwork submit(Map<String, Collection<Boolean>> obs) {
		generateBayesNet(obs);
		logger.debug("BNet created");
		// logger.info(IceXMLSerializer.toXMLString(bn.getGraph().getVertices()));
		ls.run(bn);
		return bn;

	}
}
