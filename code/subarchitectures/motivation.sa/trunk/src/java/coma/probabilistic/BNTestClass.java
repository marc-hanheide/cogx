package coma.probabilistic;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.core.DiscreteEvidence;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
import edu.ksu.cis.bnj.ver3.inference.exact.LS;

public class BNTestClass {

	/**
	 * 
	 */
	public BNTestClass() {
		logger = Logger.getLogger(BNTestClass.class);
	}

	BeliefNetwork bn;
	List<String> roomLabels;
	List<String> objectLabels;
	Map<String, Map<String, Double>> existProbabilities;
	Logger logger;

	private String uniqueObjectId(String roomId, String objectId) {
		return roomId + "::" + objectId;
	}

	void initializeNetwork() {
		bn = new BeliefNetwork("Room classifier network");
		logger.info("created BeliefNetwork");

		Discrete Bool = new Discrete(new String[] { "True", "False" });
		for (Entry<String, Map<String, Double>> roomEntry : existProbabilities
				.entrySet()) {
			BeliefNode roomNode = new BeliefNode(roomEntry.getKey(), Bool);
			bn.addBeliefNode(roomNode);

			for (Entry<String, Double> objectEntry : roomEntry.getValue()
					.entrySet()) {
				BeliefNode objectNode = new BeliefNode(uniqueObjectId(roomEntry
						.getKey(), objectEntry.getKey()), Bool);
				bn.addBeliefNode(objectNode);
				bn.connect(roomNode, objectNode);
				objectNode.getCPF().put(0,
						new ValueDouble(objectEntry.getValue().doubleValue()));
				objectNode.getCPF().put(
						1,
						new ValueDouble(1.0 - objectEntry.getValue()
								.doubleValue()));
				objectNode.getCPF().put(
						2,
						new ValueDouble(1.0 - objectEntry.getValue()
								.doubleValue()));
				objectNode.getCPF().put(3,
						new ValueDouble(objectEntry.getValue().doubleValue()));
				objectNode.getCPF().normalizeByDomain();
			}

			roomNode.getCPF().put(0, new ValueDouble(0.5));
			roomNode.getCPF().put(1, new ValueDouble(0.5));
		}

	}

	Map<String, Double> computeRoomMarginals() {
		LS ls = new LS();
		logger.info("run exact inference");
		ls.run(bn);
		Map<String, Double> result = new HashMap<String, Double>();
		for (Entry<String, Map<String, Double>> roomEntry : existProbabilities
				.entrySet()) {
			BeliefNode roomNode = bn.findNode(roomEntry.getKey());
			assert (roomNode != null);
			CPF cpf = ls.queryMarginal(roomNode);
			logger.info("true value of marginal for hypothesis "
					+ roomNode.getName() + " is " + cpf.get(0).getExpr());
			result.put(roomNode.getName(), Double.parseDouble(cpf.get(0)
					.getExpr()));
		}
		return result;
	}

	void setEvidence(List<String> objectEvidences) {
		for (Entry<String, Map<String, Double>> roomEntry : existProbabilities
				.entrySet()) {
			for (String objectLabel : objectEvidences) {
				String uniqueLabel = uniqueObjectId(roomEntry.getKey(),
						objectLabel);
				BeliefNode objectNode = bn.findNode(uniqueLabel);
				if (objectNode != null) {
					logger
							.info("we have an evidence for object "
									+ uniqueLabel);
					objectNode.setEvidence(new DiscreteEvidence(0));
				}

			}

		}
	}

	public String toString() {
		String result = "";
		BeliefNode[] Nodes = bn.getNodes();
		for (int i = 0; i < Nodes.length; i++) {
			if (bn.getParents(Nodes[i]).length == 0) {
				result += Nodes[i].getName() + "("
						+ Nodes[i].getCPF().get(0).getExpr() + "):";
				BeliefNode[] Children = bn.getChildren(Nodes[i]);
				for (BeliefNode cNode : Children) {
					result += " " + cNode.getName() + "("
							+ cNode.getCPF().get(1).getExpr() + ")";
				}
				result += "\n";
			}
		}

		return result;

	}

	public static void main(String[] args) throws Exception {

		BNTestClass test = new BNTestClass();
		test.existProbabilities = new HashMap<String, Map<String, Double>>();
		test.existProbabilities.put("room a", new HashMap<String, Double>());
		test.existProbabilities.put("room b", new HashMap<String, Double>());
		test.existProbabilities.get("room a").put("object a", new Double(0.8));
		test.existProbabilities.get("room a").put("object b", new Double(0.1));
		test.existProbabilities.get("room b").put("object b", new Double(0.7));
		test.existProbabilities.get("room b").put("object c", new Double(0.9));
		// some complexity tests:
		for (int i = 0; i < 10; i++) {
			String fakeRoomName = "fakeRoom " + Integer.toString(i);
			test.existProbabilities.put(fakeRoomName,
					new HashMap<String, Double>());
			for (int j = 0; j < 10; j++) {

				test.existProbabilities.get(fakeRoomName).put(
						"fake" + Integer.toString(j), new Double(0.5));
			}
		}

		test.initializeNetwork();
		System.out.println(test);
		List<String> evidences = new LinkedList<String>();
		evidences.add("object c");
		evidences.add("object b");
		evidences.add("object a");
		test.setEvidence(evidences);
		test.computeRoomMarginals();
	}
}
