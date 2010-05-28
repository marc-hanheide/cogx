package eu.cogx.beliefs;

import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;

public class SimpleBeliefConstruction {
	public static void main(String[] args) {
		FeatureBeliefProxy bel = new FeatureBeliefProxy("test belief");
		IndependentFeatureDistributionProxy content = bel.getContent();
		content.put(new BasicProbDistributionProxy("test", new LinkedList<FormulaProbPair>()));
		content.remove("test");
		BasicProbDistributionProxy c = content.get("test");
		
		
//		bel.put("place_id", new Formula[] { new Formula("5", 0.6), new Formula("6", 0.4)});
//		Formula[] alternative_values = bel.get("place_id");
		
		
		
		

	}
}
