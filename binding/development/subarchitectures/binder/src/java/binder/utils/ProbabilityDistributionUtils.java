package binder.utils;

import java.util.Enumeration;
import java.util.Vector;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.combined.OperationType;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.featvalues.StringValue;


public class ProbabilityDistributionUtils {



	public static ProbabilityDistribution generateProbabilityDistribution (PerceivedEntity entity) {
		DiscreteProbabilityDistribution distrib = new DiscreteProbabilityDistribution();

		Vector<Feature> features = new Vector<Feature>();
		for (int i = 0; i < entity.features.length ; i++) {
			features.add(entity.features[i]);
		}
/**		Feature exists = new Feature();
		exists.featlabel = "exists";
		exists.alternativeValues = new FeatureValue[1];
		exists.alternativeValues[0] = new StringValue(entity.probExists, "true");
		features.add(exists); */
		
		Vector<DiscreteProbabilityAssignment> assignments = generateProbabilityDistribution (features, new Vector<DiscreteProbabilityAssignment>());
		distrib.assignments = new DiscreteProbabilityAssignment[assignments.size()];
		distrib.assignments = assignments.toArray(distrib.assignments);
		return distrib;
	}


	public static float getProbabilityValue(ProbabilityDistribution distrib, DiscreteProbabilityAssignment assignment) {
		if (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			return getProbabilityValue((DiscreteProbabilityDistribution)distrib, assignment);
		}
		else if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			return getProbabilityValue((CombinedProbabilityDistribution)distrib, assignment);
		}
		else {
			log("sorry, only discrete and combined probability distribution are supported at the moment!");
			return 0.0f;
		}
	}

	public static float getProbabilityValue (CombinedProbabilityDistribution distrib, DiscreteProbabilityAssignment assignment) {
		float result = 1.0f;
		
		for (int i = 0; i < distrib.distributions.length; i++) {
			ProbabilityDistribution subdistrib = distrib.distributions[i];
				if (distrib.opType.equals(OperationType.MULTIPLIED)) {
					float newvalue = getProbabilityValue(subdistrib, assignment);
					result = result * newvalue;
				}
				else if (distrib.opType.equals(OperationType.DIVIDED)) {
					result = result / getProbabilityValue(subdistrib, assignment);
				}
		}
		
		return result;
	}
	
	public static float getProbabilityValue (DiscreteProbabilityDistribution distrib, DiscreteProbabilityAssignment assignment) {
		float result = 0.0f;
		
		Vector<FeatureValuePair> featpairsAssignment = new Vector<FeatureValuePair>();
		for (int i = 0; i < assignment.featurepairs.length; i++) {
			featpairsAssignment.add(assignment.featurepairs[i]);
		}
		
		for (int i = 0; i < distrib.assignments.length; i++) {
			DiscreteProbabilityAssignment curAssignment = distrib.assignments[i];
			Vector<FeatureValuePair> featpairsInDistrib = new Vector<FeatureValuePair>();
			for (int j = 0; j < curAssignment.featurepairs.length; j++) {
				featpairsInDistrib.add(curAssignment.featurepairs[j]);
			}
			if (containsAll(featpairsAssignment, featpairsInDistrib)) {
				return curAssignment.prob;
			}
		}
		
		log("WARNING, no probability value found for assignment");
		return result;
	}
	
	
	
	public static boolean containsAll(Vector<FeatureValuePair> pairs1, Vector<FeatureValuePair> pairs2) {
		boolean result = true;
		
		for (Enumeration<FeatureValuePair> e = pairs2.elements() ; e.hasMoreElements(); ) {
			FeatureValuePair pair2 = e.nextElement();
			boolean foundMatch = false;
			for (Enumeration<FeatureValuePair> f = pairs1.elements() ; f.hasMoreElements()  && !foundMatch; ) {
				FeatureValuePair pair1 = f.nextElement();
				if (pair1.featlabel.equals(pair2.featlabel) && ((StringValue)pair1.featvalue).val.equals(((StringValue)pair2.featvalue).val)) {
					foundMatch = true;
				}
			}
			if (!foundMatch) {
				return false;
			}
		}
		
		return result;
	}
	
	public static Vector<DiscreteProbabilityAssignment> generateProbabilityDistribution 
	(Vector<Feature> features, Vector<DiscreteProbabilityAssignment> prevAssignments) {

		Vector<DiscreteProbabilityAssignment> newAssignments = new Vector<DiscreteProbabilityAssignment>();

		Feature feat = features.remove(0);

		if (prevAssignments.size() == 0) {

			for (int i = 0; i < feat.alternativeValues.length ; i++) {
				FeatureValue fv = feat.alternativeValues[i];
				DiscreteProbabilityAssignment newAss = new DiscreteProbabilityAssignment();
				newAss.featurepairs = new FeatureValuePair[1];
				newAss.featurepairs[0] = new FeatureValuePair(feat.featlabel, fv);
				newAss.prob = fv.independentProb;
				newAssignments.add(newAss);
			}
		}

		else {
			for (int i = 0; i < feat.alternativeValues.length ; i++) {
				FeatureValue fv = feat.alternativeValues[i];

				for (Enumeration<DiscreteProbabilityAssignment> e = prevAssignments.elements() ; e.hasMoreElements(); ) {
					DiscreteProbabilityAssignment ass = e.nextElement();

					DiscreteProbabilityAssignment newAss = new DiscreteProbabilityAssignment();
					Vector<FeatureValuePair> featpairs = new Vector<FeatureValuePair>();

					for (int j = 0 ; j <ass.featurepairs.length ; j++) {
						featpairs.add(ass.featurepairs[j]);
					}
					FeatureValuePair newPair = new FeatureValuePair(feat.featlabel, fv);
					featpairs.add(newPair);
					newAss.featurepairs = new FeatureValuePair[featpairs.size()];
					newAss.featurepairs = featpairs.toArray(newAss.featurepairs);
					newAss.prob = ass.prob * fv.independentProb;
					newAssignments.add(newAss);
				}
			}
		}

		if (features.size() > 0) {
			return generateProbabilityDistribution(features, newAssignments);
		}
		else  {
			return newAssignments;
		}
}
	
public static String getDiscreteProbabilityDistributionPrettyPrint(DiscreteProbabilityDistribution distrib) {
	String text = "";
	
	for (int i = 0; i < distrib.assignments.length; i++) {
		DiscreteProbabilityAssignment assignment = distrib.assignments[i];
		text += getDiscreteProbabilityAssignmentPrettyPrint(assignment) + "\n";
	}
	
	return text;
}

public static String getDiscreteProbabilityAssignmentPrettyPrint(DiscreteProbabilityAssignment assignment) {
	
	String text = "P ( " ;
	for (int j = 0; j < assignment.featurepairs.length ; j++) {
		text += assignment.featurepairs[j].featlabel + " = " + ((StringValue)assignment.featurepairs[j].featvalue).val ;
		if (j < (assignment.featurepairs.length - 1)) {
			text += ", ";
		}
	}
	text += " ) = " + assignment.prob ;
	return text;
}

public static void log(String s) {
	System.out.println("[ProbabilityDistributionUtils] " + s);
}
	
}
