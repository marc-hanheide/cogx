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


public class ProbDistribUtils {



	public static ProbabilityDistribution generateProbabilityDistribution (PerceivedEntity entity) {
		DiscreteProbabilityDistribution distrib = new DiscreteProbabilityDistribution();

		if (entity.features.length > 0) {
			Vector<Feature> features = new Vector<Feature>();
			for (int i = 0; i < entity.features.length ; i++) {
				features.add(entity.features[i]);
			}

			Vector<DiscreteProbabilityAssignment> assignments = generateProbabilityDistribution (features, new Vector<DiscreteProbabilityAssignment>());
			distrib.assignments = new DiscreteProbabilityAssignment[assignments.size()];
			distrib.assignments = assignments.toArray(distrib.assignments);

			for (int i = 0 ; i < distrib.assignments.length ; i++) {
				distrib.assignments[i].prob = distrib.assignments[i].prob * entity.probExists;
			}

		}
		return ProbDistribUtils.normaliseDistribution(distrib, 1.0f);
	}


	public static Feature[] addIndeterminateFeatureValues (Feature[] features) {

		for (int i = 0 ; i < features.length ; i++) {

			float totalProb = 0.0f;
			Vector<FeatureValue> values = new Vector<FeatureValue>();
			for (int j= 0 ; j < features[i].alternativeValues.length ; j++) {
				values.add(features[i].alternativeValues[j]);
				totalProb += features[i].alternativeValues[j].independentProb;
			}
			if (totalProb < 1.0f) {
				features[i].alternativeValues = new FeatureValue[values.size() + 1];
				for (int j = 0 ; j < values.size(); j++ ) {
					features[i].alternativeValues[j] = values.elementAt(j);
				}
				features[i].alternativeValues[values.size()] = new StringValue((1.0f - totalProb), "indeterminate");
			}
		}

		return features;
	}


	public static Vector<DiscreteProbabilityAssignment> getAssignmentsIncludingSmallerAssignment(
			DiscreteProbabilityDistribution distrib, DiscreteProbabilityAssignment smallassign) {

		Vector<DiscreteProbabilityAssignment> assignments = new Vector<DiscreteProbabilityAssignment>();

		for (int i =0; i < distrib.assignments.length; i++) {

			DiscreteProbabilityAssignment bigassign = distrib.assignments[i];
			
			if (containsAll(bigassign, smallassign)) {
				assignments.add(bigassign);
			}
		}

		return assignments;
	}



	

	public static float getMarginalProbabilityValue(ProbabilityDistribution distrib, FeatureValuePair pair) {
		float result = 0.0f;
		
		DiscreteProbabilityAssignment smallassignment = new DiscreteProbabilityAssignment();
		smallassignment.featurepairs = new FeatureValuePair[1];
		smallassignment.featurepairs[0] = pair;
		
		if (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			DiscreteProbabilityDistribution distrib2 = (DiscreteProbabilityDistribution) distrib;
			
			Vector<DiscreteProbabilityAssignment> assignments = getAssignmentsIncludingSmallerAssignment(distrib2, smallassignment);
			
			for (Enumeration<DiscreteProbabilityAssignment> enu = assignments.elements() ; enu.hasMoreElements() ; ) {
				result += enu.nextElement().prob;
			}
		}
		
		if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			CombinedProbabilityDistribution distrib2 = (CombinedProbabilityDistribution) distrib;
			
			
			if (distrib2.distributions.length > 0) {
			DiscreteProbabilityDistribution firstdistrib = (DiscreteProbabilityDistribution) distrib2.distributions[0];
			
			Vector<DiscreteProbabilityAssignment> assignments = getAssignmentsIncludingSmallerAssignment(firstdistrib, smallassignment);
			
			if (assignments.size() > 0) {
				
				for (Enumeration<DiscreteProbabilityAssignment> enu = assignments.elements() ; enu.hasMoreElements() ; ) {
					DiscreteProbabilityAssignment assign = enu.nextElement();
				//	log("ass: " + getDiscreteProbabilityAssignmentPrettyPrint(assign));
					float partialresult = getProbabilityValue(distrib2, assign);
					result += partialresult;
				}
			}
			
			}
		}
		
		return result;
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
				float subdistribresult = getProbabilityValue(subdistrib, assignment); 
				result = result * subdistribresult;
			}
		}

		return result;
	}

	
	public static CombinedProbabilityDistribution normaliseDistribution(CombinedProbabilityDistribution distrib, float probExists) {

		DiscreteProbabilityDistribution firstDistrib = (DiscreteProbabilityDistribution) distrib.distributions[0];
		float total = 0.0f;
		for (int i = 0; i < firstDistrib.assignments.length; i++) {
			
			DiscreteProbabilityAssignment assignment = firstDistrib.assignments[i];
					
			float probValue = ProbDistribUtils.getProbabilityValue(distrib, assignment);
			total += probValue;
		}
		
		for (int i = 0; i < firstDistrib.assignments.length; i++) {
			((DiscreteProbabilityDistribution)distrib.distributions[0]).assignments[i].prob = firstDistrib.assignments[i].prob / (total * probExists);
		}
		
		return distrib;
	}
	
	
	public static ProbabilityDistribution normaliseDistribution(ProbabilityDistribution distrib, float probExists) {
		
		if (distrib.getClass().equals(CombinedProbabilityDistribution.class)) {
			return normaliseDistribution((CombinedProbabilityDistribution)distrib, probExists);
		}
		else if  (distrib.getClass().equals(DiscreteProbabilityDistribution.class)) {
			return normaliseDistribution((DiscreteProbabilityDistribution)distrib, probExists);
		}
		
		return distrib;
	}
	
	

	public static DiscreteProbabilityDistribution normaliseDistribution(DiscreteProbabilityDistribution distrib, float probExists) {

		float total = 0.0f;
		if (distrib.assignments != null) {
			for (int i = 0; i < distrib.assignments.length; i++) {
				total += distrib.assignments[i].prob;
			}

			for (int i = 0; i < distrib.assignments.length; i++) {
				distrib.assignments[i].prob = distrib.assignments[i].prob / (total * probExists);
			}
		}
		return distrib;
	}



	public static DiscreteProbabilityDistribution multiplyDistributionWithConstantValue (
			DiscreteProbabilityDistribution distrib, float constantValue) {

		for (int i = 0; i < distrib.assignments.length; i++) {
			distrib.assignments[i].prob = distrib.assignments[i].prob * constantValue;
		}

		return distrib;
	}


	public static DiscreteProbabilityDistribution invertDistribution (
			DiscreteProbabilityDistribution distrib) {

		for (int i = 0; i < distrib.assignments.length; i++) {
			if (distrib.assignments[i].prob > 0)
				distrib.assignments[i].prob = 1 / distrib.assignments[i].prob ;
		}

		return distrib;
	}

	public static float getProbabilityValue (DiscreteProbabilityDistribution distrib, DiscreteProbabilityAssignment assignment) {
		float result = 0.0f;

		if (distrib.assignments != null) {
		for (int i = 0; i < distrib.assignments.length; i++) {
			DiscreteProbabilityAssignment curAssignment = distrib.assignments[i];
			if (containsAll(assignment, curAssignment)) {
				return curAssignment.prob;
			}
		}
		}

		log("WARNING, no probability value found for assignment: " + getDiscreteProbabilityAssignmentPrettyPrint(assignment));
		log("distrib: " + getDiscreteProbabilityDistributionPrettyPrint(distrib));
		return result;
	}


	public static boolean containsAll(DiscreteProbabilityAssignment bigassign, DiscreteProbabilityAssignment smallassign) {
		
		Vector<FeatureValuePair> pairs1 = new Vector<FeatureValuePair>();
		for (int j = 0; j < bigassign.featurepairs.length; j++) {
			pairs1.add(bigassign.featurepairs[j]);
		}
		
		Vector<FeatureValuePair> pairs2 = new Vector<FeatureValuePair>();
		for (int j = 0; j < smallassign.featurepairs.length; j++) {
			pairs2.add(smallassign.featurepairs[j]);
		}
		
		return containsAll(pairs1, pairs2);	
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

		if (distrib.assignments != null) {
		for (int i = 0; i < distrib.assignments.length; i++) {
			DiscreteProbabilityAssignment assignment = distrib.assignments[i];
			text += getDiscreteProbabilityAssignmentPrettyPrint(assignment) + "\n";
		}
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
