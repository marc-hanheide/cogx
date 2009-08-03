package binder.bayesiannetwork;

import java.util.Enumeration;
import java.util.Vector;

import binder.autogen.bayesiannetworks.BayesianNetwork;
import binder.autogen.bayesiannetworks.BayesianNetworkNode;
import binder.autogen.bayesiannetworks.FeatureValueCorrelation;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.featvalues.StringValue;
import binder.utils.BayesianNetworkUtils;

public class BayesianNetworkManager {

	BayesianNetwork network;
	
	boolean logging = false;
	
	public BayesianNetworkManager() {		
	
		log("Start building the bayesian network...");
		network = BayesianNetworkUtils.constructNetwork();
		log("Construction of bayesian network successfull!");
		log("number of nodes: " + network.nodes.length);		
		log("number of edges: " + network.edges.length);
		
	}
	

	
	public DiscreteProbabilityDistribution getPriorDistribution(Feature[] features) {
		log("Start computing the prior distribution...");
		DiscreteProbabilityDistribution priorDistrib = new DiscreteProbabilityDistribution();
		priorDistrib.assignments = new DiscreteProbabilityAssignment[features.length];
		
		Vector<DiscreteProbabilityAssignment[]> priorDistribsForFeature = new Vector<DiscreteProbabilityAssignment[]>();
		
		if (features.length > 0) {
		for (int i = 0 ; i < features.length; i ++) {
			log("Current feature: \"" + features[i].featlabel + "\"");

			DiscreteProbabilityAssignment[] priorDistribForFeature = getPriorDistribution(features[i]);
			priorDistribsForFeature.add(priorDistribForFeature);
		}
		
		Vector<DiscreteProbabilityAssignment> jointDistribV = computeJointDistribution(priorDistribsForFeature);
		priorDistrib.assignments = new DiscreteProbabilityAssignment[jointDistribV.size()];
		priorDistrib.assignments = jointDistribV.toArray(priorDistrib.assignments);
		}
		return priorDistrib;
	}


	private Vector<DiscreteProbabilityAssignment> computeJointDistribution(Vector<DiscreteProbabilityAssignment[]> priorDistribs) {
		
		log("Computing the joint distribution, vector size: " + priorDistribs.size());
		
		if (priorDistribs.size() == 1) {
			Vector<DiscreteProbabilityAssignment> finalAssignment = new Vector<DiscreteProbabilityAssignment>();
			DiscreteProbabilityAssignment[] curAssignments = priorDistribs.elementAt(0);
			for (int i = 0; i < curAssignments.length ; i++) {
				finalAssignment.add(priorDistribs.elementAt(0)[i]);
			}
			return finalAssignment;
		}
		
		else {
			DiscreteProbabilityAssignment[] independentProbs = priorDistribs.remove(0);
			Vector<DiscreteProbabilityAssignment> previousAssignments = computeJointDistribution(priorDistribs);
			Vector<DiscreteProbabilityAssignment> newAssignments = new Vector<DiscreteProbabilityAssignment>();
			
			for (int i = 0 ; i < independentProbs.length; i++) {
				
				for (Enumeration<DiscreteProbabilityAssignment> e = previousAssignments.elements(); e.hasMoreElements() ; ) {
					DiscreteProbabilityAssignment assignment = e.nextElement();
										
					DiscreteProbabilityAssignment newAssignment = new DiscreteProbabilityAssignment();
					Vector<FeatureValuePair> featpairs = new Vector<FeatureValuePair>();
					for (int j = 0 ; j < independentProbs[i].featurepairs.length ; j++) {
						featpairs.add(independentProbs[i].featurepairs[j]);
					}
					for (int j = 0 ; j < assignment.featurepairs.length ; j++) {
						featpairs.add(assignment.featurepairs[j]);
					}
					
					float prob = (float) getJointProbabilityValue(featpairs); 

					
					newAssignment.featurepairs = new FeatureValuePair[featpairs.size()];
					newAssignment.featurepairs = featpairs.toArray(newAssignment.featurepairs);
					newAssignment.prob = prob;
					newAssignments.add(newAssignment);
					log("prob: " + prob);
				}
			}
			
			return newAssignments;
		}
		
	}
	
	public DiscreteProbabilityAssignment[] getPriorDistribution (Feature feature) {
		
		if (feature.alternativeValues != null) {
			DiscreteProbabilityAssignment[] assignments = new DiscreteProbabilityAssignment[feature.alternativeValues.length];

		log("Computing prior distribution with feature: \"" + feature.featlabel + "\"");
			
		for (int i = 0; i < feature.alternativeValues.length; i++) {
			FeatureValue featvalue = feature.alternativeValues[i];
			log("Feature value currently looked at: " + ((StringValue) featvalue).val);
			DiscreteProbabilityAssignment assignment = new DiscreteProbabilityAssignment();
			assignment.featurepairs = new FeatureValuePair[1];
			assignment.featurepairs[0] = new FeatureValuePair();
			assignment.featurepairs[0].featlabel = feature.featlabel;
			assignment.featurepairs[0].featvalue = featvalue;
			assignment.prob = getIndependentProb(feature.featlabel, featvalue);
			log("Independent probability: " + assignment.prob);
			assignments[i] = assignment;
		}
		return assignments;
		}
		else {
			log("ERROR: alternative values in feature are not specified");
			return null;
		}
	}
	

	
	public double getJointProbabilityValue(Vector<FeatureValuePair> featpairs) {
		double result = 1.0f;
		
		for(Enumeration<FeatureValuePair> e = featpairs.elements(); e.hasMoreElements(); ) {
			FeatureValuePair featpair1 = e.nextElement();
			
			double condProbs = 1.0f;
			
			for(Enumeration<FeatureValuePair> f = featpairs.elements(); f.hasMoreElements() ; ) {
				FeatureValuePair featpair2 = f.nextElement();
				if (featpair1 != featpair2) {
					FeatureValueCorrelation corr = BayesianNetworkUtils.getCorrelationsBetweenFVs(network, featpair1, featpair2);
					if (corr != null) {
						condProbs = condProbs * corr.condProb;
					}
				}
			}
			
			double finalProbForFeatpair1;
			if (condProbs < 1.0f) {
				finalProbForFeatpair1 = Math.sqrt(condProbs);
			}
			else {
				finalProbForFeatpair1 = getIndependentProb(featpair1.featlabel, featpair1.featvalue);
			}

			result = result * finalProbForFeatpair1;
		}
			
		log("joint probability value: " + result);
		return result;
	}
	
	
public float getIndependentProb (String featlabel, FeatureValue featvalue) {
		
		BayesianNetworkNode node = BayesianNetworkUtils.getNodeWithFeature(network, featlabel);
		
	//	log("equivalent BN node found? " + (node != null));

		if (node != null) {
		for (int i = 0; i < node.feat.alternativeValues.length ; i++) {
			StringValue featurevalue = (StringValue) node.feat.alternativeValues[i];
			if (featurevalue.val.equals(((StringValue)featvalue).val)) {
				if (featurevalue.independentProb > 0) {
					return featurevalue.independentProb;
				}
				else {
					return (1.0f / node.feat.alternativeValues.length);   // INCORRECT
				}
			}
		}
		}
		else  {
			return 0.5f;
		}
		log("WARNING: Independent probability for " + featlabel + " = " + featvalue + " is not specified");
		return 0.0f;
	}

public void log(String s) {
	if (logging) {
		System.out.println("[BayesianNetworkManager] " + s);
	}
}
}
