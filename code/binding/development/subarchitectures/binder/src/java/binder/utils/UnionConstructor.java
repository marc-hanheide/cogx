package binder.utils;

import java.util.Enumeration;
import java.util.Vector;

import cast.architecture.UnmanagedComponent;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.combined.OperationType;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.featvalues.StringValue;
import binder.bayesiannetwork.BayesianNetworkManager;

public class UnionConstructor extends UnmanagedComponent {

	

	// The Bayesian network manager
	BayesianNetworkManager BNManager;

	// Constant controlling the greediness of the binder
	public float ALPHA_CONST = 0.2f;

	// Prior probability of the existence of a proxy
	public float PRIOR_PEXISTS = 0.4f;

	
	public UnionConstructor() {
		BNManager = new BayesianNetworkManager();
	}
	
	public void setAlphaParam(float alpha) {
		ALPHA_CONST = alpha;
	}
	
	public void setPriorParam(float prior) {
		PRIOR_PEXISTS = prior;
	}
	
	
	/**
	 * Construct a new union based on the merge of several perceptual entities
	 * (which may be proxies or unions themselves)
	 * 
	 * @param includedEntities as vector of entities (with size > 0 )
	 * @return the new union
	 */
	
	public Union constructNewUnion(Vector<PerceivedEntity> includedEntities) {
		//	log("***** Constructing a new union ****");
		
		// Create a new union with a new data ID
		Union union = new Union();
		union.entityID = newDataID();
	
		// Specify the proxies included in the union
		Vector<Proxy> includedProxies = getProxies(includedEntities);
		union.includedProxies = new Proxy[includedProxies.size()];
		union.includedProxies = includedProxies.toArray(union.includedProxies);

		// Compute the existence probability of the union
		union.probExists = computeProbExists(includedEntities);

		// Extract the possible features for the union
		Vector<Feature> features = getFeatures(includedEntities);
		union.features = new Feature[features.size()];
		union.features = features.toArray(union.features);
		
		// Finally, compute the union distribution
		if (includedEntities.size() > 1) {
			union.distribution = computeUnionDistribution(union);
		}
		else {
			union.distribution = includedEntities.elementAt(0).distribution;
		}

		return union;
	}
	

	private Vector<Feature> getFeatures (Vector<PerceivedEntity> includedEntities) {
		Vector<Feature> features = new Vector<Feature>();

		for (Enumeration<PerceivedEntity> e = includedEntities.elements(); e.hasMoreElements();) {
			PerceivedEntity prox = e.nextElement();
			for (int i = 0; i < prox.features.length ; i++) {
				Feature feat = new Feature();
				feat.featlabel = prox.features[i].featlabel;
				feat.alternativeValues = new FeatureValue[prox.features[i].alternativeValues.length];
				for (int j =0; j < prox.features[i].alternativeValues.length ; j++) {
					feat.alternativeValues[j] = new FeatureValue();
					if (prox.features[i].alternativeValues[j].getClass().equals(StringValue.class)) {
						feat.alternativeValues[j] = 
							new StringValue(prox.features[i].alternativeValues[j].independentProb,
								((StringValue)prox.features[i].alternativeValues[j]).val);
					}
				}
				features.add(feat);
			}
		}
		return features;
	}
	

	
	
	/**
	 * Get all the proxies included in a set of perceptual entities (in case the entity
	 * is a proxy, it is simply added, and in case it is an union, the set of all included
	 * proxies is added to the resulting set)
	 * @param includedEntities the set of perceptual entities
	 * @return the resulting set of proxies
	 */
	private Vector<Proxy> getProxies (Vector<PerceivedEntity> includedEntities) {
		Vector<Proxy> includedProxies = new Vector<Proxy>();
		for (Enumeration<PerceivedEntity> en = includedEntities.elements() ; en.hasMoreElements() ;) {
			PerceivedEntity entity = en.nextElement();
			if (entity.getClass().equals(Proxy.class)) {
				includedProxies.add((Proxy)entity);
			}
			else if (entity.getClass().equals(Union.class)) {
				Union includedUnion = (Union)entity;
				for (int i = 0 ; i < includedUnion.includedProxies.length ; i++) {
					includedProxies.add(includedUnion.includedProxies[i]);
				}
			}
		}
		return includedProxies;
	}
	

	private float computeProbExists (Vector<PerceivedEntity> includedEntities) {
		float probExists = PRIOR_PEXISTS;
		float probNotExists = (1 - PRIOR_PEXISTS);

		for (Enumeration<PerceivedEntity> e = includedEntities.elements(); e.hasMoreElements();) {
			PerceivedEntity prox = e.nextElement();
			probExists = probExists * prox.probExists; 
			probNotExists = probNotExists * (1- prox.probExists); 
		}	
		
		probExists = probExists / ((float) Math.pow(PRIOR_PEXISTS,(includedEntities.size())));
		probNotExists = probNotExists / ((float) Math.pow((1-PRIOR_PEXISTS),(includedEntities.size())));
		
		float normConstant = 1.0f / (probExists + probNotExists);
		probExists = normConstant * probExists;
		return probExists;
	}
	


	/** 
	 * Compute the probability distribution of the union
	 * @param union
	 * @return
	 */
	public ProbabilityDistribution computeUnionDistribution(Union union) {

		// Compute the prior distribution for the union
		DiscreteProbabilityDistribution priorDistrib =  BNManager.getPriorDistribution(union.features);

//		log("Maximum for prior distribution of the union: " + GradientDescent.getMaximum(priorDistrib));

		Vector<CombinedProbabilityDistribution> proxiesDistrib = 
			new Vector<CombinedProbabilityDistribution>();

		for (int i = 0 ; i < union.includedProxies.length ; i++) {
			Proxy proxy = union.includedProxies[i];

			//		log("Maximum for observation-driven distribution of the proxy " + i +  ": "
			// + GradientDescent.getMaximum(proxy.distribution));

			DiscreteProbabilityDistribution priorDistribForProxy = BNManager.getPriorDistribution(proxy);
			
			//		log("Maximum for prior distribution of the proxy " + i +  
			// ": " + GradientDescent.getMaximum(priorDistribForProxy));

			CombinedProbabilityDistribution finalProxyDistrib = new CombinedProbabilityDistribution();
			finalProxyDistrib.opType = OperationType.MULTIPLIED;
			finalProxyDistrib.distributions = new DiscreteProbabilityDistribution[2];
			finalProxyDistrib.distributions[0] = proxy.distribution;
			finalProxyDistrib.distributions[1] = 
				ProbDistribUtils.invertDistribution(priorDistribForProxy);
			finalProxyDistrib.distributions[1] = 
				ProbDistribUtils.multiplyDistributionWithConstantValue
				((DiscreteProbabilityDistribution)finalProxyDistrib.distributions[1], ALPHA_CONST);
	
			//		log("Maximum for final distribution of the proxy " + 
			// i +  ": " + GradientDescent.getMaximum(finalProxyDistrib));

			proxiesDistrib.add(finalProxyDistrib);
		}

		CombinedProbabilityDistribution finalDistrib = new CombinedProbabilityDistribution();
		finalDistrib.opType = OperationType.MULTIPLIED;
		finalDistrib.distributions = new ProbabilityDistribution[proxiesDistrib.size() + 1];

		finalDistrib.distributions[0] = 
			ProbDistribUtils.multiplyDistributionWithConstantValue(priorDistrib, ALPHA_CONST);

		for (int i = 1 ; i < proxiesDistrib.size() + 1 ; i++ ) {
			finalDistrib.distributions[i] = proxiesDistrib.elementAt(i-1);
		}

		//		log("Maximum for final distribution of the union: " + GradientDescent.getMaximum(finalDistrib));

		// finalDistrib = ProbDistribUtils.normaliseDistribution(finalDistrib, 1.0f);
		return finalDistrib;
	}




	public Union getInitialUnion(Proxy proxy) {
		Vector<PerceivedEntity> curProxyV = new Vector<PerceivedEntity>();
		curProxyV.add(proxy);
		Union union = constructNewUnion(curProxyV);
		return union;
	} 

}
