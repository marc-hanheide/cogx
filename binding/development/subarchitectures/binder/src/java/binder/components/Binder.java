package binder.components;

import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.combined.OperationType;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.featvalues.StringValue;
import binder.bayesiannetwork.BayesianNetworkManager;
import binder.utils.GradientDescent;
import binder.utils.ProbDistribUtils;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class Binder extends ManagedComponent  {

	BayesianNetworkManager BNManager;

	//  HashMap<String,Union> curUnions = new HashMap<String,Union>();
	HashMap<Union, Float> maxValues = new HashMap<Union,Float>();

	public float ALPHA_CONST = 0.2f;

	public float PRIOR_PEXISTS = 0.4f;

	public float FACTOR = 0.99f;

	public boolean incrementalBinding = true;


	Vector<UnionConfiguration> currentUnionConfigurations ;

	
	@Override
	public void start() {

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Proxy.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					proxyUpdate(_wmc);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});


		BNManager = new BayesianNetworkManager();
		initializeUnionConfigurations();

		log("Binding Monitor successfully started");
	}


	public void initializeUnionConfigurations () {
		currentUnionConfigurations = new Vector<UnionConfiguration>();
		UnionConfiguration initialConfig = new UnionConfiguration();
		initialConfig.includedUnions = new Union[0];
		currentUnionConfigurations.add(initialConfig);
	}

	
	
	public Union constructNewUnion(Vector<PerceivedEntity> includedEntities) {
		//	log("***** Constructing a new union ****");
		Union union = new Union();
		union.entityID = newDataID();
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

		union.includedProxies = new Proxy[includedProxies.size()];
		union.includedProxies = includedProxies.toArray(union.includedProxies);

		Vector<Feature> features = new Vector<Feature>();

		union.probExists = PRIOR_PEXISTS;
		float probNotExists = (1.0f- PRIOR_PEXISTS);
		for (Enumeration<PerceivedEntity> e = includedEntities.elements(); e.hasMoreElements();) {
			PerceivedEntity prox = e.nextElement();
			for (int i = 0; i < prox.features.length ; i++) {
				Feature feat = new Feature();
				feat.featlabel = prox.features[i].featlabel;
				feat.alternativeValues = new FeatureValue[prox.features[i].alternativeValues.length];
				for (int j =0; j < prox.features[i].alternativeValues.length ; j++) {
					feat.alternativeValues[j] = new FeatureValue();
					if (prox.features[i].alternativeValues[j].getClass().equals(StringValue.class)) {
						feat.alternativeValues[j] = new StringValue(0.0f,
								((StringValue)prox.features[i].alternativeValues[j]).val);
					}
				}
				features.add(feat);
			}
			union.probExists = union.probExists * prox.probExists; 
			probNotExists = probNotExists * (1-prox.probExists);
		}
		union.probExists = union.probExists / ((float) Math.pow(PRIOR_PEXISTS,(includedEntities.size())));
		probNotExists = probNotExists / ((float) Math.pow((1.0f- PRIOR_PEXISTS),(includedEntities.size())));
		float normConstant = 1.0f / (union.probExists + probNotExists);

		union.probExists = normConstant * union.probExists;

		//	log("union prob exists:  " + union.probExists);

		union.features = new Feature[features.size()];
		union.features = features.toArray(union.features);

		if (includedEntities.size() > 1) {
			union.distribution = computeUnionDistribution(union);
		}
		else {
			union.distribution = includedEntities.elementAt(0).distribution;
		}


		return union;
	}



	private ProbabilityDistribution computeUnionDistribution(Union union) {

		DiscreteProbabilityDistribution priorDistrib =  BNManager.getPriorDistribution(union.features);

//		log("Maximum for prior distribution of the union: " + GradientDescent.getMaximum(priorDistrib));

		Vector<CombinedProbabilityDistribution> proxiesDistrib = new Vector<CombinedProbabilityDistribution>();

		for (int i = 0 ; i < union.includedProxies.length ; i++) {
			Proxy proxy = union.includedProxies[i];

			//		log("Maximum for observation-driven distribution of the proxy " + i +  ": " + GradientDescent.getMaximum(proxy.distribution));

			DiscreteProbabilityDistribution priorDistribForProxy =  BNManager.getPriorDistribution(proxy.features);
			//		log("Maximum for prior distribution of the proxy " + i +  ": " + GradientDescent.getMaximum(priorDistribForProxy));

			CombinedProbabilityDistribution finalProxyDistrib = new CombinedProbabilityDistribution();
			finalProxyDistrib.opType = OperationType.MULTIPLIED;
			finalProxyDistrib.distributions = new DiscreteProbabilityDistribution[2];
			finalProxyDistrib.distributions[0] = proxy.distribution;
			finalProxyDistrib.distributions[1] = 
				ProbDistribUtils.invertDistribution(priorDistribForProxy);
			finalProxyDistrib.distributions[1] = 
				ProbDistribUtils.multiplyDistributionWithConstantValue((DiscreteProbabilityDistribution)finalProxyDistrib.distributions[1], ALPHA_CONST);
			//		log("Maximum for final distribution of the proxy " + i +  ": " + GradientDescent.getMaximum(finalProxyDistrib));

			proxiesDistrib.add(finalProxyDistrib);
		}

		CombinedProbabilityDistribution finalDistrib = new CombinedProbabilityDistribution();
		finalDistrib.opType = OperationType.MULTIPLIED;
		finalDistrib.distributions = new ProbabilityDistribution[proxiesDistrib.size() + 1];

		finalDistrib.distributions[0] = ProbDistribUtils.multiplyDistributionWithConstantValue(priorDistrib, ALPHA_CONST);

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


	@Override
	public void configure(Map<String, String> _config) {
		if (_config.containsKey("--alpha")) {
			ALPHA_CONST = Float.parseFloat(_config.get("--alpha"));
		} 
		if (_config.containsKey("--incremental")) {
			incrementalBinding = Boolean.parseBoolean(_config.get("--incremental"));
		} 
	}



	/**	private void deleteOldUnions() {
		try {
			CASTData<Union>[] unions = getWorkingMemoryEntries(Union.class);
			for (int i = 0; i < unions.length; i++) {
				deleteFromWorkingMemory(unions[i].getID());
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	} */



	/**
	public void deleteUnionFromWM(Union oldUnion) {
		try {
			CASTData<UnionDistribution>[] uniondistribs = getWorkingMemoryEntries(UnionDistribution.class);
			for (int i = 0; i < uniondistribs.length; i++) {
				for (int j = 0 ; j < uniondistribs[i].getData().alternativeUnions.length ; j++) {
				if (uniondistribs[i].getData().alternativeUnions[j].equals(oldUnion)) {
					deleteFromWorkingMemory(uniondistribs[i].getID());
				}
				}
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
	public void deleteUnionDistributionFromWM(UnionDistribution uniondistrib) {
		try {
			CASTData<UnionDistribution>[] uniondistribs = getWorkingMemoryEntries(UnionDistribution.class);
			for (int i = 0; i < uniondistribs.length; i++) {
				if (uniondistribs[i].getData().equals(uniondistrib)) {
					deleteFromWorkingMemory(uniondistribs[i].getID());
				}
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
*/



	
	public void fullRebinding() {
		try {
			log("Perform full rebinding...");
		
			initializeUnionConfigurations();

			CASTData<Proxy>[] proxies = getWorkingMemoryEntries(Proxy.class);
			for (int i = 0 ; i < proxies.length; i++) {
				incrementalBinding(proxies[i].getData());
			}

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	

	public void proxyUpdate(WorkingMemoryChange wmc) {
		log("--------START BINDING----------");
		log("binder working memory updated with a new proxy!");

		long initTime = System.currentTimeMillis();

		if (incrementalBinding) {
			try {
			Proxy newProxy = getMemoryEntry(wmc.address, Proxy.class);
			incrementalBinding(newProxy);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
		}
		else  {
			fullRebinding();
		}

		long finalTime = System.currentTimeMillis();
		log("Total binding time: " + (finalTime - initTime)/1000.0 + " seconds");
		log("--------STOP BINDING----------");

	}

//	Vector<UnionDistribution> curUnionDistributions = new Vector<UnionDistribution>();
//	HashMap<Union, Vector<UnionDistribution>> includingDistributions = new HashMap<Union, Vector<UnionDistribution>>();

	
	
	
	
	
	
	public void incrementalBinding(Proxy newProxy) {
		try {
			log("Perform incremental binding...");

			log("Proxy ID: " + newProxy.entityID);

			Union newUnion = getInitialUnion(newProxy);
			
			//	sleepComponent(250);

			log("Construction of initial unions finished, moving to unions of more than 1 proxy...");
			log("current number of recorded unions in maxvalues: " + maxValues.size());

			
			log("Incremental binding algorithm finished!");
			
			Vector<UnionConfiguration> newUnionConfigurations = new Vector<UnionConfiguration>();
			HashMap<String,Union> alreadyMergedUnions = new HashMap<String, Union>();
			
			for (Enumeration<UnionConfiguration> configs = currentUnionConfigurations.elements() ; configs.hasMoreElements(); ) {
				
				UnionConfiguration existingUnionConfig = configs.nextElement();				
				
				UnionConfiguration newConfigWithSingleUnion = createNewUnionConfiguration (existingUnionConfig, newUnion);
				newUnionConfigurations.add(newConfigWithSingleUnion);	
				
				
				for (int i = 0 ; i < existingUnionConfig.includedUnions.length; i++) {
					
					Union existingUnion = existingUnionConfig.includedUnions[i];
					
					Union newMergedUnion;
					if (!hasConflictingSubarch(existingUnion, newUnion)) {
						
						if (!alreadyMergedUnions.containsKey(existingUnion.entityID)) {
						Vector<PerceivedEntity> unionsToMerge = new Vector<PerceivedEntity>();
						unionsToMerge.add(existingUnion);
						unionsToMerge.add(newUnion);
						newMergedUnion = constructNewUnion(unionsToMerge);
						alreadyMergedUnions.put(existingUnion.entityID, newMergedUnion);
						}
						else {
							newMergedUnion = alreadyMergedUnions.get(existingUnion.entityID);
						}
						UnionConfiguration newConfigWithMergedUnion = createNewUnionConfiguration (existingUnionConfig, newMergedUnion, existingUnion);
						newUnionConfigurations.add(newConfigWithMergedUnion);
						
					}
				}
			}
			
			for (Enumeration<UnionConfiguration> configs = currentUnionConfigurations.elements() ; configs.hasMoreElements(); ) {
				
			}
			currentUnionConfigurations = newUnionConfigurations;
			
			log("Total number of union configurations generated: " + currentUnionConfigurations.size());
			
	/**		AlternativeUnionConfigurations alters = new AlternativeUnionConfigurations();
			alters.alterconfigs = new UnionConfiguration[currentUnionConfigurations.size()];
			for (int i = 0; i < alters.alterconfigs.length ; i++) {
				alters.alterconfigs[i] = currentUnionConfigurations.elementAt(i); 
			}
			addEntityToWM(alters); */

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

	
	
	

	private UnionConfiguration createNewUnionConfiguration(UnionConfiguration existingUnionConfig, Union unionToAdd) {			
		return createNewUnionConfiguration(existingUnionConfig, unionToAdd, new Vector<Union>());
	}
	
	
	
	private UnionConfiguration createNewUnionConfiguration(UnionConfiguration existingUnionConfig, Union unionToAdd, Union unionToRemove) {
		Vector<Union> unionsToRemove = new Vector<Union>();
		unionsToRemove.add(unionToRemove);
		return createNewUnionConfiguration(existingUnionConfig, unionToAdd, unionsToRemove);
	}
	
	
	
	private UnionConfiguration createNewUnionConfiguration(UnionConfiguration existingUnionConfig, 
			Union unionToAdd, Vector<Union> unionsToRemove) {
		
		UnionConfiguration newConfig = new UnionConfiguration();
		newConfig.includedUnions = new Union[existingUnionConfig.includedUnions.length + 1 - unionsToRemove.size()];
		
		int count = 0;
		for (int i = 0 ; i < existingUnionConfig.includedUnions.length; i++) {
			if (!unionsToRemove.contains(existingUnionConfig.includedUnions[i])) {
				newConfig.includedUnions[i- count] = existingUnionConfig.includedUnions[i];
			}
			else {
				count ++;
			}
		}
		
		newConfig.includedUnions[existingUnionConfig.includedUnions.length - unionsToRemove.size()] = unionToAdd;
		
		return newConfig;
		
	}

	private boolean hasConflictingSubarch(Union union1, Union union2) {	
		for (int i = 0 ; i < union1.includedProxies.length ; i++) {
			Proxy proxyi = union1.includedProxies[i];
			for (int j = 0 ; j < union2.includedProxies.length ; j++) {
				Proxy proxyj = union2.includedProxies[j];
				if (proxyi.subarchId.equals(proxyj.subarchId)) {
					return true;
				}
			}
		}		
		return false;
	}


/**
	private void addEntityToWM(Union entity) {

		try {
			UnionDistribution uniondistrib = new UnionDistribution();
			uniondistrib.alternativeUnions = new Union[1];
			uniondistrib.alternativeUnions[0] = entity;
			
			addToWorkingMemory(entity.entityID, uniondistrib);
			log("new Union distribution succesfully added to the binding working memory");
			log("Union ID: " + entity.entityID);
			log("Union has " + entity.includedProxies.length + " included proxies");

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	

	
	private void addEntityToWM(UnionDistribution distrib) {

		try {
			
			addToWorkingMemory(newDataID(), distrib);
			log("new Union distribution succesfully added to the binding working memory");
			log("number of unions in distrib: " +  distrib.alternativeUnions.length);
			for (int i = 0 ; i < distrib.alternativeUnions.length ; i++) {
				Union union = distrib.alternativeUnions[i];
			log("Union ID: " + union.entityID);
			log("Union has " + union.includedProxies.length + " included proxies");
			}

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	*/
	
	private void addEntityToWM(UnionConfiguration config) {

		try {
			
			addToWorkingMemory(newDataID(), config);
			log("new Union distribution succesfully added to the binding working memory");
			log("number of unions in config: " +  config.includedUnions.length);
			for (int i = 0 ; i < config.includedUnions.length ; i++) {
				Union union = config.includedUnions[i];
			log("Union ID: " + union.entityID);
			log("Union has " + union.includedProxies.length + " included proxies");
			}

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

/**
	private void addEntityToWM(AlternativeUnionConfigurations configs) {

		try {
			
			addToWorkingMemory(newDataID(), configs);

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
*/	
}
