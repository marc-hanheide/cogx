package binder.components;

import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Vector;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.combined.OperationType;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.bayesiannetwork.BayesianNetworkManager;
import binder.utils.GradientDescent;
import binder.gui.BinderMonitorGUI;
import binder.utils.ProbabilityDistributionUtils;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class Binder extends ManagedComponent  {

	BayesianNetworkManager BNManager;
	
	HashMap<String,Union> curUnions = new HashMap<String,Union>();
	HashMap<Union, Float> maxValues = new HashMap<Union,Float>();


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
		
		log("Binding Monitor successfully started");
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
		union.probExists = 1.0f;
		for (Enumeration<PerceivedEntity> e = includedEntities.elements(); e.hasMoreElements();) {
			PerceivedEntity prox = e.nextElement();
			for (int i = 0; i < prox.features.length ; i++) {
				features.add(prox.features[i]);
			}
			union.probExists = union.probExists * prox.probExists;  // INCORRECT!!!
		}
		union.features = new Feature[features.size()];
		union.features = features.toArray(union.features);
		
		union.distribution = computeUnionDistribution(union);
		
		return union;
	}
	
	
	private ProbabilityDistribution computeUnionDistribution(Union union) {
		
		DiscreteProbabilityDistribution priorDistrib =  BNManager.getPriorDistribution(union.features);
	//	log("Maximum for prior distribution of the union: " + GradientDescent.getMaximum(priorDistrib));
		
		
		Vector<CombinedProbabilityDistribution> proxiesDistrib = new Vector<CombinedProbabilityDistribution>();
		
		for (int i = 0 ; i < union.includedProxies.length ; i++) {
			Proxy proxy = union.includedProxies[i];

	//		log("Maximum for observation-driven distribution of the proxy " + i +  ": " + GradientDescent.getMaximum(proxy.distribution));
	
			DiscreteProbabilityDistribution priorDistribForProxy =  BNManager.getPriorDistribution(proxy.features);
	//		log("Maximum for prior distribution of the proxy " + i +  ": " + GradientDescent.getMaximum(priorDistribForProxy));
	
			CombinedProbabilityDistribution finalProxyDistrib = new CombinedProbabilityDistribution();
			finalProxyDistrib.opType = OperationType.DIVIDED;
			finalProxyDistrib.distributions = new DiscreteProbabilityDistribution[2];
			finalProxyDistrib.distributions[0] = proxy.distribution;
			finalProxyDistrib.distributions[1] = priorDistribForProxy;
	//		log("Maximum for final distribution of the proxy " + i +  ": " + GradientDescent.getMaximum(finalProxyDistrib));
		
			proxiesDistrib.add(finalProxyDistrib);
		}
		
		CombinedProbabilityDistribution finalDistrib = new CombinedProbabilityDistribution();
		finalDistrib.opType = OperationType.MULTIPLIED;
		finalDistrib.distributions = new ProbabilityDistribution[proxiesDistrib.size() + 1];
		finalDistrib.distributions[0] = priorDistrib;
		for (int i = 1 ; i < proxiesDistrib.size() + 1 ; i++ ) {
			finalDistrib.distributions[i] = proxiesDistrib.elementAt(i-1);
		}
		
	//	log("Maximum for final distribution of the union: " + GradientDescent.getMaximum(finalDistrib));
		return finalDistrib;
	}
	
	
	
	public HashMap<String,Union> getInitialUnions(Vector<Proxy> proxies) {
		HashMap<String,Union> initialUnions = new HashMap<String,Union>();
		for (Enumeration<Proxy> e = proxies.elements(); e.hasMoreElements() ; ) {
			Proxy curProxy = e.nextElement();
			if (!initialUnions.containsKey(curProxy.entityID)) {
				Vector<PerceivedEntity> curProxyV = new Vector<PerceivedEntity>();
				curProxyV.add(curProxy);
				Union union = constructNewUnion(curProxyV);
				initialUnions.put(curProxy.entityID, union);
		}
		}
		
		return initialUnions;
	}
	
	
	public Union getInitialUnion(Proxy proxy) {
				Vector<PerceivedEntity> curProxyV = new Vector<PerceivedEntity>();
				curProxyV.add(proxy);
				Union union = constructNewUnion(curProxyV);
		return union;
	} 
	
	
	private void deleteOldUnions() {
		try {
			CASTData<Union>[] unions = getWorkingMemoryEntries(Union.class);
			for (int i = 0; i < unions.length; i++) {
				deleteFromWorkingMemory(unions[i].getID());
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	

	public void deleteUnionFromWM(Union oldUnion) {
		try {
			CASTData<Union>[] unions = getWorkingMemoryEntries(Union.class);
			for (int i = 0; i < unions.length; i++) {
				if (unions[i].getData().equals(oldUnion)) {
					deleteFromWorkingMemory(unions[i].getID());
				}
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
	
	
	public void proxyUpdate(WorkingMemoryChange wmc) {
		log("--------START BINDING----------");
		log("binder working memory updated with a new proxy!");
		try {
			long initTime = System.currentTimeMillis();
			Proxy newProxy = getMemoryEntry(wmc.address, Proxy.class);
			log("Proxy ID: " + newProxy.entityID);

			Union curUnionToAdd = getInitialUnion(newProxy);
			float maxValueForCurUnionToAdd = GradientDescent.getMaximum(curUnionToAdd.distribution);

			HashMap<Union, Float> newMaxValues = new HashMap<Union, Float>();
			
		//	sleepComponent(250);
			
			log("Construction of initial unions finished, moving to unions of more than 1 proxy...");
			log("current number of recorded unions in maxvalues: " + maxValues.size());
			
				for (Iterator<Union> it = maxValues.keySet().iterator() ; it.hasNext();) {
					
					Union existingUnion = it.next();

					if (!hasConflictingSubarch(curUnionToAdd, existingUnion)) {
						Vector<PerceivedEntity> unionsToMerge = new Vector<PerceivedEntity>();
						unionsToMerge.add(curUnionToAdd);
						unionsToMerge.add(existingUnion);
						Union newMergedUnion = constructNewUnion(unionsToMerge);
						float maxOfNewMergedUnion = GradientDescent.getMaximum(newMergedUnion.distribution);
						
						log("maxOfnewMergedUnion: " + maxOfNewMergedUnion );
						if (maxOfNewMergedUnion > maxValueForCurUnionToAdd && 
								maxOfNewMergedUnion > maxValues.get(existingUnion)) {
	
						deleteUnionFromWM(existingUnion);
						
						curUnionToAdd = newMergedUnion;
						maxValueForCurUnionToAdd = maxOfNewMergedUnion;
						}
						else {
							newMaxValues.put(existingUnion, maxValues.get(existingUnion));
						}
					}
					else {
						newMaxValues.put(existingUnion, maxValues.get(existingUnion));
					}
				
			}
				
				newMaxValues.put(curUnionToAdd, maxValueForCurUnionToAdd);

				maxValues = newMaxValues;
				addEntityToWM(curUnionToAdd);
				log("Binding algorithm finished!");
				long finalTime = System.currentTimeMillis();
				log("Total binding time: " + (finalTime - initTime)/1000.0 + " seconds");
				log("--------STOP BINDING----------");
 		}
		catch (Exception e) {
			e.printStackTrace();
		}
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
	
	
	
	private void addEntityToWM(Union entity) {

		try {
			addToWorkingMemory(entity.entityID, entity);
			log("new Union succesfully added to the binding working memory");
			log("Union ID: " + entity.entityID);
			log("Union has " + entity.includedProxies.length + " included proxies");

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
}
