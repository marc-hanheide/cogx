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
	
	HashMap<String,Union> curBestUnions;
	HashMap<Union, Float> maxValues;
	
	@Override
	public void start() {
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Proxy.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
				proxyUpdate();
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		curBestUnions = new HashMap<String,Union>();
		maxValues = new HashMap<Union,Float>();
		
		BNManager = new BayesianNetworkManager();
		
		log("Binding Monitor successfully started");
	}
	
	
	public Union constructNewUnion(Vector<Proxy> includedProxies) {
	//	log("***** Constructing a new union ****");
		Union union = new Union();
		union.entityID = newDataID();
		union.includedProxies =new Proxy[includedProxies.size()];
		union.includedProxies = includedProxies.toArray(union.includedProxies);
		
		Vector<Feature> features = new Vector<Feature>();
		union.probExists = 0.0f;
		for (Enumeration<Proxy> e = includedProxies.elements(); e.hasMoreElements();) {
			Proxy prox = e.nextElement();
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
	//	log("number of included proxies: " + union.includedProxies.length);
		
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
		
//		log("Maximum for final distribution of the union: " + GradientDescent.getMaximum(finalDistrib));
		return finalDistrib;
	}
	
	public HashMap<String,Union> getInitialUnions(Vector<Proxy> proxies) {
				
		for (Enumeration<Proxy> e = proxies.elements(); e.hasMoreElements() ; ) {
			Proxy curProxy = e.nextElement();
			if (!curBestUnions.containsKey(curProxy.entityID)) {
				Vector<Proxy> curProxyV = new Vector<Proxy>();
				curProxyV.add(curProxy);
				Union union = constructNewUnion(curProxyV);
				curBestUnions.put(curProxy.entityID, union);
				maxValues.put(union, GradientDescent.getMaximum(union.distribution));
		}
		}
		
		return curBestUnions;
	}
	 
	
	public void deleteOldUnions() {
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
	
	
	
	
	public void proxyUpdate() {
		log("------------------");
		log("binder working memory updated with a new proxy!");
		try {
			CASTData<Proxy>[] proxies = getWorkingMemoryEntries(Proxy.class);
			log("Current number of proxies in WM: " + proxies.length);
			
			Vector<Proxy> proxiesV = new Vector<Proxy>();
			HashMap<String,String> proxySubarchs = new HashMap<String,String>();
			for (int i = 0; i < proxies.length; i++) {
				proxiesV.add(proxies[i].getData());
				proxySubarchs.put(proxies[i].getData().entityID, proxies[i].getData().subarchId);
			}
			
			curBestUnions = getInitialUnions(proxiesV);
			
			String[] keyArray = new String[curBestUnions.size()];
			keyArray = curBestUnions.keySet().toArray(keyArray);
			
			for (int i = 0; i < (keyArray.length - 1 ); i++ ) {
				String proxyID1 = keyArray[i];
				for (int j = i+1 ; j < keyArray.length ; j++) {
					String proxyID2 = keyArray[j];
					if (!proxyID1.equals(proxyID2) && !proxySubarchs.get(proxyID1).equals(proxySubarchs.get(proxyID2))) {
						Union union1 = curBestUnions.get(proxyID1);
						Union union2 = curBestUnions.get(proxyID2);
						Vector<Proxy> unions = new Vector<Proxy>();
						unions.add(union1.includedProxies[0]);
						unions.add(union2.includedProxies[0]);
						Union newUnion = constructNewUnion(unions);
						
						float maxNewUnion = GradientDescent.getMaximum(newUnion.distribution);
						maxValues.put(newUnion, maxNewUnion);
						
						if (maxNewUnion > maxValues.get(union1) && maxNewUnion > maxValues.get(union2)) {
							
						deleteUnionFromWM(curBestUnions.get(proxyID1));
						deleteUnionFromWM(curBestUnions.get(proxyID2));
						
						curBestUnions.put(proxyID1, newUnion);
						curBestUnions.put(proxyID2, newUnion);
						}
					}
				}
			}
						
			for (Iterator<String> e = curBestUnions.keySet().iterator() ; e.hasNext() ; ) {
				String proxyID = e.next();
				Union union = curBestUnions.get(proxyID);
				if (! unionAlreadyOnWM(union)) {
					addEntityToWM(union);				
				}
				
			}
 		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	

	private boolean unionAlreadyOnWM(Union union) {
		try {

			CASTData<Union>[] unions = getWorkingMemoryEntries(Union.class);
				
			for (int i = 0 ; i < unions.length ; i++) {
				if (unions[i].getData().equals(union)) {
					return true;
				}
			}
			
			for (int k = 0 ; k < union.includedProxies.length ; k++) {

				for (int i = 0 ; i < unions.length ; i++) {
					Union otherUnion = unions[i].getData();
					if (otherUnion.equals(union)) {
						return true;
					}
					for (int j = 0 ; j < otherUnion.includedProxies.length ; j++) {
						if (otherUnion.includedProxies[j].entityID.equals(union.includedProxies[k].entityID)) {
							return true;
						}
					}
				}

			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return false;
	}
	
	
	private void addEntityToWM(PerceivedEntity entity) {

		try {
			addToWorkingMemory(entity.entityID, entity);
			log("new Union succesfully added to the binding working memory");

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
}
