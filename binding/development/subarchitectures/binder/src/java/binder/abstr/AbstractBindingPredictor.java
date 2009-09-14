package binder.abstr;

import java.util.HashMap;

import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.specialentities.PhantomProxy;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class AbstractBindingPredictor extends BindingWorkingMemoryWriter {

	static PhantomProxy lastPhantomProxy = new PhantomProxy();
	static WorkingMemoryChangeReceiver receiverForPhantomProxies;	
	static HashMap<String, Union> predictedUnions = new HashMap<String, Union>();



	protected Union getPredictedUnion (PhantomProxy phantomProxy, boolean deleteProxyAfterBinding) {
		try {

			addPhantomProxyToWM (phantomProxy);

			while (!predictedUnions.containsKey(phantomProxy.entityID)) {
				sleepComponent(20);
			}
			log("Predicted union for phantom proxy is sucessfully retrieved");

			if (deleteProxyAfterBinding) {
				deleteEntityInWM(phantomProxy);
			}
			
			return predictedUnions.get(phantomProxy.entityID);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}



	private void addPhantomProxyToWM (PhantomProxy phantomProxy) {

		lastPhantomProxy = phantomProxy;

		receiverForPhantomProxies =  new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					UnionConfiguration config = 
						getMemoryEntry(_wmc.address, UnionConfiguration.class);
					for (int i = 0 ; i < config.includedUnions.length ; i++) {
						Union u = config.includedUnions[i];
						for (int j = 0 ; j < u.includedProxies.length ; j++) {
							if (u.includedProxies[j].equals(lastPhantomProxy)) {
								predictedUnions.put(lastPhantomProxy.entityID, u);
								removeChangeFilter(receiverForPhantomProxies);
							}
						}
					}
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		};
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(UnionConfiguration.class,
				WorkingMemoryOperation.WILDCARD), receiverForPhantomProxies);
		addProxyToWM (phantomProxy);
	}

}
