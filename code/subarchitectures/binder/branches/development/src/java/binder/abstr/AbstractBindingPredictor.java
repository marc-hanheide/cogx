package binder.abstr;

import java.util.Enumeration;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
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
	static Vector<Union> predictedUnions = new Vector<Union>();


	protected Vector<Union> getPredictedUnions (PhantomProxy phantomProxy, boolean deleteProxyAfterBinding) {
		try {

			predictedUnions = new Vector<Union>();

			addPhantomProxyToWM (phantomProxy);

			while (predictedUnions.size() == 0) {
				sleepComponent(20);
			}
			
			log("Predicted union for phantom proxy is sucessfully retrieved");

			if (deleteProxyAfterBinding) {
				deleteEntityInWM(phantomProxy);
			}

			return predictedUnions;
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}


	
	protected Union getBestPredictedUnion 
	(PhantomProxy phantomProxy, boolean deleteProxyAfterBinding) {
		return getMaximum(getPredictedUnions(phantomProxy, deleteProxyAfterBinding));
	}
	
	

	public Union getMaximum (Vector<Union> unions) {

		Union maxUnion = new Union();
		float maxValue = -1.0f;

		for (Enumeration<Union> e = unions.elements(); e.hasMoreElements() ; ) {
			Union curU = e.nextElement();
	//		log("confidence score for "  + curU.entityID + " is: "  + curU.probExists);
			if (curU.probExists > maxValue) {
				maxValue = curU.probExists;
				maxUnion = curU;
			}
		}
		return maxUnion;
	}

	private void addPhantomProxyToWM (PhantomProxy phantomProxy) {

		lastPhantomProxy = phantomProxy;

		receiverForPhantomProxies =  new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					AlternativeUnionConfigurations configs = 
						getMemoryEntry(_wmc.address, AlternativeUnionConfigurations.class);
					
					Vector<Union> newPredictedUnions = new Vector<Union>();
					for (int k = 0 ; k < configs.alterconfigs.length ; k++) {
						UnionConfiguration curConfig = configs.alterconfigs[k];
						for (int i = 0 ; i < curConfig.includedUnions.length ; i++) {
							Union u = curConfig.includedUnions[i];
							for (int j = 0 ; j < u.includedProxies.length ; j++) {
								if (u.includedProxies[j].equals(lastPhantomProxy)) {
									newPredictedUnions.add(u);
								}
							}
						}
					}
					predictedUnions = newPredictedUnions;
					if (predictedUnions.size() > 0) {
					removeChangeFilter(receiverForPhantomProxies);
					}
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		};
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(AlternativeUnionConfigurations.class,
				WorkingMemoryOperation.WILDCARD), receiverForPhantomProxies);
		addProxyToWM (phantomProxy);
	}

}
