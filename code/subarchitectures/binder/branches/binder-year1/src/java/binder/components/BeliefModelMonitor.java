


package binder.components;


import beliefmodels.adl.Belief;
import beliefmodels.adl.BeliefModel;
import beliefmodels.domainmodel.cogx.GroundedBelief;
import binder.utils.BeliefModelUtils;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;


/**
 * Monitors belief models present on the binder working memory (inspection of the WM content 
 * and print the results on screen)
 * 
 * @author Pierre Lison
 * @version 15/09/2009
 */

public class BeliefModelMonitor extends ManagedComponent {

	
	
	/**
	 * Initialisation of the monitor
	 * 
	 * TODO: extend this to AlternativeUnionConfigurations
	 */
	@Override
	public void start() {

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Belief.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					if (existsOnWorkingMemory(_wmc.address)) {
					Belief belief = getMemoryEntry(_wmc.address, Belief.class);
					log("Updated belief : \n" + BeliefModelUtils.getBeliefPrettyPrint(belief, 1));
					log("----------------------");
					}
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(BeliefModel.class,
				WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					BeliefModel bmodel = getMemoryEntry(_wmc.address, BeliefModel.class);
					CASTData<Belief>[] beliefs = getWorkingMemoryEntries(Belief.class);
					log("Number of beliefs: " + beliefs.length);
					log("Updated belief model: \n" + BeliefModelUtils.getBeliefModelPrettyPrint(bmodel, 1));
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		
		log("Binding Monitor successfully started");
	}
	
	
	public void run () {
	/**	while (isRunning()) {
			sleepComponent(500);
			try {
				CASTData<Belief>[] beliefs = getWorkingMemoryEntries(Binder.BINDER_SA, Belief.class);
				log("number of beliefs in WM: " + beliefs.length);
				
				for (int i = 0 ; i < beliefs.length ; i++) {
					log("belief " + i + " " + beliefs[i].getData().id); 
				}
				
				CASTData<GroundedBelief>[] beliefs2 = getWorkingMemoryEntries(Binder.BINDER_SA, GroundedBelief.class);
				log("number of grounded beliefs in WM: " + beliefs2.length);
				
				for (int i = 0 ; i < beliefs2.length ; i++) {
					log("gbelief " + i + " " + beliefs2[i].getData().id); 
				}
				
				CASTData<BeliefModel>[] bmodel = getWorkingMemoryEntries(Binder.BINDER_SA, BeliefModel.class);
				log("number of beliefs in model: " + bmodel[0].getData().k.length);
				
				for (int i = 0 ; i < bmodel[0].getData().k.length ; i++) {
					log("belief in model " + i + " " + bmodel[0].getData().k[i]); 
				}
			}
			catch (Exception e) {
				e.printStackTrace();
			}
		} */
	}
}
