package binder.components;


import beliefmodels.adl.Belief;
import beliefmodels.adl.BeliefModel;
import binder.utils.BeliefModelUtils;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;


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
					Belief belief = getMemoryEntry(_wmc.address, Belief.class);
					log("Updated belief : \n" + BeliefModelUtils.getBeliefPrettyPrint(belief, 1));
					log("----------------------");
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
					log("Updated belief model: \n" + BeliefModelUtils.getBeliefModelPrettyPrint(bmodel, 1));
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		
		log("Binding Monitor successfully started");
	}
}
