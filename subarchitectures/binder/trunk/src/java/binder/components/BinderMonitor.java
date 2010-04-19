package binder.components;

import beliefmodels.autogen.beliefs.AttributedBelief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.beliefs.SharedBelief;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import binder.gui.BeliefGraph;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
    
 
/**
 * Simple class to demonstrate how to monitor the working memory for new changes 
 * 
 * @author plison
 *
 */

public class BinderMonitor extends ManagedComponent {
	
	BeliefGraph beliefGraph = new BeliefGraph();

 
	public void start() {
		
		System.out.println("Entering start");
		
		beliefGraph.init();
		
		//percept belief
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						

						newPerceptBeliefAdded(_wmc);
						
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						perceptBeliefUpdated(_wmc);
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						perceptBeliefDeleted(_wmc);
					}
				}
		);
		
		//perceptunion
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						unionBeliefAdded(_wmc);
					}
				}
		);
		
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptUnionBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
					}
				}
		);
		
		//multimodel
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<MultiModalBelief> beliefData = getMemoryEntryWithData(_wmc.address,
									MultiModalBelief.class);
							MultiModalBelief newmBelief = beliefData.getData();
							
							beliefGraph.createGraph(newmBelief, _wmc.address);
						
							
						} catch (DoesNotExistOnWMException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(MultiModalBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
					}
				}
		);
		
		//temporal
		
		
		
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					
						try {
							CASTData<TemporalUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address,
									TemporalUnionBelief.class);
							TemporalUnionBelief newtBelief = beliefData.getData();
							beliefGraph.createGraph(newtBelief, _wmc.address);
							
						} catch (DoesNotExistOnWMException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
						
						
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(TemporalUnionBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
					}
				}
		);
		
		//stable
		

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<StableBelief> beliefData = getMemoryEntryWithData(_wmc.address,
									StableBelief.class);
							
							StableBelief newSBelief = beliefData.getData();
							
							beliefGraph.createGraph(newSBelief, _wmc.address);

						} catch (DoesNotExistOnWMException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
						
						
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
					}
				}
		);
		
		//attributed
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(AttributedBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<AttributedBelief> beliefData = getMemoryEntryWithData(_wmc.address,
									AttributedBelief.class);
							
							AttributedBelief newABelief = beliefData.getData();
							
							beliefGraph.createGraph(newABelief, _wmc.address);

						} catch (DoesNotExistOnWMException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
						
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(AttributedBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(AttributedBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					}
				}
		);
		
		//Shared
		
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(SharedBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<SharedBelief> beliefData = getMemoryEntryWithData(_wmc.address,
									SharedBelief.class);
							
							SharedBelief newABelief = beliefData.getData();
							
							beliefGraph.createGraph(newABelief, _wmc.address);

						} catch (DoesNotExistOnWMException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
						
						
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(SharedBelief.class,
						WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					}
				}
		);
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(SharedBelief.class,
						WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					}
				}
		);
		
		beliefGraph.display();		
		
		
		
	}  
	
	
	protected void unionBeliefAdded(WorkingMemoryChange _wmc) {
		try {
			CASTData<PerceptUnionBelief> beliefData = getMemoryEntryWithData(_wmc.address,
					PerceptUnionBelief.class);
			PerceptUnionBelief newBelief = beliefData.getData();
			
			beliefGraph.createGraph(newBelief, _wmc.address);
			
			// here you can do what you want with the belief
			
		}
		 catch (DoesNotExistOnWMException e) {
				e.printStackTrace();
			}
		 catch (UnknownSubarchitectureException e) {	
			e.printStackTrace();
		} 
		
	}


	protected void perceptBeliefDeleted(WorkingMemoryChange _wmc) {
		// TODO Auto-generated method stub
		

		try {
			CASTData<PerceptBelief> beliefData = getMemoryEntryWithData(_wmc.address,
					PerceptBelief.class);
			PerceptBelief newpBelief = beliefData.getData();
			// here you can do what you want with the belief

			beliefGraph.deleteGraph(newpBelief);
			
			
		}
		 catch (DoesNotExistOnWMException e) {
				e.printStackTrace();
			}
		 catch (UnknownSubarchitectureException e) {	
			e.printStackTrace();
		}
		
	}


	protected void perceptBeliefUpdated(WorkingMemoryChange _wmc) {
		// TODO Auto-generated method stub
		
	}


	private void newPerceptBeliefAdded(WorkingMemoryChange _wmc) {
		// TODO Auto-generated method stub
		
		try {
			CASTData<PerceptBelief> beliefData = getMemoryEntryWithData(_wmc.address,
					PerceptBelief.class);
			PerceptBelief newpBelief = beliefData.getData();
			// here you can do what you want with the belief
			System.out.println("adding Percept Belief");

			beliefGraph.createGraph(newpBelief,_wmc.address);
			
			
			
		}
		 catch (DoesNotExistOnWMException e) {
				e.printStackTrace();
			}
		 catch (UnknownSubarchitectureException e) {	
			e.printStackTrace();
		}
	}
	

}
