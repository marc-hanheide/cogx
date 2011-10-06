package eu.cogx.goals.george;

import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;

public class AnswerQuestionMotiveGenerator extends ManagedComponent {

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				BaseIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						newInterpretedIntention(
								_wmc.address,
								getMemoryEntry(_wmc.address,
										BaseIntention.class));

					}
				});
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(BaseIntention.class), new WorkingMemoryChangeReceiver() {
			
			@Override
			public void workingMemoryChanged(WorkingMemoryChange arg0)
					throws CASTException {
				println("SEEEEEEEEEEEEEN INTENTION");
				
			}
		});
	}

	private void newInterpretedIntention(WorkingMemoryAddress _intentionAddr,
			BaseIntention _intention) {
		println("newInterpretedIntention");

		println("stringContent");
		for (String key : _intention.stringContent.keySet()) {
			println(key + " -> " + _intention.stringContent.get(key));
		}
		
		println("");
		println("addressContent");
		for (String key : _intention.addressContent.keySet()) {
			println(key + " -> " + CASTUtils.toString(_intention.addressContent.get(key)));
		}
	}
	
	

}
