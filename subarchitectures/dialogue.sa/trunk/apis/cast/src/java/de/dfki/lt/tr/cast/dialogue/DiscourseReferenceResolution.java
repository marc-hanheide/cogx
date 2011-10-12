package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionToAct;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.cast.dialogue.DiscourseReferenceResolution.LastMentionedReferenceResolver;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolver;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.util.EpistemicStatusFactory;

public class DiscourseReferenceResolution
extends AbstractReferenceResolutionComponent<LastMentionedReferenceResolver> {

	private WorkingMemoryAddress lastMentioned;

	public DiscourseReferenceResolution() {
		super();
		lastMentioned = null;
	}

	@Override
	protected LastMentionedReferenceResolver initResolver() {
		return new LastMentionedReferenceResolver();
	}

	@Override
	protected void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				BaseIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

							@Override
							public void execute(WorkingMemoryAddress addr) {
								try {
									BaseIntention bint = getMemoryEntry(addr, BaseIntention.class);

									WorkingMemoryAddress aboutAddr = bint.addressContent.get("about");
									if (aboutAddr != null) {
										setLastMentioned(aboutAddr);
									}

								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}
							
						});
					}
				});
	}

	private void setLastMentioned(WorkingMemoryAddress wma) {
		lastMentioned = wma;
	}

	private WorkingMemoryAddress getLastMentioned() {
		return lastMentioned;
	}

	public class LastMentionedReferenceResolver implements ReferenceResolver {

		private final EpistemicStatus epst = EpistemicStatusFactory.newSharedEpistemicStatus(IntentionManagementConstants.thisAgent, IntentionManagementConstants.humanAgent);

		@Override
		public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr, WorkingMemoryAddress origin) {
			ReferenceResolutionResult result = ReferenceUtils.newEmptyResolutionResult(rr, origin, "last-mention");
			if (rr.sort.equals(ReferenceResolver.SORT_DISCOURSE)) {
				WorkingMemoryAddress qudAboutAddr = getLastMentioned();
				if (qudAboutAddr != null) {
					dFormula referent = new PointerFormula(0, qudAboutAddr, "");
					double score = 1.0;
					result.hypos.add(new EpistemicReferenceHypothesis(epst, referent, score));
				}
			}
			return result;
		}
		
	}

}
