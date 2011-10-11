package de.dfki.lt.tr.cast.dialogue;

import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.dialogue.FakeReferenceResolution.FakeReferenceResolver;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolver;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.util.EpistemicStatusFactory;
import java.util.LinkedList;
import java.util.List;

public class FakeReferenceResolution
extends AbstractReferenceResolutionComponent<FakeReferenceResolver> {

	public FakeReferenceResolution() {
		super();
	}

	@Override
	protected FakeReferenceResolver initResolver() {
		return new FakeReferenceResolver();
	}

	@Override
	protected void onStart() {
		super.onStart();
		for (int i = 0; i < 4; i++) {
			addFakeReferent();
		}
	}

	protected void addFakeReferent() {
		WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(), this.getSubarchitectureID());
		getResolver().addReferent(new WorkingMemoryPointer(wma, dBelief.class.getCanonicalName()));
	}

	public static class FakeReferenceResolver implements ReferenceResolver {

		private List<WorkingMemoryPointer> wmptrs = new LinkedList<WorkingMemoryPointer>();
		
		public void addReferent(WorkingMemoryPointer wmptr) {
			wmptrs.add(wmptr);
		}

		@Override
		public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr, WorkingMemoryAddress origin) {
			ReferenceResolutionResult result = ReferenceUtils.newEmptyResolutionResult(rr, origin, ReferenceResolver.SORT_OBJECT);

			EpistemicStatus epst = EpistemicStatusFactory.newSharedEpistemicStatus(IntentionManagementConstants.thisAgent, IntentionManagementConstants.humanAgent);
			double score = 0.7;
			for (WorkingMemoryPointer wmptr : wmptrs) {
				dFormula referent = new PointerFormula(0, wmptr.address, wmptr.type);
				result.hypos.add(new EpistemicReferenceHypothesis(epst, referent, score));
				score *= 0.85;
			}

			return result;
		}
		
	}

}
