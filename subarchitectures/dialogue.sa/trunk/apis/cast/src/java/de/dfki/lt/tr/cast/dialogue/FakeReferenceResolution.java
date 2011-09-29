package de.dfki.lt.tr.cast.dialogue;

import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
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

public class FakeReferenceResolution
extends AbstractReferenceResolutionComponent<FakeReferenceResolver> {

	public FakeReferenceResolution() {
		super(new FakeReferenceResolver());
	}

	@Override
	protected void onStart() {
		super.onStart();
		WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(), this.getSubarchitectureID());
		getResolver().setReferent(new WorkingMemoryPointer(wma, dBelief.class.getCanonicalName()));
	}

	public static class FakeReferenceResolver implements ReferenceResolver {

		private WorkingMemoryPointer wmptr = null;
		
		public void setReferent(WorkingMemoryPointer wmptr) {
			this.wmptr = wmptr;
		}

		@Override
		public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr, WorkingMemoryAddress origin) {
			ReferenceResolutionResult result = ReferenceUtils.newEmptyResolutionResult(rr, origin, "fake");

			dFormula referent = new PointerFormula(0, wmptr.address, wmptr.type);
			EpistemicReferenceHypothesis hypo = new EpistemicReferenceHypothesis(EpistemicStatusFactory.newSharedEpistemicStatus(IntentionManagementConstants.humanAgent, IntentionManagementConstants.thisAgent), referent, 0.9F);
			result.hypos.add(hypo);

			return result;
		}
		
	}

}
