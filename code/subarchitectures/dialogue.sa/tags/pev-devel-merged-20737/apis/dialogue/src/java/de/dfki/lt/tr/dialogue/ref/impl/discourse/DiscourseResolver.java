package de.dfki.lt.tr.dialogue.ref.impl.discourse;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolver;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.slice.discourse.DialogueMove;
import de.dfki.lt.tr.dialogue.util.EpistemicStatusFactory;
import java.util.Stack;

public class DiscourseResolver implements ReferenceResolver {

	private static final String METHOD_ID = "discourse";
	private final Stack<DialogueMove> dst;

	public DiscourseResolver() {
		dst = new Stack<DialogueMove>();
	}

	@Override
	public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr, WorkingMemoryAddress origin) {
		ReferenceResolutionResult result = ReferenceUtils.newEmptyResolutionResult(rr, origin, METHOD_ID);
		if (rr.sort.equals(SORT_DISCOURSE)) {
			dFormula referent = getTopReferent();
			if (referent != null) {
				EpistemicReferenceHypothesis hypo = new EpistemicReferenceHypothesis(EpistemicStatusFactory.newSharedEpistemicStatus(IntentionManagementConstants.thisAgent, IntentionManagementConstants.humanAgent), referent, 0.9F);
				result.hypos.add(hypo);
			}
		}
		return result;
	}

	public void addDialogueMove(DialogueMove dm) {
		dst.push(dm);
	}

	protected dFormula getTopReferent() {
		if (!dst.empty()) {
			DialogueMove top = dst.peek();
			if (top != null) {
				if (top.topic != null) {
					return top.topic.referent;
				}
			}
		}
		return null;
	}

}
