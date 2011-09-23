package de.dfki.lt.tr.dialogue.ref.impl.discourse;

import de.dfki.lt.tr.dialogue.ref.ReferenceResolver;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.slice.discourse.DialogueMove;
import java.util.LinkedList;
import java.util.Stack;

public class DiscourseResolver implements ReferenceResolver {

	private final Stack<DialogueMove> dst;

	public DiscourseResolver() {
		dst = new Stack<DialogueMove>();
	}

	@Override
	public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr) {
		ReferenceResolutionResult result = newEmptyResolutionResult(rr);
		if (rr.sort.equals(SORT_DISCOURSE)) {
			dFormula referent = getTopReferent();
			if (referent != null) {
				EpistemicReferenceHypothesis hypo = new EpistemicReferenceHypothesis(newSharedEpistemicStatus(IntentionManagementConstants.thisAgent, IntentionManagementConstants.humanAgent), referent, 0.9F);
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

	public static SharedEpistemicStatus newSharedEpistemicStatus(String ag1, String ag2) {
		SharedEpistemicStatus epst = new SharedEpistemicStatus(new LinkedList<String>());
		epst.cgagents.add(ag1);
		epst.cgagents.add(ag2);
		return epst;
	}

	public static ReferenceResolutionResult newEmptyResolutionResult(ReferenceResolutionRequest rr) {
		return new ReferenceResolutionResult(rr.nom, new LinkedList<EpistemicReferenceHypothesis>());
	}

}
