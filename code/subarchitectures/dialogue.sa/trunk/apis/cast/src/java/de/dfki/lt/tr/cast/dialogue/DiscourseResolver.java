package de.dfki.lt.tr.cast.dialogue;

import de.dfki.lt.tr.dialogue.ref.ReferenceResolver;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.ref.Constraint;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.slice.discourse.DialogueMove;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Stack;

public class DiscourseResolver implements ReferenceResolver {

	private final Stack<DialogueMove> dst;

	public DiscourseResolver() {
		dst = new Stack<DialogueMove>();
	}

	@Override
	public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr) {
		Map<String, String> constraints = constraintsToMap(rr.constraints);
		ReferenceResolutionResult result = newEmptyResolutionResult(rr);
		if (constraints.get("Type").equals("it")) {
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
		DialogueMove top = dst.peek();
		if (top != null) {
			if (top.topic != null) {
				return top.topic.referent;
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

	public static Map<String, String> constraintsToMap(List<Constraint> constraints) {
		Map<String, String> result = new HashMap<String, String>();
		for (Constraint c : constraints) {
			assert result.put(c.feature, c.value) == null;
		}
		return result;
	}

	public static ReferenceResolutionResult newEmptyResolutionResult(ReferenceResolutionRequest rr) {
		return new ReferenceResolutionResult(rr.nom, new LinkedList<EpistemicReferenceHypothesis>());
	}

}
