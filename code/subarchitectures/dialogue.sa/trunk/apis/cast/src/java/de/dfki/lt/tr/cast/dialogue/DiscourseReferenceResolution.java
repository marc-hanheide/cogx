package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.ref.Constraint;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ResolutionResult;
import de.dfki.lt.tr.dialogue.slice.discourse.DialogueMove;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Stack;

public class DiscourseReferenceResolution
extends AbstractReferenceResolutionComponent {

	private final Stack<DialogueMove> dst;

	public DiscourseReferenceResolution() {
		super();
		dst = new Stack<DialogueMove>();
	}

	@Override
	protected void onStart() {
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				DialogueMove.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleDialogueMove(_wmc);
					}
				});
	}

	private void handleDialogueMove(WorkingMemoryChange _wmc) {
		try {
			DialogueMove dm = getMemoryEntry(_wmc.address, DialogueMove.class);
			dst.push(dm);
		}
		catch (SubarchitectureComponentException e) {
			getLogger().error("exception in handling a new dialogue move", e);
		}
	}

	@Override
	public ResolutionResult resolve(ResolutionRequest rr) {
		Map<String, String> constraints = constraintsToMap(rr.constraints);
		ResolutionResult result = newEmptyResolutionResult(rr);

		if (constraints.get("Type").equals("it")) {
			dFormula referent = getTopReferent();
			if (referent != null) {
				EpistemicReferenceHypothesis hypo = new EpistemicReferenceHypothesis(
					newSharedEpistemicStatus(IntentionManagementConstants.thisAgent, IntentionManagementConstants.humanAgent),
					referent, 0.9f);
				result.hypos.add(hypo);
			}
		}
		
		return result;
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

	public static ResolutionResult newEmptyResolutionResult(ResolutionRequest rr) {
		return new ResolutionResult(rr.nom, new LinkedList<EpistemicReferenceHypothesis>());
	}

	public dFormula getTopReferent() {
		DialogueMove top = dst.peek();
		if (top != null) {
			if (top.topic != null) {
				return top.topic.referent;
			}
		}
		return null;
	}

}
