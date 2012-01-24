package de.dfki.lt.tr.dialogue.ref.util;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.ref.Constraint;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import java.util.LinkedList;
import org.apache.log4j.Logger;

public abstract class ReferenceUtils {

	private static Logger logger = Logger.getLogger("reference-utils");

	public static String resolutionRequestToString(ReferenceResolutionRequest rr) {
		String s = "";

		s += rr.nom + " <- {\n";
		s += "  sort = \"" + rr.sort + "\"\n";
		s += "  constraints = [\n";
		for (Constraint c : rr.constraints) {
			s += "    (\"" + c.feature + "\", \"" + c.value + "\")\n";
		}
		s += "  ]\n";
		s += "}";

		return s;
	}

	public static String resolutionResultToString(ReferenceResolutionResult rr) {
		String s = "";

		s += rr.nom + " (method=\"" + rr.method + "\") -> {\n";
		for (EpistemicReferenceHypothesis hypo : rr.hypos) {
			s += "  (" + BeliefIntentionUtils.dFormulaToString(hypo.referent) + "  | " + BeliefIntentionUtils.epistemicStatusToString(hypo.epst)+ ") = " + hypo.score + "\n";
		}
		s += "}";

		return s;
	}

	public static ReferenceResolutionResult newEmptyResolutionResult(ReferenceResolutionRequest rr, WorkingMemoryAddress origin, String method) {
		return new ReferenceResolutionResult(rr.nom, origin, method, new LinkedList<EpistemicReferenceHypothesis>());
	}

}