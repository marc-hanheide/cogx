package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.ref.Constraint;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.slice.lf.Feature;
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.slice.time.Interval;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
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

		s += rr.nom + " -> {\n";
		for (EpistemicReferenceHypothesis hypo : rr.hypos) {
			s += "  (" + BeliefIntentionUtils.dFormulaToString(hypo.referent) + "  | " + BeliefIntentionUtils.epistemicStatusToString(hypo.epst)+ ") = " + hypo.score + "\n";
		}
		s += "}";

		return s;
	}

	public static ReferenceResolutionRequest nominalToConstraints(LFNominal nom, TimeInterval ival) {
		List<Constraint> cs = new ArrayList<Constraint>();
		Iterator<Feature> iter = LFUtils.lfNominalGetFeatures(nom);
		log("converting nominal to constraints");

		String sort = nom.sort;

		if (!nom.prop.prop.equals("context")) {
			String type = nom.prop.prop;
			cs.add(new Constraint("Type", type));
		}

		while (iter.hasNext()) {
			Feature feat = iter.next();
			Constraint c = new Constraint(feat.feat, feat.value);
			cs.add(c);
		}
		log("conversion done");
		return new ReferenceResolutionRequest(nom.nomVar, sort, cs, ival.toIce());
	}

	private static void log(String s) {
		logger.debug(s);
	}

}