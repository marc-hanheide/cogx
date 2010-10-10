package de.dfki.lt.tr.dialogue.discourse;

import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.slice.discourse.DialogueMove;
import de.dfki.lt.tr.dialogue.slice.ref.NominalReference;
import de.dfki.lt.tr.infer.weigabd.MercuryUtils;
import de.dfki.lt.tr.infer.weigabd.TermAtomFactory;
import de.dfki.lt.tr.infer.weigabd.slice.FunctionTerm;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public class DialogueMoveTranslator {

	List<String> lines = new LinkedList<String>();
	FunctionTerm currentTopic = null;

	public void addDialogueMove(DialogueMove dm) {
		NominalReference nr = dm.topic;
		String s = "% ";
		String s2 = "";
		s += "(dm " + dm.agent + " " + dm.lform.root.nomVar + " (topic ";
		if (nr != null) {
			s += "(ref " + nr.nominal + " " + MercuryUtils.termToString(ConversionUtils.stateFormulaToTerm(nr.referent)) + ")";
			if (currentTopic == null) {
				// the latest topic!
				currentTopic = (FunctionTerm) ConversionUtils.stateFormulaToTerm(nr.referent);
				s2 = "att : linguisticsalience(" + MercuryUtils.termToString(currentTopic) + ", high).";
			}
		}
		else {
			s += "NULL";
		}
		s += "))";

		lines.add(s);
		if (!s2.isEmpty()) {
			lines.add(s2);
		}
	}

	public String toRulefileContents() {
		return "\n" + join("\n", lines) + "\n";
	}

	// Sun should burn in hell for not having such a function in the standard library!
	public static String join(String separator, List<String> args) {
	    StringBuilder sb = new StringBuilder();
	    if (args != null) {
			Iterator<String> iter = args.iterator();
			while (iter.hasNext()) {
				sb.append(iter.next());
				if (iter.hasNext()) {
					sb.append(separator);
				}
			}
		}
		return sb.toString();
    }

}
