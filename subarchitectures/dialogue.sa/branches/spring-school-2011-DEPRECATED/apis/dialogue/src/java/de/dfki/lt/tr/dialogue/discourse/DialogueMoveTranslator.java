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
	NominalReference latestTopic = null;
//	FunctionTerm lastTopic = null;

	public void addDialogueMove(DialogueMove dm) {
		NominalReference nr = dm.topic;
		if (dm.lform != null) {
			String s = "%   ";
			s += "(dm " + dm.agent + " " + dm.lform.root.nomVar + " (topic ";
			if (nr != null) {
				s += "(ref " + nr.nominal + " " + MercuryUtils.termToString(ConversionUtils.stateFormulaToTerm(nr.referent)) + ")";
				latestTopic = nr;
			}
			else {
				s += "NULL";
			}
			s += "))";

			lines.add(s);
		}
	}

	public String toRulefileContents() {
		String topS = "";
		if (latestTopic != null) {
			FunctionTerm topicTerm = (FunctionTerm) ConversionUtils.stateFormulaToTerm(latestTopic.referent);
			topS = "att : linguisticsalience(" + MercuryUtils.termToString(topicTerm) + ", high).\n";
		}
		return "\n" + "% discourse structure\n" + join("\n", lines) + "\n\n" + "% latest topic\n" + topS;
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
