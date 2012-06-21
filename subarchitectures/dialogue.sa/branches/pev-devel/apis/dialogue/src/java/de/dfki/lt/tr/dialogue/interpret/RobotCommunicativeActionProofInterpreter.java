package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.util.ProofUtils;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import org.apache.log4j.Logger;

public class RobotCommunicativeActionProofInterpreter
extends AbstractWellFormedTestingProofInterpreter<RobotCommunicativeAction> {

	public RobotCommunicativeActionProofInterpreter(Logger logger) {
		super(logger);
	}

	@Override
	protected RobotCommunicativeAction interpretWithoutWellFormednessTest(List<ModalisedAtom> matoms, double cost) {

		RobotCommunicativeAction ra = new RobotCommunicativeAction();

		List<ModalisedAtom> events = ProofUtils.filterStripByModalityPrefix(matoms,
				new ArrayList<Modality>(Arrays.asList(new Modality[] {Modality.Event})));

		LogicalForm lf = null;
//		NominalReference nr = null;

		if (events.size() == 1 && events.get(0).a.predSym.equals("produce_text") && events.get(0).a.args.size() == 1) {
//			List<Term> terms = ConversionUtils.listTermToListOfTerms(events.get(0).a.args.get(0));
			List<Term> terms = flattenListOfTerms(events.get(0).a.args.get(0));

			List<String> words = new LinkedList<String>();
			for (Term t : terms) {
				String word = null;
				if (t instanceof FunctionTerm) {
					word = ((FunctionTerm) t).functor;
				}

				if (word == null) {
					return null;
				}
				else {
					words.add(word.replace(" ", "_"));
				}
			}

			String cannedString = join("_", words);
			if (cannedString != null && !cannedString.equals("")) {
				lf = LFUtils.convertFromString("@d:dvp(<CannedText>" + cannedString + ")");
			}
		}
		else {
			lf = AbducerUtils.factsToLogicalForm(matoms, "dn1_1");
		}

		ra.setProtoLF(lf);
		return ra;
	}

	public static List<Term> flattenListOfTerms(Term t) {
		List<Term> result = new LinkedList<Term>();

		List<Term> asList = ConversionUtils.listTermToListOfTerms(t);
		if (asList != null) {
			for (Term tt : asList) {
				result.addAll(flattenListOfTerms(tt));
			}
		}
		else {
			result.add(t);
		}

		return result;
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
