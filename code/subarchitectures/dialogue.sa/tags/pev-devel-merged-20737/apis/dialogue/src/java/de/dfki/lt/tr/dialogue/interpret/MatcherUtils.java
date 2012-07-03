package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import java.util.LinkedList;
import java.util.List;

public class MatcherUtils {

	public static WorkingMemoryAddress parseTermToWorkingMemoryAddress(Term t) throws TermParsingException {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			WorkingMemoryAddress wma = ConversionUtils.termToWorkingMemoryAddress(t);
			if (wma != null) {
				return wma;
			}
			else {
				throw new TermParsingException("WorkingMemoryAddress", t);
			}
		}
		else {
			return null;
		}
	}

	public static Boolean parseTermToBoolean(Term t) throws TermParsingException {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			if (isConstTerm(ft)) {
				String functor = ft.functor;
				if (functor.equals("yes")) {
					return true;
				}
				else if (functor.equals("no")) {
					return false;
				}
				else {
					throw new TermParsingException("Boolean", t);
				}
			}
			else {
				throw new TermParsingException("Boolean", t);
			}
		}
		else {
			return null;
		}
	}

	public static String parseTermToString(Term t) throws TermParsingException {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			if (isConstTerm(ft)) {
				return ft.functor;
			}
			else {
				throw new TermParsingException("String", t);
			}
		}
		else {
			return null;
		}
	}

	private static boolean isConstTerm(FunctionTerm ft) {
		return ft.args.isEmpty();
	}

	public static dFormula parseTermToDFormula(Term t) throws TermParsingException {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			dFormula value = ConversionUtils.uniTermToFormula(ft);
			if (value != null) {
				return value;
			}
			else {
				throw new TermParsingException("dFormula", t);
			}
		}
		else {
			return null;
		}
	}

	public static EpistemicStatus parseTermToEpistemicObject(Term t) throws TermParsingException {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			EpistemicStatus epst = ConversionUtils.termToEpistemicStatus(ft);
			if (epst != null) {
				return epst;
			}
			else {
				throw new TermParsingException("EpistemicStatus", t);
			}
		}
		else {
			return null;
		}
	}

	public static List<String> parseTermToListOfStrings(Term t) throws TermParsingException {
		List<Term> listTerms = ConversionUtils.listTermToListOfTerms(t);
		if (listTerms != null) {
			List<String> strings = new LinkedList<String>();
			for (Term tt : listTerms) {
				String s = parseTermToString(tt);
				if (s != null) {
					strings.add(s);
				}
				else {
					throw new TermParsingException("List<String>", t);
				}
			}
			return strings;
			
		}
		else {
			return null;
		}
	}

}
