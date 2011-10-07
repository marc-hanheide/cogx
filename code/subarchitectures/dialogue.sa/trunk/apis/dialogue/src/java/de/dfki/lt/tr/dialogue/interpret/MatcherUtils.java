package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.Term;

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

}
