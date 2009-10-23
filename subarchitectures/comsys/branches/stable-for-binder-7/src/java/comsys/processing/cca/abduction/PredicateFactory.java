package comsys.processing.cca.abduction;

import Abducer.FunctionTerm;
import Abducer.Predicate;
import Abducer.Term;
import Abducer.VariableTerm;

public abstract class PredicateFactory {

	/**
	 * Return a new predicate.
	 * 
	 * @param predSym predicate symbol
	 * @param args arguments (terms)
	 * @return Predicate the predicate
	 */
	public static Predicate predicate(String predSym, Term[] args) {
		Predicate p = new Predicate();
		p.predSym = predSym;
		p.args = args;
		return p;
	}
	
	/**
	 * Return a two-place predicate.
	 * 
	 * @param predSym predicate symbol
	 * @param arg1 first argument
	 * @param arg2 second argument
	 * @return the predicate
	 */
	public static Predicate twoPlacePredicate(String predSym, String arg1, String arg2) {
		return predicate(predSym, new Term[] { term(arg1), term(arg2)});
	}
	
	/**
	 * Return a function term.
	 * 
	 * @param functor term functor
	 * @param args arguments (terms)
	 * @return FunctionTerm the term
	 */
	public static FunctionTerm term(String functor, Term[] args) {
		FunctionTerm f = new FunctionTerm();
		f.type = Abducer.TermType.Function;
		f.functor = functor;
		f.args = args;
		return f;
	}

	/**
	 * Return a function term with no arguments.
	 * 
	 * @param functor term functor
	 * @return FunctionTerm the term
	 */
	public static FunctionTerm term(String functor) {
		return term(functor, new Term[0]);
	}
	
	/**
	 * Return a named variable.
	 * 
	 * @param name variable name
	 * @return VariableTerm the term
	 */
	public static VariableTerm var(String name) {
		VariableTerm v = new VariableTerm();
		v.type = Abducer.TermType.Variable;
		v.name = name;
		return v;
	}

}
