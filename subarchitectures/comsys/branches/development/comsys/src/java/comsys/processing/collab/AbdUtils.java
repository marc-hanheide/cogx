package comsys.processing.collab;

import java.util.*;

import comsys.datastructs.lf.*;
import comsys.lf.utils.LFUtils;
import Abducer.*;

public class AbdUtils {

	/** Return a new modalised formula.
	 * 
	 * @param ms list of modalities, m1...mn
	 * @param p the formula to be modalised, p(t1...tn)
	 * @return ModalisedFormula the modalised formula, "m1...mn:p(t1...tn)"
	 */
	public static ModalisedFormula modalisedFormula(Modality[] ms, Predicate p) {
		ModalisedFormula mf = new ModalisedFormula();
		mf.m = ms;
		mf.p = p;
		return mf;
	}
	
	/** Return a new predicate.
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
	
	/** Return a function term.
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

	/** Return a function term with no arguments.
	 * 
	 * @param functor term functor
	 * @return FunctionTerm the term
	 */
	public static FunctionTerm term(String functor) {
		return term(functor, new Term[0]);
	}
	
	/** Return a named variable.
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

	public static InfoModality infoModality() {
		InfoModality m = new InfoModality();
		m.type = ModalityType.Info;
		return m;
	}
	
	public static EventModality eventModality() {
		EventModality m = new EventModality();
		m.type = ModalityType.Event;
		return m;
	}
	
	public static AttStateModality attStateModality() {
		AttStateModality m = new AttStateModality();
		m.type = ModalityType.AttState;
		return m;
	}

	/** Convert a logical form to an array of modalised formulas.
	 * 
	 * @param modality prefixed to every fact
	 * @param lf
	 * @return set of corresponding facts
	 */
	public static ModalisedFormula[] lfToFacts(Modality[] modality, LogicalForm lf) {
		ArrayList<ModalisedFormula> facts = new ArrayList<ModalisedFormula>();
		for (int i=0; i < lf.noms.length; i++) {
			LFNominal nom = lf.noms[i];
			addNomToFactList(facts, modality, lf.noms[i], lf);
		}
		return facts.toArray(new ModalisedFormula[0]);
	}

	private static void addNomToFactList(AbstractList<ModalisedFormula> facts, Modality[] factModality, LFNominal nom, LogicalForm lf) {
		// nominal term
		Term nomTerm = term(nom.nomVar);
		
		// sort
		facts.add(modalisedFormula(
				factModality,
				predicate("sort", new Term[] {
					nomTerm,
					term(nom.sort)
				})));

		// proposition, if there is one
		if (!nom.prop.prop.equals("")) {
			facts.add(modalisedFormula(
					factModality,
					predicate("prop", new Term[] {
						nomTerm,
						term(nom.prop.prop)
					})));		
		}

		// features
		Iterator fIter = LFUtils.lfNominalGetFeatures(nom);
		while (fIter.hasNext()) { 
			Feature feat = (Feature) fIter.next(); 
			facts.add(modalisedFormula(
					factModality,
					predicate("feat_" + feat.feat, new Term[] {
						nomTerm,
						term(feat.value)
					})));
		}

		Iterator rIter = LFUtils.lfNominalGetRelations(nom); 
		while (rIter.hasNext()) {
			LFRelation rel = (LFRelation) rIter.next();
			facts.add(modalisedFormula(
					factModality,
					predicate("rel_" + rel.mode, new Term[] {
						nomTerm, 
						term(rel.dep)
					})));
/*
			LFNominal depnom = LFUtils.lfGetNominal(lf, rel.dep); 
			if(rel.coIndexedDep == true){
				// we only want to generate <RelMode>var1:type1, not the complete nominal
				facts.add(modalisedFormula(
						factModality,
						predicate("sort", new Term[] {
							term(rel.dep),
							term(depnom.sort)
						})));
			} else {
				addNomToFactList(facts, depnom, lf);
			}
*/
		}
	}
}
