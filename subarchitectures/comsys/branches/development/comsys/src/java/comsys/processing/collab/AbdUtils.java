package comsys.processing.collab;

import java.util.*;

import comsys.datastructs.lf.*;
import comsys.lf.utils.LFUtils;
import Abducer.*;

public class AbdUtils {

	public static ModalisedFormula modFormula(Modality[] ms, Predicate p) {
		ModalisedFormula mf = new ModalisedFormula();
		mf.m = ms;
		mf.p = p;
		return mf;
	}
	
	public static Predicate predicate(String predSym, Term[] args) {
		Predicate p = new Predicate();
		p.predSym = predSym;
		p.args = args;
		return p;
	}
	
	public static FunctionTerm term(String functor, Term[] args) {
		FunctionTerm f = new FunctionTerm();
		f.type = Abducer.TermType.Function;
		f.functor = functor;
		f.args = args;
		return f;
	}

	public static FunctionTerm term(String functor) {
		return term(functor, new Term[0]);
	}
	
	public static VariableTerm var(String name) {
		VariableTerm v = new VariableTerm();
		v.type = Abducer.TermType.Variable;
		v.name = name;
		return v;
	}

	public static InfoModality modInfo() {
		InfoModality m = new InfoModality();
		m.type = ModalityType.Info;
		return m;
	}
	
	public static EventModality modEvent() {
		EventModality m = new EventModality();
		m.type = ModalityType.Event;
		return m;
	}
	
	public static void addLFAsExplicitFacts(AbducerServerPrx abducer, LogicalForm lf) {
		LFNominal rootnom = lf.root;
		Vector rootDependents = LFUtils.lfCollectNomvars(rootnom,lf);

		addNomToExplicitFacts(abducer, rootnom, lf);
/*
		for (int i=0; i < lf.noms.length; i++) {
			LFNominal nom = lf.noms[i];			
			if (!result.contains(nom.nomVar)) {
//				log("Nominal ["+nom.nomVar+"] not subordinated to the root ["+rootnom.nomVar+"]");
				addNomToExplicitFacts(abducer, nom, lf);
			} else {
//				log("Nominal ["+nom.nomVar+"] subordinated to the root ["+rootnom.nomVar+"]");				
			} 
		}
*/
	}
	
	private static void addNomToExplicitFacts (AbducerServerPrx abducer, LFNominal nom, LogicalForm lf) {
		Term nomTerm = term(nom.nomVar);
//		System.err.println("add nom: " + nom.nomVar);
		abducer.addFact(modFormula(
				new Modality[] {modInfo()},
				predicate("sort", new Term[] {nomTerm, term(nom.sort)})
				));

		// Iterator pIter = nom.getPropositions();
		boolean props = (!nom.prop.prop.equals("")); 	

		Iterator fIter = LFUtils.lfNominalGetFeatures(nom);
		boolean feats = (fIter.hasNext());
		Iterator rIter = LFUtils.lfNominalGetRelations(nom); 
		boolean rels = rIter.hasNext(); 

//		System.err.println("before props");

		//while (pIter.hasNext()) { 
		String prop = nom.prop.prop;
		if (props) {
			abducer.addFact(modFormula(
					new Modality[] {modInfo()},
					predicate("prop", new Term[] {nomTerm, term(prop)})
					));
		}

//		System.err.println("before feats");

		while (fIter.hasNext()) { 
			Feature feat = (Feature) fIter.next(); 
			abducer.addFact(modFormula(
					new Modality[] {modInfo()},
					predicate("feat_" + feat.feat, new Term[] {nomTerm, term(feat.value)})
					));
		}

//		System.err.println("before rels");

		while (rIter.hasNext()) { 
			LFRelation rel = (LFRelation) rIter.next();
			abducer.addFact(modFormula(
					new Modality[] {modInfo()},
					predicate("rel_" + rel.mode, new Term[] {nomTerm, term(rel.dep)})
					));

			LFNominal depnom = LFUtils.lfGetNominal(lf, rel.dep); 
			if(rel.coIndexedDep == true){
				// we only want to generate <RelMode>var1:type1, not the complete nominal
				abducer.addFact(modFormula(
						new Modality[] {modInfo()},
						predicate("sort", new Term[] {term(rel.dep), term(depnom.sort)})
						));
			} else {
				addNomToExplicitFacts(abducer, depnom, lf);
			}
		}
//		System.err.println("done add nom " + nom.nomVar);
	}
}
