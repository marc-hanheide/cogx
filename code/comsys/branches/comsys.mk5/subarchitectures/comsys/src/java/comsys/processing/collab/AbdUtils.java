package comsys.processing.collab;

import java.util.*;

import comsys.datastructs.lf.*;
import comsys.lf.utils.LFUtils;
import Abducer.*;

public class AbdUtils {

	public static ModalisedFormula modFormula(ModalityType mt, Predicate p) {
		ModalisedFormula mf = new ModalisedFormula();
		mf.m = new Modality(mt);
		mf.p = p;
		return mf;
	}
	
	public static Predicate predicate(String predSym, Term[] args) {
		Predicate p = new Predicate();
		p.predSym = predSym;
		p.args = args;
		return p;
	}
	
	public static Term term(String functor, Term[] args) {
		Term t = new Term();
		t.variable = false;
		t.name = functor;
		t.args = args;
		return t;
	}

	public static Term term(String functor) {
		Term t = new Term();
		t.variable = false;
		t.name = functor;
		t.args = new Term[0];
		return t;
	}
	
	public static Term var(String name) {
		Term t = new Term();
		t.variable = true;
		t.name = name;
		t.args = null;
		return t;
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
		abducer.addFact(modFormula(ModalityType.Info, predicate("sort", new Term[] {nomTerm, term(nom.sort)})));

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
			abducer.addFact(modFormula(ModalityType.Info, predicate("prop", new Term[] {nomTerm, term(prop)})));
		}

//		System.err.println("before feats");

		while (fIter.hasNext()) { 
			Feature feat = (Feature) fIter.next(); 
			abducer.addFact(modFormula(ModalityType.Info,
					predicate("feat_" + feat.feat, new Term[] {nomTerm, term(feat.value)})));
		}

//		System.err.println("before rels");

		while (rIter.hasNext()) { 
			LFRelation rel = (LFRelation) rIter.next();
			abducer.addFact(modFormula(ModalityType.Info,
					predicate("rel_" + rel.mode, new Term[] {nomTerm, term(rel.dep)})));

			LFNominal depnom = LFUtils.lfGetNominal(lf, rel.dep); 
			if(rel.coIndexedDep == true){
				// we only want to generate <RelMode>var1:type1, not the complete nominal
				abducer.addFact(modFormula(ModalityType.Info,
						predicate("sort", new Term[] {term(rel.dep), term(depnom.sort)})));
			} else {
				addNomToExplicitFacts(abducer, depnom, lf);
			}
		}
//		System.err.println("done add nom " + nom.nomVar);
	}
}
