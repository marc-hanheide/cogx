package comsys.processing.cca;

import java.util.*;

import comsys.datastructs.lf.*;
import comsys.lf.utils.LFUtils;
import Abducer.*;

import beliefmodels.adl.*;

import comsys.processing.cca.ProofUtils;
import comsys.processing.cca.abduction.PredicateFactory;

public abstract class AbducerUtils {

	/**
	 * Return a new modalised formula.
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

	private static void addNomToFactList(AbstractList<ModalisedFormula> facts, Modality[] mod, LFNominal nom, LogicalForm lf) {
		// nominal term
		Term nomTerm = PredicateFactory.term(nom.nomVar);
		
		// sort
		facts.add(modalisedFormula(
				mod,
				PredicateFactory.predicate("sort", new Term[] {
					nomTerm,
					PredicateFactory.term(nom.sort)
				})));

		// proposition, if there is one
		if (!nom.prop.prop.equals("")) {
			facts.add(modalisedFormula(
					mod,
					PredicateFactory.predicate("prop", new Term[] {
						nomTerm,
						PredicateFactory.term(nom.prop.prop)
					})));		
		}

		// features
		Iterator fIter = LFUtils.lfNominalGetFeatures(nom);
		while (fIter.hasNext()) { 
			Feature feat = (Feature) fIter.next(); 
			facts.add(modalisedFormula(
					mod,
					PredicateFactory.predicate("feat_" + feat.feat, new Term[] {
						nomTerm,
						PredicateFactory.term(feat.value)
					})));
		}

		Iterator rIter = LFUtils.lfNominalGetRelations(nom); 
		while (rIter.hasNext()) {
			LFRelation rel = (LFRelation) rIter.next();
			facts.add(modalisedFormula(
					mod,
					PredicateFactory.predicate("rel_" + rel.mode, new Term[] {
						nomTerm, 
						PredicateFactory.term(rel.dep)
					})));
		}
	}
	
	public static LogicalForm factsToLogicalForm(ModalisedFormula[] facts, String root) {
		LogicalForm lf = LFUtils.newLogicalForm();
		
		// go through the facts, extending the logical form
		for (int i = 0; i < facts.length; i++) {
			ModalisedFormula f = facts[i];
			String nomVar = "";
			
			if (f.p.predSym.equals("sort") && f.p.args.length == 2) {
				//System.err.println("sort...");
				nomVar = ProofUtils.termToString(f.p.args[0]);
				String sort = ProofUtils.termToString(f.p.args[1]);

				LFNominal nom = LFUtils.lfGetNominal(lf, nomVar);
				if (nom == null) {
					nom = LFUtils.newLFNominal(nomVar, sort);
				}
				else {
					nom.sort = sort;
				}
				//System.err.println("  " + sort);
				//System.err.println("  " + LFUtils.lfNominalToString(nom));
				lf.noms = LFUtils.lfAddNominal(lf.noms, nom);
			}

			else if (f.p.predSym.equals("prop") && f.p.args.length == 2) {
				//System.err.println("prop...");
				nomVar = ProofUtils.termToString(f.p.args[0]);
				String prop = ProofUtils.termToString(f.p.args[1]);
				LFNominal nom = LFUtils.lfGetNominal(lf, nomVar);
				if (nom == null) {
					nom = LFUtils.newLFNominal(nomVar);
				}
				nom.prop = LFUtils.lfNominalAddProposition(nom, prop);
				//System.err.println("  " + prop);
				//System.err.println("  " + LFUtils.lfNominalToString(nom));
				lf.noms = LFUtils.lfAddNominal(lf.noms, nom);
			}

			else if (f.p.predSym.matches("feat_.*") && f.p.args.length == 2) {
				//System.err.println("feat...");
				nomVar = ProofUtils.termToString(f.p.args[0]);
				Feature feat = new Feature();
				feat.feat = f.p.predSym.substring(5);
				feat.value = ProofUtils.termToString(f.p.args[1]);
				LFNominal nom = LFUtils.lfGetNominal(lf, nomVar);
				if (nom == null) {
					nom = LFUtils.newLFNominal(nomVar);
				}
				nom.feats = LFUtils.lfNominalAddFeature(nom, feat);
				//System.err.println("  " + feat.feat + " -> " + feat.value);
				//System.err.println("  " + LFUtils.lfNominalToString(nom));
				lf.noms = LFUtils.lfAddNominal(lf.noms, nom);
			}
						
			else if (f.p.predSym.matches("rel_.*") && f.p.args.length == 2) {
				//System.err.println("rel...");
				nomVar = ProofUtils.termToString(f.p.args[0]);
				String mode = f.p.predSym.substring(4);
				String dep = ProofUtils.termToString(f.p.args[1]);
				LFNominal nom = LFUtils.lfGetNominal(lf, nomVar);
				LFRelation rel = LFUtils.newLFRelation(nomVar, mode, dep);
				if (nom == null) {
					nom = LFUtils.newLFNominal(nomVar);
				}
				nom.rels = LFUtils.lfNominalAddRelation(nom, rel);
				lf.noms = LFUtils.lfAddNominal(lf.noms, nom);
			}
		}
		//System.err.println(LFUtils.lfToString(lf));

		// set the root
		LFNominal rootNom = LFUtils.lfGetNominal(lf, root);
		if (rootNom == null) {
			rootNom = LFUtils.newLFNominal(root);
		}
		lf.root = rootNom;

		//System.err.println(LFUtils.lfToString(lf));

		return lf;
	}
}
