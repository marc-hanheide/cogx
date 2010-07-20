package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.lf.Feature;
import de.dfki.lt.tr.dialogue.slice.lf.LFRelation;

import de.dfki.lt.tr.infer.wabd.FormulaFactory;
import de.dfki.lt.tr.infer.wabd.ProofUtils;
import de.dfki.lt.tr.infer.wabd.TermPredicateFactory;
import de.dfki.lt.tr.infer.wabd.slice.ModalisedFormula;
import de.dfki.lt.tr.infer.wabd.slice.Modality;
import de.dfki.lt.tr.infer.wabd.slice.Term;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public abstract class AbducerUtils {

	/** Convert a logical form to an array of modalised formulas -- facts
         *  for the abducer.
	 * 
	 * @param modality prefixed to every fact
	 * @param lf
	 * @return set of corresponding facts
	 */
	public static List<ModalisedFormula> lfToFacts(Modality[] modality, LogicalForm lf) {
		List<ModalisedFormula> facts = new LinkedList<ModalisedFormula>();
		Iterator<LFNominal> it = LFUtils.lfGetNominals(lf);
		while (it.hasNext()) {
			addNomToFactList(facts, modality, it.next());
		}
		return facts;
	}

        /** Expand a nominal to a set of abducer facts.
         *
         * @param facts the set of facts
         * @param mod modal context
         * @param nom the nominal
         */
	private static void addNomToFactList(List<ModalisedFormula> facts, Modality[] mod, LFNominal nom) {
		// nominal term
		Term nomTerm = TermPredicateFactory.term(nom.nomVar);
		
		// sort
		facts.add(FormulaFactory.modalisedFormula(
				mod,
				TermPredicateFactory.predicate("sort", new Term[] {
					nomTerm,
					TermPredicateFactory.term(nom.sort)
				})));

		// proposition, if there is one
		if (!nom.prop.prop.equals("")) {
			facts.add(FormulaFactory.modalisedFormula(
					mod,
					TermPredicateFactory.predicate("prop", new Term[] {
						nomTerm,
						TermPredicateFactory.term(nom.prop.prop)
					})));		
		}

		// features
		Iterator fIter = LFUtils.lfNominalGetFeatures(nom);
		while (fIter.hasNext()) { 
			Feature feat = (Feature) fIter.next(); 
			facts.add(FormulaFactory.modalisedFormula(
					mod,
					TermPredicateFactory.predicate("feat_" + feat.feat, new Term[] {
						nomTerm,
						TermPredicateFactory.term(feat.value)
					})));
		}

		Iterator rIter = LFUtils.lfNominalGetRelations(nom); 
		while (rIter.hasNext()) {
			LFRelation rel = (LFRelation) rIter.next();
			facts.add(FormulaFactory.modalisedFormula(
					mod,
					TermPredicateFactory.predicate("rel_" + rel.mode, new Term[] {
						nomTerm, 
						TermPredicateFactory.term(rel.dep)
					})));
		}
	}

    /** Reconstruct a logical form from a set of abduction facts.
     * 
     * @param facts the set of facts
     * @param root identifier of the root nominal
     * @return the logical form
     */
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

				boolean isNew = false;
				LFNominal nom = LFUtils.lfGetNominal(lf, nomVar);
				if (nom == null) {
					isNew = true;
					nom = LFUtils.newLFNominal(nomVar, sort);
				}
				else {
					nom.sort = sort;
				}
				//System.err.println("  " + sort);
				//System.err.println("  " + LFUtils.lfNominalToString(nom));
				if (isNew)
					lf.noms = LFUtils.lfAddNominal(lf.noms, nom);
			}

			else if (f.p.predSym.equals("prop") && f.p.args.length == 2) {
				//System.err.println("prop...");
				nomVar = ProofUtils.termToString(f.p.args[0]);
				String prop = ProofUtils.termToString(f.p.args[1]);
				LFNominal nom = LFUtils.lfGetNominal(lf, nomVar);
				if (nom == null) {
					nom = LFUtils.newLFNominal(nomVar);
					LFUtils.lfAddNominal(lf.noms, nom);
				}
				nom.prop = LFUtils.lfNominalAddProposition(nom, prop);
				//System.err.println("  " + prop);
				//System.err.println("  " + LFUtils.lfNominalToString(nom));
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
					LFUtils.lfAddNominal(lf.noms, nom);
				}
				nom.feats = LFUtils.lfNominalAddFeature(nom, feat);
				//System.err.println("  " + feat.feat + " -> " + feat.value);
				//System.err.println("  " + LFUtils.lfNominalToString(nom));
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
					LFUtils.lfAddNominal(lf.noms, nom);
				}
				nom.rels = LFUtils.lfNominalAddRelation(nom, rel);
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
