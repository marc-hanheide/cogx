package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.lf.Feature;
import de.dfki.lt.tr.dialogue.slice.lf.LFRelation;

import de.dfki.lt.tr.infer.weigabd.AbductionEngineConnection;
import de.dfki.lt.tr.infer.weigabd.MercuryUtils;
import de.dfki.lt.tr.infer.weigabd.ProofUtils;
import de.dfki.lt.tr.infer.weigabd.TermAtomFactory;
import de.dfki.lt.tr.infer.weigabd.slice.MarkedQuery;
import de.dfki.lt.tr.infer.weigabd.slice.ModalisedAtom;
import de.dfki.lt.tr.infer.weigabd.slice.Modality;
import de.dfki.lt.tr.infer.weigabd.slice.ProofWithCost;
import de.dfki.lt.tr.infer.weigabd.slice.Term;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public abstract class AbducerUtils {

	public static boolean logging = true;

	/** Convert a logical form to an array of modalised atoms -- facts
     *  for the abducer.
	 * 
	 * @param modality prefixed to every fact
	 * @param lf
	 * @return set of corresponding facts
	 */
	public static List<ModalisedAtom> lfToFacts(Modality[] modality, LogicalForm lf) {
		List<ModalisedAtom> facts = new LinkedList<ModalisedAtom>();
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
	private static void addNomToFactList(List<ModalisedAtom> facts, Modality[] mod, LFNominal nom) {
		// nominal term
		Term nomTerm = TermAtomFactory.term(nom.nomVar);
		
		// sort
		facts.add(TermAtomFactory.modalisedAtom(
				mod,
				TermAtomFactory.atom("sort", new Term[] {
					nomTerm,
					TermAtomFactory.term(nom.sort)
				})));

		// proposition, if there is one
		if (!nom.prop.prop.equals("")) {
			facts.add(TermAtomFactory.modalisedAtom(
					mod,
					TermAtomFactory.atom("prop", new Term[] {
						nomTerm,
						TermAtomFactory.term(nom.prop.prop)
					})));		
		}

		// features
		Iterator fIter = LFUtils.lfNominalGetFeatures(nom);
		while (fIter.hasNext()) { 
			Feature feat = (Feature) fIter.next(); 
			facts.add(TermAtomFactory.modalisedAtom(
					mod,
					TermAtomFactory.atom("feat_" + feat.feat, new Term[] {
						nomTerm,
						TermAtomFactory.term(feat.value)
					})));
		}

		Iterator rIter = LFUtils.lfNominalGetRelations(nom); 
		while (rIter.hasNext()) {
			LFRelation rel = (LFRelation) rIter.next();
			facts.add(TermAtomFactory.modalisedAtom(
					mod,
					TermAtomFactory.atom("rel_" + rel.mode, new Term[] {
						nomTerm, 
						TermAtomFactory.term(rel.dep)
					})));
		}
	}

    /** Reconstruct a logical form from a set of abduction facts.
     * 
     * @param facts the set of facts
     * @param root identifier of the root nominal
     * @return the logical form
     */
	public static LogicalForm factsToLogicalForm(ModalisedAtom[] facts, String root) {
		LogicalForm lf = LFUtils.newLogicalForm();
		
		// go through the facts, extending the logical form
		for (int i = 0; i < facts.length; i++) {
			ModalisedAtom f = facts[i];
			String nomVar = "";
			
			if (f.a.predSym.equals("sort") && f.a.args.length == 2) {
				//System.err.println("sort...");
				nomVar = ProofUtils.termToString(f.a.args[0]);
				String sort = ProofUtils.termToString(f.a.args[1]);

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

			else if (f.a.predSym.equals("prop") && f.a.args.length == 2) {
				//System.err.println("prop...");
				nomVar = ProofUtils.termToString(f.a.args[0]);
				String prop = ProofUtils.termToString(f.a.args[1]);
				LFNominal nom = LFUtils.lfGetNominal(lf, nomVar);
				if (nom == null) {
					nom = LFUtils.newLFNominal(nomVar);
					LFUtils.lfAddNominal(lf.noms, nom);
				}
				nom.prop = LFUtils.lfNominalAddProposition(nom, prop);
				//System.err.println("  " + prop);
				//System.err.println("  " + LFUtils.lfNominalToString(nom));
			}

			else if (f.a.predSym.matches("feat_.*") && f.a.args.length == 2) {
				//System.err.println("feat...");
				nomVar = ProofUtils.termToString(f.a.args[0]);
				Feature feat = new Feature();
				feat.feat = f.a.predSym.substring(5);
				feat.value = ProofUtils.termToString(f.a.args[1]);
				LFNominal nom = LFUtils.lfGetNominal(lf, nomVar);
				if (nom == null) {
					nom = LFUtils.newLFNominal(nomVar);
					LFUtils.lfAddNominal(lf.noms, nom);
				}
				nom.feats = LFUtils.lfNominalAddFeature(nom, feat);
				//System.err.println("  " + feat.feat + " -> " + feat.value);
				//System.err.println("  " + LFUtils.lfNominalToString(nom));
			}
						
			else if (f.a.predSym.matches("rel_.*") && f.a.args.length == 2) {
				//System.err.println("rel...");
				nomVar = ProofUtils.termToString(f.a.args[0]);
				String mode = f.a.predSym.substring(4);
				String dep = ProofUtils.termToString(f.a.args[1]);
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

	public static MarkedQuery[] bestAbductiveProof(AbductionEngineConnection abd, MarkedQuery[] goal, int timeout) {
		String listGoalsStr = "";
		for (int i = 0; i < goal.length; i++) {
			listGoalsStr += MercuryUtils.modalisedAtomToString(goal[i].atom);
			if (i < goal.length - 1) listGoalsStr += ", ";
		}
		log("abducer:" + abd.getEngineName(), "proving: [" + listGoalsStr + "]");

		abd.getProxy().startProving(goal);
		ProofWithCost[] result = abd.getProxy().getProofs(timeout);
		if (result.length > 0) {
			log("abducer:" + abd.getEngineName(), "found " + result.length + " proofs, picking the best one");
			return result[0].proof;
		}
		else {
			return null;
		}
	}

	private static void log(String logname, String str) {
		if (logging)
			System.out.println("\033[31m[" + logname + "]\t" + str + "\033[0m");
	}

}
