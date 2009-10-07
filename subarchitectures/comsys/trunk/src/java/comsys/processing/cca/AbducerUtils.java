package comsys.processing.cca;

import java.util.*;

import comsys.datastructs.lf.*;
import comsys.lf.utils.LFUtils;
import Abducer.*;

import beliefmodels.adl.*;

import comsys.processing.cca.ProofUtils;

public class AbducerUtils {

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
	
	public static Abducer.Agent toAbducerAgent(beliefmodels.adl.Agent ag) {
		if (ag.id.equals("human")) {
			return Abducer.Agent.human;
		}
		else {
			return Abducer.Agent.robot;
		}
	}

	public static KModality kModality(AgentStatus as) {
		KModality m = new KModality();
		m.type = ModalityType.K;
		if (as instanceof PrivateAgentStatus) {
			m.share = Abducer.Sharing.Private;
			m.act = toAbducerAgent(((PrivateAgentStatus) as).ag);
			m.pat = Abducer.Agent.human;
		}
		if (as instanceof AttributedAgentStatus) {
			m.share = Abducer.Sharing.Attribute;
			m.act = toAbducerAgent(((AttributedAgentStatus) as).ag);
			m.pat = toAbducerAgent(((AttributedAgentStatus) as).ag2);
		}
		if (as instanceof AttributedAgentStatus) {
			m.share = Abducer.Sharing.Mutual;
			m.act = Abducer.Agent.robot;
			m.act = Abducer.Agent.human;
		}
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

/*
	public static LFNominal factsToLFNominal(ModalisedFormula[] facts, String nomVar) {
		String sort = "";
		String prop = "";
		ArrayList<Feature> feats = new ArrayList<Feature>();
		ArrayList<LFRelation> rels = new ArrayList<LFRelation>();

		for (int i = 0; i < facts.length; i++) {

			ModalisedFormula f = facts[i];

			if (f.p.predSym.equals("sort") && ProofUtils.termToString(f.p.args[0]).equals(nomVar)) {
				sort = ProofUtils.termToString(f.p.args[1]);
			}

			if (f.p.predSym.equals("prop") && ProofUtils.termToString(f.p.args[0]).equals(nomVar)) {
				prop = ProofUtils.termToString(f.p.args[1]);
			}
			
			if (f.p.predSym.matches("feat_.*") && ProofUtils.termToString(f.p.args[0]).equals(nomVar)) {
				Feature feat = new Feature();
				feat.feat = f.p.predSym.substring(5);
				feat.value = ProofUtils.termToString(f.p.args[1]);
				feats.add(feat);
			}
			
			if (f.p.predSym.matches("rel_.*") && ProofUtils.termToString(f.p.args[0]).equals(nomVar)) {
				LFRelation rel = new LFRelation();
				rel.mode = f.p.predSym.substring(4);
				rel.head = nomVar;
				rel.dep = ProofUtils.termToString(f.p.args[1]);
				rels.add(rel);
			}
		}
		
		LFNominal nom = LFUtils.newLFNominal(nomVar, sort);
		nom.feats = feats.toArray(new Feature[0]);
		nom.rels = rels.toArray(new LFRelation[0]);
		return nom;
	}
*/
}
