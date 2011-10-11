package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.lf.Feature;
import de.dfki.lt.tr.dialogue.slice.lf.LFRelation;

import de.dfki.lt.tr.dialogue.util.NominalRemapper;
import de.dfki.lt.tr.infer.abducer.engine.FileReadErrorException;
import de.dfki.lt.tr.infer.abducer.engine.SyntaxErrorException;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.proof.MarkedQuery;
import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.util.AbductionEngineConnection;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import de.dfki.lt.tr.infer.abducer.util.ProofUtils;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import org.apache.log4j.Logger;

public abstract class AbducerUtils {

	public static Logger logger = Logger.getLogger("abducer-utils");

	/** Convert a logical form to an array of modalised atoms -- facts
     *  for the abducer.
	 * 
	 * @param modality prefixed to every fact
	 * @param lf
	 * @return set of corresponding facts
	 */
	public static List<ModalisedAtom> lfToFacts(Modality[] modality, LogicalForm lf, NominalRemapper remapper) {
		List<ModalisedAtom> facts = new LinkedList<ModalisedAtom>();
		Iterator<LFNominal> it = LFUtils.lfGetNominals(lf);
		while (it.hasNext()) {
			addNomToFactList(facts, modality, it.next(), remapper);
		}
		return facts;
	}

	/** Expand a nominal to a set of abducer facts.
	 *
	 * @param facts the set of facts
	 * @param mod modal context
	 * @param nom the nominal
	 */
	private static void addNomToFactList(List<ModalisedAtom> facts, Modality[] mod, LFNominal nom, NominalRemapper remapper) {
		// nominal term
		Term nomTerm = TermAtomFactory.term(remapper.remap(nom.nomVar));
		
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
						TermAtomFactory.term(remapper.remap(rel.dep))
					})));
		}
	}

    /** Reconstruct a logical form from a set of abduction facts.
     * 
     * @param facts the set of facts
     * @param root identifier of the root nominal
     * @return the logical form
     */
	public static LogicalForm factsToLogicalForm(List<ModalisedAtom> facts, String root) {
		LogicalForm lf = LFUtils.newLogicalForm();
		
		// go through the facts, extending the logical form
		for (ModalisedAtom f : facts) {
			String nomVar = "";
			
			if (f.a.predSym.equals("sort") && f.a.args.size() == 2) {
				//System.err.println("sort...");
				nomVar = ProofUtils.termFunctor(f.a.args.get(0));
				String sort = ProofUtils.termFunctor(f.a.args.get(1));

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

			else if (f.a.predSym.equals("prop") && f.a.args.size() == 2) {
				//System.err.println("prop...");
				nomVar = ProofUtils.termFunctor(f.a.args.get(0));
				String prop = ProofUtils.termFunctor(f.a.args.get(1));
				LFNominal nom = LFUtils.lfGetNominal(lf, nomVar);
				if (nom == null) {
					nom = LFUtils.newLFNominal(nomVar);
					LFUtils.lfAddNominal(lf.noms, nom);
				}
				nom.prop = LFUtils.lfNominalAddProposition(nom, prop);
				//System.err.println("  " + prop);
				//System.err.println("  " + LFUtils.lfNominalToString(nom));
			}

			else if (f.a.predSym.matches("feat_.*") && f.a.args.size() == 2) {
				//System.err.println("feat...");
				nomVar = ProofUtils.termFunctor(f.a.args.get(0));
				Feature feat = new Feature();
				feat.feat = f.a.predSym.substring(5);
				feat.value = ProofUtils.termFunctor(f.a.args.get(1));
				LFNominal nom = LFUtils.lfGetNominal(lf, nomVar);
				if (nom == null) {
					nom = LFUtils.newLFNominal(nomVar);
					LFUtils.lfAddNominal(lf.noms, nom);
				}
				nom.feats = LFUtils.lfNominalAddFeature(nom, feat);
				//System.err.println("  " + feat.feat + " -> " + feat.value);
				//System.err.println("  " + LFUtils.lfNominalToString(nom));
			}
						
			else if (f.a.predSym.matches("rel_.*") && f.a.args.size() == 2) {
				//System.err.println("rel...");
				nomVar = ProofUtils.termFunctor(f.a.args.get(0));
				String mode = f.a.predSym.substring(4);
				String dep = ProofUtils.termFunctor(f.a.args.get(1));
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

	public static ProofWithCost bestAbductiveProof(AbductionEngineConnection abd, List<MarkedQuery> goal, int timeout) {
		List<ProofWithCost> proofs = allAbductiveProofs(abd, goal, timeout);

		if (!proofs.isEmpty()) {
			engineLog(abd.getEngineName(), "using the best proof");

			ProofWithCost pwc = proofs.get(0);
			List<MarkedQuery> proof = pwc.proof;
			engineLog(abd.getEngineName(), "obtained the proof: " + PrettyPrint.proofToString(proof));

			return pwc;
		}
		else {
			engineLog(abd.getEngineName(), "no proofs found");
			return null;
		}
	}

	public static List<ProofWithCost> allAbductiveProofs(AbductionEngineConnection abd, List<MarkedQuery> goal, int timeout) {
		engineLog(abd.getEngineName(), "proving: " + PrettyPrint.proofToString(goal));

		abd.getEngineProxy().startProving(goal);
		List<ProofWithCost> result = abd.getEngineProxy().getProofs(timeout);

		engineLog(abd.getEngineName(), "found " + result.size() + " alternatives");
		return result;
	}

	/**
	 * Load a file with rules and facts for the abducer.
	 *
	 * @param file file name
	 */
	public static void loadFile(AbductionEngineConnection abd, String file) {
		try {
			abd.getEngineProxy().loadFile(file);
		}
		catch (FileReadErrorException ex) {
			engineLog(abd.getEngineName(), "file read error: " + ex.filename);
		}
		catch (SyntaxErrorException ex) {
			engineLog(abd.getEngineName(), "syntax error: " + ex.error + " in " + ex.filename + " on line " + ex.line);
		}
	}

	public static String getAbducerServerEndpointString(String host, int port) {
		String s = "default -p " + port;
		if (host != null && !host.equals("")) {
			s += " -h " + host;
		}
		return s;
	}

	public static List<String> getAbducerRuleFiles(Logger logger, String rulesetFile) {
		if (rulesetFile == null) {
			throw new NullPointerException("ruleset filename is null");
		}

		List<String> result = new ArrayList<String>();

		try {
			BufferedReader f = new BufferedReader(new FileReader(rulesetFile));
			String parentAbsPath = (new File((new File(rulesetFile)).getParent()).getCanonicalPath());
			if (parentAbsPath == null) {
				parentAbsPath = ""; // rulefile is in `/'
			}
			logger.info("will be looking for abducer rulefiles in `" + parentAbsPath + "'");
			String file = null;
			while ((file = f.readLine()) != null) {
				file = parentAbsPath + File.separator + file;
				result.add(file);
			}
			f.close();
		}
		catch (FileNotFoundException ex) {
			logger.error("ruleset filename not found", ex);
		}
		catch (IOException ex) {
			logger.error("I/O exception while reading files from list", ex);
		}

		return result;
	}

	private static void log(String logname, String str) {
		if (logger != null) {
			Logger lg = Logger.getLogger(logger.getName() + "." + logname);
			lg.debug(str);
		}
	}

	private static void engineLog(String engName, String str) {
		log("engine." + engName, str);
	}

	public static double weightToProb(double weight) {
		return Math.exp(-weight);
	}

	public static double probToWeight(double prob) {
		return -Math.log(prob);
	}

}
