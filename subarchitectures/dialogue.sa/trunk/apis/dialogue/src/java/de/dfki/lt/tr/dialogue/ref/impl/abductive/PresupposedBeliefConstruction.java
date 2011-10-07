// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots
// Miroslav Janicek (miroslav.janicek@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

package de.dfki.lt.tr.dialogue.ref.impl.abductive;

import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.interpret.AbducerUtils;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.util.NoOpNominalRemapper;
import de.dfki.lt.tr.infer.abducer.engine.AbductionEnginePrx;
import de.dfki.lt.tr.infer.abducer.engine.FileReadErrorException;
import de.dfki.lt.tr.infer.abducer.engine.SyntaxErrorException;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.MarkedQuery;
import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import de.dfki.lt.tr.infer.abducer.util.AbductionEngineConnection;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import de.dfki.lt.tr.infer.abducer.util.ProofUtils;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 *
 * @author janicek
 */
public class PresupposedBeliefConstruction {

	public boolean logging = true;
	private AbductionEngineConnection abd;

	private String abd_serverName = "";
	private String abd_endpoints = "";

	public static final String BELIEF_CONSTRUCTION_ENGINE = "BeliefConstruction";

	public PresupposedBeliefConstruction(String servername, String endpoints) {
		this.abd_serverName = servername;
		this.abd_endpoints = endpoints;
		init();
	}

	private void init() {
		abd = new AbductionEngineConnection();
		abd.connectToServer(abd_serverName, abd_endpoints);
		abd.bindToEngine(BELIEF_CONSTRUCTION_ENGINE);
		abd.getEngineProxy().clearContext();
	}

	public Map<String, Map<String, String>> extractPresuppositions(LogicalForm lf) {
		log("expanding LF into facts");
		for (ModalisedAtom fact : AbducerUtils.lfToFacts(new Modality[] {Modality.Truth}, lf, NoOpNominalRemapper.INSTANCE)) {
			abd.getEngineProxy().addFact(fact);
		}

		ModalisedAtom g = TermAtomFactory.modalisedAtom(
				new Modality[] { },
				TermAtomFactory.atom("presuppositions_in", new Term[] {
					TermAtomFactory.term(lf.root.nomVar)
				}));

		List<MarkedQuery> proof = bestAbductiveProof(ProofUtils.newUnsolvedProof(g));

		Map<String, Map<String, String>> result = new HashMap<String, Map<String, String>>();

		if (proof == null) {
			log("no presuppositions found");
			return result;
		}

		List<ModalisedAtom> imfs = ProofUtils.filterStripByModalityPrefix(
				ProofUtils.stripMarking(proof),
				Arrays.asList(new Modality[] {Modality.Belief}));

		// extract the feature-value pairs
		for (ModalisedAtom ma : imfs) {
			if (ma.a.predSym.equals("fv")) {
				FunctionTerm nominal = (FunctionTerm) ma.a.args.get(0);
				FunctionTerm name = (FunctionTerm) ma.a.args.get(1);
				FunctionTerm value = (FunctionTerm) ma.a.args.get(2);
				if (!result.containsKey(nominal.functor)) {
					result.put(nominal.functor, new HashMap<String, String>());
				}
				Map<String, String> fvPairs = result.get(nominal.functor);
				fvPairs.put(name.functor, value.functor);
			}
		}

		return result;
	}

	public static String presupToString(String nominal, Map<String, String> fvPairs) {
		String s = nominal + " ~";
		for (String featName : fvPairs.keySet()) {
			s += " " + featName + "=" + fvPairs.get(featName) + ";";
		}
		return s;
	}

	private List<MarkedQuery> bestAbductiveProof(List<MarkedQuery> goal) {
		String listGoalsStr = "";
		for (MarkedQuery q : goal) {
			listGoalsStr += PrettyPrint.modalisedAtomToString(q.atom) + ", ";
		}
		log("proving: [" + listGoalsStr + "]");

//		abd.getEngineProxy().startProving(goal);
		List<ProofWithCost> result = abd.getEngineProxy().getProofs(250);

		if (!result.isEmpty()) {
			log("found " + result.size() + " alternatives, using the best one");
			return result.get(0).proof;
		}
		else {
			log("no proof found");
			return null;
		}
	}

	/**
	 * Load a file with rules and facts for the abducer.
	 *
	 * @param file file name
	 */
	public void loadFile(String file) {
		try {
			abd.getEngineProxy().loadFile(file);
		}
		catch (FileReadErrorException ex) {
			log("file read error: " + ex.filename);
		}
		catch (SyntaxErrorException ex) {
			log("syntax error: " + ex.error + " in " + ex.filename + " on line " + ex.line);
		}
	}

	private void log(String str) {
		if (logging)
			System.out.println("\033[35m[BelConstruct]\t" + str + "\033[0m");
	}

}
