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

package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.util.BeliefFormulaFactory;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.slice.ref.NominalReference;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.infer.abducer.engine.FileReadErrorException;
import de.dfki.lt.tr.infer.abducer.engine.SyntaxErrorException;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.MarkedQuery;
import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import de.dfki.lt.tr.infer.abducer.util.AbductionEngineConnection;
import de.dfki.lt.tr.infer.abducer.util.ProofUtils;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public class IntentionRealization {

	private int timeout = 0;
	public boolean logging = true;
	private AbductionEngineConnection abd_realize;

//	public static Counter counter = new Counter("ir");
	public final IdentifierGenerator<String> idGen;

	private final String abd_serverName;
	private final String abd_endpoints;

	public static final String INTENTION_REALIZATION_ENGINE = "IntentionRealization";

	/**
	 * Initialise the abducer and prepare for action.
	 */
    public IntentionRealization(String servername, String endpoints, IdentifierGenerator<String> idGen_, int timeout_) {
		this.idGen = idGen_;
		this.timeout = timeout_;
		this.abd_serverName = servername;
		this.abd_endpoints = endpoints;
		init();
    }

	private void init() {
		abd_realize = new AbductionEngineConnection();
		abd_realize.connectToServer(abd_serverName, abd_endpoints);
		abd_realize.bindToEngine(INTENTION_REALIZATION_ENGINE);
		abd_realize.getEngineProxy().clearContext();
	}

	/**
	 * Try to find a proto-LF that realises the intention and the underlying
	 * beliefs.
	 *
	 * @param it communicative intention
	 * @param bels beliefs needed by itn
	 * @return a proto-LF if successful, null if not
	 */
	public ContentPlanningGoal epistemicObjectsToProtoLF(WorkingMemoryAddress itWma, Intention it, List<dBelief> bels) {

		Term itIdTerm = ConversionUtils.workingMemoryAddressToTerm(itWma);

		// update the abduction context
		for (ModalisedAtom mf : ConversionUtils.intentionToFacts(itIdTerm, it)) {  // XXX
			abd_realize.getEngineProxy().addFact(mf);
		}
/*
		for (dBelief b : bels) {
			for (ModalisedAtom mf : ConversionUtils.beliefToFacts(b)) {
				abd_realize.getProxy().addFact(mf);
			}
		}
*/

		ModalisedAtom g = TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Generation,
					Modality.Event
				},
				TermAtomFactory.atom("utter", new Term[] {
					TermAtomFactory.term(IntentionManagementConstants.thisAgent),
					TermAtomFactory.term(IntentionManagementConstants.humanAgent),
					itIdTerm
				}));

		ProofWithCost pwc = AbducerUtils.bestAbductiveProof(abd_realize, ProofUtils.newUnsolvedProof(g), timeout);
		if (pwc != null) {
			return proofToProtoLF(pwc.proof);
		}
		else {
			log("no proof found");
			return null;
		}
	}

	// Sun should burn in hell for not having such a function in the standard library!
	public static String join(String separator, List<String> args) {
		StringBuilder sb = new StringBuilder();
		if (args != null) {
			Iterator<String> iter = args.iterator();
			while (iter.hasNext()) {
				sb.append(iter.next());
				if (iter.hasNext()) {
					sb.append(separator);
				}
			}
		}
		return sb.toString();
	}

	private ContentPlanningGoal proofToProtoLF(List<MarkedQuery> proof) {

		List<ModalisedAtom> events = ProofUtils.filterStripByModalityPrefix(
				ProofUtils.stripMarking(ProofUtils.filterAssumedAndForget(proof)),
				new ArrayList<Modality>(Arrays.asList(new Modality[] {Modality.Event})));

		LogicalForm lf = null;
		NominalReference nr = null;

		if (events.size() == 1 && events.get(0).a.predSym.equals("produce_text") && events.get(0).a.args.size() == 1) {
			List<Term> terms = ConversionUtils.listTermToListOfTerms(events.get(0).a.args.get(0));

			List<String> words = new LinkedList<String>();
			for (Term t : terms) {
				String word = null;
				if (t instanceof FunctionTerm) {
					word = ((FunctionTerm) t).functor;
				}

				if (word == null) {
					return null;
				}
				else {
					words.add(word);
				}
			}

			String cannedString = join("_", words);
			if (cannedString != null && !cannedString.equals("")) {
				lf = LFUtils.convertFromString("@d:dvp(<CannedText>" + cannedString + ")");
			}
		}
		else {
			List<ModalisedAtom> imfs = ProofUtils.filterStripByModalityPrefix(
					ProofUtils.stripMarking(ProofUtils.filterAssumedAndForget(proof)),
					new ArrayList<Modality>(Arrays.asList(new Modality[] {Modality.Truth})));

			lf = AbducerUtils.factsToLogicalForm(imfs, "dn1_1");
		}

		List<ModalisedAtom> rrs = ProofUtils.stripMarking(ProofUtils.filterAssumedAndForget(proof));
		for (ModalisedAtom ma : rrs) {
			if (ma.m.size() == 1 && ma.m.get(0) == Modality.Generation && ma.a.predSym.equals("dialogue_move_topic") && ma.a.args.size() == 2) {
				String nom = ((FunctionTerm)ma.a.args.get(0)).functor;
				WorkingMemoryAddress wma = ConversionUtils.termToWorkingMemoryAddress(ma.a.args.get(1));
				if (wma != null) {
					nr = new NominalReference(nom, BeliefFormulaFactory.newPointerFormula(wma));
				}
			}
		}

		if (lf != null) {
			return new ContentPlanningGoal(idGen.newIdentifier(), lf, nr);
		}
		else {
			return null;
		}
	}

	public void clearContext() {
		abd_realize.getEngineProxy().clearContext();
	}

	/**
	 * Load a file with rules and facts for the abducer.
	 *
	 * @param file file name
	 */
	public void loadFile(String file) {
		try {
			abd_realize.getEngineProxy().loadFile(file);
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
			System.out.println("\033[32m[IR]\t" + str + "\033[0m");
	}

}
