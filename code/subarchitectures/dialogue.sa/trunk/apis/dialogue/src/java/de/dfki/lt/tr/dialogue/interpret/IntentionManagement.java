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

import de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parse.PackedLFs;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.util.Counter;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.infer.wabd.AbducerServerConnection;
import de.dfki.lt.tr.infer.wabd.FormulaFactory;
import de.dfki.lt.tr.infer.wabd.MercuryUtils;
import de.dfki.lt.tr.infer.wabd.ProofUtils;
import de.dfki.lt.tr.infer.wabd.TermPredicateFactory;
import de.dfki.lt.tr.infer.wabd.slice.FileReadErrorException;
import de.dfki.lt.tr.infer.wabd.slice.MarkedQuery;
import de.dfki.lt.tr.infer.wabd.slice.ModalisedFormula;
import de.dfki.lt.tr.infer.wabd.slice.Modality;
import de.dfki.lt.tr.infer.wabd.slice.NoProofException;
import de.dfki.lt.tr.infer.wabd.slice.ProveResult;
import de.dfki.lt.tr.infer.wabd.slice.SyntaxErrorException;
import de.dfki.lt.tr.infer.wabd.slice.Term;
import java.util.LinkedList;
import java.util.List;

/**
 * The class encapsulating the recognition and realisation of communicative
 * intentions.
 *
 * Note that the design is not optimal as it was dictated by the need to
 * use one connection to an inference engine. Most likely it will need to
 * be entirely rewritten.
 *
 * @author Miroslav Janicek
 */
public class IntentionManagement {

	public boolean logging = true;
	private AbducerServerConnection abd;

//	public static Counter counter = new Counter("ir");
	public IdentifierGenerator idGen;

	public static final String thisAgent = "robot";
	public static final String humanAgent = "human";

	public static final String refModality = "Ref";
	public static final String beliefLinkModality = "Belief";

	/**
	 * Initialise the abducer and prepare for action.
	 */
    public IntentionManagement(IdentifierGenerator idGen_) {
		this.idGen = idGen_;
		init();
    }

	private void init() {
		abd = new AbducerServerConnection();
		abd.connect("AbducerServer", "default -p 10000");
		abd.getProxy().clearRules();
		abd.getProxy().clearFacts();
		abd.getProxy().clearAssumables();
	}

	/**
	 * Try to recognise the communicative intention and underlying beliefs
	 * behind an utterance, represented by a PackedLFs object.
	 *
	 * @param plf the utterance
	 * @return recognised intentions and beliefs when successful, null if error occurred
	 */
	public LinkedList<EpistemicObject> packedLFsToEpistemicObjects(PackedLFs plf) {
		LogicalForm lf = RandomParseSelection.extractLogicalForm(plf);
		log("selected LF = " + LFUtils.lfToString(lf));
		if (lf == null) {
			log("no LF found");
			return null;
		}
		log("expanding LF into facts");

		for (ModalisedFormula fact : AbducerUtils.lfToFacts(new Modality[] {Modality.Truth}, lf)) {
			log("  add fact: " + MercuryUtils.modalisedFormulaToString(fact));
			abd.getProxy().addFact(fact);
		}

		ModalisedFormula g = FormulaFactory.modalisedFormula(
				new Modality[] {
					Modality.Understanding,
					Modality.Event
				},
				TermPredicateFactory.predicate("utter", new Term[] {
					TermPredicateFactory.term(humanAgent),
					TermPredicateFactory.term(thisAgent),
					TermPredicateFactory.term(lf.root.nomVar)
				}));

		MarkedQuery[] proof = abductiveProof(ProofUtils.newUnsolvedProof(g));
		if (proof != null) {
			return ConversionUtils.proofToEpistemicObjects(idGen, humanAgent, proof);
		}
		else {
			log("no proof found");
			return null;
		}
	}

	/**
	 * Try to find a proto-LF that realises the intention and the underlying
	 * beliefs.
	 *
	 * @param itn communicative intention
	 * @param bels beliefs needed by itn
	 * @return a proto-LF if successful, null if not
	 */
	public ContentPlanningGoal epistemicObjectsToProtoLF(Intention itn, List<dBelief> bels) {

		// update the abduction context
		for (ModalisedFormula mf : ConversionUtils.intentionToFacts(itn)) {
			abd.getProxy().addFact(mf);
		}
		for (dBelief b : bels) {
			for (ModalisedFormula mf : ConversionUtils.beliefToFacts(b)) {
				abd.getProxy().addFact(mf);
			}
		}

		ModalisedFormula g = FormulaFactory.modalisedFormula(
				new Modality[] {
					Modality.Generation,
					Modality.Event
				},
				TermPredicateFactory.predicate("utter", new Term[] {
					TermPredicateFactory.term(thisAgent),
					TermPredicateFactory.term(humanAgent),
					TermPredicateFactory.term(itn.id)
				}));

		MarkedQuery[] proof = abductiveProof(ProofUtils.newUnsolvedProof(g));
		if (proof != null) {
			return proofToProtoLF(proof);
		}
		else {
			log("no proof found");
			return null;
		}
	}

	private ContentPlanningGoal proofToProtoLF(MarkedQuery[] proof) {
		ModalisedFormula[] imfs = ProofUtils.filterStripByModalityPrefix(
				ProofUtils.stripMarking(ProofUtils.filterAssumed(proof)),
				new Modality[] {Modality.Truth});
		LogicalForm lf = AbducerUtils.factsToLogicalForm(imfs, "dn1_1");
		ContentPlanningGoal plf = new ContentPlanningGoal();
		plf.cpgid = idGen.newIdentifier();
		plf.lform = lf;
		return plf;
	}

	private MarkedQuery[] abductiveProof(MarkedQuery[] goal) {
		String listGoalsStr = "";
		for (int i = 0; i < goal.length; i++) {
			listGoalsStr += MercuryUtils.modalisedFormulaToString(goal[i].formula);
			if (i < goal.length - 1) listGoalsStr += ", ";
		}
		log("proving: [" + listGoalsStr + "]");

		ProveResult result = abd.getProxy().prove(goal);
		if (result == ProveResult.ProofFound) {
			log("proof found");
			try {
				MarkedQuery[] p = abd.getProxy().getBestProof();
				return p;
			}
			catch (NoProofException e) {
				e.printStackTrace();
				return null;
			}
		}
		else {
			return null;
		}
	}

	/**
	 * Load a file with rules and facts for the abducer.
	 *
	 * @param file file name
	 */
	public void addFile(String file) {
		try {
			abd.getProxy().loadFile(file);
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