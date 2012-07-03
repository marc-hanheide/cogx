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

import de.dfki.lt.tr.dialogue.slice.ref.NominalReference;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.slice.interpret.Interpretation;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import java.util.ArrayList;
import java.util.List;
import java.util.LinkedList;
import java.util.Set;
import java.util.TreeSet;
import org.apache.log4j.Logger;

public class IntentionRecognitionResult {

	private final LogicalForm lf;
	private final TimeInterval ival;

	private final List<Intention> ints;
	private final List<dBelief> pre;
	private final List<dBelief> post;
	private NominalReference nref;
	private final List<ReferenceResolutionRequest> rrs;
	private final List<ProofWithCost> proofs;

	public IntentionRecognitionResult(LogicalForm lf, TimeInterval ival, List<ProofWithCost> proofs) {
		ints = new LinkedList<Intention>();
		pre = new LinkedList<dBelief>();
		post = new LinkedList<dBelief>();
		rrs = new LinkedList<ReferenceResolutionRequest>();
		nref = null;
		this.lf = lf;
		this.ival = ival;
		this.proofs = proofs;
	}

	public Interpretation toInterpretation() {
		Set<String> ungrounded = new TreeSet<String>();
		for (ReferenceResolutionRequest rr : rrs) {
			ungrounded.add(rr.nom);
		}
		return new Interpretation(lf, ival.toIce(), proofs, new ArrayList<String>(ungrounded));
	}

	public static IntentionRecognitionResult extractFromInterpretation(ProofConvertor pconv, Interpretation ipret, Logger logger) {
		if (!ipret.proofs.isEmpty()) {
			ProofWithCost pwc = ipret.proofs.get(0);
			List<ReferenceResolutionRequest> rcs = ConversionUtils.extractReferenceRequests(pconv.getReferenceResolutionRequestExtractor(), ipret.lform, pwc.proof, new TimeInterval(ipret.ival));

			return pconv.proofToIntentionRecognitionResult(ipret.lform, pwc, ipret.lform.preferenceScore, new TimeInterval(ipret.ival), rcs);
		}
		else {
			logger.warn("interpretation empty!");
			return null;
		}
	}

	public void setNominalReference(NominalReference nominalReference) {
		nref = nominalReference;
	}

	@Deprecated
	public List<Intention> getIntentions() {
		return ints;
	}

	@Deprecated
	public List<dBelief> getPreconditionBeliefs() {
		return pre;
	}

	@Deprecated
	public List<dBelief> getPostconditionBeliefs() {
		return post;
	}

	@Deprecated
	public List<ReferenceResolutionRequest> getResolutionRequests() {
		return rrs;
	}

	@Deprecated
	public NominalReference getNominalReference() {
		return nref;
	}

	@Deprecated
	public LogicalForm getLogicalForm() {
		return lf;
	}

	@Deprecated
	public List<ProofWithCost> getProofs() {
		return proofs;
	}

	public TimeInterval getInterval() {
		return ival;
	}

	public Set<String> getUngroundedNominals() {
		Set<String> ungrounded = new TreeSet<String>();
		for (ReferenceResolutionRequest rr : rrs) {
			ungrounded.add(rr.nom);
		}
		return ungrounded;
	}

}
