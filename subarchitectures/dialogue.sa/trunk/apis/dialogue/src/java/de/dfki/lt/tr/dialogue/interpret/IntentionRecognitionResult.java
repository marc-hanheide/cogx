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
import de.dfki.lt.tr.dialogue.ref.ResolutionRequest;
import de.dfki.lt.tr.dialogue.slice.interpret.Interpretation;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.time.Interval;
import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import java.util.ArrayList;
import java.util.List;
import java.util.LinkedList;
import java.util.Set;
import java.util.TreeSet;

public class IntentionRecognitionResult {

	public LogicalForm lf;
	public Interval ival;

	public List<Intention> ints;
	public List<dBelief> pre;
	public List<dBelief> post;
	public NominalReference nref;
	public List<ResolutionRequest> rrs;
	public List<ProofWithCost> proofs;

	public IntentionRecognitionResult(LogicalForm lf, Interval ival, List<ProofWithCost> _proofs) {
		ints = new LinkedList<Intention>();
		pre = new LinkedList<dBelief>();
		post = new LinkedList<dBelief>();
		rrs = new LinkedList<ResolutionRequest>();
		nref = null;
		this.lf = lf;
		this.ival = ival;
		proofs = _proofs;
	}

	public Interpretation toInterpretation() {
		Set<String> ungrounded = new TreeSet<String>();
		for (ResolutionRequest rr : rrs) {
			ungrounded.add(rr.nom);
		}
		return new Interpretation(lf, ival, proofs, new ArrayList<String>(ungrounded));
	}

}
