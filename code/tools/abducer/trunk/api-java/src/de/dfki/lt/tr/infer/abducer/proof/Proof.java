package de.dfki.lt.tr.infer.abducer.proof;

import de.dfki.lt.tr.infer.abducer.lang.AssumabilityFunction;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.NullAssumabilityFunction;
import java.util.ArrayList;
import java.util.List;

public class Proof
implements Comparable<Proof> {

	private final List<MarkedQuery> proof;
	private double cost;
	
	public Proof(ProofWithCost pwc) {
		this.proof = pwc.proof;
		this.cost = pwc.cost;
	}

	public Proof(ModalisedAtom matom, AssumabilityFunction f) {
		proof = new ArrayList<MarkedQuery>();
		proof.add(new UnsolvedQuery(matom, f));
		cost = 0.0;
	}

	public Proof(ModalisedAtom matom) {
		this(matom, new NullAssumabilityFunction());
	}

	public double getCost() {
		return cost;
	}

	public List<MarkedQuery> getMarkedQueries() {
		return proof;
	}

	public List<ModalisedAtom> toModalisedAtoms() {
		List<ModalisedAtom> result = new ArrayList<ModalisedAtom>();
		for (MarkedQuery mq : proof) {
			result.add(mq.atom);
		}
		return result;
	}

	public boolean isStable() {
		for (MarkedQuery mq : proof) {
			if (mq instanceof UnsolvedQuery || mq instanceof AssertedQuery) {
				return false;
			}
		}
		return true;
	}

	public boolean isUnsolved() {
		for (MarkedQuery mq : proof) {
			if (mq instanceof UnsolvedQuery) {
				return true;
			}
		}
		return false;
	}

	public Assertion getFirstAssertion(ProofSet ps) {
		for (int i = 0; i < proof.size(); i++) {
			MarkedQuery mq = proof.get(i);
			if (mq instanceof AssertedQuery) {
				return new Assertion(ps, this, i);
			}
		}
		return null;
	}

	public <T> T interpret(ProofInterpreter<T> interpreter) {
		List<ModalisedAtom> matoms = new ArrayList<ModalisedAtom>();
		for (MarkedQuery q : proof) {
			matoms.add(q.atom);
		}
		return interpreter.interpret(matoms, getCost());
	}

	public boolean hasAssertions() {
		for (MarkedQuery mq : proof) {
			if (mq instanceof AssertedQuery) {
				return true;
			}
		}
		return false;
	}

	@Override
	public int compareTo(Proof other) {
		return Double.compare(getCost(), other.getCost());
	}

}
