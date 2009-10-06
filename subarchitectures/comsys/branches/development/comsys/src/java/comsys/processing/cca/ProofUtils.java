package comsys.processing.cca;

import java.util.*;
import beliefmodels.adl.*;
import beliefmodels.domainmodel.cogx.*;
import Abducer.*;

public class ProofUtils {

	/**
	 * Given a sequence of queries, return all queries marked as asserted.
	 * 
	 * @param qs the sequence of queries (proof)
	 * @return all elements of qs that are of type AssertedQuery
	 */
	public static AssertedQuery[] filterAsserted(MarkedQuery[] qs) {
		ArrayList<AssertedQuery> list = new ArrayList<AssertedQuery>();
		for (int i = 0; i < qs.length; i++) {
			if (qs[i] instanceof AssertedQuery) {
				list.add((AssertedQuery) qs[i]);
			}
		}
		return list.toArray(new AssertedQuery[0]);
	}
	
	/**
	 * Given a sequence of queries, return all queries that are marked as assumed.
	 * 
	 * @param qs the sequence of queries (proof)
	 * @return all elements of qs that are of type AssumedQuery
	 */
	public static AssumedQuery[] filterAssumed(MarkedQuery[] qs) {
		ArrayList<AssumedQuery> list = new ArrayList<AssumedQuery>();
		for (int i = 0; i < qs.length; i++) {
			if (qs[i] instanceof AssumedQuery) {
				list.add((AssumedQuery) qs[i]);
			}
		}
		return list.toArray(new AssumedQuery[0]);		
	}

	/**
	 * Extract asserted beliefs from a proof.
	 * 
	 * @param proof the proof
	 * @return list of beliefs based of the proof's assertions
	 */
	public static Belief[] extractAssertedBeliefs(MarkedQuery[] proof) {
		AssertedQuery[] assertions = filterAsserted(proof);
		ArrayList<Belief> list = new ArrayList<Belief>();

		for (int i = 0; i < assertions.length; i++) {
			Belief b = assertionToBelief(assertions[i]);
			if (b != null) {
				list.add(b);
			}
		}
		return list.toArray(new Belief[0]);
	}
	
	/**
	 * Extract assumed beliefs from a proof.
	 * 
	 * @param proof the proof
	 * @return list of beliefs based on the proof's assumptions
	 */
	public static Belief[] extractAssumedBeliefs(MarkedQuery[] proof) {
		AssumedQuery[] assumptions = filterAssumed(proof);
		ArrayList<Belief> list = new ArrayList<Belief>();

		for (int i = 0; i < assumptions.length; i++) {
			Belief b = assumptionToBelief(assumptions[i]);
			if (b != null) {
				list.add(b);
			}
		}
		return list.toArray(new Belief[0]);
	}

	//-----------------------------------------------------------------------------
	
	/**
	 * Convert a query marked as `asserted' to a belief.
	 * 
	 * @param q the query
	 * @return the belief, null if the query cannot be converted to a belief
	 */
	public static Belief assertionToBelief(AssertedQuery q) {
		if (q.body.m.length == 1) {
			Belief b = modalityToBeliefStub(q.body.m[0]);
			if (b != null) {
				ContinualFormula f = predicateToContinualFormula(q.body.p);
//				if (f == null) System.err.println("formula null");
				f.cstatus = ContinualStatus.assertion;
				b.phi = f;
			}
			return b;
		}
		else {
			return null;
		}
	}
	
	/**
	 * Convert a query marked as `assumed' to a belief.
	 * 
	 * @param q the query
	 * @return the belief, null if the query cannot be converted to a belief
	 */
	public static Belief assumptionToBelief(AssumedQuery q) {
		if (q.body.m.length == 1) {
			Belief b = modalityToBeliefStub(q.body.m[0]);
			if (b != null) {
				ContinualFormula f = predicateToContinualFormula(q.body.p);
//				if (f == null) System.err.println("formula null");
				f.cstatus = ContinualStatus.proposition;
				b.phi = f;
			}
			return b;
		}
		else {
			return null;
		}
	}
	
	/**
	 * Create a belief stub (a belief with a null formula) from a modality.
	 * 
	 * FIXME: currently, the spatiotemporal frame is set to null. 
	 * 
	 * @param m the modality
	 * @return belief stub, null if the modality is not convertible
	 */
	public static Belief modalityToBeliefStub(Modality m) {
		if (m instanceof KModality) {
			Belief b = new Belief();
			b.ags = kModalityToAgents((KModality) m);
			b.sigma = kModalityToSpatioTemporalFrame((KModality) m);
			b.phi = null;
			return b;
		}
		else {
			// not a K modality
			return null;
		}
	}

	/**
	 * Convert the abducer representation of agent to a belief model agent.
	 * 
	 * TODO: this shouldn't be hardcoded.
	 * 
	 * @param aAgent
	 * @return
	 */
	public static beliefmodels.adl.Agent agent(Abducer.Agent aAgent) {
		beliefmodels.adl.Agent bAgent = new beliefmodels.adl.Agent();
		switch (aAgent) {
			case Human: bAgent.id = "human"; break;
			case Robot: bAgent.id = "robot"; break;
			default: bAgent = null; break;
		}
		return bAgent;
	}
	
	public static beliefmodels.adl.Agent[] kModalityToAgents(KModality m) {
		switch (m.share) {

			case Private:
				return new beliefmodels.adl.Agent[] { agent(m.act) };

			case Mutual:
				return new beliefmodels.adl.Agent[] { agent(m.act), agent(m.pat) };
			
			case Attribute:
				return new beliefmodels.adl.Agent[] { agent(m.act), agent(m.pat) }; // FIXME: this isn't right
				
			default:
				// not a recognised agent relation
				return null;
		}
	}

	public static SpatioTemporalFrame kModalityToSpatioTemporalFrame(KModality m) {
		return null;
	}

	public static String termToString(Term t) {
		if (t instanceof FunctionTerm)
			return ((FunctionTerm) t).functor;
		else
			return null;	
	}
	
	public static Color termToColor(Term t) {
		if (t instanceof FunctionTerm)
			return Color.convert(((FunctionTerm) t).functor);
		else
			return null;
	}
	
	public static Shape termToShape(Term t) {
		if (t instanceof FunctionTerm)
			return Shape.convert(((FunctionTerm) t).functor);
		else
			return null;
	}

	public static ObjectType termToObjectType(Term t) {
		if (t instanceof FunctionTerm)
			return ObjectType.convert(((FunctionTerm) t).functor);
		else
			return null;
	}
	
	/**
	 * Convert predicate to a continual formula.
	 * 
	 * NOTE: the formula's continual status is unset.
	 *
	 * @param p the predicate
	 * @return corresponding continual formula, null if conversion not possible
	 */
	public static ContinualFormula predicateToContinualFormula(Predicate p) {

//		System.out.println("p2cf: " + p.predSym);
		
		if (p.predSym.equals("color")) {
//			System.err.println("color");
			// color(Object, Value)
			ColorProperty prop = new ColorProperty();
			prop.id = termToString(p.args[0]);  // Object
			prop.colorValue = termToColor(p.args[1]);  // Value
			prop.prob = 1.0f;
			return prop;
		}
		else if (p.predSym.equals("shape")) {
			// shape(Object, Value)
			ShapeProperty prop = new ShapeProperty();
			prop.id = termToString(p.args[0]);  // Object
			prop.shapeValue = termToShape(p.args[1]);  // Value
			prop.prob = 1.0f;
			return prop;
		}
		else if (p.predSym.equals("objecttype")) {
			// objecttype(Object, Value)
			ObjectTypeProperty prop = new ObjectTypeProperty();
			prop.id = termToString(p.args[0]); // Object
			prop.typeValue = termToObjectType(p.args[1]); // Value
			prop.prob = 1.0f;
			return prop;
		}
		
		return null;
	}

	public static ModalisedFormula[] proofToFacts(MarkedQuery[] proof) {
		ModalisedFormula[] result = new ModalisedFormula[proof.length];
		for (int i = 0; i < proof.length; i++) {
			result[i] = proof[i].body;
		}
		return result;
	}
}
