package comsys.processing.cca;

import java.util.*;
import beliefmodels.adl.*;
import beliefmodels.domainmodel.cogx.*;
import comsys.processing.reference.belieffactories.AbstractBeliefFactory;
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
				ComplexFormula f = predicateToComplexFormula(q.body.p, ContinualStatus.assertion);
//				if (f == null) System.err.println("formula null");
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
				ComplexFormula f = predicateToComplexFormula(q.body.p, ContinualStatus.proposition);
//				if (f == null) System.err.println("formula null");
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
			b.ags = kModalityToAgentStatus((KModality) m);
			System.out.println(b.ags.getClass());
			b.sigma = kModalityToSpatioTemporalFrame((KModality) m);
			b.phi = null;
			//System.out.println("in modalityToBeliefStub: b.ags=" + PrettyPrinting.agentStatusToString(b.ags));
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
			case human: bAgent.id = "human"; break;
			case robot: bAgent.id = "robot"; break;
			default: bAgent = null; break;
		}
		return bAgent;
	}
	
	public static AgentStatus kModalityToAgentStatus(KModality m) {
		//System.out.println("kModalityToAgentStatus: share=" + m.share.toString() + ", act=" + m.act.toString() + ", pat=" + m.pat.toString());
		switch (m.share) {

			case Private:
				return AbstractBeliefFactory.createPrivateAgentStatus(m.ag.toString());

			case Mutual:
				return AbstractBeliefFactory.createMutualAgentStatus(new String[] {m.ag.toString(), m.ag2.toString()});
			
			case Attribute:
				return AbstractBeliefFactory.createAttributedAgentStatus(m.ag.toString(), m.ag2.toString());
				
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
	 * @param p the predicate
	 * @return corresponding continual formula, null if conversion not possible
	 */
	public static ComplexFormula predicateToComplexFormula(Predicate p, ContinualStatus cstatus) {

		ComplexFormula f = new ComplexFormula();
		f.id = "p2cf";
		f.op = LogicalOp.and;
		f.formulae = new ContinualFormula[2];
		
		String ref = "";
			
		System.out.println("p2cf: " + p.predSym);
		
		ContinualFormula cprop = null;
		
		if (p.predSym.equals("color")) {
			System.err.println("color");
			// color(Object, Value)
			ColorProperty prop = new ColorProperty();
			ref = termToString(p.args[0]);  // Object
			prop.colorValue = termToColor(p.args[1]);  // Value
			cprop = prop;
		}
		else if (p.predSym.equals("shape")) {
			System.err.println("shape");
			// shape(Object, Value)
			ShapeProperty prop = new ShapeProperty();
			ref = termToString(p.args[0]);  // Object
			prop.shapeValue = termToShape(p.args[1]);  // Value
			cprop = prop;
		}
		else if (p.predSym.equals("objecttype")) {
			System.err.println("objecttype");
			// objecttype(Object, Value)
			ObjectTypeProperty prop = new ObjectTypeProperty();
			ref = termToString(p.args[0]); // Object
			prop.typeValue = termToObjectType(p.args[1]); // Value
			cprop = prop;
		}
		
		UnionRefProperty unionRef = new UnionRefProperty();
		unionRef.cstatus = ContinualStatus.proposition;
		unionRef.prob = 1.0f;
		unionRef.unionRef = ref;
		unionRef.id = "ref";
		f.formulae[0] = unionRef;
		
		cprop.cstatus = cstatus;
		cprop.prob = 1.0f;
		cprop.id = "prop";
		f.formulae[1] = cprop;
		
		return f;
	}

	public static ModalisedFormula[] proofToFacts(MarkedQuery[] proof) {
		ModalisedFormula[] result = new ModalisedFormula[proof.length];
		for (int i = 0; i < proof.length; i++) {
			result[i] = proof[i].body;
		}
		return result;
	}
}
