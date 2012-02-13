package comsys.processing.cca.abduction;

import java.util.*;

import cast.cdl.CASTTime;
import beliefmodels.adl.*;
import beliefmodels.adl.Agent;
import beliefmodels.domainmodel.cogx.*;
import binder.abstr.BeliefModelInterface;
import comsys.processing.cca.BeliefUtils;
import comsys.processing.cca.Counter;
import comsys.processing.cca.MercuryUtils;
import comsys.processing.reference.belieffactories.AbstractBeliefFactory;
import Abducer.*;

public class ProofUtils {

	private static Counter counter = new Counter("proofs");
	
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
		Vector<Belief> bs = new Vector<Belief>();

		for (int i = 0; i < assertions.length; i++) {
			Belief b = assertionToBelief(assertions[i]);
			// find a belief that has the same agent status and unionref
			for (int j = 0; j < bs.size(); j++) {
				Belief oldB = bs.elementAt(j);
//				System.err.println("looking at it");
//				System.err.println("ref old=" + BeliefModelInterface.referringUnionId(oldB) + ", new=" + BeliefModelInterface.referringUnionId(b));
				if (BeliefUtils.agentStatusesEqual(oldB.ags, b.ags) 
						&& BeliefModelInterface.referringUnionId(oldB).equals(BeliefModelInterface.referringUnionId(b))) {
//					System.err.println("merging");
//					String unionId = BeliefModelInterface.referringUnionId(oldB);
					// merge them
//					oldB.phi = BeliefUtils.removeUnionRefs((SuperFormula) oldB.phi);
					SuperFormula addF = BeliefUtils.removeUnionRefs((SuperFormula) b.phi);
					BeliefUtils.mergeFormulaIntoBelief(oldB, addF);
//					oldB.phi = BeliefUtils.addUnionRef(f, unionId)addUnionRef()
					b = null;
					break;
				}
			}
			if (b != null) {
				bs.add(b);
			}
		}
		return bs.toArray(new Belief[0]);
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
	 * @param m the modality
	 * @return belief stub, null if the modality is not convertible
	 */
	public static Belief modalityToBeliefStub(Modality m) {
		if (m instanceof KModality) {
			Belief b = new Belief(
					"stub",
					AbstractBeliefFactory.createHereNowFrame(new Agent[]{new Agent("robot")}),
					kModalityToAgentStatus((KModality) m),
					null,
					new CASTTime(0, 0));
			//System.out.println(b.ags.getClass());
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
	
	public static beliefmodels.adl.Agent proofAgentStringToAgent(String s) {
		beliefmodels.adl.Agent bAgent = new beliefmodels.adl.Agent();
		if (s.equals("h")) {
			bAgent.id = "human";
			return bAgent;
		}
		else if (s.equals("r")) {
			bAgent.id = "robot";
			return bAgent;
		}
		else {
			return null;
		}
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
	
	public static boolean termPolarity(Term t) {
		if (t instanceof FunctionTerm) {
			if (((FunctionTerm)t).functor.equals("not")) {
				System.err.println("neg polarity at " + MercuryUtils.termToString(t));
				return false;
			}
			else {
				return true;
			}
		}
		else {
			return true;
		}
	}
	
	public static Color termToColor(Term t) {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			if (ft.functor.equals("not"))
				return Color.convert(((FunctionTerm)ft.args[0]).functor);
			else
				return Color.convert(ft.functor);
		}
		else {
			return null;
		}
	}
	
	public static Shape termToShape(Term t) {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			if (ft.functor.equals("not"))
				return Shape.convert(((FunctionTerm)ft.args[0]).functor);
			else
				return Shape.convert(ft.functor);
		}
		else {
			return null;
		}
	}

	public static ObjectType termToObjectType(Term t) {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			if (ft.functor.equals("not"))
				return ObjectType.convert(((FunctionTerm)ft.args[0]).functor);
			else
				return ObjectType.convert(ft.functor);
		}
		else {
			return null;
		}
	}
	
	/**
	 * Convert predicate to a continual formula.
	 * 
	 * @param p the predicate
	 * @return corresponding continual formula, null if conversion not possible
	 */
	public static ComplexFormula predicateToComplexFormula(Predicate p, ContinualStatus cstatus) {

		ComplexFormula f = new ComplexFormula();
		f.id = counter.inc("pred2cplxf");
		f.op = LogicalOp.and;
		f.formulae = new ContinualFormula[2];
		f.prob = 1.0f;
		
		String ref = "";
			
		//System.out.println("p2cf: " + p.predSym);
		
		ContinualFormula cprop = null;
		
		if (p.predSym.equals("color")) {
			//System.err.println("color");
			// color(Object, Value)
			ColorProperty prop = new ColorProperty();
			ref = termToString(p.args[0]);  // Object
			prop.polarity = termPolarity(p.args[1]);
			prop.colorValue = termToColor(p.args[1]);  // Value
			prop.prob = 1.0f;
			cprop = prop;
		}
		else if (p.predSym.equals("shape")) {
			//System.err.println("shape");
			// shape(Object, Value)
			ShapeProperty prop = new ShapeProperty();
			ref = termToString(p.args[0]);  // Object
			prop.polarity = termPolarity(p.args[1]);
			prop.shapeValue = termToShape(p.args[1]);  // Value
			prop.prob = 1.0f;
			cprop = prop;
		}
		else if (p.predSym.equals("objecttype")) {
			//System.err.println("objecttype");
			// objecttype(Object, Value)
			ObjectTypeProperty prop = new ObjectTypeProperty();
			ref = termToString(p.args[0]); // Object
			prop.polarity = termPolarity(p.args[1]);
			prop.typeValue = termToObjectType(p.args[1]); // Value
			prop.prob = 1.0f;
			cprop = prop;
		}
		
		UnionRefProperty unionRef = new UnionRefProperty();
		unionRef.cstatus = ContinualStatus.proposition;
		unionRef.prob = 1.0f;
		unionRef.unionRef = ref;
		unionRef.id = counter.inc("ref");
		f.formulae[0] = unionRef;
		
		cprop.cstatus = cstatus;
		cprop.prob = 1.0f;
		cprop.id = counter.inc("prop");
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