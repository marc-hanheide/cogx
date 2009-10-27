package comsys.processing.cca;

import java.util.HashSet;
import java.util.Vector;

import comsys.processing.reference.belieffactories.AbstractBeliefFactory;

import beliefmodels.adl.Agent;
import beliefmodels.adl.AgentStatus;
import beliefmodels.adl.AttributedAgentStatus;
import beliefmodels.adl.Belief;
import beliefmodels.adl.MutualAgentStatus;
import beliefmodels.adl.PrivateAgentStatus;
import beliefmodels.adl.SpatioTemporalFrame;
import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.ContinualFormula;
import beliefmodels.domainmodel.cogx.ContinualStatus;
import beliefmodels.domainmodel.cogx.LogicalOp;
import beliefmodels.domainmodel.cogx.SuperFormula;
import beliefmodels.domainmodel.cogx.UnionRefProperty;

public class BeliefUtils {

	public static SuperFormula removeUnionRefs(SuperFormula f) {
		if (f instanceof ComplexFormula) {
			ComplexFormula cf = (ComplexFormula) f;
			Vector<SuperFormula> newFormulae = new Vector<SuperFormula>();
			for (int i = 0; i < cf.formulae.length; i++) {
				SuperFormula tmp = removeUnionRefs(cf.formulae[i]);
				if (tmp != null) {
					newFormulae.add(tmp);
				}
			}
			// TODO: copy constructor
			ComplexFormula newCf = new ComplexFormula();
			newCf.id = cf.id;
			newCf.op = cf.op;
			newCf.prob = cf.prob;
			newCf.formulae = newFormulae.toArray(new SuperFormula[]{});
			return newCf;
		}
		if (f instanceof UnionRefProperty) {
			return null;
		}
		return f;
	}

	public static ComplexFormula addUnionRef(ComplexFormula f, String unionId) {
		SuperFormula[] newFormulae = new SuperFormula[f.formulae.length+1];
		UnionRefProperty u = new UnionRefProperty();
		u.id = "ur";
		u.cstatus = ContinualStatus.proposition;
		u.polarity = true;
		u.prob = 1.0f;
		u.unionRef = unionId;
		newFormulae[0] = u;
		for (int i = 0; i < f.formulae.length; i++) {
			newFormulae[i+1] = f.formulae[i];
		}
		f.formulae = newFormulae;
		return f;
	}

	public static void mergeFormulaIntoBelief (Belief initBelief, SuperFormula formula) {
		
		AgentStatus ags = initBelief.ags;
		String id = initBelief.id;
		SpatioTemporalFrame sigma = initBelief.sigma;
		
		SuperFormula phi;

		if (initBelief.phi instanceof ComplexFormula) {
			
			Vector<SuperFormula> formulae = new Vector<SuperFormula>();
			
			for (int i = 0; i < ((ComplexFormula)initBelief.phi).formulae.length; i++) {
				formulae.add(((ComplexFormula)initBelief.phi).formulae[i]);
			}
			
			if (formula instanceof ComplexFormula) {
			for (int i = 0; i < ((ComplexFormula)formula).formulae.length ; i++) {
				if (!formulae.contains(((ComplexFormula)formula).formulae[i])) {
					formulae.add(((ComplexFormula)formula).formulae[i]);
				}
			}
			}
			else {
				if (!formulae.contains(formula)) {
					formulae.add(formula);
				}
			}
			
			SuperFormula[] formulaeArray = new SuperFormula[formulae.size()];
			formulaeArray = formulae.toArray(formulaeArray);
			LogicalOp op = ((ComplexFormula)initBelief.phi).op;
			float prob = ((ComplexFormula)initBelief.phi).prob;
			String phiId = initBelief.phi.id;
			
			phi = new ComplexFormula(phiId, prob, op, formulaeArray);
		}
		else {
			SuperFormula[] formulaeArray = new SuperFormula[2];
			formulaeArray[0] = (SuperFormula) initBelief.phi;
			formulaeArray[1] = formula;
			String phiId = initBelief.phi.id;
			
			phi = new ComplexFormula(phiId, 1.0f, LogicalOp.and, formulaeArray);
		}
	}
	
	public static SuperFormula changeAssertionsToPropositions(SuperFormula sf, float prob) {
		if (sf instanceof ComplexFormula) {
			ComplexFormula cf = (ComplexFormula) sf;
			for (int i = 0; i < cf.formulae.length; i++) {
				cf.formulae[i] = changeAssertionsToPropositions(cf.formulae[i], prob);
			}
			return cf;
		}
		if (sf instanceof ContinualFormula) {
			ContinualFormula cof = (ContinualFormula) sf;
			if (cof.cstatus == ContinualStatus.assertion) {
				cof.cstatus = ContinualStatus.proposition;
				cof.prob = prob;
			}
			return cof;
		}
		return sf;
	}

	public static SuperFormula swapPolarityOfAssertions(SuperFormula sf) {
		if (sf instanceof ComplexFormula) {
			ComplexFormula cf = (ComplexFormula) sf;
			for (int i = 0; i < cf.formulae.length; i++) {
				cf.formulae[i] = swapPolarityOfAssertions(cf.formulae[i]);
			}
			return cf;
		}
		if (sf instanceof ContinualFormula) {
			ContinualFormula cof = (ContinualFormula) sf;
			cof.polarity = !cof.polarity;
			return cof;
		}
		return sf;
	}

	
	public static boolean formulaHasAssertions(SuperFormula f) {
		if (f instanceof ComplexFormula) {
			ComplexFormula cf = (ComplexFormula) f;
			for (int i = 0; i < cf.formulae.length; i++) {
				if (formulaHasAssertions(cf.formulae[i])) {
					return true;
				}
			}
			return false;
		}
		if (f instanceof ContinualFormula) {
			ContinualFormula cf = (ContinualFormula) f;
			return cf.cstatus == ContinualStatus.assertion;
		}
		return false;
	}

	public static boolean agentStatusesEqual(AgentStatus as1, AgentStatus as2) {
		if (as1 instanceof PrivateAgentStatus && as2 instanceof PrivateAgentStatus) {
			PrivateAgentStatus p1 = (PrivateAgentStatus) as1;
			PrivateAgentStatus p2 = (PrivateAgentStatus) as2;
			return p1.ag.id.equals(p2.ag.id);
		}
		else if (as1 instanceof AttributedAgentStatus && as2 instanceof AttributedAgentStatus) {
			AttributedAgentStatus a1 = (AttributedAgentStatus) as1;
			AttributedAgentStatus a2 = (AttributedAgentStatus) as2;
			return a1.ag.id.equals(a2.ag.id) && a1.ag2.id.equals(a2.ag2.id);
		}
		else if (as1 instanceof MutualAgentStatus && as2 instanceof MutualAgentStatus) {
			MutualAgentStatus m1 = (MutualAgentStatus) as1;
			MutualAgentStatus m2 = (MutualAgentStatus) as2;
	
			HashSet<String> ids1 = new HashSet<String>();
			HashSet<String> ids2 = new HashSet<String>();
			for (int i = 0; i < m1.ags.length; i++) {
				ids1.add(m1.ags[i].id);
			}
			for (int j = 0; j < m2.ags.length; j++) {
				ids2.add(m2.ags[j].id);
			}
			return ids1.equals(ids2);
		}
		return false;
	}

	public static AgentStatus raise(AgentStatus as, Agent a) {
		if (as instanceof PrivateAgentStatus) {
			PrivateAgentStatus pas = (PrivateAgentStatus) as;
			if (!pas.ag.id.equals(a.id)) {
				return attribute(pas, a);
			}
		}
		else if (as instanceof AttributedAgentStatus) {
			AttributedAgentStatus aas = (AttributedAgentStatus) as;
			if (aas.ag.id.equals(a.id)) {
				return raiseToMutual(aas);
			}
		}
		else if (as instanceof MutualAgentStatus) {
			MutualAgentStatus mas = (MutualAgentStatus) as;
			return addToGroup(mas, a);
		}
		
		return null;
	}
	
	public static AttributedAgentStatus attribute(PrivateAgentStatus as, Agent a) {
		return new AttributedAgentStatus(as.ag, a);
	}

	public static MutualAgentStatus raiseToMutual(AttributedAgentStatus as) {
		return new MutualAgentStatus(new Agent[] {as.ag, as.ag2});
	}
	
	public static MutualAgentStatus addToGroup(MutualAgentStatus as, Agent a) {
		for (int i = 0; i < as.ags.length; i++) {
			if (as.ags[i].id.equals(a.id)) {
				// already present
				return as;
			}
		}
		Agent[] newAgs = new Agent[as.ags.length+1];
		for (int i = 0; i < as.ags.length; i++) {
			newAgs[i] = as.ags[i];
		}
		newAgs[newAgs.length-1] = a;
		
		MutualAgentStatus result = new MutualAgentStatus();
		result.ags = newAgs;
		return result;
	}

}
