package comsys.processing.cca.abduction;

import java.util.*;

import comsys.utils.Pair;
import comsys.utils.Triple;
import comsys.processing.cca.AbducerUtils;
import comsys.processing.cca.BeliefUtils;
import comsys.processing.cca.MercuryUtils;
import comsys.processing.cca.PrettyPrinting;
import comsys.processing.reference.belieffactories.AbstractBeliefFactory;

import Abducer.*;
import beliefmodels.adl.*;
import beliefmodels.domainmodel.cogx.ColorProperty;
import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.ContinualFormula;
import beliefmodels.domainmodel.cogx.ContinualStatus;
import beliefmodels.domainmodel.cogx.ObjectTypeProperty;
import beliefmodels.domainmodel.cogx.ShapeProperty;
import beliefmodels.domainmodel.cogx.SuperFormula;
import beliefmodels.domainmodel.cogx.UncertainSuperFormula;
import binder.utils.BeliefModelUtils;
import binder.abstr.BeliefModelInterface;

public class BeliefModelSynchronization {

	private static final float COST_MIN = 1.0f;
	private static final float COST_MAX = 10.0f;
	
	private static final String ASSUMABLE_FUNC = "belief_model";
	
	public static boolean logging = true;
	
	public static void sync(AbducerServerPrx abducer, Belief[] kBeliefs) {
		log("syncing abducer with the given set of beliefs");
		
		String ls = "will use these beliefs: { ";
		for (int i = 0; i < kBeliefs.length; i++) {
			ls += kBeliefs[i].id;
			ls += i < kBeliefs.length-1 ? ", " : "";
		}
		ls += " }";
		log(ls);

		log("clearing K facts in the abducer");
		abducer.clearFactsByModality(Abducer.ModalityType.K);
		
		log("clearing belief model assumability function");
		abducer.clearAssumableFunction(ASSUMABLE_FUNC);

		log("adding assumables");
		// for all beliefs
		for (int i = 0; i < kBeliefs.length; i++) {
			Belief b = kBeliefs[i];

			String unionId = BeliefModelInterface.referringUnionId(b);
			AgentStatus[] as = expandedAgentStatuses(b.ags);
			
			// for all agent statuses
			for (int j = 0; j < as.length; j++) {

				addAssumablesForFormula(abducer, as[j], unionId, (SuperFormula) b.phi);
			}
		}
		log("done adding assumables");
		
		// map (unionId -> ( (expandedAgentStatusSTRING, predSym) -> (expandedAgentStatus, originAgentStatus, value) )
		// Ice doesn't override hashCode() ;( ... we need to make the key hashable, so we convert it to
		// string... nasty, but works
		Map<String, HashMap<Pair<String, String>, Triple<AgentStatus, AgentStatus, String>>> toAdd
				= new HashMap<String, HashMap<Pair<String, String>, Triple<AgentStatus, AgentStatus, String>>>();

		// for all beliefs
		for (int i = 0 ; i < kBeliefs.length; i++) {
			Belief b = kBeliefs[i];

			String unionId = BeliefModelInterface.referringUnionId(b);
			AgentStatus[] as = expandedAgentStatuses(b.ags);
			
			HashMap<Pair<String, String>, Triple<AgentStatus, AgentStatus, String>> predMap = toAdd.get(unionId);

			if (predMap == null) {
				predMap = new HashMap<Pair<String, String>, Triple<AgentStatus, AgentStatus, String>>();
			}

			// for all agent statuses
			for (int j = 0; j < as.length; j++) {

				Vector<Pair<String, String>> preds = formulaToPredSymValuePairs((SuperFormula) b.phi);
				
				// for all predicate symbol - property value pairs
				for (int k = 0; k < preds.size(); k++) {
					Pair<String, String> key = new Pair<String, String>(asToString(as[j]), preds.elementAt(k).fst);

					Triple<AgentStatus, AgentStatus, String> currentValue = predMap.get(key);
					Triple<AgentStatus, AgentStatus, String> newValue
							= new Triple<AgentStatus, AgentStatus, String>(as[j], b.ags, preds.elementAt(k).snd);
					boolean addIt = true;
					
					if (currentValue != null && implies(currentValue.second, newValue.second)) {
						// we already have this belief covered by a more grounded one
						addIt = false;
					}
					
					if (addIt) {
						predMap.put(key, newValue);
					}
				}
			}
			toAdd.put(unionId, predMap);
		}

		Iterator<String> it = toAdd.keySet().iterator();
		while (it.hasNext()) {
			String unionId = it.next();
			
			log("  " + unionId);
			HashMap<Pair<String, String>, Triple<AgentStatus, AgentStatus, String>> valMap = toAdd.get(unionId);
			
			Iterator<Pair<String, String>> predSymIt = valMap.keySet().iterator();
			while (predSymIt.hasNext()) {
				Pair<String, String> key = predSymIt.next();
				Triple<AgentStatus, AgentStatus, String> value = valMap.get(key);

				AgentStatus as = value.first;
				String predSym = key.snd;
				String valueStr = value.third;

//				log("    (" + PrettyPrinting.agentStatusToString(as) + ", " + predSym + ")");
				//log("    (" + key.fst + ", " + key.snd + ")");
				//log("     " + PrettyPrinting.agentStatusToString(as) + ", " + predSym + ")");
				//log("    =" + new Integer(key.hashCode()).toString() + "; " + new Integer(key.fst.hashCode()).toString() + ", " + new Integer(key.snd.hashCode()).toString());
				
				Modality[] mod = new Modality[] { ModalityFactory.kModality(as) };
				ModalisedFormula mf = AbducerUtils.modalisedFormula(mod,
						PredicateFactory.predicate(predSym, new Term[] { PredicateFactory.term(unionId), PredicateFactory.term(valueStr) }));
				log("    adding fact: " + MercuryUtils.modalisedFormulaToString(mf));
				abducer.addFact(mf);
				
//				log("id=" + unionId + ", ags=" + PrettyPrinting.agentStatusToString(as) + ", sym=" + predSym + ", value=" + valueStr);
			}
		}
		log("sync done");
	}

	public static final String asToString(AgentStatus as) {
		String s = "";
		if (as instanceof PrivateAgentStatus) {
			PrivateAgentStatus p = (PrivateAgentStatus) as;
			s = p.ag.id;
		}
		else if (as instanceof AttributedAgentStatus) {
			AttributedAgentStatus a = (AttributedAgentStatus) as;
			s = a.ag.id + "[" + a.ag2.id + "]";
		}
		else if (as instanceof MutualAgentStatus) {
			MutualAgentStatus m = (MutualAgentStatus) as;
			HashSet<String> ids = new HashSet<String>();
			for (int i = 0; i < m.ags.length; i++) {
				ids.add(m.ags[i].id);
			}
			Iterator<String> it = ids.iterator();
			while (it.hasNext()) {
				s += it.next();
				if (it.hasNext())
					s += ",";
			}
		}
		return s;
	}

	public static String negate(boolean polarity, String s) {
		if (polarity)
			return s;
		else
			return "not(" + s + ")";
	}
	
	public static Term toMaybeNegatedTerm(boolean polarity, String s) {
		Term arg = PredicateFactory.term(s);
		if (polarity) {
			return arg; 
		}
		else {
			return PredicateFactory.term("not", new Term[] {arg});
		}
	}
	
	public static boolean isUnknown(String s) {
		return s.startsWith("unknown");
	}
	
	public static float probToCost(float prob) {
		return COST_MAX - prob * (COST_MAX - COST_MIN);
	}

	public static void addAssumablesForFormula(AbducerServerPrx abducer, AgentStatus as, String unionId, SuperFormula sf) {
		if (sf instanceof ComplexFormula) {
    		ComplexFormula cf = (ComplexFormula) sf;
    		for (int i = 0; i < cf.formulae.length; i++) {
    			addAssumablesForFormula(abducer, as, unionId, cf.formulae[i]);
    		}
		}
		
		boolean polarity = true;
		
    	if (sf instanceof ContinualFormula) {
    		if (((ContinualFormula)sf).cstatus == ContinualStatus.assertion) {
    			//log("NOT adding an asserted formula");
    			return;
    		}
    		polarity = ((ContinualFormula)sf).polarity;
    	}

    	float prob = 1.0f;
    	if (sf instanceof UncertainSuperFormula) {
    		prob = ((UncertainSuperFormula)sf).prob;
    	}

    	float cost = probToCost(prob);
    	
    	Modality[] mod = new Modality[] {
    		ModalityFactory.kModality(as)
    	};
    	Term unionTerm = PredicateFactory.term(unionId);

    	if (sf instanceof ObjectTypeProperty) {
    		String valueString = ((ObjectTypeProperty)sf).typeValue.toString();
    		if (!isUnknown(valueString)) {
	    		Predicate p = PredicateFactory.predicate("objecttype", new Term[] {
	    				unionTerm,
	    				toMaybeNegatedTerm(polarity, valueString)
	    			});
	    		ModalisedFormula mf = AbducerUtils.modalisedFormula(mod, p);
	    		log("    adding assumable: " + MercuryUtils.modalisedFormulaToString(mf) + " / " + ASSUMABLE_FUNC + " = " + cost);
	    		abducer.addAssumable(ASSUMABLE_FUNC, mf, cost);
    		}
    	}
    	if (sf instanceof ColorProperty) {
    		String valueString = ((ColorProperty)sf).colorValue.toString();
    		if (!isUnknown(valueString)) {
	    		Predicate p = PredicateFactory.predicate("color", new Term[] {
	    				unionTerm,
	    				toMaybeNegatedTerm(polarity, valueString)
	    			});
	    		ModalisedFormula mf = AbducerUtils.modalisedFormula(mod, p);
	    		log("    adding assumable: " + MercuryUtils.modalisedFormulaToString(mf) + " / " + ASSUMABLE_FUNC + " = " + cost);
	    		abducer.addAssumable(ASSUMABLE_FUNC, mf, cost);
    		}
    	}
    	if (sf instanceof ShapeProperty) {
    		String valueString = ((ShapeProperty)sf).shapeValue.toString();
    		if (!isUnknown(valueString)) {
	    		Predicate p = PredicateFactory.predicate("shape", new Term[] {
	    				unionTerm,
	    				toMaybeNegatedTerm(polarity, valueString)
	    			});
	    		ModalisedFormula mf = AbducerUtils.modalisedFormula(mod, p);
	    		log("    adding assumable: " + MercuryUtils.modalisedFormulaToString(mf) + " / " + ASSUMABLE_FUNC + " = " + cost);
	    		abducer.addAssumable(ASSUMABLE_FUNC, mf, cost);
    		}
    	}

	}

    public static Vector<Pair<String, String>> formulaToPredSymValuePairs(SuperFormula sf) {
    	Vector<Pair<String, String>> preds = new Vector<Pair<String, String>>();
    	
    	Pair<String, String> pair = new Pair<String, String>();
    	
    	if (sf instanceof ComplexFormula) {
    		ComplexFormula cf = (ComplexFormula) sf;
    		for (int i = 0; i < cf.formulae.length; i++) {
    			preds.addAll(formulaToPredSymValuePairs(cf.formulae[i]));
    		}
    	}

    	boolean polarity = true;

    	if (sf instanceof ContinualFormula) {
    		if (((ContinualFormula)sf).cstatus == ContinualStatus.assertion) {
    			//log("NOT adding an asserted formula");
    			return preds;
    		}
    		polarity = ((ContinualFormula)sf).polarity;
    	}
    	
    	if (sf instanceof ObjectTypeProperty) {
    		String valueString = negate(polarity, ((ObjectTypeProperty)sf).typeValue.toString());
    		preds.add(new Pair<String, String>("objecttype", valueString));
    	}
    	if (sf instanceof ColorProperty) {
    		String valueString = negate(polarity, ((ColorProperty)sf).colorValue.toString());
    		preds.add(new Pair<String, String>("color", valueString));
    	}
    	if (sf instanceof ShapeProperty) {
    		String valueString = negate(polarity, ((ShapeProperty)sf).shapeValue.toString());
    		preds.add(new Pair<String, String>("shape", valueString));
    	}
    	
    	return preds;
    }

	public static AgentStatus[] expandedAgentStatuses(AgentStatus s) {
		if (s instanceof AttributedAgentStatus) {
			AttributedAgentStatus a = (AttributedAgentStatus) s;
			PrivateAgentStatus p = AbstractBeliefFactory.createPrivateAgentStatus(a.ag.id);
			return new AgentStatus[] { a, p }; 
		}
		if (s instanceof PrivateAgentStatus) {
			PrivateAgentStatus p = (PrivateAgentStatus) s;
			return new AgentStatus[] { p };
		}
		if (s instanceof MutualAgentStatus) {
			MutualAgentStatus m = (MutualAgentStatus) s;
			
			Vector<AgentStatus> ss = new Vector<AgentStatus>();
			
			// mutual belief
			ss.add(m);
			
			// TODO: attributed beliefs
			
			// private beliefs
			for (int i = 0; i < m.ags.length; i++) {
				ss.add(AbstractBeliefFactory.createPrivateAgentStatus(m.ags[i].id));
			}

			return ss.toArray(new AgentStatus[] {});
		}
		return null;
	}
   
    /**
     * True iff K(as1 PHI) implies K(as2 PHI), where PHI is a belief formula.
     * 
     * @param as1
     * @param as2
     * @return
     */
    public static boolean implies(AgentStatus as1, AgentStatus as2) {
    	AgentStatus[] expanded = expandedAgentStatuses(as1);
    	
    	for (int i = 0; i < expanded.length; i++) {
    		if (BeliefUtils.agentStatusesEqual(expanded[i], as2)) {
    	    	//log("implies(" + PrettyPrinting.agentStatusToString(as1) + ", " + PrettyPrinting.agentStatusToString(as2) + ")");
    			return true;
    		}
    	}
    	//log("!implies(" + PrettyPrinting.agentStatusToString(as1) + ", " + PrettyPrinting.agentStatusToString(as2) + ")");
    	return false;
    }

    private static void log(String str) {
		if (logging)
			System.out.println("\033[36m[BeliefModelSynchronization] " + str  + "\033[0m");
	}

}