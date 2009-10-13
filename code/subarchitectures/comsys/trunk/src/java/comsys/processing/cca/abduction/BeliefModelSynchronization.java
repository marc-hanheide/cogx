package comsys.processing.cca.abduction;

import java.util.*;

import comsys.processing.cca.AbducerUtils;
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
import binder.utils.BeliefModelUtils;
import binder.abstr.BeliefModelInterface;

public class BeliefModelSynchronization {

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
				
				Modality[] mod = new Modality[] { AbducerUtils.kModality(as) };
				ModalisedFormula mf = AbducerUtils.modalisedFormula(mod,
						AbducerUtils.predicate(predSym, new Term[] { AbducerUtils.term(unionId), AbducerUtils.term(valueStr) }));
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
    		if (agentStatusesEqual(expanded[i], as2)) {
    	    	//log("implies(" + PrettyPrinting.agentStatusToString(as1) + ", " + PrettyPrinting.agentStatusToString(as2) + ")");
    			return true;
    		}
    	}
    	//log("!implies(" + PrettyPrinting.agentStatusToString(as1) + ", " + PrettyPrinting.agentStatusToString(as2) + ")");
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
    		return a1.ag.id.equals(a2.ag.id) && a1.ag2.equals(a2.ag2.id);
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

	private static void log(String str) {
		if (logging)
			System.out.println("\033[36m[BeliefModelSynchronization] " + str  + "\033[0m");
	}

}