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
		log("commencing sync");
		log("clearing K facts");
		abducer.clearFactsByModality(Abducer.ModalityType.K);
		
		// map (unionId -> ( (expandedAgentStatus, predSym) -> (originAgentStatus, value) )
		Map<String, HashMap<Pair<AgentStatus, String>, Pair<AgentStatus, String>>> toAdd
				= new HashMap<String, HashMap<Pair<AgentStatus, String>, Pair<AgentStatus, String>>>();
		
		// for all beliefs
		for (int i = 0 ; i < kBeliefs.length; i++) {
			Belief b = kBeliefs[i];

			String unionId = BeliefModelInterface.referringUnionId(b);
			AgentStatus[] as = expandedAgentStatuses(b.ags);
			
			HashMap<Pair<AgentStatus, String>, Pair<AgentStatus, String>> predMap = toAdd.get(unionId);

			if (predMap == null) {
				predMap = new HashMap<Pair<AgentStatus, String>, Pair<AgentStatus, String>>();
			}

			// for all agent statuses
			for (int j = 0; j < as.length; j++) {

				Vector<Pair<String, String>> preds = formulaToPredSymValuePairs((SuperFormula) b.phi);
				
				// for all predicate symbol - property value pairs
				for (int k = 0; k < preds.size(); k++) {
					Pair<AgentStatus, String> key = new Pair<AgentStatus, String>(as[j], preds.elementAt(k).fst);

					Pair<AgentStatus, String> currentValue = predMap.get(key);
					Pair<AgentStatus, String> newValue = new Pair<AgentStatus, String>(b.ags, preds.elementAt(k).snd);
					boolean addIt = true;
					
					if (currentValue != null && !implies(currentValue.fst, newValue.fst)) {
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
			
			HashMap<Pair<AgentStatus, String>, Pair<AgentStatus, String>> valMap = toAdd.get(unionId);
			
			Iterator<Pair<AgentStatus, String>> predSymIt = valMap.keySet().iterator();
			while (predSymIt.hasNext()) {
				Pair<AgentStatus, String> key = predSymIt.next();
				Pair<AgentStatus, String> value = valMap.get(key);
				
				AgentStatus as = key.fst;
				String predSym = key.snd;
				String valueStr = value.snd;
				
				Modality[] mod = new Modality[] { AbducerUtils.kModality(as) };
				ModalisedFormula mf = AbducerUtils.modalisedFormula(mod,
						AbducerUtils.predicate(predSym, new Term[] { AbducerUtils.term(unionId), AbducerUtils.term(valueStr) }));
				log("adding fact: " + MercuryUtils.modalisedFormulaToString(mf));
				abducer.addFact(mf);
				
//				log("id=" + unionId + ", ags=" + PrettyPrinting.agentStatusToString(as) + ", sym=" + predSym + ", value=" + valueStr);
			}
		}
/*
			//log("got a belief, id=" + model.k[i]);
			Modality[] mod = new Modality[] { AbducerUtils.kModality(b.ags) };
			String unionId = BeliefModelInterface.referringUnionId(b);

			//log("inspecting feats");
			
			Predicate[] preds = formulaToPredicates(unionId, (SuperFormula) b.phi).toArray(new Predicate[] {});
			for (int j = 0; j < preds.length; j++) {
				ModalisedFormula mf = AbducerUtils.modalisedFormula(mod, preds[j]);
				log("adding " + MercuryUtils.modalisedFormulaToString(mf) + " ... from " + kBeliefs[i].id);
			}
		}
*/
		log("sync done");
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
   
    	if (sf instanceof ContinualFormula) {
    		if (((ContinualFormula)sf).cstatus == ContinualStatus.assertion) {
    			//log("NOT adding an asserted formula");
    			return preds;
    		}
    	}
    	
    	if (sf instanceof ObjectTypeProperty) {
    		String valueString = ((ObjectTypeProperty)sf).typeValue.toString();
    		preds.add(new Pair<String, String>("objecttype", valueString));
    	}
    	if (sf instanceof ColorProperty) {
    		String valueString = ((ColorProperty)sf).colorValue.toString();
    		preds.add(new Pair<String, String>("color", valueString));
    	}
    	if (sf instanceof ShapeProperty) {
    		String valueString = ((ShapeProperty)sf).shapeValue.toString();
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
    		if (expanded[i].equals(as2)) {
    			return true;
    		}
    	}
    	return false;
    }
    
	private static void log(String str) {
		if (logging)
			System.out.println("\033[36m[BeliefModelSynchronization] " + str  + "\033[0m");
	}

}