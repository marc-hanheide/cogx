package comsys.processing.cca;

import java.lang.Class;
import java.util.*;

import comsys.datastructs.lf.*;
import comsys.lf.utils.LFUtils;
import Abducer.*;

public class MercuryUtils {

	public static String termStringEscape(String s) {
		if (s.contains("-") || Character.isUpperCase(s.charAt(0))) {
			return "'" + s + "'";
		}
		else {
			return s;
		}
	}
	
	public static String termToString(Term t) {
		String s = "";
		
		switch (t.type) {
		
		case Variable: {
				VariableTerm v = (VariableTerm) t;
				s = "V_" + v.name;
			}
			break;
		
		case Function: {
				FunctionTerm f = (FunctionTerm) t;
				s = termStringEscape(f.functor);
				if (f.args.length > 0) {
					s += "(";
					for (int i = 0; i < f.args.length; i++) {
						s += termToString(f.args[i]);
						s += (i == f.args.length-1 ? "" : ", ");
					}
					s += ")";
				}
			}
			break;
		
		default:
			s = null;
		}
		
		return s;
	}
	
	public static String predicateToString(Predicate p) {
		String s = termStringEscape(p.predSym) + "(";
		for (int i = 0; i < p.args.length; i++) {
			s += termToString(p.args[i]);
			s += (i == p.args.length-1 ? "" : ", ");
		}
		s += ")";
		
		return s;
	}

	public static String modalisedFormulaToString(ModalisedFormula mf) {
		String modStr = modalitySeqToString(mf.m);
		String predStr = predicateToString(mf.p);
		return !modStr.equals("") ? modStr + ":" + predStr : predStr;
	}
	
	public static String modalitySeqToString(Modality[] m) {
		String s = "";
		for (int i = 0; i < m.length; i++) {
			s += modalityToString(m[i]);
			s += (i == m.length-1 ? "" : ":");
		}
		return s;
	}

	public static String agentToString(Agent a) {
		switch (a) {
		case human:
			//System.err.println("hum");
			return "h";
		case robot:
			//System.err.println("rob");
			return "r";
		default:
			//System.err.println("unk");
			return "?";
		}
	}
	
	public static String modalityToString(Modality m) {
		String s = "";
		switch (m.type) {
		
		case Understanding:
			s = "understand";
			break;
		
		case Generation:
			s = "generate";
			break;
		
		case Event:
			s = "event";
			break;
			
		case Intention:
			s = "intention";
			break;
			
		case Info:
			s = "i";
			break;
		
		case AttState:
			s = "a";
			break;
		
		case K:
			s += kModalityToString((KModality) m);
			break;
			
		default:
			s = "other";
			break;
		}
		return s;
	}

	public static String kModalityToString(KModality km) {
		String s = "k(";
		switch (km.share) {
		case Private:
			//System.err.println("priv");
			s += agentToString(km.ag);
			break;
		case Attribute:
			//System.err.println("attrib");
			s += agentToString(km.ag) + "[" + agentToString(km.ag2) + "]";
			break;
		case Mutual:
			//System.err.println("mutu");
			s += "{" + agentToString(km.ag) + "," + agentToString(km.ag2) + "}";
			break;
		default:
			//System.err.println("unknown");
			s += "!";
		}
		s += ")";
		return s;
	}

}
