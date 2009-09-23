package comsys.processing.collab;

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
	
	public static Predicate lfToPredicate(LogicalForm lf) {
		return null;
	}

	public static String termToString(Term t) {
		String s = "";
		// determine whether `a' is a Term or a Var
		if (t.variable) {
			s = "V" + t.name;
		}
		else {
			s = termStringEscape(t.name);
			if (t.args.length > 0) {
				s += "(";
				for (int i = 0; i < t.args.length; i++) {
					s += termToString(t.args[i]);
					s += (i == t.args.length-1 ? "" : ", ");
				}
				s += ")";
			}
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
		return modalitySeqToString(mf.m) + ":" + predicateToString(mf.p);
	}
	
	public static String modalitySeqToString(Modality[] m) {
		String s = "[";
		for (int i = 0; i < m.length; i++) {
			s += modalityToString(m[i]);
			s += (i == m.length-1 ? "" : ", ");
		}
		s += "]";
		return s;
	}

	public static String agentToString(Agent a) {
		switch (a) {
		case Human:
			return "h";
		case Robot:
			return "r";
		default:
			return "?";
		}
	}
	
	public static String modalityToString(Modality m) {
		String s = "";
		switch (m.type) {
		
		case Event:
			s = "e";
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
			s += agentToString(km.act);
			break;
		case Attribute:
			s += agentToString(km.act) + "[" + agentToString(km.pat) + "]";
			break;
		case Mutual:
			s += "{" + agentToString(km.act) + "," + agentToString(km.pat) + "}";
			break;
		default:
			s += "!";
		}
		s += ")";
		return s;
	}

/*
 * This has been taken from comsys.lf.utils.LFUtils
	
	private static String depNomToMercString (LFNominal nom, LogicalForm lf) { 
		if (nom == null) { 
			// System.out.println("WARNING!!"); 
			return null; 
		} else { 
			String result = "\"" + nom.nomVar+"\"::\""+nom.sort + "\""; 

			boolean props = (!nom.prop.prop.equals("")); 	

			Iterator fIter = lfNominalGetFeatures(nom);
			boolean feats = (fIter.hasNext());

			Iterator rIter = lfNominalGetRelations(nom);
			boolean rels = (rIter.hasNext()); 

			// add a conjunction after the nominal if there is more
			if (props || feats || rels) { result = result + " ^ "; }
			// while (pIter.hasNext()) { 
			//	String prop = (String) pIter.next();
			String prop = nom.prop.prop;
			if (!prop.equals("")) { 
				result = result + "p(\"" + prop + "\")";
				//if (pIter.hasNext()) { result = result + " ^ ";} 
				props = true;
			} // end if over proposition
			//if (props) {if (feats == false && rels==false) { result = result + ")"; }}
			//if (props && !(feats || rels)) { result = result + ")"; }
			// the result now does not have a conjunction at the end: 
			// we need one if there are both props and feats
			if (props & feats) { result = result + " ^ ";}
			while (fIter.hasNext()) { 
				Feature feat = (Feature) fIter.next(); 
				String val  = feat.value;
				result = result + "r(\""+feat.feat+"\", p(\""+val + "\"))";
				if (fIter.hasNext()) { result = result + " ^ "; }
			} // end for over features
			// the result now does not have a conjunction at the end.  if
			// there were no feats, just props, then there is no
			// conjunction either
			// if (props || feats) { result=(rIter.hasNext())?result+" ^ ":result+"#)"; }    // CLOSING BRACKET
			if (props) { 
				if (feats == true && rels == false) { 
					// if features but no more relations
					//result = result + ")"; 
				} else { 
					// if no features, or there are relations ...
					// but we do not want this if there were no features
					if (rels) { result = result + " ^ "; } 
				} 
			} // if propositions

			if (props == false) { if (feats==true && rels==false) { result = result + ")"; } else { result = result + " ^ ";} }

			while (rIter.hasNext()) { 
				LFRelation rel = (LFRelation) rIter.next();
				result = result + " r(\""+rel.mode+"\", ";
				LFNominal depnom = lfGetNominal(lf,rel.dep); 
				log("depNomToString: looking for dependent nom of head: "+rel.head+" with rel "+rel.mode+" and nomid "+rel.dep+", finding: "+depnom);
				if(rel.coIndexedDep ==true) { // we only want to generate <RelMode>var1:type1, not the complete nominal
					log("depnomtostr GENERATE coindexed!");
					result = result + "\"" + rel.dep + "\"::\"" +depnom.sort + "\"";
				}else {
					// added by plison: avoid running into infinite loops in case 
					// cycles are present in the graph
					
					boolean hasCycle = false ;
					for (int i = 0 ; i < depnom.rels.length & !hasCycle; i++) {
						String dep1 = depnom.rels[i].dep;
						
						if (dep1.equals(nom.nomVar)) {
							hasCycle = true ;
						}
						LFNominal nomdep1 = lfGetNominal(lf,dep1);
						for (int j = 0 ; j < nomdep1.rels.length & !hasCycle; j++) {
							if (nomdep1.rels[j].dep.equals(nom.nomVar)) {
								hasCycle = true;
							}
						}
					}
					if (!hasCycle) {
						String depStr = depNomToMercString(depnom,lf);
						if (depStr != null) { result = result +depNomToMercString(depnom,lf); }
					}
				}
				result += ")";
				// ADD CLOSING BRACKET
				// result = result + ")"; 
				if (rIter.hasNext()) { result = result + " ^ "; } 
			} // end for over relations
			// if (rels) { result = result + "+"+nom.getNomvar()+")";} 
//			if (rels) { result = result + ")";} 
			// System.out.println("***** return from depNomToString: "+result);
			return result; 
		}
	} // end depNomToString


	private static String rootNomToMercString (LFNominal nom, LogicalForm lf) { 
		String result = "@(\""+nom.nomVar+"\"::\""+nom.sort+"\", "; 

		// Iterator pIter = nom.getPropositions();
		boolean props = (!nom.prop.prop.equals("")); 	

		Iterator fIter = lfNominalGetFeatures(nom);
		boolean feats = (fIter.hasNext());
		Iterator rIter = lfNominalGetRelations(nom); 
		boolean rels = rIter.hasNext(); 

		//while (pIter.hasNext()) { 
		String prop = nom.prop.prop;
		if (props) { 
			result = result + "p(\"" + prop + "\")";
		} // end for over propositions
		// the result now does not have a conjunction at the end: 
		// we need one if there are both props and feats
		if (props & feats) { result = result + " ^ ";}
		while (fIter.hasNext()) { 
			Feature feat = (Feature) fIter.next(); 
			result = result + "r(\""+feat.feat+"\", p(\""+feat.value +"\"))";
			if (fIter.hasNext()) { result = result + " ^ "; }
		} // end for over features
		// the result now does not have a conjunction at the end.  if
		// there were no feats, just props, then there is no
		// conjunction either
		if (props || feats) { result=(rIter.hasNext())?result+" ^ ":result; } 
		while (rIter.hasNext()) { 
			LFRelation rel = (LFRelation) rIter.next();
			result = result + " r(\""+rel.mode+"\", ";

			log("Now trying to print dep with nomvar "+rel.dep+" in relation "+rel.mode+" from "+rel.head);

			LFNominal depnom = lfGetNominal(lf, rel.dep); 
			if(rel.coIndexedDep == true){ // we only want to generate <RelMode>var1:type1, not the complete nominal
				log("root nom 2 str GENERATE coindexed!");
				result = result + "\"" + rel.dep+"\"::\""+depnom.sort + "\"";
			} else {
				result = result + depNomToMercString(depnom,lf);
			}
			// ADD CLOSING BRACKET
			result += ")";
			// result = result + "/"+nom.getNomvar()+")";
			if (rIter.hasNext()) { result = result + " ^ "; } 
		} // end for over relations
		// if (rels) { result = result + "-)";} 
		//	 CAUTION : sw 051015 changed here, (removed "if (rels), because something goes wrong in QuestionLF.constructLFSubtree
		// have not checked if everything else still works!
		//	if (rels) { result = result + ")";}  
		//{ result = result + ")";} 
		result += ")";
		return result; 
	} // end rootNomToString

	
	public static String lfToMercString (LogicalForm lf) { 
		String result = ""; 
		LFNominal rootnom = lf.root;

		// JUST IN CASE the LF is empty...
		if (rootnom==null) {
			return null;
		} 		
		else {
			// the root is not a dialogue point, so just do the
			// root and embed everything directly under it.

			// collect the root dependents
			Vector rootDependents = LFUtils.lfCollectNomvars(rootnom,lf);

			result = result + rootNomToMercString(rootnom,lf);
			
			for (int i=0; i < lf.noms.length; i++) {
				LFNominal nom = lf.noms[i];
				
				if (!result.contains(nom.nomVar)) {
					log("Nominal ["+nom.nomVar+"] not subordinated to the root ["+rootnom.nomVar+"]");
					result = result + " ^ " + rootNomToMercString(nom,lf);
					log("Result: "+result);
				} else {
					log("Nominal ["+nom.nomVar+"] subordinated to the root ["+rootnom.nomVar+"]");				
				} 
			}
			
			// result = result + "*)";
			result = result + "";
		}  // end if..else check whether dialogue act or not
		return result;
	} // end toString(); 

	public static String lfNominalToMercString (LFNominal nom) { 
		if (nom.nomVar.equals("")) { System.out.println("ERROR! EMPTY NOMINAL"); }
	
		String res= "xx@(\""+nom.nomVar+"\"::\""+nom.sort+"\", ";
		// NEXT IS OLD
		//Iterator piter = this.getPropositions();
		//while (piter.hasNext()) { 
		//	String p = (String) piter.next(); 
		//	res = res+p;
		//	if (piter.hasNext()) { res=res+" ^"; }
		//}
		// NEXT IS NEW
		res = res+nom.prop.prop;
		// Cycle over features
		Iterator fiter = lfNominalGetFeatures(nom);
		if (!nom.prop.prop.equals("")) { 
			if (fiter.hasNext() || java.lang.reflect.Array.getLength(nom.rels) > 0) { res=res+" ^"; }		
		}
		while (fiter.hasNext()) { 
			Feature feat = (Feature)fiter.next();
			String f = feat.feat;
			String v = feat.value;
			res=res+" r(\""+f+"\", p(\""+v+"\")";
			if (fiter.hasNext() || java.lang.reflect.Array.getLength(nom.rels) > 0) { res=res+" ^"; }
		} // end while over features
		Iterator riter = lfNominalGetRelations(nom);
		while (riter.hasNext()) { 
			LFRelation r = (LFRelation) riter.next();
			res=res+"r(\""+r.mode+"\""; 
			String dp = r.dep; 
			res = res+dp;
			if (riter.hasNext()) { res=res+" ^"; } 
		} // end while over relations
		res=res+")";
		return res;
	} // end lfNominalToString

*/

}
