//=================================================================
// Copyright (C) 2005,2006 Geert-Jan M. Kruijff (gj@acm.org)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
//=================================================================

package comsys.utils;

//=================================================================
// IMPORTS
//=================================================================

// ---------------------------------------------------------------
// JAVA
// ---------------------------------------------------------------
import java.util.*;


import comsys.datastructs.lf.*;
import comsys.datastructs.comsysEssentials.*;
import comsys.lf.utils.LFUtils;
import comsys.datastructs.CacheWrapper;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
    The class <b>SDRSWrapper</b> implements an SDRT-style discourse
    structure. As per (Asher and Lascarides 2003) (p.138ff) such a
    structure includes:
   
    <ul>

    <li> a set of labels <i>A</i>; here, these are <b>SDRSLabel</b>
         objects </li>

    <li> a special label <i>LAST</i> which -intuitively- indicates the
         last "clause" that was added to the structure </li> 

    <li> a function <i>F</i> that assigns to each label a formula,
         i.e. an <b>SDRSFormula</b> object. </li>

    </ul>

    In addition, the class keeps track of what nominal variables have
    been used, and in what SDRSLf objects they appear.

	<h4>Comsys.mk3: Specifics for porting to ComsysEssentials.idl LFEssentials.idl, LFUtils</h4>
	
	The current <b>SDRSSstructure</b> (comsys.mk3) differs from the previous 
	version (comsys.mk2) in several significant ways. These all arise from 
	the fact that we now use the IDL structs for representing logical forms, 
	(accessible through the LFUtils class),	and the structures in the comsys 
	working memory. 
	
	<ul>
	<li> variable binding: an LFBindings object represents the bindings for the 
		 variables in a logical form; the object refers to another LogicalForm 
		 object, by ID. The Hashtable <i>varbindings</i> stores LFBindings, 
		 keyed by the label for the SDRS structure in which the logical form 
		 is stored.  
		 
		 
		 Both LFBindings and LogicalForm
		 are stored in the BoundLF (wrapper) object.  
	</ul>
	

    @version 061026 (started 050609)
    @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class  SDRSUtils {
	
	static boolean  logging = true ;
	
	static int lCounter = 0 ;
	
	public static final String PLF_TYPE = "plf";
	public static final String RELATION_TYPE = "relation";
    
    //-----------------------------------------------------------------
    // ADD METHODS
    //-----------------------------------------------------------------
   
    /**
       The method <i>addFormula</i> adds a formula to the SDRS
       structure. Should the formula object not yet have a suitable
       label, then a new label is automatically generated. Should the
       formula be a logical form, we check that the nominal variables
       introduced in the logical form have unique names wrt the set of
       variables already in use in the SDRS structure.

       <h4>Note</h4>

       We assume that that the formula/logical form has identifiers
       that are all appropriate given the namespace of the dialogue
       context model. 

     */

    public static SDRS addFormula (SDRS sdrs, SDRSFormula f) { 
		log("Adding a formula to the discourse model");
    	String fLabel = f.label ;
		if (fLabel.equals("unknown")) { 
			fLabel = generateLabel();
			f.label = fLabel;
		} 
		// end if.. check whether a label is needed
		// Check for logical forms; if LF, then ensure proper naming
		// of nominal identifiers.
		SDRSType fType = getFormulaType(f); 
			f.tprec = sdrs.LAST;
			sdrs.LAST = fLabel;
		
			// Add the formula to the SDRS
			sdrs.A = (String[]) LFUtils.resizeArray(sdrs.A, sdrs.A.length+1);			
			sdrs.A[sdrs.A.length-1] = fLabel;
			sdrs.F.mapping = (LabelFormulaPair[]) LFUtils.resizeArray(sdrs.F.mapping, sdrs.F.mapping.length+1);
			LabelFormulaPair lfp = new LabelFormulaPair() ;
			lfp.formula = f;
			lfp.label = fLabel;
			if (f != null && f.caches != null) 
				lfp.formula.caches = f.caches ;
			else {
				lfp.formula.caches = new Cache[0];
			}
			sdrs.F.mapping[sdrs.F.mapping.length-1] = lfp;
	
			if (fType.type.equals(PLF_TYPE)) { 
					
			// ------------
			log("Formula will be stored with id "+fLabel);
			//----
			Iterator<PackedNominal> nomIter = Arrays.asList(LFUtils.plfGetNominals(lfp.formula.type.plf.packedLF)).iterator();

			String nomseq = "";
			while (nomIter.hasNext()) { 
				PackedNominal nom = (PackedNominal) nomIter.next();
				String nomv = nom.nomVar;
			//	Vector<String> occV = (nomvars.containsKey(nomv))? (Vector<String>)nomvars.get(nomv): new Vector<String>();
			//	occV.addElement(fLabel); 
			//	nomvars.put(nomv,occV);
				nomseq = (nomIter.hasNext())?nomseq+nomv+", ":nomseq+nomv;
			} // end while
			log("Nominals stored for ["+fLabel+"]: "+nomseq);
		} 
    	
		return sdrs;
    } // end method


    public static void addCacheToFormula(SDRSFormula f, Cache cache) {
    	if (f.caches != null) {
    		f.caches = (Cache[]) LFUtils.resizeArray(f.caches, f.caches.length +1);
    		f.caches[f.caches.length-1] = cache;
    	}
    	else {
    		f.caches = new Cache[1];
    		f.caches[0] = cache;
    	} 	    	
    }
    

    public static void replaceCacheInFormula(SDRSFormula f, Cache cache) {
    	for (int i=0; i < f.caches.length ; i++) {
    		if (f.caches[i].CacheId.equals(cache.CacheId)) {
    			f.caches[i] = cache;
    		}
    	}
    }
    
    /**
       The method <i>lfNamespaceTransfer</i> takes a logical form, and
       ensures that the variable names are unique in the variable
       namespace for the (current) SDRS structure. 
       
       <b>NOTE</b>: if the lf contains variables with the same name, we take
       this as intended and do not rename them (sawi, 060106)

       <h4>Warning</h4> We're using the name of the SDA-lf nominal
       variable as the name for its equivalence class. Also, we're
       initializing the CPP result as <tt>unknown_t</tt> by default.
    */

    public static PackedLFs plfNamespaceTransfer (PackedLFs plf) { 
    	// WARNING: IMPLEMENTATION PROBABLY BUGGY !! - plison
    	// METHOD SHOULD IDEALLY BE REWRITTEN FROM SCRATCH
	
    	/**
    	String rootDVP = plf.packedLF.root;
		HashMap<String, String> replacedIds = new HashMap<String, String>();
		HashSet<String> lfIds = new HashSet<String>();
	
		Cache bindings = new Cache() ;
		int maxcnt = -1;
		
		PackedLogicalForm plfcopy = LFUtils.plfClone(plf.packedLF);
		
		// First, update the counters for variables in the namespace,
		// to ensure that each counter is higher than the max number
		// for a letter in the logical form.
		for (Iterator<PackedNominal> nomCIter = Arrays.asList(LFUtils.plfGetNominals(plf.packedLF)).iterator(); nomCIter.hasNext(); ) { 
			PackedNominal nom = (PackedNominal) nomCIter.next();
			String nomvar = nom.nomVar;
			String[] split = null;
			try { 
				split = nomvar.split("\\d");
			} catch (Exception e) { 
				System.out.println(e.getMessage());
			} // end try..catch
			String letter = split[0]; // nomvar.substring(0,1);
			String number = nomvar.substring(letter.length(),nomvar.length());
			
			if (nomvar.indexOf("dvprob") == -1) { 
				if (nomvarCntrs.containsKey(letter)) {
					int cntr = ((Integer)nomvarCntrs.get(letter)).intValue();
					int realnum = (new Integer(number)).intValue(); 
					if (realnum >= cntr) { cntr = realnum+1; }
					nomvarCntrs.put(letter,new Integer(cntr));
				} else { 
					if (!number.equals("")) { 
						int realnum = (new Integer(number)).intValue(); 
						nomvarCntrs.put(letter,new Integer(realnum+1));
					} else { 
						nomvarCntrs.put(letter,new Integer(1));
					} // end if..else whether number been set 
				} // end if.. check whether to raise the number
			}
		} // end for over the nominals to establish namespace extension
		// Next, iterate over the nominals to establish new names. 
		
		log("NOMINALS <<");
		for (Iterator<PackedNominal> nomIter = Arrays.asList(LFUtils.plfGetNominals(plf.packedLF)).iterator(); nomIter.hasNext(); ) {
			PackedNominal lfn = (PackedNominal) nomIter.next();
			log(lfn.toString());
		}
		log(">> NOMINALS");
		
		for (Iterator<PackedNominal> nomIter = Arrays.asList(LFUtils.plfGetNominals(plf.packedLF)).iterator(); nomIter.hasNext(); ) { 
			PackedNominal nom = (PackedNominal) nomIter.next();
			String nomvar = nom.nomVar;
			
			String[] split = null;
			try { 
				split = nomvar.split("\\d");
			} catch (Exception e) { 
				System.out.println(e.getMessage());
			} // end try..catch
			String letter = split[0]; 
			String number = nomvar.substring(letter.length(),nomvar.length());
			if (nomvarCntrs.containsKey(letter)) {
				int cntr = ((Integer)nomvarCntrs.get(letter)).intValue();
				cntr = cntr+1;
				nomvarCntrs.put(letter,new Integer(cntr));
				String newnomvar = letter+(cntr);
				// if this nomvar occurred before in the formula, we don't want to generate a new name
				if(lfIds.contains(nomvar)){
					newnomvar = replacedIds.get(nomvar);
				} 
				replacedIds.put(nomvar,newnomvar);

				PackedNominal nomcopy = LFUtils.plfNominalCloneUnderReplacements(nom,replacedIds);

				nomcopy.nomVar = newnomvar;

				plfcopy = LFUtils.plfReplaceNomvar(plfcopy,nomvar,newnomvar);
				
				
			} else { 
				if (nomvar.indexOf("dvprob") == -1) { 
					if (number.equals("")) { number="1"; }
					nomvarCntrs.put(letter,new Integer(number));
				} 	
			} // end if..else.. check whether seen nom id before
			
			lfIds.add(nomvar); //store the ids of the current lf
		} // end for.. over lf nominals
		// Construct and return the BoundLF object
		// log("lfNamespaceTransfer bindings: " + bindings.toMsgElementBody());
		
		PackedLFs newPlf = new PackedLFs();
		newPlf.packedLF = plfcopy;
		newPlf.id = plf.id ;
		newPlf.phon = plf.phon ;
		newPlf.finalized = plf.finalized;
		
		return newPlf ;
		*/
    	return plf;
    } // end lfNamespaceTransfer

    //-----------------------------------------------------------------
    // GET METHODS
    //-----------------------------------------------------------------

    /**
       The method <i>getFirstMention</i> returns the identifier of the
       first SDRSFormula in which the given nominal variable was
       mentioned. If no such SDRSFormula exists, then "unknown" is
       returned.
    */

    public static String getFirstMention (SDRS sdrs, String nv) { 
		String result = "unknown";
		for (int i=0; i < sdrs.F.mapping.length ; i++) {
			if (getFormulaType(sdrs.F.mapping[i].formula).type.equals(PLF_TYPE)) {
				PackedLFs plf = sdrs.F.mapping[i].formula.type.plf;
				for (int j=0; j < plf.packedLF.pNodes.length;j++) {
					for (int k=0 ; k < plf.packedLF.pNodes[j].packedNoms.length ; k++) {
						if (plf.packedLF.pNodes[j].packedNoms[k].nomVar.equals(nv)) {
							return sdrs.F.mapping[i].formula.label ;
						}
					}
				}
			}
		} 
		return result;
    } 

    /**
       The method <i>getFormula</i> returns the formula with the given
       identifier. If there is no formula with the given identifier,
       <tt>null</tt> is returned.
    */

    public static SDRSFormula getFormula (SDRS sdrs, String id) { 
		SDRSFormula result = null ;
		for (int i=0; i < sdrs.F.mapping.length; i++) {
			if (sdrs.F.mapping[i] != null && sdrs.F.mapping[i].label.equals(id)) {
				return sdrs.F.mapping[i].formula ;
			}
		}
		return result;
    } // end method

    /**
       The method <i>getFormulas</i> returns an iterator over the
       identifiers for formulas. Each formula can then be retrieved
       using the <i>getFormula</i> method.

       @see #getFormula
    */
    
    public static Iterator<SDRSFormula> getFormulas (SDRS sdrs) { 
    	Vector<SDRSFormula> formulas = new Vector<SDRSFormula>();
    	for (int i=0;i<sdrs.F.mapping.length;i++) {
    		formulas.add(sdrs.F.mapping[i].formula);
    	} 
    	return formulas.iterator();
    }

    
    public static String getRefNomvar (SDRSFormula form, String initNomvar) {    	
    	String nomvar = initNomvar;  
    	for (int i=0; i < form.caches.length ; i++) {
    		Cache cache = form.caches[i];
    		if (cache.CacheId.equals("discRef")) {
    			CacheWrapper bindingsWrapper = new CacheWrapper(cache);
    			nomvar = bindingsWrapper.getDiscRef(initNomvar);
    		}
    	}   	
    	return nomvar;
    }
    
    /**
       The method <i>getFormulaType</i> returns the formula type.
    */

    public static SDRSType getFormulaType (SDRSFormula f) { 
    	if (f.type != null) {
    		return f.type;
    	}
    		else return null;
    	
    } // end method

    /**
       The method <i>getLAST</i> returns the identifier of the formula
       (clause) that was last added.
    */

    public static String getLAST (SDRS sdrs) { return sdrs.LAST; }

    

    public static SDRSFormula getLastFormula (SDRS sdrs) { 
    	return getFormula(sdrs, sdrs.LAST);
    }

    
    /**
       The method <i>getLastMention</i> returns the identifier of the
       most recent SDRSFormula in which the given nominal variable was
       mentioned. If no such SDRSFormula exists, then "unknown" is
       returned.
    */

    public static String getLastMention (SDRS sdrs, String nv) { 
		String result = "unknown";
		for (int i=sdrs.F.mapping.length-1; i <= 0 ; i--) {
			if (getFormulaType(sdrs.F.mapping[i].formula).type.equals(PLF_TYPE)) {
				PackedLFs plf = sdrs.F.mapping[i].formula.type.plf;
				for (int j=0; j < plf.packedLF.pNodes.length;j++) {
					for (int k=0 ; k < plf.packedLF.pNodes[j].packedNoms.length ; k++) {
						if (plf.packedLF.pNodes[j].packedNoms[k].nomVar.equals(nv)) {
							return sdrs.F.mapping[i].formula.label ;
						}
					}
				}
			}
		} // end if.. check for existence
		return result;
    } // end getLastMention

    /**
       The method <i>getMentions</i> returns a Vector object with the
       labels of the SDRSFormula objects in which the given nominal
       variable was mentioned. The first element in the Vector is its
       first mention, the last element the last (and thus most recent)
       mention. If the nominal variable has not been mentioned/used,
       then the Vector will be empty (size=0).
    */

    public static Vector<String> getMentions (SDRS sdrs, String nv) { 
		Vector<String> result = new Vector<String>();
		for (int i=0; i < sdrs.F.mapping.length ; i++) {
			if (getFormulaType(sdrs.F.mapping[i].formula).type.equals(PLF_TYPE)) {
				PackedLFs plf = sdrs.F.mapping[i].formula.type.plf;
				for (int j=0; j < plf.packedLF.pNodes.length;j++) {
					for (int k=0 ; k < plf.packedLF.pNodes[j].packedNoms.length ; k++) {
						if (plf.packedLF.pNodes[j].packedNoms[k].nomVar.equals(nv)) {
							result.add(sdrs.F.mapping[i].formula.label) ;
						}
					}
				}
			}
		} 
		return result;
    } // end getMentions

    
    //=================================================================
    // COMPUTATION METHODS
    //=================================================================

    /**
       The method <i>generateLabel</i> generates a new (unique) label.
    */

    public static String generateLabel () { 
		String labelid = "pi"+lCounter;
		lCounter++;
		return labelid;
    } // end method

    

    public static String getNextLabel () { 
		String labelid = "pi"+ new Integer(lCounter+1).toString();
		return labelid;
    } // end method

    //=================================================================
    // I/O METHODS
    //=================================================================

    
    public static String createDOTSpecsLastFormulaAndLastDM(SDRSFormula formula, SDRSFormula dm) {
    
    	String formSpecs = createDOTSpecsLastFormula(formula);
    	String result = formSpecs.substring(0, formSpecs.length() -2);
    	result += "\n compound=true;\n";
    	String dmSpecs = createDOTSpecsLastFormula(dm);
    	result += dmSpecs.substring(12, dmSpecs.length()-2).replace("clusterSDRSFormula", "clusterSDRSFormula2");
    	String rootName = LFUtils.plfGetPackingNode
    	(formula.type.plf.packedLF, formula.type.plf.packedLF.root).root;
    	result += dm.type.relation.relType + " -> " + rootName + 
    		" [label=\"tprec\", lhead=clusterSDRSFormula, ltail=clusterSDRSFormula2];\n";
    	result += "\n}\n";
    	return result;
    }
    
    
    public static String createDOTSpecsLastFormula(SDRSFormula formula) {

    	String result = "digraph G {\n";

    	result += "subgraph clusterSDRSFormula" +" {\n";
   // 	result += "label = \"SDRS Label #" + "\";\n";
    	result += "color=red;\n";
    	if (getFormulaType(formula).type.equals(PLF_TYPE)) {
    		result += "subgraph clusterPLF"+" {\n";
    		String specsPLF = LFUtils.createDOTSpecs(formula.type.plf.packedLF);
    		result += specsPLF.substring(12, specsPLF.length()-2);
    		result += "color=blue;\n";
    		result += "label=\"Packed Logical Form " +"\";\n";
    		result += "\n}\n";

        	for (int j=0; j < formula.caches.length ; j++) {
        		String countCaches = (new Integer(j+1)).toString();
        		result += "subgraph clusterCache" + countCaches + " {\n";
        		result += "color=blue;\n";
        		result += "label=\""+ formula.caches[j].CacheId +"\";\n";
        		for (int k=0; k < formula.caches[j].mapping.associations.length ; k++) {
        			if (formula.caches[j].mapping.associations[k].relType.equals("SINGULAR")) {
        				long id1 = formula.caches[j].mapping.associations[k].id1[0];
        				long id2 = formula.caches[j].mapping.associations[k].id2[0];
        				String label = formula.caches[j].content1[(int)id1] ;
        				String nominal = formula.caches[j].content2[(int)id2] ;
        				result += label + "c"  + "[label=\"" + label + "\"];\n";
        				result += label + "c"  + " -> " + nominal + " [style=dashed];\n";
        			}
        		}
        		result += "\n}\n";
        	}
    	}
    	else if (getFormulaType(formula).type.equals(RELATION_TYPE)) {
    		result += formula.type.relation.relType + " [shape=box, label=\"" + 
    		formula.type.relation.relType + "(" + formula.type.relation.args[0] + ", " + 
    		formula.type.relation.args[1] + ")\"];\n"; 
    	}

    	result += "\n}\n";

    	result = result + "\n}";
    	return result;    	
    }

    public static String createDOTSpecsFullDisc (SDRS sdrs) {
    	String result = "digraph G {\n";
    	result += "compound=true;\n";
    	/**
    	 result += "{rank=source;\n";
    	result += "masterRoot[style=invis];\n";
    	result += "}\n";
    	result += "{rank=same;\n";
      	for (int i=0; i < sdrs.F.mapping.length -1 ; i++) {
      		result += "relation" + (new Integer(i+1)).toString() + 
      		(new Integer(i+2)).toString() + "[label=\"tprec\"];\n";
      	}
      	result += "}\n compound=true;\n";
  		result += "nodesep=1.0 ;\n";    	
      	for (int i=0; i < sdrs.F.mapping.length -1 ; i++) {
      		result += "masterRoot -> " + "relation" + (new Integer(i+1)).toString() + 
      		(new Integer(i+2)).toString() + "[style=invis];\n";
      	}
      	
    	
      	for (int i=0; i < sdrs.F.mapping.length -1 ; i++) {
      		SDRSFormula form1 = sdrs.F.mapping[i].formula;
      		SDRSFormula form2 = sdrs.F.mapping[i+1].formula;
      		String curIncr = new Integer(i+1).toString();
      		String nextIncr = new Integer(i+2).toString();
      		String head = "invis" +  curIncr;
      		String tail = "invis" +  nextIncr;

     		result += "relation" + curIncr + nextIncr + " -> " + head + 
     				"[tailport=w, headport=n, lhead=clusterSDRSFormula" + curIncr + 
     				", style=bold, constraint=true];\n";
    		result += tail + " -> " + "relation" + curIncr + nextIncr + 
    		"[arrowhead=none, tailport=n, headport=e, ltail=clusterSDRSFormula" + nextIncr + 
				 ", style=bold, constraint=true];\n";
      	}
      	

      	for (int i=1; i < sdrs.F.mapping.length  ; i=i+2) {
  		result += 	"relation" + (new Integer(i)).toString() + (new Integer(i+1)).toString()
  					 + " -> " + "invis" + (new Integer(i)).toString()
  					 + "[tailport=w, headport=n, lhead=clusterSDRSFormula" + (new Integer(i)).toString() + 
  					 ", style=bold, constraint=true];\n";
  		result +=    "rootNom" + (new Integer(i+1)).toString() + " -> " +
  					 "relation" + (new Integer(i)).toString() + (new Integer(i+1)).toString() + 
  					 "[arrowhead=none, tailport=n, headport=e, ltail=clusterSDRSFormula" + (new Integer(i+1)).toString() + 
  					 ", style=bold, constraint=true];\n";
		}
	*/
      	
      	int countPLF = 1;
      	String afterSpecs = "";
    	for (int i=0; i < sdrs.F.mapping.length ; i++) {
    		String count = (new Integer(i+1)).toString();
    		result += "subgraph clusterSDRSFormula"+ count +" {\n";
    		result += "label = \"SDRS Formula #"+ count + "\";\n";
    		result += "color=red;\n";
    		SDRSFormula formula = sdrs.F.mapping[i].formula;
    		
    		if (getFormulaType(formula).type.equals(PLF_TYPE)) {
    			result += "ordering=out;\n";
				result += "invis" + count + " [style=invis];\n";
				result += "subgraph clusterPLF"+count+" {\n";
    			String specsPLF = LFUtils.createDOTSpecs(formula.type.plf.packedLF);
    			result += specsPLF.substring(12, specsPLF.length()-2);
    			result += "color=blue;\n";
    			result += "label=\"Packed Logical Form "+ countPLF +"\";\n";
    			countPLF++;
    			result += "\n}\n";
    		}
    		
   			else if (getFormulaType(formula).type.equals(RELATION_TYPE)) {
    			result += "ordering=out;\n";
    			result += "invis" + count + " [style=invis];\n";	
    			result += formula.type.relation.relType  + count + " [shape=box, label=\"" + 
    	    		formula.type.relation.relType + "(" + formula.type.relation.args[0] + ", " + 
    	    		formula.type.relation.args[1] + ")\"];\n";
    			if ((i +1 < sdrs.F.mapping.length) && 
    					getFormulaType(sdrs.F.mapping[i+1].formula).type.equals(PLF_TYPE) &&
    					sdrs.F.mapping[i+1].formula.label.equals(formula.type.relation.args[1])) {
    				afterSpecs = "invis" + count + " -> " + "invis" + (new Integer(i+2)).toString() +
    				" [label=\"dialogue move\" ltail=clusterSDRSFormula"+count + " lhead=clusterSDRSFormula" + 
    				(new Integer(i+2)).toString() + "];\n";
    			}
    	    	}
    		
    		for (int j=0; j < sdrs.F.mapping[i].formula.caches.length ; j++) {
    			String countCaches = (new Integer(j+1)).toString();
    			result += "subgraph clusterCache"+ count + countCaches + " {\n";
    			result += "color=blue;\n";
    			result += "label=\""+ sdrs.F.mapping[i].formula.caches[j].CacheId +"\";\n";
    			for (int k=0; k < sdrs.F.mapping[i].formula.caches[j].mapping.associations.length ; k++) {
    				if (sdrs.F.mapping[i].formula.caches[j].mapping.associations[k].relType.equals("SINGULAR")) {
    					long id1 = sdrs.F.mapping[i].formula.caches[j].mapping.associations[k].id1[0];
    					long id2 = sdrs.F.mapping[i].formula.caches[j].mapping.associations[k].id2[0];
    					String label = sdrs.F.mapping[i].formula.caches[j].content1[(int)id1];
    					String nominal = sdrs.F.mapping[i].formula.caches[j].content2[(int)id2];
    					result += label + "c" + count + "[label=\"" + label + "\"];\n";
    					result += label + "c" + count  + " -> " + nominal + " [style=dashed];\n";
    				}
    			}
    			result += "}\n\n";
    		}	
    		result += "}\n\n";
    		result += afterSpecs;
    		afterSpecs = "";
    	}
    	   	
    	result = result + "}\n";
    	return result;
    }


    public static SDRSFormula getFormulaFromPLF (SDRS sdrs, String plfId) {
    	for (int i=0; i < sdrs.F.mapping.length ; i++) {
    		SDRSFormula form = sdrs.F.mapping[i].formula ;
    		if (getFormulaType(form).type.equals(PLF_TYPE)) {
    			String plfId2 = form.type.plf.id;
    			if (plfId2.equals(plfId)) {
    				return form;
    			}
    		}
    	}
    	return null;
    }
    
    public static int getIndexInSDRS (SDRS sdrs, SDRSFormula formula) {
    	for (int i=0; i < sdrs.F.mapping.length ; i++) {
    		SDRSFormula form = sdrs.F.mapping[i].formula ;
    		if (formula.equals(form)) {
    			return i+1;
    		}
    	}
    	return -1;
    }
	public static void SDRSToGraph(SDRS sdrs, String graphName, boolean generatePNG) {

		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

		String DOTText = createDOTSpecsFullDisc(sdrs) ;

		if (!DOTText.equals("")) {
			LFUtils.writeDOTFile(DOTText,DOTFile);

			if (generatePNG) { 
				try	{
					Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			}

		}

		// showPNGGraph(PNGFile);
	}
	

	public static void FormulaToGraph(SDRSFormula formula, String graphName, boolean generatePNG) {

		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

		String DOTText = createDOTSpecsLastFormula(formula) ;

		if (!DOTText.equals("")) {
			LFUtils.writeDOTFile(DOTText,DOTFile);

			if (generatePNG) {
			try	{
				Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			}
		}

		// showPNGGraph(PNGFile);
	}

	

	public static void FormulaAndDMToGraph(SDRSFormula formula, SDRSFormula dm, String graphName, boolean generatePNG) {

		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

		log("before generation of DOT specs");
		String DOTText = createDOTSpecsLastFormulaAndLastDM(formula, dm) ;
		log("DOT specs for last formula and DM generated");
		if (!DOTText.equals("")) {
			LFUtils.writeDOTFile(DOTText,DOTFile);
			log("DOT file written");
			if (generatePNG) {
			try	{
				Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
				log("After runtime");
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			}
		}

		log("end of method");
		// showPNGGraph(PNGFile);
	}

    //=================================================================
    // MISCELLANEOUS METHODS
    //=================================================================

    private static void log (String m) {
        if (logging) { System.out.println("[SDRSUtils] "+m); }
    }

    public static String toString (SDRS sdrs) { 
    	
    	// TO BE REWRITTEN (NO STRING VERSION OF PACKED LOGICAL FORMS YET)
    	/**
		String result = "\n\nSDRStructure size: "+F.size();

		Iterator fIter = F.keySet().iterator();
		while (fIter.hasNext()) { 
			String fId = (String) fIter.next();
			 SDRSLf lf  = (SDRSLf) F.get(fId);
			result = result+"\n"+(lf.toString()); 
		} // end while
		*/
		return "";
    } // end toString()

    //=================================================================
    // MAIN METHOD
    //=================================================================

    public static void main (String[] args) { 
		/**
		SDRSStructure struct = new SDRSStructure (); 	
		SDRSLf lf = new SDRSLf ();
		LogicalForm l1 = LFUtils.convertFromString("@b1:state(be ^ <Mood>ind ^ <Restr>(t1:thing ^ this ^ <VisCtxt>(a1:thing ^ ant ^ <Number>sg ^ <Proximity>proximal)) ^ <Scope>(b2:thing ^ ball ^  <Delimitation>existential ^  <Quantification>specific_singular ^ <Property>(r1:color ^ red)))");
		lf.setLf(l1);
		struct.addFormula(lf);
		SDRSLf lf2 = new SDRSLf ();
		LogicalForm l2 = LFUtils.convertFromString("@b1:state(be ^ <Mood>ind ^ <Restr>(i1:thing ^ it) ^ <Scope>(b2:size ^ big)) ");
		lf2.setLf(l2);
		struct.addFormula(lf2);
		struct.toString();
		*/
    } // end main

} // end class definition 
