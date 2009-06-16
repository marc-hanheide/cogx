//=================================================================
//Copyright (C) 2006 Geert-Jan M. Kruijff (gj@dfki.de)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION 
//=================================================================

package comsys.lf.utils;

//=================================================================
//IMPORTS
//=================================================================

//-----------------------------------------------------------------
//COMSYS IMPORTS
//-----------------------------------------------------------------


//-----------------------------------------------------------------
//LOGICAL FORM REPRESENTATION IMPORTS
//-----------------------------------------------------------------

import comsys.datastructs.lf.*;

import comsys.arch.ComsysException;
//-----------------------------------------------------------------
//JAVA IMPORTS
//-----------------------------------------------------------------

import java.io.*;
import java.util.*;

import javax.swing.JOptionPane;
import javax.swing.ImageIcon;
//-----------------------------------------------------------------
//OpenCCG IMPORTS
//-----------------------------------------------------------------

import opennlp.ccg.synsem.*;
import opennlp.ccg.realize.*;
import java.util.Enumeration;

import org.jdom.*;
import org.jdom.output.XMLOutputter;

//=================================================================
//CLASS DOCUMENTATION 
//=================================================================

/**

The class <b>LFUtils</b> implements a set of utilities operating on 
the IDL objects for representing logical forms. 

<p>

@version 071031 (started 061023)
@author	 Geert-Jan M. Kruijff (gj@dfki.de)
 */

public class LFUtils { 


	//=================================================================
	// CLASS-INTERNAL GLOBAL VARIABLES
	//=================================================================

	static boolean logOutput = false; 
	

	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================

	/** Creates a new LFNominal object, with properly initialized 
		(non-null) fields 

		@return LFNominal the initialized object
	 */ 

	public static LFNominal newLFNominal () { 
		LFNominal result = new LFNominal ();
		result.nomVar = "";
		result.sort   = ""; 
		result.prop = newProposition();
		result.feats = new Feature[0];
		result.rels = new LFRelation[0];
		return result;
	} // end newLFNominal

	/** Creates a new LFNominal object, with properly initialized 
		(non-null) fields, and the nomVar set to the given variable 

		@param id The nominal variable for the nominal
		@return LFNominal the initialized object
	 */ 

	public static LFNominal newLFNominal (String id) { 
		LFNominal result = new LFNominal ();
		result.nomVar = id;
		result.sort   = "";
		result.prop = newProposition();
		result.feats = new Feature[0];
		result.rels = new LFRelation[0];
		return result;
	} // end newLFNominal

	/** Creates a new LFNominal object, with properly initialized 
		(non-null) fields, setting the nomVar to the given variable
		and the sort of the nominal to the given sort.  

		@param id The nominal variable for the nominal
		@param sort The sort for the nominal		
		@return LFNominal the initialized object
	 */ 

	public static LFNominal newLFNominal (String id, String sort) { 
		LFNominal result = new LFNominal ();
		result.nomVar = id;
		result.sort   = sort;
		result.prop = newProposition();
		result.feats = new Feature[0];
		result.rels = new LFRelation[0];
		return result;
	} // end newLFNominal 

	/** Creates a new LogicalForm object */ 

	public static LogicalForm newLogicalForm () { 
		LogicalForm result = new LogicalForm  (); 
		result.logicalFormId = "";
		result.noms = new LFNominal[0];
		result.root = newLFNominal();
		result.stringPos = 0;
		return result;
	} // end newLogicalForm

	/** Creates a new LFComponent object */ 

	public static LFComponent newLFComponent () { 
		LFComponent result = new LFComponent();
		result.componentName = "";
		result.lf = newLogicalForm();
		return result;
	} // end LFComponent	

	/** Creates a new LFRelation object, with properly initialized 
		(non-null) fields.

		@return LFRelation the initialized object
	 */ 

	public static LFRelation newLFRelation () { 
		LFRelation result = new LFRelation();
		result.head = "";
		result.mode = "";		
		result.dep = "";
		return result;
	} // end newLFRelation


	/** Creates a new LFRelation object, with properly initialized 
		(non-null) fields.

		@return LFRelation the initialized object
	 */ 

	public static LFRelation newLFRelation (String h, String m, String d) { 
		LFRelation result = new LFRelation();
		result.head = h;
		result.mode = m;		
		result.dep = d;
		return result; 
	} // end newLFRelation


	/** Creates a new Proposition object, with properly initialized 
		(non-null) fields.

		@return Proposition the initialized object
	 */ 

	public static comsys.datastructs.lf.Proposition newProposition () { 
		comsys.datastructs.lf.Proposition result = new comsys.datastructs.lf.Proposition();
		result.prop = "";
		result.connective = ConnectiveType.NONE;
//		result.rhsProp = new comsys.datastructs.lf.Proposition[0];
		return result;
	} // end newProposition


	/** Creates a new Proposition object, with properly initialized 
		(non-null) fields.
		@param  The proposition label
		@return Proposition the initialized object
	 */ 

	public static comsys.datastructs.lf.Proposition newProposition (String propLabel) { 
		comsys.datastructs.lf.Proposition result = new comsys.datastructs.lf.Proposition();
		result.prop = propLabel;
		result.connective = ConnectiveType.NONE;
//		result.rhsProp = new comsys.datastructs.lf.Proposition[0];
		return result;
	} // end newProposition


	//=================================================================
	// ACCESSOR METHODS
	//=================================================================

	/** Returns a resized LFNominal[] array including the already 
		present LFNominal objects, and the given LFNominal object. 
		<p>
		The method checks whether the array already contains a nominal 
		with the variable of the given LFNominal object. If this is the
		case then the old Nominal object is overwritten by the new 
		object (through a remove, add sequence).  

		@param	LFNominal[] array with already present LFNominal objects
		@param  LFNominal	to-be-added LFNominal object
		@return LFNominal[] array with LFNominal objects
	 */

	public static LFNominal[] lfAddNominal (LFNominal[] noms, LFNominal nom) { 
		LFNominal[] resizingArray = null; 
		if (noms == null) { 
			resizingArray = new LFNominal[0]; 
		} else if (lfHasNomvar(noms,nom.nomVar)) { 
			resizingArray = lfRemoveNominal(noms,nom.nomVar);
			resizingArray = lfAddNominal(resizingArray,nom);
		} else { 
			resizingArray = noms;
		} // end if..else check whether to replace nominal or not
		int newSize = 1+java.lang.reflect.Array.getLength(resizingArray);
		LFNominal[] result = (LFNominal[]) resizeArray(resizingArray,newSize);
		result[newSize-1] = nom;
		return result;
	} // end lfAddNominal 


	
	/**
	 * Verify the equivalence of two logical forms
	 */
	public static boolean compareLFs (LogicalForm lf1, LogicalForm lf2) {
		
		log("Logical form 1: " + LFUtils.lfToString(lf1));
		log("Logical form 2: " + LFUtils.lfToString(lf2));
		
		
		if (lf1.noms.length != lf2.noms.length) {
			log("Different number of nominals in each LF");
			return false;
		}
		
		for (int i = 0 ; i < lf1.noms.length ; i++) {
			boolean foundEquiv = false;
			LFNominal nom1 = lf1.noms[i];
			log("loop1 on nominal " + nom1.nomVar);
			for (int j = 0 ; j < lf2.noms.length && !foundEquiv; j++) {
				LFNominal nom2 = lf2.noms[j];
				if (nom1.prop.prop.equals(nom2.prop.prop) &&
						nom1.sort.equals(nom2.sort) &&
						nom1.feats.length == nom2.feats.length && 
						nom1.rels.length == nom2.rels.length) {
					
					log("found possible match");
					
					boolean EquivalentFeatures = true;
					for (int k = 0 ; k < nom1.feats.length ; k++) {
						boolean foundEquivFeat = false;
						Feature feat1 = nom1.feats[k];
						for (int l = 0 ; l < nom2.feats.length && !foundEquivFeat ; l++) {
							Feature feat2 = nom2.feats[l];
							if (feat1.feat.equals(feat2.feat) && 
									feat1.value.equals(feat2.value)) {
								foundEquivFeat = true;
							}
						}
						
						if (!foundEquivFeat) {
							log("Missing feature: [" + feat1.feat + ":" + feat1.value + "]");
							EquivalentFeatures = false;
						}
					}
					
					log("Equivalent features? " + EquivalentFeatures);
					
					boolean EquivalentRels = true;
					for (int k = 0 ; k < nom1.rels.length ; k++) {
						boolean foundEquivRel = false;
						LFRelation rel1 = nom1.rels[k];
						for (int l = 0 ; l < nom2.rels.length && !foundEquivRel ; l++) {
							LFRelation rel2 = nom2.rels[l];
							if (rel1.mode.equals(rel2.mode)) {
								foundEquivRel = true;
							}
						}
						
						if (!foundEquivRel) {
							log("Missing relation: [" + rel1.head + "->" + rel1.dep + ":" + rel1.mode + "]");
							EquivalentRels = false;
						}
					}
					
					log("Equivalent relations? " + EquivalentRels);
				
				if (EquivalentFeatures && EquivalentRels) {
					foundEquiv = true;
				}
					
				}
			}
		
			if (!foundEquiv) {
				log("Unfortunately, not equivalent found for nominal " + nom1.nomVar);
				return false;
			}
			
		}		
		
		return (lf1.root.sort.equals(lf2.root.sort) && 
				lf1.root.prop.prop.equals(lf2.root.prop.prop));
		
	}
	

	
	/**
	 * Check the equivalence between two sets of logical forms
	 * @param lfs1
	 * @param lfs2
	 * @return
	 */
	public static boolean compareLFSets (LogicalForm[] lfs1, LogicalForm[] lfs2) {
		 boolean allParsesCorrect = true;
		 
		 for (int i= 0; i < lfs1.length ; i++) {
			 LogicalForm lf = lfs1[i];
			 
		//	 String strLF = LFUtils.lfToString(lf);
			 log(LFUtils.lfToString(lf));
			 
			 boolean foundEquivalentParse = false;
			 			 
			 for (int j=0 ; j < lfs2.length && !foundEquivalentParse ; j++) {
				 LogicalForm lf2 = lfs2[j];
				 if (LFUtils.compareLFs(lf, lf2)) {
					 log("the generated parse is correct!");
					 foundEquivalentParse = true;
				 }
			 }
			 
			 if (!foundEquivalentParse) {
				 log("aie, no parse corresponding to the generated one");
				 allParsesCorrect = false;
			 }
		 }
		 
		 return allParsesCorrect;
	}
	
	
	public static PackedNominal[] plfAddNominal (PackedNominal[] noms, PackedNominal nom) { 
		PackedNominal[] resizingArray = null; 
		if (noms == null) { 
			resizingArray = new PackedNominal[0]; 
		} else if (plfHasNomvar(noms,nom.nomVar)) { 
			resizingArray = plfRemoveNominal(noms,nom.nomVar);
			resizingArray = plfAddNominal(resizingArray,nom);
		} else { 
			resizingArray = noms;
		} // end if..else check whether to replace nominal or not
		int newSize = 1+java.lang.reflect.Array.getLength(resizingArray);
		PackedNominal[] result = (PackedNominal[]) resizeArray(resizingArray,newSize);
		result[newSize-1] = nom;
		return result;
	} // end lfAddNominal 


	public static LFNominal[] lfAddNominal (LogicalForm lf, LFNominal nom) { 
		// System.out.println("Adding nominal ["+nom.nomVar+"]");
		LFNominal[] resizingArray = null; 
		LFNominal[] noms = lf.noms;
/**		if (noms == null) { 
			// System.out.println("Null noms, so resize to 1");
			resizingArray = new LFNominal[1]; 
			resizingArray[0] = nom;
			return resizingArray;
		} else { 	
*/		
		if (noms == null) { 
			resizingArray = new LFNominal[0]; 
		} else if (lfHasNomvar(noms,nom.nomVar)) { 
			resizingArray = lfRemoveNominal(noms,nom.nomVar);
			resizingArray = lfAddNominal(resizingArray,nom);
		} else { 
			resizingArray = noms;
		} // end if..else check whether to replace nominal or not
		int newSize = 1+java.lang.reflect.Array.getLength(resizingArray);
		LFNominal[] result = (LFNominal[]) resizeArray(resizingArray,newSize);
		result[newSize-1] = nom;
		return result;
	} // end lfAddNominal 

	
	/**
	 * Returns true if the logical form includes a specific dependency, false otherwise
	 * @param lf logical form
	 * @param headNomVar the nominal variable head of the dependency
	 * @param depNomVar the nominal variable dependent of the dependency
	 * @param mode the mode of the dependency
	 * @return
	 */
	public static boolean hasDependency(LogicalForm lf, String headNomVar, String depNomVar, String mode) {
		
		LFNominal headNom = lfGetNominal(lf, headNomVar);
		
		if (headNom != null) {
			for (int i=0; i < headNom.rels.length ; i++) {
				LFRelation rel = headNom.rels[i];
				if (rel.dep.equals(depNomVar) && rel.mode.equals(mode)) {
					return true;
				}
			}
		}
		
		return false;
	}
	

	public static PackedNominal[] plfAddNominal (PackedLogicalForm plf, PackedNominal nom) { 
		PackedNominal[] resizingArray = null; 
		PackedNominal[] noms =  plfGetNominals(plf);
		if (noms == null) { 
			resizingArray = new PackedNominal[0]; 
		} else if (plfHasNomvar(noms,nom.nomVar)) { 
			resizingArray = plfRemoveNominal(noms,nom.nomVar);
			resizingArray = plfAddNominal(resizingArray,nom);
		} else { 
			resizingArray = noms;
		} // end if..else check whether to replace nominal or not
		int newSize = 1+java.lang.reflect.Array.getLength(resizingArray);
		PackedNominal[] result = (PackedNominal[]) resizeArray(resizingArray,newSize);
		result[newSize-1] = nom;
		return result;
	} // end lfAddNominal 




	/** Returns a cloned copy of the given packed logical form, under clone(). 

		@param lf The logical form to be cloned
		@return LogicalForm The clone
	 */

	public static PackedLogicalForm plfClone (PackedLogicalForm plf) { 
		PackedLogicalForm result = new PackedLogicalForm(); 
		result.packedLFId = plf.packedLFId ;
		result.root = plf.root ;
		result.pNodes = new PackingNode[plf.pNodes.length];
		for (int i=0;i<plf.pNodes.length;i++) {
			result.pNodes[i] = new PackingNode();
			result.pNodes[i].pnId = plf.pNodes[i].pnId;
			result.pNodes[i].root = plf.pNodes[i].root;
			result.pNodes[i].lfIds = new String[plf.pNodes[i].lfIds.length];
			for (int j=0;j<plf.pNodes[i].lfIds.length;j++) {
				result.pNodes[i].lfIds[j] = plf.pNodes[i].lfIds[j] ;
			}
			result.pNodes[i].nomsPePairs = 
				new NominalPackingEdgePair[plf.pNodes[i].nomsPePairs.length];
			for (int j=0;j<plf.pNodes[i].nomsPePairs.length;j++) {
				result.pNodes[i].nomsPePairs[j] = new NominalPackingEdgePair();
				result.pNodes[i].nomsPePairs[j].head = 
					plf.pNodes[i].nomsPePairs[j].head ;
				result.pNodes[i].nomsPePairs[j].pe = new PackingEdge() ;
				result.pNodes[i].nomsPePairs[j].pe.head = 
					plf.pNodes[i].nomsPePairs[j].pe.head ;
				result.pNodes[i].nomsPePairs[j].pe.coIndexedDep = 
					plf.pNodes[i].nomsPePairs[j].pe.coIndexedDep ;
				result.pNodes[i].nomsPePairs[j].pe.mode = 
					plf.pNodes[i].nomsPePairs[j].pe.mode;
				result.pNodes[i].nomsPePairs[j].pe.peId = 
					plf.pNodes[i].nomsPePairs[j].pe.peId;
				result.pNodes[i].nomsPePairs[j].pe.preferenceScore = 
					plf.pNodes[i].nomsPePairs[j].pe.preferenceScore;
				result.pNodes[i].nomsPePairs[j].pe.targets = 
					new PackingNodeTarget[plf.pNodes[i].nomsPePairs[j].pe.targets.length] ;
				for (int k=0;k<plf.pNodes[i].nomsPePairs[j].pe.targets.length;k++) {
					result.pNodes[i].nomsPePairs[j].pe.targets[k] = new PackingNodeTarget();
					result.pNodes[i].nomsPePairs[j].pe.targets[k].pnId = 
						plf.pNodes[i].nomsPePairs[j].pe.targets[k].pnId;
					result.pNodes[i].nomsPePairs[j].pe.targets[k].lfIds = 
						new String[plf.pNodes[i].nomsPePairs[j].pe.targets[k].lfIds.length];
					for (int l=0;l<plf.pNodes[i].nomsPePairs[j].pe.targets[k].lfIds.length;l++) {
						result.pNodes[i].nomsPePairs[j].pe.targets[k].lfIds[l] = 
							plf.pNodes[i].nomsPePairs[j].pe.targets[k].lfIds[l] ;
					}			
				}
			}
			result.pNodes[i].packedNoms = 
				new PackedNominal[plf.pNodes[i].packedNoms.length];
			for (int j=0;j<plf.pNodes[i].packedNoms.length;j++) {
				result.pNodes[i].packedNoms[j] = plfNominalClone(plf.pNodes[i].packedNoms[j]);
			}
		}

		return result;
	} // end lfClone



	/** Returns a cloned copy of the given logical form, under clone(). 

		@param lf The logical form to be cloned
		@return LogicalForm The clone
	 */

	public static LogicalForm lfClone (LogicalForm lf) { 
		LogicalForm result = newLogicalForm(); 
		result.logicalFormId = lf.logicalFormId; 
		result.root = lfNominalClone(lf.root);
		for (int i=0; i < java.lang.reflect.Array.getLength(lf.noms); i++ ) { 
			LFNominal nom = (LFNominal) lf.noms[i];
			LFNominal nomcopy = lfNominalClone(nom);
			result.noms = lfAddNominal(result.noms,nomcopy);
		} // end for
		return result;
	} // end lfClone

	/** Returns the size of the logical form (number of nominals) */

	public static int lfSize (LogicalForm lf) { 
		return java.lang.reflect.Array.getLength(lf.noms);
	} // end lfSize


	/**
       The method <i>lfCollectNomvars</i> recursively descends down the
       subtree starting at the given root, and constructs a Vector
       with the identifiers of all the nominal variables that are
       governed by the root.

	   @param root  The root nominal to be descending from
	   @param lf	The logical form with all the nominals
	   @return Vector A vector with all the nominal variables under the root
	 */

	/** 
    public static Vector lfCollectNomvars (LFNominal root, LogicalForm lf) 
	{ 
		Vector nomvars = new Vector(); 
		Iterator relsIter = LFUtils.lfNominalGetRelations(root);
		while (relsIter.hasNext()) {
			LFRelation rel = (LFRelation) relsIter.next();
			String depnomvar = rel.dep;
			LFNominal depnom = LFUtils.lfGetNominal(lf,depnomvar);
			if (depnom != null) {
				nomvars.add(depnomvar);
				nomvars.addAll(lfCollectNomvars(depnom,lf));
			} // end if.. else 
		} // end while over relations
		return nomvars; 
    } // end lfCollectNomvars;
	 */

	public static Vector lfCollectNomvars (LFNominal root, LogicalForm lf) { 
		Vector occurrences = new Vector();
		return lfCollectNomvars(root,lf,occurrences);
	}

	public static Vector lfCollectNomvars (LFNominal root, LogicalForm lf, Vector occurrences) 
	{ 
		Vector nomvars = new Vector(); 
		Iterator relsIter = LFUtils.lfNominalGetRelations(root);
		while (relsIter.hasNext()) {
			LFRelation rel = (LFRelation) relsIter.next();
			String depnomvar = rel.dep;
			LFNominal depnom = LFUtils.lfGetNominal(lf,depnomvar);
			if (depnom != null) {
				nomvars.add(depnomvar);
				if (!occurrences.contains(depnomvar)) { 
					occurrences.add(depnomvar);
					nomvars.addAll(lfCollectNomvars(depnom,lf,occurrences));
				} // end if.. occurrence check 
			} // end if.. else 
		} // end while over relations
		return nomvars; 
	} // end lfCollectNomvars;








	/**
       The method <i>lfConstructSubtree</i> constructs a subtree of
       the logical form, by recursively descending from the given
       nominal down to the leaves of the logical form, collecting the
       nominals that are governed by the nominal. From the resulting
       list of nominals a new logical form is constructed, with the
       given nominal as root.

		@param root The root from which the subtree should be built
		@param lf	The logical form with the nominals
		@return LogicalForm the logical form subtree

	 */

	public static LogicalForm lfConstructSubtree (LFNominal root, LogicalForm lf) { 
		LogicalForm result = newLogicalForm ();
		result.noms = lfAddNominal(result.noms, root);
		result.root = root;
		Vector govdnomvars = lfCollectNomvars(root,lf);
		Iterator nomvarsIter = govdnomvars.iterator();
		while (nomvarsIter.hasNext()) { 
			String nomvar = (String) nomvarsIter.next();
			LFNominal nom = lfGetNominal(lf,nomvar);
			result.noms = lfAddNominal(result.noms, lfNominalClone(nom));
		} // end while
		return result;
	} // end lfConstructSubtree


	/** Returns the nominal with the given identifier, or <tt>null</tt> 
		if no such nominal is present. 

		@param lf the logical form from which the nominal is to be retrieved
		@param nomVar the identifier of the nominal to be retrieved
		@return LFNominal the nominal (if present; else null) 	
	 */ 

	public static LFNominal lfGetNominal (LogicalForm lf, String nomVar) { 
		LFNominal result = null;
		LFNominal[] noms = lf.noms;
		for (int i=0; i < java.lang.reflect.Array.getLength(noms); i++) { 
			LFNominal nom = (LFNominal) noms[i];
			if (nom.nomVar.equals(nomVar)) { 
				result = nom; 
				break; // exit the for
			} // end if 
		} // end for		
		return result; 
	} // end getNominal

	/** Returns an iterator over the nominals in the given logical form */

	public static Iterator<LFNominal> lfGetNominals (LogicalForm lf) { 
		LFNominal[] noms = lf.noms;
		return (Iterator<LFNominal>)(new ArrayIterator<LFNominal>(noms));
	} // end lfGetNominals


	public static PackedNominal[] plfGetNominals (PackedLogicalForm plf) { 
		Vector<PackedNominal> noms = new Vector<PackedNominal>();
		for (int i=0 ; i < plf.pNodes.length ; i++) {
			for (int j=0 ; j < plf.pNodes[i].packedNoms.length ; j++) {
				noms.add(plf.pNodes[i].packedNoms[j]);
			}
		}
		return noms.toArray(new PackedNominal[noms.size()]);
	} // end lfGetNominals


	public static LFRelation plfGetRelation(PackedNominal nom, String mode) {
		for (int i=0; i < nom.rels.length ; i++) {
			LFRelation rel = nom.rels[i];
			if (rel.mode.equals(mode)) {
				return rel;
			}
		}
		return null;
	}

	/** Returns a boolean indicating whether the given array of LFNominal
		objects contains an object with the given nominal variable as id. 

		@param	lf  the logical form with the nominals array to be searched
		@param  nomVar the variable to be searched for
		@return boolean whether the nomvar is present
	 */

	private static boolean lfHasNomvar(LFNominal[] noms, String nomVar) { 
		boolean result = false;
		for (int i=0; i < java.lang.reflect.Array.getLength(noms); i++) { 
			LFNominal nom = (LFNominal) noms[i];
			if (nom.nomVar.equals(nomVar)) { 
				result = true; 
				break; // exit the for
			} // end if 
		} // end for	
		return result;
	} // end lfHasNomVar

	private static boolean plfHasNomvar(PackedNominal[] noms, String nomVar) { 
		boolean result = false;
		for (int i=0; i < java.lang.reflect.Array.getLength(noms); i++) { 
			PackedNominal nom = (PackedNominal) noms[i];
			if (nom.nomVar.equals(nomVar)) { 
				result = true; 
				break; // exit the for
			} // end if 
		} // end for	
		return result;
	} // end lfHasNomVar

	public static boolean lfHasNomvar(LogicalForm lf, String nomVar) { 
		LFNominal[] noms = lf.noms;
		return lfHasNomvar(noms,nomVar);
	} // end hasNomvar

	/** Returns a new LFNominal[] array from which any nominal with the given
		nominal variable has been removed */

	public static LFNominal[] lfRemoveNominal (LFNominal[] noms, String nomVar) { 
		LFNominal[] result = new LFNominal[0];
		for (int i=0; i < java.lang.reflect.Array.getLength(noms); i++) { 
			LFNominal nom = (LFNominal) noms[i];
			if (!nom.nomVar.equals(nomVar)) { 
				result = LFUtils.lfAddNominal(result,nom);
			} // end if 			
		} // end for
		return result;
	} // end lfRemoveNominal


	public static LFNominal[] lfUpdateNominal (LFNominal[] noms, LFNominal nom) { 
		LFNominal[] result = lfRemoveNominal(noms,nom.nomVar);
		result = lfAddNominal(result,nom);
		return result;
	} // end update	
		
	
	
	public static PackedNominal[] plfRemoveNominal (PackedNominal[] noms, String nomVar) { 
		PackedNominal[] result = new PackedNominal[0];
		for (int i=0; i < java.lang.reflect.Array.getLength(noms); i++) { 
			PackedNominal nom = (PackedNominal) noms[i];
			if (!nom.nomVar.equals(nomVar)) { 
				result = LFUtils.plfAddNominal(result,nom);
			} // end if 			
		} // end for
		return result;
	} // end lfRemoveNominal

	/** Returns a new LFNominal[] array in which every occurrence of a nominal 
		with the given variable name has been renamed to the given new variable 
		name. 

		@param noms   the array with nominals
		@param oldVar the old nominal variable name
		@param newVar the new nominal variable name
		@return LFNominal[] the updated array 
	 */

	public static LFNominal[] lfReplaceNomvar(LFNominal[] noms, String oldVar, String newVar) { 
		LFNominal[] result = new LFNominal[0];
		for (int i=0; i < java.lang.reflect.Array.getLength(noms); i++) { 
			LFNominal nom = (LFNominal) noms[i];
			LFRelation[] newRels = new LFRelation[java.lang.reflect.Array.getLength(nom.rels)];
			// first, cycle over the relations of the nominal

			for (int j=0; j < java.lang.reflect.Array.getLength(nom.rels); j++) { 
				LFRelation rel = (LFRelation) nom.rels[j];
				if (rel.head.equals(oldVar)) { 
					rel.head = newVar; 
				} // end check head
				if (rel.dep.equals(oldVar)) { 
					rel.dep = newVar; 
				} // end check dependent
				newRels[j] = rel;
			} // end cycling over relations

			// update the relations in the nominal
			nom.rels = newRels;

			// next, check whether the nominal variable itself
			// must be changed, and the store the nominal
			if (!nom.nomVar.equals(oldVar)) { 
				result = LFUtils.lfAddNominal(result,nom);					
			} else {
				LFNominal nomcopy = LFUtils.lfNominalClone(nom);
				nomcopy.nomVar = newVar;
				result = LFUtils.lfAddNominal(result,nomcopy);
			} // end if 			
		} // end for		
		return result;
	} // end lfReplaceNomvar


	/** Returns a new LFNominal[] array in which every occurrence of a nominal 
		with the given variable name has been renamed to the given new variable 
		name. 

		@param noms   the array with nominals
		@param oldVar the old nominal variable name
		@param newVar the new nominal variable name
		@return LFNominal[] the updated array 
	 */

	public static PackedLogicalForm plfReplaceNomvar(PackedLogicalForm plf, String oldVar, String newVar) { 
		PackedLogicalForm newPlf = plfClone(plf);
		for (int i=0; i< plf.pNodes.length ; i++) {
			if (plf.pNodes[i].root.equals(oldVar)) {
				newPlf.pNodes[i].root = newVar ;
			}
			for (int j=0; j < plf.pNodes[i].packedNoms.length ; j++) {
				if (plf.pNodes[i].packedNoms[j].nomVar.equals(oldVar)) {
					newPlf.pNodes[i].packedNoms[j].nomVar = newVar ;
				}
				for (int k=0 ; k < plf.pNodes[i].packedNoms[j].rels.length ; k++) {
					if (plf.pNodes[i].packedNoms[j].rels[k].head.equals(oldVar)) {
						newPlf.pNodes[i].packedNoms[j].rels[k].head = newVar ;
					}
					if (plf.pNodes[i].packedNoms[j].rels[k].dep.equals(oldVar)) {
						newPlf.pNodes[i].packedNoms[j].rels[k].dep = newVar ;
					}
				}
			}

		}
		return newPlf;
	} // end lfReplaceNomvar

	public static PackingEdge plfGetPackingEdge(PackingNode pn, String mode) {
		for (int i=0; i < pn.nomsPePairs.length; i++) {
			if (pn.nomsPePairs[i].pe.mode.equals(mode)) {
				return pn.nomsPePairs[i].pe ;
			}
		}
		return null;
	}

	public static String plfGetFeatureValue(PackedNominal pn, String feat) {
		for (int i=0; i < pn.feats.length ; i++) {
			if (pn.feats[i].feat.equals(feat)) {
				return pn.feats[i].value ;
			}
		}
		return null;
	}

	public static PackingNode plfGetPackingNode(PackedLogicalForm plf, String pnId) {
		for (int i=0; i < plf.pNodes.length ; i++) {
			if (plf.pNodes[i].pnId.equals(pnId)) {
				return plf.pNodes[i];
			}
		}
		return null;
	}

	public static PackedNominal plfGetPackedNominal(PackingNode pn, String nomVar) {
		for (int i=0; i < pn.packedNoms.length ; i++) {
	//		System.out.println("nomVar: " + pn.packedNoms[i].nomVar);
			if (pn.packedNoms[i].nomVar.equals(nomVar)) {
				return pn.packedNoms[i];
			}
		}
		return null;
	}
	
	public static PackedNominal plfGetPackedNominal(PackedLogicalForm plf, String nomVar) {
		for (int i=0; i < plf.pNodes.length ; i++) {
	//		System.out.println("PNId: " + plf.pNodes[i].pnId);
			PackedNominal result = plfGetPackedNominal(plf.pNodes[i], nomVar);
			if (result != null) {
				return result;
			}
		}
		return null;
	}
	


	/** Returns an Feature array including the nominal's features,
		with provided feature added.

		@param nom The nominal with the set of relations
		@param ft  The feature to be added
		@return Features[] the array with features
	 */

	public static Feature[] lfNominalAddFeature (LFNominal nom, Feature ft) {
		int newSize = 1+java.lang.reflect.Array.getLength(nom.feats);
		Feature[] fts = (Feature[]) resizeArray(nom.feats,newSize);
		fts[newSize-1] = ft; 
		return fts;
	} // end lfNominalAddFeature


	/** Returns a Proposition object including the nominals propositions,
		with provided proposition added.

		@param nom The nominal with the set of relations
		@param prop  The proposition to be added
		@return Proposition The updated proposition
	 */

	public static comsys.datastructs.lf.Proposition lfNominalAddProposition (LFNominal nom, String prop) {
		comsys.datastructs.lf.Proposition result = newProposition(prop); 

		if (nom.prop == null ||
				nom.prop.prop.equals("")) 
		{ 
			return result;
		} else {
			comsys.datastructs.lf.Proposition nomProp = nom.prop;
			// .... 
			return result;
		}
	} // end lfNominalAddRelation

	/** Returns an LFRelation array including the nominal's relations,
		with provided relation added.

		@param nom The nominal with the set of relations
		@param rl  The relation to be added
		@return LFRelation[] the array with relations
	 */

	public static LFRelation[] lfNominalAddRelation (LFNominal nom, LFRelation rl) {
		int newSize = 1+java.lang.reflect.Array.getLength(nom.rels);
		LFRelation[] rels = (LFRelation[]) resizeArray(nom.rels,newSize);
		rels[newSize-1] = rl; 	
		return rels;
	} // end lfNominalAddRelation



	/** Returns an LFRelation array including the nominal's relations,
		with provided relation added.

		@param nom The nominal with the set of relations
		@param rl  The relation to be added
		@return LFRelation[] the array with relations
	 */

	public static LFRelation[] plfNominalAddRelation (PackedNominal nom, LFRelation rl) {
		int newSize = 1+java.lang.reflect.Array.getLength(nom.rels);
		LFRelation[] rels = (LFRelation[]) resizeArray(nom.rels,newSize);
		rels[newSize-1] = rl; 	
		return rels;
	} // end lfNominalAddRelation

	
	
	/** Returns a clone of the given LFNominal object */

	public static LFNominal lfNominalClone (LFNominal nom) {
		LFNominal result = new LFNominal();
		result.nomVar = nom.nomVar;
		result.sort   = nom.sort;
		result.prop = LFUtils.propositionClone(nom.prop);
		result.feats = nom.feats.clone();
		result.rels = nom.rels.clone();
		return result;
	} // end lfNominalClone



	/** Returns a clone of the given PackedNominal object */

	public static PackedNominal plfNominalClone (PackedNominal nom) {
		PackedNominal result = new PackedNominal() ;
		result.nomVar = nom.nomVar;
		result.prop = propositionClone(nom.prop);
		result.feats = new PackedFeature[nom.feats.length];
		for (int k=0;k<nom.feats.length ; k++){
			result.feats[k] = new PackedFeature();
			result.feats[k].feat = nom.feats[k].feat;
			result.feats[k].lfIds = new String[nom.feats[k].lfIds.length];
			result.feats[k].value = nom.feats[k].value;
			for (int l=0;l<nom.feats[k].lfIds.length;l++) {
				result.feats[k].lfIds[l] = 
					nom.feats[k].lfIds[l] ;
			}			
		}
		result.packedSorts = new PackedOntologicalSort[nom.packedSorts.length];
		for (int k=0;k<nom.packedSorts.length ; k++){
			result.packedSorts[k] = new PackedOntologicalSort();
			result.packedSorts[k].sort = nom.packedSorts[k].sort;
			result.packedSorts[k].lfIds = new String[nom.packedSorts[k].lfIds.length];
			for (int l=0;l<nom.packedSorts[k].lfIds.length;l++) {
				result.packedSorts[k].lfIds[l] = 
					nom.packedSorts[k].lfIds[l] ;
			}			
		}
		result.rels = nom.rels.clone();
		return result;
	} // end lfNominalClone




	/** Returns a clone of the given LFNominal object, with nominal variables pointing to dependents
		being replaced with the new identifiers given in the replacedIds map

		@param nom The nominal to be copied
		@param replacedIds The map with variable names and their replacements
		@return LFNominal The copy
	 */

	public static LFNominal lfNominalCloneUnderReplacements (LFNominal nom, HashMap replacedIds) {
		LFNominal result = new LFNominal();
		result.nomVar = nom.nomVar;
		result.sort   = nom.sort;
		result.prop = LFUtils.propositionClone(nom.prop);
		result.feats = nom.feats.clone();
		result.rels = nom.rels.clone();
		LFRelation[] updRels = new LFRelation[java.lang.reflect.Array.getLength(result.rels)];
		for (int i=0; i < java.lang.reflect.Array.getLength(nom.rels) ; i++)  { 
			LFRelation rel = result.rels[i];
			if (replacedIds.containsKey(rel.dep)) {
				rel.dep = (String)replacedIds.get(rel.dep);
			} // end if
			updRels[i] = rel;
		} // end while
		result.rels = updRels;		
		return result;
	} // end lfNominalClone



	/** Returns a clone of the given PackedNominal object, with nominal variables pointing to dependents
		being replaced with the new identifiers given in the replacedIds map

		NOTE: TO BE VERIFIED !! -- plison

		@param nom The nominal to be copied
		@param replacedIds The map with variable names and their replacements
		@return LFNominal The copy
	 */

	public static PackedNominal plfNominalCloneUnderReplacements (PackedNominal nom, HashMap replacedIds) {

		PackedNominal result = new PackedNominal();
		result.nomVar = nom.nomVar;
		result.prop = propositionClone(nom.prop);
		result.feats = new PackedFeature[nom.feats.length];
		for (int k=0;k<nom.feats.length ; k++){
			result.feats[k].feat = nom.feats[k].feat;
			result.feats[k].lfIds = new String[nom.feats[k].lfIds.length];
			result.feats[k].value = nom.feats[k].value;
			for (int l=0;l<nom.feats[k].lfIds.length;l++) {
				result.feats[k].lfIds[l] = 
					nom.feats[k].lfIds[l] ;
			}			
		}
		result.packedSorts = new PackedOntologicalSort[nom.packedSorts.length];
		for (int k=0;k<nom.packedSorts.length ; k++){
			result.packedSorts[k].sort = nom.packedSorts[k].sort;
			result.packedSorts[k].lfIds = new String[nom.packedSorts[k].lfIds.length];
			for (int l=0;l<nom.packedSorts[k].lfIds.length;l++) {
				result.packedSorts[k].lfIds[l] = 
					nom.packedSorts[k].lfIds[l] ;
			}			
		}
		result.rels = nom.rels.clone();
		LFRelation[] updRels = new LFRelation[java.lang.reflect.Array.getLength(result.rels)];
		for (int i=0; i < java.lang.reflect.Array.getLength(nom.rels) ; i++)  { 
			LFRelation rel = result.rels[i];
			if (replacedIds.containsKey(rel.dep)) {
				rel.dep = (String)replacedIds.get(rel.dep);
			} // end if
			updRels[i] = rel;
		} // end while
		result.rels = updRels;		
		return result;
	} // end lfNominalClone


	/** Returns an iterator over the features in the given nominal */

	public static Iterator lfNominalGetFeatures (LFNominal nom) { 
		Feature[] feats = nom.feats;
		return (Iterator)(new ArrayIterator(feats));
	} // end lfNominalGetRelations


	/** Returns a value for the given feature in the given nominal 

		@param nom	 The nominal to check
		@param fName The feature to check for
		@return String The value for the feature
	 */

	public static String lfNominalGetFeature (LFNominal nom, String fName) { 
		String result = ""; 
		Iterator featsIter = lfNominalGetFeatures(nom); 
		while (featsIter.hasNext()) { 
			Feature feat = (Feature) featsIter.next();
			if (feat.feat.equals(fName)) { 
				result = feat.value;
				break; // no need to look further
			} // end if check for feature
		} // end while
		return result;
	} // end lfNominalGetFeature

	/** Returns a boolean indicating whether the given nominal has the given feature 

		@param nom	 The nominal to check
		@param fName The name of the feature to check for
		@return boolean Whether the given nominal has a feature with the given name
	 */

	public static boolean lfNominalHasFeature (LFNominal nom, String fName) { 
		boolean result = false; 
		Iterator featsIter = lfNominalGetFeatures(nom);
		while (featsIter.hasNext()) { 
			Feature feat = (Feature) featsIter.next();
			if (feat.feat.equals(fName)) { 
				result = true;
				break; // no need to look further
			} // end if check for feature
		} // end while
		return result;
	} // end lfNominalHasFeature



	/** Returns a boolean indicating whether the given packed nominal has the given feature 

	@param nom	 The nominal to check
	@param fName The name of the feature to check for
	@return boolean Whether the given nominal has a feature with the given name
	 */

	public static boolean plfNominalHasFeature (PackedNominal nom, String fName) { 
		if (nom.feats!=null) {
		for (int i=0;i<nom.feats.length;i++){
			if (nom.feats[i].feat != null && nom.feats[i].feat.equals(fName)) {
				return true;
			}
		}
		}
		return false;
	} // end plfNominalHasFeature


	/** Returns a value for the given feature in the given nominal 

	@param nom	 The nominal to check
	@param fName The feature to check for
	@return String The value for the feature
	 */

	public static String plfNominalGetFeature (PackedNominal nom, String fName) { 
		String result = ""; 
		for (int i=0;i<nom.feats.length;i++){
			if (nom.feats[i].feat.equals(fName)) {
				return nom.feats[i].value;
			}
		}
		return result;
	} // end lfNominalGetFeature




	/** Returns an LFRelation object for the given label, if any. If none, then null is returned */

	public static LFRelation lfNominalGetRelation (LFNominal nom, String rName) { 
		LFRelation result = null; 
		Iterator relsIter = lfNominalGetRelations(nom); 
		while (relsIter.hasNext()) { 
			LFRelation rel = (LFRelation) relsIter.next();
			if (rel.mode.equals(rName)) { 
				result = rel;
				break; // no need to look further
			} // end if check for feature
		} // end while
		return result; 
	} // end lfNominalGetFeature

	/** Returns an iterator over the relations in the given nominal */

	public static Iterator lfNominalGetRelations (LFNominal nom) { 
		LFRelation[] rels = nom.rels;
		return (Iterator)(new ArrayIterator(rels));
	} // end lfNominalGetRelations


	/** Returns a boolean whether the given nominal has the given propositional name*/

	public static boolean lfNominalHasProposition (LFNominal nom, String prop) { 
		return (nom.prop.prop.equals(prop));
	} // end lfNominalHasProposition 


	/** Returns whether the given nominal has an LFRelation object for the given label */

	public static boolean lfNominalHasRelation (LFNominal nom, String rName) { 
		boolean result = false; 
		Iterator relsIter = lfNominalGetRelations(nom); 
		while (relsIter.hasNext()) { 
			LFRelation rel = (LFRelation) relsIter.next();
			if (rel.mode.equals(rName)) { 
				result = true;
				break; // no need to look further
			} // end if check for feature
		} // end while
		return result; 
	} // end lfNominalGetFeature


	public static boolean lfNominalHasRelations (LFNominal nom) { 
		return lfNominalGetRelations(nom).hasNext();

	} 



	/** Returns a LogicalForm in which the given relations are removed from under the given nominal (including depending nominals) */ 

	public static LFRelation[] lfNominalRemoveRelation (LFNominal nom, String xrel) { 
		Vector xrels = new Vector();
		xrels.add(xrel);
		return lfNominalRemoveRelations(nom,xrels);
	} // end lfNominalRemoveRelation


	/** Returns a LogicalForm in which the given relations are removed from under the given nominal (including depending nominals) */ 

	public static LFRelation[] lfNominalRemoveRelations (LFNominal nom, Vector xrels) { 
		Vector keptRels = new Vector(); 
		// cycle over the relations in the nominal, copy over those not in the exclude set, gather dependents for those that are
		for (int i=0; i < java.lang.reflect.Array.getLength(nom.rels); i++) { 
			LFRelation rel = nom.rels[i];
			if (xrels.contains(rel.mode)) { 
				// this relation needs to be removed
			} else {
				// keep this relation
				keptRels.add(rel);
			} // end if..else check whether to update relation object
		} // end for over relations in nom
		// add the relations to be kept
		int counter = 0;
		LFRelation[] result = new LFRelation[keptRels.size()];
		for (Iterator<LFRelation> keptIter = keptRels.iterator(); keptIter.hasNext(); ) { 
			result[counter] = keptIter.next();
			counter++;
		} // end for	
		return result; 
	} // end lfNominalRemoveRelations

	
	
	
	
	public static LFRelation[] lfNominalReplaceRelation (LFNominal nom, LFRelation oldRel, LFRelation newRel) { 
		LFRelation[] result = new LFRelation[java.lang.reflect.Array.getLength(nom.rels)];
		for (int i=0; i < java.lang.reflect.Array.getLength(nom.rels); i++) { 
			LFRelation rel = nom.rels[i];
			if (rel.mode.equals(oldRel.mode) && rel.dep.equals(oldRel.dep)) { 
				result[i] = newRel;
			} else {
				result[i] = rel;
			} // end if..else check whether to update relation object
		} // end for over relations in nom
		return result;
	} // end lfNominalReplaceRelation
	
	

	/** Returns an updated list of relations for the nominal. The method scans for a relation with the given mode
		and given dependent, and changes that relation so as to point to the nominal variable of the new dependent.

		@param  nom  The nominal for which the relations should be updated
		@param  mode The mode of the relation to be updated
		@param  oldDep The nominal variable to be changed
		@param  newDep The new nominal variable
		@return LFRelation[] The updated relations array
	 */

	public static LFRelation[] lfNominalReplaceDependent (LFNominal nom, String mode, String oldDep, String newDep) { 
		LFRelation[] result = new LFRelation[java.lang.reflect.Array.getLength(nom.rels)];
		for (int i=0; i < java.lang.reflect.Array.getLength(nom.rels); i++) { 
			LFRelation rel = nom.rels[i];
			if (rel.mode.equals(mode)) { 
				if(rel.dep.equals(oldDep)) { 
					LFRelation newRel = new LFRelation(nom.nomVar,mode,newDep,false);
					newRel.coIndexedDep = rel.coIndexedDep;
					result[i] = newRel;
				} else {
					result[i] = rel;
				} // end if..else
			} else {
				result[i] = rel;
			} // end if..else check whether to update relation object
		} // end for over relations in nom
		return result;
	} // end lfNominalReplaceDependent

	/** Returns a clone of the given Proposition object */

	public static comsys.datastructs.lf.Proposition propositionClone (comsys.datastructs.lf.Proposition prop) { 
		comsys.datastructs.lf.Proposition result = new comsys.datastructs.lf.Proposition();
		result.prop = prop.prop;
		result.connective  = prop.connective;
//		result.rhsProp	   = prop.rhsProp.clone();
		return result;
	} // end 

	

	public static PackedNominal plfGetRoot (PackedLogicalForm plf) {
		PackingNode root = plfGetPackingNode (plf, plf.root) ;
		for (int p=0 ; p < root.packedNoms.length ; p++) {
			if ( root.packedNoms[p].nomVar.contains("rootNom") && root.packedNoms[p].rels.length > 0) {
				for (int q = 0; q < root.packedNoms[p].rels.length ; q++ ) {
					if (root.packedNoms[p].rels[q].mode.equals("root")) {
						return plfGetPackedNominal(root, root.packedNoms[p].rels[q].dep) ;
					}
				}
			}
		}
		System.out.println("WARNING: the packed logical form seems to have several competing roots");
		return null;
	}

	/**
	 * Returns the number of LF identifiers included in a given PLF
	 * @param plf
	 * @return
	 */
	public static int plfNumberOfLFs(PackedLogicalForm plf) {
		int result = 0;
		PackingNode pn = plfGetPackingNode(plf, plf.root);
		result = pn.lfIds.length;
		return result;
	}
	

	
	public static Iterator<PackedNominal> plfGetRoots (PackedLogicalForm plf) {
		List<PackedNominal> roots = new ArrayList<PackedNominal>();
		PackedNominal usualRoot = plfGetRoot(plf);
		if (usualRoot != null) {
			roots.add(usualRoot);
			return roots.iterator();
		}
		else {
			PackingNode rootPN = plfGetPackingNode (plf, plf.root) ;
			PackedNominal rootNominal = plfGetPackedNominal(rootPN, rootPN.root);
			for (int i=0; i < rootNominal.pEdges.length ; i++) {
				for (int j=0; j < rootNominal.pEdges[i].targets.length ; j++) {
					PackingNode depPN = plfGetPackingNode (plf, rootNominal.pEdges[i].targets[j].pnId);
					PackedNominal nom = plfGetPackedNominal(depPN, depPN.root);
					roots.add(nom);
				}
			}
		}
		if (roots.size() == 0)
			System.out.println("PROBLEM: unable to find any roots!");
		return roots.iterator();
	}

	

	/** Reallocates an array with a new size, and copies the contents
		of the old array to the new array.
		@param oldArray  the old array, to be reallocated.
		@param newSize   the new array size.
		@return          A new array with the same contents.
	 */
	public static Object resizeArray (Object oldArray, int newSize) {
		int oldSize = java.lang.reflect.Array.getLength(oldArray);
		Class elementType = oldArray.getClass().getComponentType();
		Object newArray = java.lang.reflect.Array.newInstance(
				elementType,newSize);
		int preserveLength = Math.min(oldSize,newSize);
		if (preserveLength > 0) System.arraycopy (oldArray,0,newArray,0,preserveLength);
		return newArray; 
	}

	//=================================================================
	// COMPUTATION METHODS
	//=================================================================
	
	/** This method creates a new diamond-element representing a feature of
	 *  the given LFNominal. It appends this new Element to the given one and
	 *  returns the combined Element. It is a help-method for creating an XML-doument
	 *  that represents a given LogicalForm
	 * @return Element 
	 * @param LogicalForm complete, LFNominal to extract feature value from, String containing the feature,
	 * Element to which will be appended
	 */
	private static Element editFeatures(LogicalForm input,LFNominal nom,String feature,Element el) {
		
		// create root diamond node
		Element diamond = new Element("diamond");
		diamond = diamond.setAttribute("mode",feature);
		// create its proposition node
		
		Element prop = new Element("prop");
		
		for(Iterator<comsys.datastructs.lf.Feature> featsIter = LFUtils.lfNominalGetFeatures(nom); featsIter.hasNext(); ) { 
			comsys.datastructs.lf.Feature nomFeature = featsIter.next();
			if (nomFeature.feat.equals(feature)) {
				prop = prop.setAttribute("name",LFUtils.lfNominalGetFeature(nom,nomFeature.feat));		
			} // end if 
		}
		// add all in a tree-like manner
		diamond = diamond.addContent(prop);
		el = el.addContent(diamond);
		if (el == null) { System.out.println("Empty element created in editFeatures for feature ["+feature+"]"); }
		return el;
		
	} // end editFeatures
	
	
	/** This method creates a new diamond-element representing the given relation of
	 *  the given LFNominal. If the relation itself has relations, recursively call
	 *  this method to append the relations to the current element. The this new Element is
	 *  appended to the originally given one and that combined Element is returned.
	 *  It is a help-method for creating an XML-doument that represents a given LogicalForm
	 * @return Element 
	 * @param LogicalForm complete, LFRelation, Element to which will be appended
	 */	
    private static Element editRelations(LogicalForm input, LFRelation rel, Element el) {
		
		// System.out.println("Trying to add relation ["+rel.mode+"]");
		
		
		//--------------------------------------------
		//GJ: let's have a look at what's going on ...
		//-------------------------------------------- 
		
		// create root diamond node
		Element diamond = new Element("diamond");
		diamond = diamond.setAttribute("mode",rel.mode);
		
		// create its nom node containing dependent and type
		
		Element nom = new Element("nom");
		LFNominal helper = LFUtils.lfGetNominal(input,rel.dep);
		
		String s = helper.nomVar +":"+ helper.sort;
		nom = nom.setAttribute("name",s);
		// create its proposition node
		Element prop = null; 
		if (!helper.prop.prop.equals("")) { 
			prop = new Element("prop");
			prop = prop.setAttribute("name",helper.prop.prop);
		}
		// add them
		diamond = diamond.addContent(nom);
		
		
		
		
		if (!rel.mode.equals("Subject") && !rel.mode.equals("Scope")) { 
			
			if (rel.mode.equals("Wh-Restr")) { 
				if (helper.sort.equals("specifier")) { 
					if (prop != null) { 			
						diamond = diamond.addContent(prop);
					}
				} // end
			} else if (prop != null) { 			
				diamond = diamond.addContent(prop);
			}
			// if the diamond itself has any features add those recursively
			if (LFUtils.lfNominalGetFeatures(helper).hasNext()) {
				Iterator iter = LFUtils.lfNominalGetFeatures(helper);
				while (iter.hasNext()) {
					comsys.datastructs.lf.Feature feature = (comsys.datastructs.lf.Feature) iter.next();
					diamond = editFeatures(input,helper,feature.feat,diamond);
				}
			}
			// same for relations
			if (LFUtils.lfNominalHasRelations(helper)) {
				Iterator iter = LFUtils.lfNominalGetRelations(helper);
				while (iter.hasNext()) {
					LFRelation hrel = (LFRelation) iter.next();
					diamond = editRelations(input,hrel,diamond);
				}
			}
		} // end if.. exclude features and relations for "Subject" diamonds
		
		
		el = el.addContent(diamond);
		if (el == null) { System.out.println("Empty element created in editRelations for relation ["+rel.mode+"]"); }
		
       	return el;
    } // end editRelations
    
	
	/** 
		Returns an OpenCCG LF object, constructed from a Logical Form. 
	*/

	public static LF convertToLF(LogicalForm input) { 
		Realizer realizer = new Realizer(null);
		XMLOutputter out = new XMLOutputter();
		Document doc;
		LF lf = null;
		try { 
			// LogicalForm to xml-File
      		// -------------------------------------------
			// GJ: Make it a proper XML document, with a
			// document type and an indication of the root
			// element.
			// -------------------------------------------
			
			DocType doctype = new DocType("xml");
			Element root1 = new Element("xml");
			doc = new Document(root1,doctype);
			
			// basic Element "lf"
			
			Element lf1 = new Element("lf");
			
			//----------------------------------------------
			//GJ: First, deal with the root of the LF
			//----------------------------------------------
			
			LFNominal inputNom = (LFNominal)input.root;
			
			// System.out.println("Working on nominal: "+LFUtils.lfNominalToString(inputNom));
			
			
			// basic Element "satop" with nom = "id:type"
			Element sat = new Element("satop");
			String s = inputNom.nomVar;
			s = s + ":" + inputNom.sort;
			sat = sat.setAttribute("nom",s);
			
			
			// basic proposition, added to "satop"
			if (!inputNom.prop.prop.equals("")) { 
				Element prop = new Element("prop");
				prop = prop.setAttribute("name",inputNom.prop.prop);
				sat = sat.addContent(prop);
			}
			// System.out.println("Satop: "+out.outputString(sat));
			
			
			//----------------------------------------------
			//GJ: Now the root has a nominal, type, and 
			//proposition. 
			//----------------------------------------------
			
			// this test shows whether a satop-element has been created, 
			// it is only successful if it contains a NomVar and a Proposition
			
			//----------------------------------------------
			// if there are features create and add elements in 'editFeatures()'
			// GJ: still dealing with the root at this point.
			//---------------------------------------------- 
			
			if (LFUtils.lfNominalGetFeatures(inputNom).hasNext()) {
				Iterator iter = LFUtils.lfNominalGetFeatures(inputNom); 
				while (iter.hasNext()) {
					comsys.datastructs.lf.Feature feature = (comsys.datastructs.lf.Feature) iter.next();
					sat = editFeatures(input,inputNom,feature.feat,sat);
				}
			} else { 
				//System.out.println("Nominal ["+inputNom.nomVar+"] has no features");
			} 
			
			// System.out.println("Satop with features: "+out.outputString(sat));
			
			
			
			//----------------------------------------------
			//GJ: okay, so now we start some recursion down
			//the relations of the nominal ...  
			//----------------------------------------------
			
			// if there are Relations create and add elements in 'editRelations()' 
			if (LFUtils.lfNominalHasRelations(inputNom)) { 
				Iterator it = LFUtils.lfNominalGetRelations(inputNom);
				while (it.hasNext()) {
					sat = editRelations(input,(LFRelation)it.next(),sat);
				} // end while over relations
			} // end if.. check for relations
			
			//System.out.println("Sat with relations:\n"+sat.toString());
			
			//----------------------------------------------
			//GJ: now we supposedly have a complete structure 
			// ----------------------------------------------
			
			lf1 = lf1.addContent(sat);
			root1 = doc.getRootElement().addContent(lf1);			
			Element targetElt = new Element("target");
			targetElt = targetElt.setText("*** dummy target***");
			root1 = root1.addContent(targetElt);
			
			// msg("\nSending to realizer:\n"+out.outputString(doc));
			
			//------------- openccg - Realize -----------------
			// call ccg.Realizer with XML-document which extracts the contained LF and returns it  
			lf = realizer.getLfFromDoc(doc);
			
		} catch (Exception e) {
			log("error in LF: "+e.getMessage()); 
		} 
		// return resulting LF
		return lf;
	} 



	/** Returns a LogicalForm object constructed from an OpenCCG LF object. 
		The method fills the <i>nominals</i> field in the LogicalForm object; 
		it does not set the identifier for the logical form. The method creates
		a string representation of the OpenCCG LF, and then parses that string
		using the <i>convertFromString</i> method. 

		@see   #convertFromString(String)
		@param lf the opencc logical form
		@return LogicalForm the IDL-based object 
	 */

	public static comsys.datastructs.lf.LogicalForm convertFromLF (LF lf) { 
		return convertFromString(lf.toString());
	} // end convertFromLF

	/** The method <i>convertFromString</i> converts a String representation of a logical form into 
		a proper LogicalForm object. The method is based on a parser, which parses a String representation
		of using the following strategy: 

        <ul>
        <li> Use a StringTokenizer using "(" as delimiter to break up the formula into embeddings.</li>
        <li> Use a StringTokenizer using "^" as delimiter to break up an embedding into conjuncts.</li>
	    <li> Keep track of the current node using a variable <tt>nom</tt> of type <tt>LFNominal</tt>.</li>
        <li> Keep track of governing nodes (given embeddings) using a stack.</li>
        </ul>

		Breaking up into embeddings means that we first see a relation
		and its governor, and only on the next pass through the
		"("-loop, the nominal to which the relation points (the
		dependent, or governed, node). To this end, we keep track of
		the relation in the <tt>rel</tt> variable of type
		<tt>LFRelation</tt>.

		<h4>Warning</h4>
		The parser can deal with multiple at-signs in the grammar,
		though should be expected to do only reliably so for up to two
		at-signs. <p>

		@param s the string representation of the logical form
		@return LogicalForm the IDL-based object 
	 */

	public static comsys.datastructs.lf.LogicalForm convertFromString(String s) { 

		
	//	System.out.println("Converting from String: ["+s+"]");


		LogicalForm resultLF = newLogicalForm();
		
		resultLF.root = null;
		s.replaceAll("\n","");
		int fatpos = s.indexOf("@");
		int latpos = s.lastIndexOf("@");

//		lf.Proposition rhsStub[] = new lf.Proposition[0];

		LFNominal nom  = newLFNominal();

		Stack nomstack = new Stack();
		LFRelation rel = null;

		// check whether we have a restart, i.e. a logical form of the shape
		// @A & @B. (not completely flattened). 

		if (fatpos != latpos) { 
			StringTokenizer satisoper = new StringTokenizer(s,"@");
			Vector<LogicalForm> subLFs = new Vector<LogicalForm>();
			Vector<LFNominal> subNominals = new Vector<LFNominal>();
			LogicalForm returnLF = new LogicalForm();
			while (satisoper.hasMoreTokens()) { 
				String subLFstring = "@" + (String) satisoper.nextToken();
				subLFstring = subLFstring.trim();
				if (subLFstring.endsWith(") ^")) {
					subLFstring = subLFstring.substring(0, subLFstring.length() - 2);
				}
				log("PART: " + subLFstring);
				LogicalForm subLF = convertFromString(subLFstring);
				subLFs.add(subLF);
				subNominals.addAll(Arrays.asList(subLF.noms));
			}
			LFNominal[] nominals= new LFNominal[subNominals.size()];
			returnLF.noms = subNominals.toArray(nominals);
			returnLF.root = subLFs.elementAt(0).root;
			return returnLF;
			
		}
		
		else {

		StringTokenizer openbrckts = new StringTokenizer(s,"(");
		while (openbrckts.hasMoreTokens()) { 
			String form = (String) openbrckts.nextToken();
			log("new openbracket: "+form);
			int atpos = form.indexOf("@");
			if (atpos > -1) { // head nominal
				// int clpos = form.indexOf(":");
				// System.out.println("@@@@ Form: ["+form+"] with clpos index "+form.indexOf(":"));
				int clpos = form.lastIndexOf(":");
				String nomvar = "";
				String type = "";
				if (clpos != -1 && atpos < clpos) { 
					nomvar = form.substring(atpos+1,clpos);
					type   = form.substring(clpos+1,form.length());
				} else {
					nomvar = form.substring(atpos+1,form.length());
				} // end if..else
				// ---- 
				// System.out.println("@@@@ New nominal "+nomvar+":"+type); 
				nom = newLFNominal(nomvar,type);
				nom.nomVar = nomvar;
				if (resultLF.root == null) { resultLF.root = nom; } 
				// ----
				resultLF.noms = lfAddNominal(resultLF.noms,nom);
				// ----
			} else { // conjuncts and/or embedding 
				StringTokenizer conjuncts = new StringTokenizer(form,"^");
				while (conjuncts.hasMoreTokens()) { 
					String conj = conjuncts.nextToken(); 
					// log(conj,nomstack,nom);
					int ospos = conj.indexOf("<"); 
					int cbpos = conj.indexOf(")");
					int clpos = conj.indexOf(":");
					if (clpos > -1 && ospos < 0) { 
						String nomvar = form.substring(0,clpos);
						String type; 
						if (form.indexOf(" ") > -1) { 
							type = form.substring(clpos+1,form.indexOf(" "));
						} else { 
							type = form.substring(clpos+1,form.indexOf(")"));
						} 
						//---
						log("New nominal "+nomvar+":"+type);
						nom = newLFNominal(nomvar,type);
						//---
						resultLF.noms = lfAddNominal(resultLF.noms,nom);
						//---
						if (rel != null) {
							rel.dep = nomvar;
						}
						else {
							rel = new LFRelation();
							rel.dep = nomvar;
						}
						//---
						log("relation --"+rel.toString());
						log("CURRENT NOMINAL: "+nom.nomVar);
					}
					
					else if (ospos < 0 && rel != null && rel.dep.equals("")) {
						if (cbpos < 0) { 
							String nomvar = conj;
							nomvar = strip(nomvar);
							nom = newLFNominal(nomvar,"");
							//---
							resultLF.noms = lfAddNominal(resultLF.noms,nom);
							//---
							rel.dep = nomvar;
							//---
							log("relation --"+rel.toString());
							log("CURRENT NOMINAL: "+nom.nomVar);
						}
					}
					else {
						if (ospos < 0) { // proposition
							if (cbpos > -1) { 
								String brackets = conj.substring(cbpos,conj.length()); 
								brackets = strip(brackets);
								String proposition = conj.substring(0,cbpos);
								proposition = strip(proposition);
								log("proposition "+proposition+" for "+nom.nomVar);
								// ----
//								nom.prop = new lf.Proposition (proposition, ConnectiveType.NONE, rhsStub);
								nom.prop = new comsys.datastructs.lf.Proposition (proposition, ConnectiveType.NONE);
								log("brackets: "+brackets+" length:"+brackets.length());
								log("stack: "+nomstack.toString());
								// for (int i=0; i < brackets.length()-1; i++) { if (!nomstack.empty()) { nom = (LFNominal) nomstack.pop(); log("pop - prop!--"+i);} }
								for (int i=0; i < brackets.length(); i++) { if (!nomstack.empty()) { nom = (LFNominal) nomstack.pop(); log("pop - prop!--"+i);} }
							} else {
								String proposition = conj;
								proposition = strip(proposition);
//								nom.prop = new lf.Proposition (proposition, ConnectiveType.NONE, rhsStub);
								nom.prop = new comsys.datastructs.lf.Proposition (proposition, ConnectiveType.NONE);
								log("proposition "+proposition+" for "+nom.nomVar);
							} // end if..else check need to pop stack
						} else { // relation or feature
							int cspos = conj.indexOf(">"); 
							if (cspos == conj.length()-1) { // relation (to not coindexed nominal)
								conj = conj.substring(ospos+1,cspos);
								log("Adding relation "+conj+" to "+nom.nomVar);
								// --- 
								rel = newLFRelation();
								rel.mode = conj;
								rel.head = nom.nomVar;
								nom.rels = lfNominalAddRelation(nom,rel);
								nomstack.push(nom);
								// --- 
								log("relation push "+nom.nomVar);
							} else { // feature or relation to coindexed (bare, without brackets) dependent
								String feat = conj.substring(ospos+1,cspos);
								String val;
								String brackets = null;
								if (cbpos > -1) {
									brackets = conj.substring(cbpos,conj.length());
									log("Brackets!!");
									val  = strip(conj.substring(cspos+1,cbpos));
								}  else {
									val  = strip(conj.substring(cspos+1,conj.length()));
								} // end if..else check need to pop stack
								if (val.contains(":")){ // relation to typed and coindexed (bare, without brackets) nominal
									String nomid = val.substring(0,val.indexOf(":"));
									String type = val.substring(val.indexOf(":")+1);
									log("Adding bracketless relation "+feat+" to "+nom.nomVar);
									// ---
									rel = newLFRelation (nom.nomVar, feat, nomid);
									rel.coIndexedDep = true;
									nom.rels = lfNominalAddRelation(nom,rel);
									// --- 
									if(!lfHasNomvar(resultLF,nomid)){
										log("nominal is new");
										LFNominal nomidNom = newLFNominal(nomid,type);
										resultLF.noms = lfAddNominal(resultLF.noms,nomidNom);
			
									} else{
										log("nominal exists already");
										// check if types match
										LFNominal checknom = lfGetNominal(resultLF,nomid);
										if(!checknom.sort.equals(type)){
											log("WARNING!! types mismatch");
										}  // end if.. check for type
									} // check for presence of nominal
								} // end of relation to typed and coindexed nominal
								else { //feature
									Feature nomFeat = new Feature(feat,val);
									nom.feats = lfNominalAddFeature(nom,nomFeat);
									log("Adding feature/value pair "+feat+"/"+val+" to "+nom.nomVar);
								} // end of feature

								if(brackets != null) {
									for (int i=0; i < brackets.length()-1; i++) { 
										if (!nomstack.empty()) { nom = (LFNominal) nomstack.pop(); log("pop-feat!"); } }
								}
							} // end if..else 

						} // end if..else proposition or relation/feature
					} // end if..else new nominal or conjuncts
				} // end while over conjuncts
			} // end if..else whether @head or conjuncts
		} // end while over embeddings
		return resultLF;
		}
	} // end convertFromString


	//=================================================================
	// META-LOGICAL FORM METHODS
	//=================================================================

	/**
       The method <i>determineCopula</i> determines whether the
       logical form expresses a copula-based assertion. If it does,
	   the nominal variable for the copula root is returned, else
	   null is returned

	   @param lf The logical form
	   @return String the identifier for the copula root, if any; else null
	 */

	public static String determineCopula (LogicalForm lf) {
		boolean result = false;
		String copNomvar = null;
		LFNominal root = lf.root;
		log("Root: "+lfNominalToString(root)+" with type ["+root.sort+"]"); 
		if (root.sort.equals("dvp")) { 
			LFRelation cbrel = lfNominalGetRelation(root,"ContentBody"); 
			if (cbrel != null) { 
				String cbRootNV = cbrel.dep;
				root = lfGetNominal(lf,cbRootNV); 
				log("Revised root: "+root.toString()+" with type ["+root.sort+"]"); 
			} else { 
				log("Root is dvp but has no contentbody");	
			} // end if.. check for existing content body
		} // end if.. check for revising root. 
		// Second, check whether we have a root that is a discourse connective; if so, 
		// descend to the Body. 
		if (root.sort.equals("dconn")) {
			LFRelation bodyR = lfNominalGetRelation(root,"Body");
			if (bodyR != null) {
				String bodyNV = bodyR.dep;
				root = lfGetNominal(lf,bodyNV);
			} // end if.. check for Body 
		} // end if.. check for discourse connective
		if (root != null) { 
			if (root.sort.equals("state")) { 
				if (root.prop.prop.equals("be") ||
						root.prop.prop.equals("there-be")) { 
					result = true;
					copNomvar = root.nomVar;
					log("Copula nomvar: "+copNomvar);
				} // end if.. check for be
			} // end if.. check root is state
		} // end if.. check root non-null just to make sure
		log("Assertion is copula-construction: "+result);
		return copNomvar;
	} // end determineCopula



	//=================================================================
	// MISC METHODS
	//=================================================================

	/** Returns a string presentation of the given logical form. The
		presentation is just a list of nominals! 


	public static String lfToString (LogicalForm lf) { 
		String result = "Logical form ["+lf.id+"]:\n";
		Iterator nomIter = lfGetNominals(lf); 
		while (nomIter.hasNext()) { 
			LFNominal nom = (LFNominal) nomIter.next();
			result = result+lfNominalToString(nom)+"\n";
		} // end while over nominals
		return result;
	} // end lfToString
	 */

	/**
	 *  The function <i>depNomToString</i> produces a String
	 *  representation for the given nominal, based on the fact that
	 *  this nominal is embedded in a structure (i.e. not a root).
	 */

	private static String depNomToString (LFNominal nom, LogicalForm lf) { 
		if (nom == null) { 
			// System.out.println("WARNING!!"); 
			return null; 
		} else { 
			String result = nom.nomVar+":"+nom.sort; 

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
				result = result + prop;
				//if (pIter.hasNext()) { result = result + " ^ ";} 
				props = true;
			} // end if over proposition
			if (props) {if (feats == false && rels==false) { result = result + ")"; }}
			//if (props && !(feats || rels)) { result = result + ")"; }
			// the result now does not have a conjunction at the end: 
			// we need one if there are both props and feats
			if (props & feats) { result = result + " ^ ";}
			while (fIter.hasNext()) { 
				Feature feat = (Feature) fIter.next(); 
				String val  = feat.value;
				result = result + "<"+feat.feat+">"+val;
				if (fIter.hasNext()) { result = result + " ^ "; }
			} // end for over features
			// the result now does not have a conjunction at the end.  if
			// there were no feats, just props, then there is no
			// conjunction either
			// if (props || feats) { result=(rIter.hasNext())?result+" ^ ":result+"#)"; }    // CLOSING BRACKET
			if (props) { 
				if (feats == true && rels == false) { 
					// if features but no more relations
					result = result + ")"; 
				} else { 
					// if no features, or there are relations ...
					// but we do not want this if there were no features
					if (rels) { result = result + " ^ "; } 
				} 
			} // if propositions

			if (props == false) { if (feats==true && rels==false) { result = result + ")"; } else { result = result + " ^ ";} }

			while (rIter.hasNext()) { 
				LFRelation rel = (LFRelation) rIter.next();
				result = result + " <"+rel.mode+">";
				LFNominal depnom = lfGetNominal(lf,rel.dep); 
				log("depNomToString: looking for dependent nom of head: "+rel.head+" with rel "+rel.mode+" and nomid "+rel.dep+", finding: "+depnom);
				if(rel.coIndexedDep ==true) { // we only want to generate <RelMode>var1:type1, not the complete nominal
					log("depnomtostr GENERATE coindexed!");
					result = result + rel.dep+":"+depnom.sort;
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
						String depStr = depNomToString(depnom,lf);
						if (depStr != null) { result = result + "(" +depNomToString(depnom,lf); }
					}
				}
				// ADD CLOSING BRACKET
				// result = result + ")"; 
				if (rIter.hasNext()) { result = result + " ^ "; } 
			} // end for over relations
			// if (rels) { result = result + "+"+nom.getNomvar()+")";} 
			if (rels) { result = result + ")";} 
			// System.out.println("***** return from depNomToString: "+result);
			return result; 
		}
	} // end depNomToString


	private static String rootNomToString (LFNominal nom, LogicalForm lf) { 
		String result = "@"+nom.nomVar+":"+nom.sort+"("; 

		// Iterator pIter = nom.getPropositions();
		boolean props = (!nom.prop.prop.equals("")); 	

		Iterator fIter = lfNominalGetFeatures(nom);
		boolean feats = (fIter.hasNext());
		Iterator rIter = lfNominalGetRelations(nom); 
		boolean rels = rIter.hasNext(); 

		//while (pIter.hasNext()) { 
		String prop = nom.prop.prop;
		if (props) { 
			result = result + prop;
		} // end for over propositions
		// the result now does not have a conjunction at the end: 
		// we need one if there are both props and feats
		if (props & feats) { result = result + " ^ ";}
		while (fIter.hasNext()) { 
			Feature feat = (Feature) fIter.next(); 
			result = result + "<"+feat.feat+">"+feat.value;
			if (fIter.hasNext()) { result = result + " ^ "; }
		} // end for over features
		// the result now does not have a conjunction at the end.  if
		// there were no feats, just props, then there is no
		// conjunction either
		if (props || feats) { result=(rIter.hasNext())?result+" ^ ":result; } 
		while (rIter.hasNext()) { 
			LFRelation rel = (LFRelation) rIter.next();
			result = result + " <"+rel.mode+">";

			log("Now trying to print dep with nomvar "+rel.dep+" in relation "+rel.mode+" from "+rel.head);

			LFNominal depnom = lfGetNominal(lf, rel.dep); 
			if(rel.coIndexedDep == true){ // we only want to generate <RelMode>var1:type1, not the complete nominal
				log("root nom 2 str GENERATE coindexed!");
				result = result + rel.dep+":"+depnom.sort;
			} else {
				result = result + "(" + depNomToString(depnom,lf);
			}
			// ADD CLOSING BRACKET
			// result = result + "/"+nom.getNomvar()+")";
			if (rIter.hasNext()) { result = result + " ^ "; } 
		} // end for over relations
		// if (rels) { result = result + "-)";} 
		//	 CAUTION : sw 051015 changed here, (removed "if (rels), because something goes wrong in QuestionLF.constructLFSubtree
		// have not checked if everything else still works!
		//	if (rels) { result = result + ")";}  
		{ result = result + ")";} 
		return result; 
	} // end rootNomToString




	public static void writeDOTFile(String DOTText, String DOTFile) {		
		try {
			FileOutputStream fout = new FileOutputStream (DOTFile);
			new PrintStream(fout).println (DOTText);
			fout.close();		
		}
		catch (java.io.IOException e) {
			e.printStackTrace();
		}
	}

	private static void showPNGGraph (String PNGFile) {
		ImageIcon i = new ImageIcon(PNGFile) ;
		JOptionPane.showMessageDialog(null,i,"Created graph",JOptionPane.INFORMATION_MESSAGE, null);
	}

	private static String getPropositionString(comsys.datastructs.lf.Proposition prop) {		
		String propString = prop.prop ;

		// Since the recursive definition of the Proposition was removed by Nick, 
		// we don't treat complex propositions anymore

		/**	String conn = "";
		if (prop.connective == ConnectiveType.CONJUNCTIVE) conn = " & " ;
		if (prop.connective == ConnectiveType.DISJUNCTIVE) conn = " or " ;
		if (prop.connective == ConnectiveType.XDISJUNCTIVE) conn = " xor " ;
		if (!conn.equals("")) {
			for (int i=0; i < prop.rhsProp.length ; i++ ) {
				propString = conn + getPropositionString(prop.rhsProp[i])  + "\\n" ;
			}
		} */ 
		return propString ;
	}

	public static String createDOTSpecs(LogicalForm lf) {

		if (lf==null) {
			return "" ;
		}

		String text = "digraph G {\n";
		LFNominal[] lfNoms =  lf.noms;
		for (int i = 0; i < lfNoms.length; i++) {
			String nomVarAndSort = "\\{"+lfNoms[i].nomVar+":"+lfNoms[i].sort+"\\}" ;

			String features = "" ;
			for (int j=0;j<lfNoms[i].feats.length;j++) {
				features += lfNoms[i].feats[j].feat+":"+lfNoms[i].feats[j].value  + "\\n" ;
				if (j < lfNoms[i].feats.length-1) 
					features += " & ";
			}

			String proposition = getPropositionString(lfNoms[i].prop) ;

			text += lfNoms[i].nomVar + " [shape=Mrecord fontsize=10 label=\"{" ;

			if (!proposition.equals("")) {
				text += proposition + "|" ;
			}
			text += nomVarAndSort ;

			if (!features.equals(""))
				text += "|"+features ;

			text += "}\"]"+";\n" ;

			String relations = "" ;
			for (int j=0;j<lfNoms[i].rels.length;j++) {
				LFRelation rel = lfNoms[i].rels[j] ;
				text += rel.head + " -> " + rel.dep + "[label=\"" + rel.mode + "\"];\n" ;
				if (rel.coIndexedDep) {
					text += rel.dep + " -> " + rel.head +";\n"; 
				}
			}

		}
		text += "\n}";
		return text ;
	}
	
	
	static int PLFIncrement = 0;

	/**
	 * Generate a DOT specification for the packed logical form
	 * @param plf packed logical form
	 * @return text containing the DOT specs
	 */
	public static String createDOTSpecs(PackedLogicalForm plf) {

		if (plf==null) {
			return "" ;
		}

		String text = "digraph G {\n";

		String edgeText = "";


		// loop on packing nodes
		for (int i=0 ; i< plf.pNodes.length ; i++) {

			// we create a cluster for each packing node
			text += "subgraph cluster"+PLFIncrement + "{\n" ;
			PLFIncrement++;

			for (int n=0; n<plf.pNodes[i].packedNoms.length; n++) {
				// we insert the packed nominal
				PackedNominal nom = plf.pNodes[i].packedNoms[n];

				text += nom.nomVar + " [shape=Mrecord fontsize=10 label=\"{" ;

				String proposition = getPropositionString(nom.prop) ;
				if (!proposition.equals("")) {
					text += proposition + "|" ;
				}

				String sort = "" ;
				String[] nomVars = nom.nomVar.split("_");
				if (nom.packedSorts.length == 1) {
					sort = "\\{"+nom.nomVar+":"+nom.packedSorts[0].sort+"\\}" ;
				}
				else {
					for (int s=0 ; s < nom.packedSorts.length ; s++) {
						sort += "\\{"+nomVars[0]+":"+nom.packedSorts[s].sort+"\\}\t" 
						+ " (" + nom.packedSorts[s].lfIds.length +" LFs) \\n";
					}
				}

				text += sort ;

				// we generate the features text
				String features = "" ;
				for (int j=0;j<nom.feats.length;j++) {
					features += nom.feats[j].feat+":"+nom.feats[j].value  + " (" + nom.feats[j].lfIds.length +" LFs) \\n" ;
					if (j < nom.feats.length-1) {
						features += " & ";
					}
				}
				if (!features.equals("")) {
					text += "|"+features ;
				}

				text += "}\"]"+";\n" ;

				for (int r=0 ; r < nom.rels.length ; r++) {
					text += nom.rels[r].head + " -> " + nom.rels[r].dep + "[label=\"" + nom.rels[r].mode + "\"];\n";
				}

			}

			for (int j=0 ; j <plf.pNodes[i].nomsPePairs.length ; j++) {
				PackingEdge pe = plf.pNodes[i].nomsPePairs[j].pe ;
				text += pe.peId  + " " +
				"[shape=polygon, sides=6, label=\"\"];\n" ;
				text += pe.head + " -> " + pe.peId +"[label=\""+ pe.mode +"\"];\n" ;
				for (int k=0 ; k< pe.targets.length ; k++) {
					edgeText += pe.peId + "-> " + pe.targets[k].pnId.substring(0,pe.targets[k].pnId.length() -3)  +
					"[fontsize=8, style=dotted, label=\"(" + pe.targets[k].lfIds.length + " LFs)\"];\n" ;
				}
			}

			String prefScore = (new Float(plf.pNodes[i].preferenceScore)).toString() ;
			if (prefScore.length() > 4) {
				prefScore = prefScore.substring(0, 4);
			}
			text += "style=filled ;\n";
			text += "color=lightgrey ; \n " ;
			text += "fontsize=10 ;\n";
			text += "label=\" "+ plf.pNodes[i].pnId.substring(0,plf.pNodes[i].pnId.length() -3) +
			" (" + plf.pNodes[i].lfIds.length+" LFs, pref. score=" + 
			prefScore +")\";\n" ;

			text += "}\n";

		}
		return text+edgeText+"\n}";
	}
	
	

	public static void lfToGraph(LogicalForm lf, String graphName, boolean generatePNG) {

		log ("Start generating the graphical version of the Logical form " + lf.logicalFormId + "...");
		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

		String DOTText = createDOTSpecs(lf) ;

		if (!DOTText.equals("")) {
			writeDOTFile(DOTText,DOTFile);

			if (generatePNG) {
			try	{
				Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			}
		}
		log("File " + PNGFile + "successfully written");

		// showPNGGraph(PNGFile);
	}


	public static void plfToGraph(PackedLogicalForm plf, String graphName, boolean generatePNG) {

		if (plf != null) {
		log ("Start generating the graphical version of the Packed Logical form " + plf.packedLFId + "...");
		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

//		String DOTText = "digraph G { rnode -> dnode1 ; rnode -> dnode2,}" ;
		String DOTText = createDOTSpecs(plf) ;

		if (!DOTText.equals("")) {
			writeDOTFile(DOTText,DOTFile);

			if (generatePNG) {
			try	{
				Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			}
		}
		log("File " + PNGFile + "successfully written");
		
		// showPNGGraph(PNGFile);
		}
	}


	/**
	 *  The method <i>toString</i> creates a String representation of
	 *  the given logical form. 
	 * 
	 *  <h4>Warning</h4>
	 * 
	 *  Only if the root nominal is of type <tt>disc-vantagepoint</tt>
	 *  do we create a logical form of the type
	 *  <tt>@root:t&lt;rel&gt;nom ^ @nom:t2&lt;rel2&gt;...</tt>.
	 *
	 *	@since 061102 (061102)
	 *
	 */

	public static String lfToString (LogicalForm lf) { 
		String result = ""; 
		LFNominal rootnom = lf.root;

		// JUST IN CASE the LF is empty...
		if (rootnom==null) {
			return null;
		} 
		
		/** 
		else if (rootnom.sort.equals("dvp")) { 
			log("okay do discourse");
			result = result+"(@"+rootnom.nomVar+":"+rootnom.sort;
			result = result + "(";
			// Iterator pIter = root.getPropositions(); 
			// while (pIter.hasNext()) {
			//	String p = (String) pIter.next();
			//	result = result + p + " ^ ";
			//} 
			result = result+rootnom.prop.prop + " ^ ";
			Iterator rIter = lfNominalGetRelations(rootnom);
			while (rIter.hasNext()) { 
				LFRelation rel = (LFRelation) rIter.next();
				String depvar  = rel.dep;
				LFNominal dep  = lfGetNominal(lf,depvar);
				// System.out.println("Printing LF for dep ["+dep+"] with depvar ["+depvar+"]");
				result = result + "<"+rel.mode+">"+depvar+":"+dep.sort+")";
			}
			rIter = lfNominalGetRelations(rootnom);
			if (rIter.hasNext()) { result = result + " ^ "; }
			while (rIter.hasNext()) { 
				LFRelation rel = (LFRelation) rIter.next();
				String depvar  = rel.dep;
				LFNominal dep  = lfGetNominal(lf,depvar);
				result = result + rootNomToString(dep,lf);
				if (rIter.hasNext()) { result = result + " ^ "; }
				result = result + ")";
			}	    
			result = result+")";
		} 
		*/ 
		
		else {
			// the root is not a dialogue point, so just do the
			// root and embed everything directly under it.

			// collect the root dependents
			Vector rootDependents = LFUtils.lfCollectNomvars(rootnom,lf);

			result = "(";
			result = result + rootNomToString(rootnom,lf);
			
			for (int i=0; i < lf.noms.length; i++) {
				LFNominal nom = lf.noms[i];
				
				if (!result.contains(nom.nomVar)) {
					log("Nominal ["+nom.nomVar+"] not subordinated to the root ["+rootnom.nomVar+"]");
					result = result + " ^ " + rootNomToString(nom,lf);
					log("Result: "+result);
				} else {
					log("Nominal ["+nom.nomVar+"] subordinated to the root ["+rootnom.nomVar+"]");				
				} 
			}
			
			// result = result + "*)";
			result = result + ")";
		}  // end if..else check whether dialogue act or not
		return result;
	} // end toString(); 

	/** Returns a string presentation of the given nominal */
	public static String lfNominalToString (LFNominal nom) { 
		if (nom.nomVar.equals("")) { System.out.println("ERROR! EMPTY NOMINAL"); }
	
		String res= "@"+nom.nomVar+":"+nom.sort+"(";
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
			res=res+" <"+f+">"+v;
			if (fiter.hasNext() || java.lang.reflect.Array.getLength(nom.rels) > 0) { res=res+" ^"; }
		} // end while over features
		Iterator riter = lfNominalGetRelations(nom);
		while (riter.hasNext()) { 
			LFRelation r = (LFRelation) riter.next();
			res=res+"<"+r.mode+">"; 
			String dp = r.dep; 
			res = res+dp;
			if (riter.hasNext()) { res=res+" ^"; } 
		} // end while over relations
		res=res+")";
		return res;
	} // end lfNominalToString


	public static PackingNode plfGetPackingNodeFromNomVar(PackedLogicalForm plf, String nomVar) {
		for (int i=0; i < plf.pNodes.length ; i++) {
			PackingNode pn = plf.pNodes[i];
			for (int j=0; j < pn.packedNoms.length ; j++) {
				PackedNominal pnom = pn.packedNoms[j];
				if (pnom.nomVar.equals(nomVar)) {
					return pn;
				}
			}
		}
		return null;
	}
	
	public static List copyList(List list) {
		List newList = new ArrayList ();
		Iterator it = list.iterator();
		while (it.hasNext()) {
			Object obj = it.next();
			newList.add(obj);
		}
		
		return newList;
	}
	
	/**
	 * Remove a dependence (either an internal relation or a packing edge) attached
	 * to a specific nominal in a packed logical form
	 * @param plf the packed logical form
	 * @param nomVar the nominal variable identifier
	 * @param dep the mode of the dependend
	 * @return the modifier packed logical form
	 */
	public static PackedLogicalForm plfRemoveDependence(PackedLogicalForm plf, String sourceNomVar, String targetNomVar, String dep) {

		// this is the nominal from which the dependence will b deleted
		PackedNominal sourceNom = plfGetPackedNominal(plf, sourceNomVar);

		if (sourceNom != null) {

			// we first look at internal relations
			List<LFRelation> relsList = new ArrayList<LFRelation>
				(Arrays.asList(sourceNom.rels));
			Iterator<LFRelation> it = ((List<LFRelation>)copyList(relsList)).iterator();
			int nbDeletions = 0;
			while (it.hasNext()) {
				LFRelation rel = it.next();
				if (rel.mode.equals(dep) && rel.dep.equals(targetNomVar)) {
					relsList.remove(rel);
					log("internal relation deleted");
					nbDeletions++;
				}
			}
			sourceNom.rels = (LFRelation[]) resizeArray(sourceNom.rels, sourceNom.rels.length - nbDeletions);
			sourceNom.rels = relsList.toArray(sourceNom.rels);

			// then at packing edges
			List<PackingEdge> peList = new ArrayList<PackingEdge>
			(Arrays.asList(sourceNom.pEdges));
			Vector<PackingEdge> deletedEdges = new Vector<PackingEdge>();
			Iterator<PackingEdge> it2 = ((List<PackingEdge>)copyList(peList)).iterator();
			while (it2.hasNext()) {
			
				PackingEdge pe = it2.next();
				if (pe.mode.equals(dep)) {
					List<PackingNodeTarget> petList = new ArrayList<PackingNodeTarget> (Arrays.asList(pe.targets));
					Iterator<PackingNodeTarget> it4 = ((List<PackingNodeTarget>) copyList(petList)).iterator() ;
					while (it4.hasNext()) {
						PackingNodeTarget pnt = it4.next();
						PackingNode PN = plfGetPackingNode(plf, pnt.pnId);
						
						if (PN != null && PN.root.equals(targetNomVar)) {
							petList.remove(pnt);
 							if (petList.size() == 0) {
								peList.remove(pe);
								log("packing edge deleted");
								deletedEdges.add(pe);		
							}
						}
						pe.targets = (PackingNodeTarget[]) resizeArray(pe.targets, petList.size());
						pe.targets = petList.toArray(pe.targets);
					}

				}
			}
			sourceNom.pEdges = (PackingEdge[]) resizeArray(sourceNom.pEdges, sourceNom.pEdges.length - deletedEdges.size());
			sourceNom.pEdges = peList.toArray(sourceNom.pEdges);

			// we also have to delete the related sourceNominal packing edge pairs
			PackingNode pn = plfGetPackingNodeFromNomVar(plf, sourceNomVar) ;
			List<NominalPackingEdgePair> nominalPackingEdgePairsList = new ArrayList<NominalPackingEdgePair>
			(Arrays.asList(pn.nomsPePairs));
			Iterator<NominalPackingEdgePair> it3 = ((List<NominalPackingEdgePair>)copyList(nominalPackingEdgePairsList)).iterator();
			while (it3.hasNext()) {
				NominalPackingEdgePair pair = it3.next();
				if (deletedEdges.contains(pair.pe)) {
					if (nominalPackingEdgePairsList.contains(pair)) {
						nominalPackingEdgePairsList.remove(pair);
						log("packing edge pair deleted");
					}
				}
			}
			pn.nomsPePairs = (NominalPackingEdgePair[]) resizeArray(pn.nomsPePairs, pn.nomsPePairs.length - deletedEdges.size());
			pn.nomsPePairs = nominalPackingEdgePairsList.toArray(pn.nomsPePairs);
	}
		else {
			System.out.println("Problem, nominal not found in the packed logical form");
		}
		
		// we return the modified packed logical form
		return plf;
	}

	
	
/**
 * Remove a set of dependencies which are "incompatible" with a supported one, ie remove the set of 
 * packing edges which have the same target nominal (but a different source nominal or relation label)
 * @param plf the packed logical form
 * @param sourceNomVar the source nominal identifier of the supported edge
 * @param targetNomVar the target nominal identifier of the supported edge
 * @param dep the relation label of the supported edge
 * @return the modified logical form
 */
	public static PackedLogicalForm plfRemoveIncompatibleDependencies (PackedLogicalForm plf, 
			String sourceNomVar, String targetNomVar, String dep) {

		// this is the nominal from which the dependence will b deleted
		PackingNode sourcePN = plfGetPackingNodeFromNomVar(plf, sourceNomVar);
		for (int i=0; i < plf.pNodes.length ; i++) {
			PackingNode PN = plf.pNodes[i];
			
			List<NominalPackingEdgePair> nominalPackingEdgePairsList = new ArrayList<NominalPackingEdgePair>
			(Arrays.asList(PN.nomsPePairs));
			Iterator<NominalPackingEdgePair> it2 = ((List<NominalPackingEdgePair>)
					copyList(nominalPackingEdgePairsList)).iterator();
			while (it2.hasNext()) {
					NominalPackingEdgePair pair = it2.next();
				List<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget> 
				(Arrays.asList(pair.pe.targets));
				Iterator<PackingNodeTarget> it3 = ((List<PackingNodeTarget>)
						copyList(targets)).iterator();
				while (it3.hasNext()) {
					PackingNodeTarget target = it3.next();
					PackingNode depPN = plfGetPackingNode(plf, target.pnId);
					
					if ((!pair.head.equals(sourceNomVar) || !pair.pe.mode.equals(dep)) && depPN.root.equals(targetNomVar)) {
						targets.remove(target);
						if (targets.size() == 0) {
							nominalPackingEdgePairsList.remove(pair);
							PackedNominal nom = plfGetPackedNominal(PN, pair.head);
							List<PackingEdge> edges = new ArrayList<PackingEdge>
								(Arrays.asList(nom.pEdges));
							edges.remove(pair.pe);
							nom.pEdges = (PackingEdge[]) resizeArray(nom.pEdges, nom.pEdges.length -1);
							nom.pEdges = edges.toArray(nom.pEdges);
						}
					}
					pair.pe.targets = (PackingNodeTarget[]) resizeArray(pair.pe.targets, targets.size());
					pair.pe.targets = targets.toArray(pair.pe.targets);
				}
		}
		PN.nomsPePairs = (NominalPackingEdgePair[]) resizeArray(PN.nomsPePairs, nominalPackingEdgePairsList.size());
		PN.nomsPePairs = nominalPackingEdgePairsList.toArray(PN.nomsPePairs);

		}
		return plf;
	}

	
	/** Return the number of packing edge targets in a given packed logical form
	 * (useful for pruning)
	 * @param plf
	 * @return
	 */
	public static int plfGetNumberOfPackingEdgeTargets(PackedLogicalForm plf) {
		int result = 0;
		for (int i=0; i< plf.pNodes.length; i++) {
			for (int j=0; j < plf.pNodes[i].nomsPePairs.length ; j++) {
				for (int k=0; k < plf.pNodes[i].nomsPePairs[j].pe.targets.length ; k++) {
					result++;
				}
			}
		}
		return result;
	}
	
	
	/**
	 * Returns the set of logical forms for which the relation (specified by its source and
	 * target nominals and its mode) is being matched
	 * @param plf
	 * @param sourceNomVar
	 * @param targetNomVar
	 * @param mode
	 * @return
	 */
	public static Vector<String> getMatchedLFs (PackedLogicalForm plf, String sourceNomVar, String targetNomVar, String mode) {
		Vector<String> matchedLFs = new Vector<String>();

		for (int i=0; i < plf.pNodes.length ; i++) {
			for (int j=0; j < plf.pNodes[i].packedNoms.length ; j++) {
				PackedNominal nom = plf.pNodes[i].packedNoms[j] ;
				if (nom.nomVar.equals(sourceNomVar)) {
					for (int k=0; k < nom.rels.length ; k++) {
						if (nom.rels[k].dep.equals(targetNomVar) && nom.rels[k].mode.equals(mode)) {
							ArrayList<String> lfIds = new ArrayList<String>(Arrays.asList(plf.pNodes[i].lfIds));
							matchedLFs.addAll(lfIds);
						}
					}
				}
			}
			for (int j=0; j < plf.pNodes[i].nomsPePairs.length ; j++) {
				String nomVar = plf.pNodes[i].nomsPePairs[j].pe.head ;
				if (nomVar.equals(sourceNomVar) && plf.pNodes[i].nomsPePairs[j].pe.mode.equals(mode)) {
					for (int k=0; k < plf.pNodes[i].nomsPePairs[j].pe.targets.length; k++) {
						String targetPnId = plf.pNodes[i].nomsPePairs[j].pe.targets[k].pnId;
						PackingNode targetPN = plfGetPackingNode(plf, targetPnId);
						if (targetPN.root.equals(targetNomVar)) {
							ArrayList<String> lfIds = new ArrayList<String>(Arrays.asList(plf.pNodes[i].nomsPePairs[j].pe.targets[k].lfIds));
							matchedLFs.addAll(lfIds);
						}
					}
				}
			}
		}
		
		return matchedLFs;
	}
	

	/**
	 * Returns the set of logical forms for which the relation (specified by its source and
	 * target nominals and its mode) is NOT being matched
	 * @param plf
	 * @param sourceNomVar
	 * @param targetNomVar
	 * @param mode
	 * @return
	 */
	public static Vector<String> getUnmatchedLFs (PackedLogicalForm plf, String sourceNomVar, String targetNomVar, String mode) {
		Vector<String> unmatchedLFs = new Vector<String>();
		
		Vector<String> matchedLFs = getMatchedLFs(plf, sourceNomVar, targetNomVar, mode);
		
		PackingNode PN = plfGetPackingNode(plf, plf.root);
		for (int i=0; i < PN.lfIds.length ; i++) {
			if (!matchedLFs.contains(PN.lfIds[i])) {
				unmatchedLFs.add(PN.lfIds[i]);
			}
		}
		return unmatchedLFs;
	}
	
	
	/**
	 * Returns the set of nominals contained in a packed logical form, 
	 * according to a particular dag traversal.
	 * 
	 * @param plf the packed logical form
	 * 
	 * @param traversalType the DAG traversal type: either "BF" (breadth-first) or "DF" (depth-first).  
	 *                      (DF is assumed by default)
	 *                      
	 * @param skipRedundantPaths true if the redundant paths (ie. if multiple paths in the dag can 
	 *                           lead to the same nominals) are to be skipped, false otherwise
	 *                           
	 * @return a vector containing the set of nominals
	 */
	private static Vector<PackedNominal> getNominals (PackedLogicalForm plf, 
			String traversalType, boolean skipRedundantPaths) throws ComsysException {
		
		// the set of nominals
		Vector<PackedNominal> nominals = new Vector<PackedNominal>();
		
		// the data struct containing all "unseen" nominals
		LinkedList<Object[]> unseenNominals = new LinkedList<Object[]>();
		
		// We add the root nominal
		PackingNode rootPN = LFUtils.plfGetPackingNode(plf, plf.root);
		Object[] tuple = new Object[2];
		tuple[0] = rootPN;
		tuple[1] = LFUtils.plfGetPackedNominal(rootPN, rootPN.root);
		unseenNominals.addLast(tuple);

		// Loop till the set of unseen nominals is empty
		while (!unseenNominals.isEmpty()){
			
			// Depending on the traversal type, remove the first or the last 
			// element of the unseen nominals
			if (traversalType.equals("DF")) {
				tuple = unseenNominals.removeLast();
			}
			else if (traversalType.equals("BF")) {
				tuple = unseenNominals.removeFirst();
			}
			else {
				log("Warning: traversal type not defined.  We assume depth-first search");
				tuple = unseenNominals.removeLast();
			}
			
			// packing node and packed nominal
			PackingNode packingNode = (PackingNode) tuple[0];
			PackedNominal packedNominal = (PackedNominal) tuple[1];

			// If redundant paths are to be skipped, verify if the nominal is 
			// not already in the set
			if (!skipRedundantPaths || !nominals.contains(packedNominal)) {

				// We add the nominal
				nominals.add(packedNominal);

				// Loop on the packing edges attached to the nominal
				for (int i=0; i < packingNode.nomsPePairs.length; i++) {
					if (packingNode.nomsPePairs[i].head.equals(packedNominal.nomVar)) {
						for (int j=0; j < packingNode.nomsPePairs[i].pe.targets.length; j++) {
							PackingNode depPN = LFUtils.plfGetPackingNode(plf, packingNode.nomsPePairs[i].pe.targets[j].pnId);
							if (depPN != null) {
								Object[] newTuple = new Object[2];
								newTuple[0] = depPN;
								newTuple[1] = LFUtils.plfGetPackedNominal(depPN, depPN.root);
								unseenNominals.addLast(newTuple);
							}
							else {
								throw new ComsysException("ERROR: one packing edge points to a packing node which no longer exists");
							}
						}
					}
				}

				// Loop on the relations attached to the nominal
				for (int i=0; i < packedNominal.rels.length; i++) {
					PackedNominal depNom = LFUtils.plfGetPackedNominal(packingNode, packedNominal.rels[i].dep);
					Object[] newTuple = new Object[2];
					newTuple[0] = packingNode;
					newTuple[1] = depNom;
					unseenNominals.addLast(newTuple);
				}
			}

		}
		// return the set of nominals
		return nominals;
	}

	/**
	* Returns an iterator on the set of nominals contained in a packed logical form, 
	* according to a particular dag traversal
	* 
	* @param plf the packed logical form
	* 
	* @param traversalType the DAG traversal type: either "BF" (breadth-first) or "DF" (depth-first).  
	*                      (depth-first search is assumed by default)
	*                      
	* @param skipRedundantPaths true if the redundant paths (ie. if multiple paths in the dag can 
	*                           lead to the same nominals) are to be skipped, false otherwise
	*                           
	* @return a vector containing the set of nominals
	*/
	public static Iterator<PackedNominal> getNominalsIterator (PackedLogicalForm plf, 
			String traversalType, boolean skipRedundantPaths) {	
		try {
			return getNominals(plf, traversalType, skipRedundantPaths).iterator();
		}
		catch (ComsysException e) {
			System.out.println("[LFUtils] " + e.getMessage());
			return null;
		}
	}


	/** 
		The method <i>getPackedNominalSort</i> returns the (stable) ontological sort of the given nominal. 
		
		@param nom The packed nominal
		@return String	The ontological sort
	*/ 

	public static String getPackedNominalSort (PackedNominal nom) { 
		TreeSet addedSorts = new TreeSet();
		ArrayList<PackedOntologicalSort> packedSorts = new ArrayList<PackedOntologicalSort>(Arrays.asList(nom.packedSorts));
		Iterator sortsIter = packedSorts.iterator();
		while (sortsIter.hasNext()) { 
			PackedOntologicalSort pSort = (PackedOntologicalSort) sortsIter.next();
			String sort = pSort.sort;
			if (!addedSorts.contains(sort)) { 
				addedSorts.add(sort);
			} // end if..check whether already added
		} // end while over sorts
		return (String) addedSorts.first();
	} // end getPackedNominalSort

	/** 
		The method <i>createNomTreeMap</i> creates a TreeMap keyed by the unique nominal variables present in the 
		packed logical form, returning for each variable a pointer to the PackedNominal object representing it in 
		the packed logical form. 
		
		@param plf The packed logical form
		@return TreeMap The treemap &lt;String,PackedNominal&gt; over packed nominal objects
	
	*/ 

	public static TreeMap<String,PackedNominal> createNomTreeMap (PackedLogicalForm plf) { 
		TreeMap<String,PackedNominal> packedNoms = new TreeMap<String,PackedNominal>();
		for (int i=0; i< plf.pNodes.length ; i++) {
			PackingNode node = (PackingNode) plf.pNodes[i]; 
			// get the nominal variable
			String nomVar = node.root;
			// get the data: get the list of nominals
			ArrayList<PackedNominal> packedNominals = new ArrayList<PackedNominal>(Arrays.asList(node.packedNoms));		
			Iterator pNomsIter = packedNominals.iterator();
			while (pNomsIter.hasNext()) { 			
				PackedNominal nom = (PackedNominal) pNomsIter.next();
				if (!packedNoms.containsKey(nom.nomVar) && !nom.nomVar.equals("rootNom")) { 
					packedNoms.put(nom.nomVar,nom); 
				} else { 
					log("Non-unique packed nominal variable spotted: ["+nom.nomVar+"] -- not added");
				} // end if..else
			} // end while over nominals
		} // end for over the packing nodes
		return packedNoms;
	} // end createNomTreeMap


	

	private static String strip (String s) { 
		StringTokenizer tok = new StringTokenizer(s);
		String result = "";
		while (tok.hasMoreTokens()) { 
			String c = tok.nextToken();
			if (!c.equals(" ")) { result=result+c; } 
		} 
		return result;
	} // end strip


	public static void log (String m) { 
		if (logOutput) { 
			System.out.println(m);
		} // end if
	} // end log

} // end class
