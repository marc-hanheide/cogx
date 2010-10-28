// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Pierre Lison (plison@dfki.de)
// Geert-Jan M. Kruijff (gj@dfki.de)
//                                                                                                                          
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION 
package de.dfki.lt.tr.dialogue.util;

//=================================================================
// IMPORTS 

// Java
import java.util.*;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.lf.*;

// OpenCCG 
import opennlp.ccg.synsem.*;
import opennlp.ccg.realize.*;

// JDom
import org.jdom.*;
import org.jdom.output.XMLOutputter;

/**
	The class <tt>LFUtils</tt> implements a set of utilities operating on 
	the slice data structures for representing (packed) logical forms. 
	
	@version 100607 (heavy refactoring)
	@since 	 061023
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/

public class LFUtils {

	// flag to turn logging on/off
	public static boolean logOutput = false; 
	
	/**
	 * Writes the provided message m to system output, if logging has been turned on. 
	 * Logging can be turned on/off by setting the <tt>logOutput</tt> flag (public access). 
	 * @param m	The message to be written
	 */
	public static void log (String m) { 
		if (logOutput) { 
			System.out.println(m);
		} // end if
	} // end log
	
	/** 
	 * Reallocates an array with a new size, and copies the contents of the old array to the new array.
	 * @param oldArray  the old array, to be reallocated.
	 * @param newSize   the new array size.
	 * @return          A new array with the same contents.
	 */
	
	public static Object resizeArray (Object oldArray, int newSize) {
		int oldSize = java.lang.reflect.Array.getLength(oldArray);
		Class elementType = oldArray.getClass().getComponentType();
		Object newArray = java.lang.reflect.Array.newInstance(
				elementType,newSize);
		int preserveLength = Math.min(oldSize,newSize);
		if (preserveLength > 0) System.arraycopy (oldArray,0,newArray,0,preserveLength);
		return newArray; 
	} // end resizeArray
	
	private static String strip (String s) { 
		StringTokenizer tok = new StringTokenizer(s);
		String result = "";
		while (tok.hasMoreTokens()) { 
			String c = tok.nextToken();
			if (!c.equals(" ")) { result=result+c; } 
		} 
		return result;
	} // end strip
	
	/** 
	 * Creates a new LFNominal object, with properly initialized (non-null) fields 
	 * 
	 * @return LFNominal the initialized object
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

	/** 
	 * Creates a new LFNominal object, with properly initialized (non-null) fields, 
	 * and the nomVar set to the given variable 
	 * 
	 * @param id The nominal variable for the nominal
	 * @return LFNominal the initialized object
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

	/** 
	 * Creates a new LFNominal object, with properly initialized 
	 * (non-null) fields, setting the nomVar to the given variable
	 * and the sort of the nominal to the given sort.  
	 * 
	 * @param id The nominal variable for the nominal
	 * @param sort The sort for the nominal		
	 * @return LFNominal the initialized object
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

	/** 
	 * Creates a new LFRelation object, with properly initialized (non-null) fields.
	 * @return LFRelation the initialized object
	 */ 

	public static LFRelation newLFRelation () { 
		LFRelation result = new LFRelation();
		result.head = "";
		result.mode = "";		
		result.dep = "";
		return result;
	} // end newLFRelation


	/** 
	 * Creates a new LFRelation object, with properly initialized (non-null) fields.
	 * @return LFRelation the initialized object
	 */ 

	public static LFRelation newLFRelation (String h, String m, String d) { 
		LFRelation result = new LFRelation();
		result.head = h;
		result.mode = m;		
		result.dep = d;
		return result; 
	} // end newLFRelation
	
	
	/** Creates a new LogicalForm object, with properly initialized (non-null) fields
	 *  @return LogicalForm 	The initialized logical form 
	 */ 

	public static LogicalForm newLogicalForm () { 
		LogicalForm result = new LogicalForm  (); 
		result.logicalFormId = "";
		result.noms = new LFNominal[0];
		result.root = newLFNominal();
		result.stringPos = 0;
		return result;
	} // end newLogicalForm
	
	/** 
	 * Creates a new Proposition object, with properly initialized (non-null) fields.
	 * 
	 * @return Proposition the initialized object
	 */ 

	public static de.dfki.lt.tr.dialogue.slice.lf.Proposition newProposition () { 
		de.dfki.lt.tr.dialogue.slice.lf.Proposition result = new de.dfki.lt.tr.dialogue.slice.lf.Proposition();
		result.prop = "";
		result.connective = ConnectiveType.NONE;
		//	result.rhsProp = new comsys.datastructs.lf.Proposition[0];
		return result;
	} // end newProposition


	/** 
	 * Creates a new Proposition object, with properly initialized (non-null) fields.
	 * @param  The proposition label
	 * @return Proposition the initialized object
	 */ 

	public static de.dfki.lt.tr.dialogue.slice.lf.Proposition newProposition (String propLabel) { 
		de.dfki.lt.tr.dialogue.slice.lf.Proposition result = new de.dfki.lt.tr.dialogue.slice.lf.Proposition();
		result.prop = propLabel;
		result.connective = ConnectiveType.NONE;
		//	result.rhsProp = new comsys.datastructs.lf.Proposition[0];
		return result;
	} // end newProposition

	/** Returns a clone of the given Proposition object */

	public static de.dfki.lt.tr.dialogue.slice.lf.Proposition propositionClone (de.dfki.lt.tr.dialogue.slice.lf.Proposition prop) { 
		de.dfki.lt.tr.dialogue.slice.lf.Proposition result = new de.dfki.lt.tr.dialogue.slice.lf.Proposition();
		result.prop = prop.prop;
		result.connective  = prop.connective;
		return result;
	} // end 
	
		
	/** 
	 * Returns a cloned copy of the given packed logical form, under clone().
	 * @param lf The packed logical form to be cloned
	 * @return PackedLogicalForm The clone
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
	} // end plfClone

	/** 
	 * Returns a clone of the given PackedNominal object 
	 * @param nom	The packed nominal to be cloned
	 * @return PackedNominal The cloned object
	 */

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
	} // end plfNominalClone
	

	
	/**
	 * Returns the packed nominal with the given nominal variable, stored at the node. 
	 * If the given node does not contain the given nominal, then <tt>null</tt> is returned.
	 * 
	 * @param pn		A packing node in a packed logical form
	 * @param nomVar	The nominal variable to be looked for
	 * @return			The packed nominal at the node, with the given variable
	 */

	public static PackedNominal plfGetPackedNominal(PackingNode pn, String nomVar) {
		for (int i=0; i < pn.packedNoms.length ; i++) {
			if (pn.packedNoms[i].nomVar.equals(nomVar)) {
				return pn.packedNoms[i];
			} // end if
		} // end for
		return null;
	} // end plfGetPackedNominal
	
	/**
	 * Returns the packing node with the given id, stored in the given packed logical form
	 * @param plf	The packed logical form from which to retrieve the node
	 * @param pnId	The id of the node to be retrieved
	 * @return PackingNode The retrieved node (or null when the id is not present)
	 */
	
	public static PackingNode plfGetPackingNode(PackedLogicalForm plf, String pnId) {
		for (int i=0; i < plf.pNodes.length ; i++) {
			if (plf.pNodes[i].pnId.equals(pnId)) {
				return plf.pNodes[i];
			}
		}
		return null;
	} // end plfGetPackingNode
	
	/**
	 * Add the given nominal to the given logical form, returning an updated list of nominals for 
	 * the provided logical form
	 * @param lf	The logical form 
	 * @param nom	The nominal to be added
	 * @return LFNominal[]	The updated list of nominals for the given lf (corresponds to lf.noms) 
	 */
	
	public static LFNominal[] lfAddNominal (LogicalForm lf, LFNominal nom) { 
		// System.out.println("Adding nominal ["+nom.nomVar+"]");
		LFNominal[] resizingArray = null; 
		LFNominal[] noms = lf.noms;	
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
	 * Returns a resized LFNominal[] array including the already present LFNominal objects, 
	 * and the given LFNominal object. <p>
	 * The method checks whether the array already contains a nominal 
	 * with the variable of the given LFNominal object. If this is the
	 * case then the old Nominal object is overwritten by the new 
	 * object (through a remove, add sequence).  
	 * 
	 * @param	LFNominal[] array with already present LFNominal objects
	 * @param  LFNominal	to-be-added LFNominal object
	 * @return LFNominal[] array with LFNominal objects
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
	 * Returns the nominal with the given identifier, or <tt>null</tt> if no such nominal is present. 
	 * @param lf the logical form from which the nominal is to be retrieved
	 * @param nomVar the identifier of the nominal to be retrieved
	 * @return LFNominal the nominal (if present; else null) 	
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

	
	
	/** 
	 * Returns a boolean indicating whether the given array of LFNominal
	 * objects contains an object with the given nominal variable as id. 
	 *
	 * @param	lf  the logical form with the nominals array to be searched
	 * @param  nomVar the variable to be searched for
	 * @return boolean whether the nomvar is present
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
	

	public static boolean lfHasNomvar(LogicalForm lf, String nomVar) { 
		LFNominal[] noms = lf.noms;
		return lfHasNomvar(noms,nomVar);
	} // end hasNomvar
	
	
	/** 
	 * Returns a new LFNominal[] array from which any nominal with the 
	 * given nominal variable has been removed 
	 */

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
	
	/**
	 * The method <i>lfConstructSubtree</i> constructs a subtree of
	 * the logical form, by recursively descending from the given 
	 * nominal down to the leaves of the logical form, collecting the
	 * nominals that are governed by the nominal. From the resulting 
	 * list of nominals a new logical form is constructed, with the
	 * given nominal as root.
	 * 
	 * @param root The root from which the subtree should be built
	 * @param lf	The logical form with the nominals
	 * @return LogicalForm the logical form subtree
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
	
	/**
	 * The method <i>lfCollectNomvars</i> recursively descends down the
	 * subtree starting at the given root, and constructs a Vector
	 * with the identifiers of all the nominal variables that are
	 * governed by the root.
	 * 
	 * @param root  The root nominal to be descending from
	 * @param lf	The logical form with all the nominals
	 * @return Vector A vector with all the nominal variables under the root
	 */

	public static Vector lfCollectNomvars (LFNominal root, LogicalForm lf) { 
		Vector occurrences = new Vector();
		return lfCollectNomvars(root,lf,occurrences);
	}

	private static Vector lfCollectNomvars (LFNominal root, LogicalForm lf, Vector occurrences) 
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
	 * Returns a Proposition object including the nominals propositions,
	 * with provided proposition added.
	 *
	 * @param nom The nominal with the set of relations
	 * @param prop  The proposition to be added
	 * @return Proposition The updated proposition
	 */

	public static Proposition lfNominalAddProposition (LFNominal nom, String prop) {
		Proposition result = newProposition(prop);

		if (nom.prop == null ||
			nom.prop.prop.equals(""))
		{
			return result;
		} else {
			Proposition nomProp = nom.prop;
			// ....
			return result;
		}
	}
	
	/** 
	 * Returns an Feature array including the nominal's features, with provided feature added.
	 * @param nom The nominal with the set of relations
	 * @param ft  The feature to be added
	 * @return Features[] the array with features
	 */

	public static Feature[] lfNominalAddFeature (LFNominal nom, Feature ft) {
		if (LFUtils.lfNominalGetFeature(nom, ft.feat).equals("")) {
			int newSize = 1+java.lang.reflect.Array.getLength(nom.feats);
			Feature[] fts = (Feature[]) resizeArray(nom.feats,newSize);
			fts[newSize-1] = ft;
			return fts;
		}
		else {
			return nom.feats;
		}
	} // end lfNominalAddFeature

	/** 
	 * Returns an LFRelation array including the nominal's relations, with provided relation added.
	 * @param nom The nominal with the set of relations
	 * @param rl  The relation to be added
	 * @return LFRelation[] the array with relations
	 */

	public static LFRelation[] lfNominalAddRelation (LFNominal nom, LFRelation rl) {
		if (LFUtils.lfNominalGetRelation(nom, rl.mode) == null) {
			int newSize = 1+java.lang.reflect.Array.getLength(nom.rels);
			LFRelation[] rels = (LFRelation[]) resizeArray(nom.rels,newSize);
			rels[newSize-1] = rl;
			return rels;
		}
		else {
			return nom.rels;
		}
	} // end lfNominalAddRelation
	
	/** 
	 * Returns a clone of the given LFNominal object 
	 * 
	 * @param nom	The nominal to be cloned
	 * @return LFNominal A clone of the provided nominal 
	 */

	public static LFNominal lfNominalClone (LFNominal nom) {
		LFNominal result = new LFNominal();
		result.nomVar = nom.nomVar;
		result.sort   = nom.sort;
		result.prop = LFUtils.propositionClone(nom.prop);
		result.feats = nom.feats.clone();
		result.rels = nom.rels.clone();
		return result;
	} // end lfNominalClone
	
	/** 
	 * Returns a value for the given feature in the given nominal 
	 * 
	 * @param nom	 The nominal to check
	 * @param fName The feature to check for
	 * @return String The value for the feature
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
	
	/** Returns an iterator over the features in the given nominal */

	public static Iterator lfNominalGetFeatures (LFNominal nom) { 
		Feature[] feats = nom.feats;
		return (Iterator)(new ArrayIterator(feats));
	} // end lfNominalGetRelations
	
	
	/** 
	 * Returns an LFRelation object for the given label (the first found), if any. 
	 * If none, then null is returned 
	 * 
	 * @param nom	The nominal under which the relation is stored
	 * @param rName	The label of the relation to be retrieved
	 * @return LFRelation The first found occurrance of a relation with the given label (if any)
	 */

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
	} // end lfNominalGetRelation
	
	/** 
	 * Returns an iterator over the relations in the given nominal 
	 *
	 * @param nom	The nominal under which the relations are stored
	 * @return Iterator	The iterator over LFRelation objects (from LFRelation[])
	 */

	public static Iterator lfNominalGetRelations (LFNominal nom) { 
		LFRelation[] rels = nom.rels;
		return (Iterator)(new ArrayIterator(rels));
	} // end lfNominalGetRelations
	
	
	/**
	 * Returns whether the nominal has relations
	 * @param nom The nominal to be checked
	 * @return boolean Whether the nominal has any relations stored under it
	 */
	
	public static boolean lfNominalHasRelations (LFNominal nom) { 
		return lfNominalGetRelations(nom).hasNext();

	} 
	
	/**
	 * Returns a boolean indicating whether the first logical form is included in the logical 
	 * form provided as second argument
	 * @param lf1	The first logical form, to be checked for inclusion in lf2
	 * @param lf2	The second logical form, inclusion of lf1 to be checked for
	 * @return	boolean	Whether lf1 is included in lf2
	 */
	
	public static boolean isLf1IncludedInLf2 (LogicalForm lf1, LogicalForm lf2) {
		for (int i = 0 ; i < lf1.noms.length ; i++) {
			boolean foundEquiv = false;
			LFNominal nom1 = lf1.noms[i];
			for (int j = 0 ; j < lf2.noms.length && !foundEquiv; j++) {
				LFNominal nom2 = lf2.noms[j];
				if (
						nom1.feats.length <= nom2.feats.length && 
						nom1.rels.length <= nom2.rels.length) {					
					if (!nom1.prop.prop.equals("") && 
							!nom1.prop.prop.equals(nom2.prop.prop)) {
						foundEquiv = false;
					}
					if (!nom1.sort.contains("event") && 
							!nom1.sort.contains("entity") &&
							!nom1.sort.equals(nom2.sort)) {
								foundEquiv = false;
					}
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
							EquivalentFeatures = false;
						}
					}										
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
							EquivalentRels = false;
						}
					}
				if (EquivalentFeatures && EquivalentRels) {
					foundEquiv = true;
				}
					
				}
			}
		
			if (!foundEquiv) {
				return false;
			}
		}		
		return true;
	} // end islf11includedinlf2
	
	
	/** 
	 * The method <i>convertFromString</i> converts a String representation of a logical form into 
	 * a proper LogicalForm object. The method is based on a parser, which parses a String representation
	 * of using the following strategy: 
	 * <ul>
	 * <li> Use a StringTokenizer using "(" as delimiter to break up the formula into embeddings.</li>
	 * <li> Use a StringTokenizer using "^" as delimiter to break up an embedding into conjuncts.</li>
	 * <li> Keep track of the current node using a variable <tt>nom</tt> of type <tt>LFNominal</tt>.</li>
	 * <li> Keep track of governing nodes (given embeddings) using a stack.</li>
	 * </ul>
	 * 	Breaking up into embeddings means that we first see a relation 
	 * and its governor, and only on the next pass through the
	 * "("-loop, the nominal to which the relation points (the
	 * dependent, or governed, node). To this end, we keep track of
	 * the relation in the <tt>rel</tt> variable of type <tt>LFRelation</tt>.
	 * 
	 * <h4>Warning</h4>
	 * The parser can deal with multiple at-signs in the grammar, though should be expected to do only 
	 * reliably so for up to two at-signs. <p>
	 * 
	 * @param s the string representation of the logical form
	 * @return LogicalForm the slice-based object 
	 */

	public static de.dfki.lt.tr.dialogue.slice.lf.LogicalForm convertFromString(String s) { 


		//	System.out.println("Converting from String: ["+s+"]");


		LogicalForm resultLF = newLogicalForm();

		resultLF.root = null;
		s.replaceAll("\n","");
		int fatpos = s.indexOf("@");
		int latpos = s.lastIndexOf("@");

		//	comsys.datastructs.lf.Proposition rhsStub[] = new comsys.datastructs.lf.Proposition[0];

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
									nom.prop = new de.dfki.lt.tr.dialogue.slice.lf.Proposition (proposition, ConnectiveType.NONE);
									log("brackets: "+brackets+" length:"+brackets.length());
									log("stack: "+nomstack.toString());
									// for (int i=0; i < brackets.length()-1; i++) { if (!nomstack.empty()) { nom = (LFNominal) nomstack.pop(); log("pop - prop!--"+i);} }
									for (int i=0; i < brackets.length(); i++) { if (!nomstack.empty()) { nom = (LFNominal) nomstack.pop(); log("pop - prop!--"+i);} }
								} else {
									String proposition = conj;
									proposition = strip(proposition);
									nom.prop = new de.dfki.lt.tr.dialogue.slice.lf.Proposition (proposition, ConnectiveType.NONE);
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


	/** 
	 * Returns a LogicalForm object constructed from an OpenCCG LF object. 
	 * The method fills the <i>nominals</i> field in the LogicalForm object; 
	 * it does not set the identifier for the logical form. The method creates
	 * a string representation of the OpenCCG LF, and then parses that string
	 * using the <i>convertFromString</i> method. 
	 * 
	 * @see   #convertFromString(String)
	 * @param lf the opencc logical form
	 * @return LogicalForm the slice-based object 
	 */

	public static de.dfki.lt.tr.dialogue.slice.lf.LogicalForm convertFromLF (LF lf) { 
		return convertFromString(lf.toString());
	} // end convertFromLF

	/** 
	 * Returns an OpenCCG LF object, constructed from a Logical Form. 
	 */

	public static LF convertToLF(LogicalForm input) { 
		Realizer realizer = new Realizer(null);
		XMLOutputter out = new XMLOutputter();
		Document doc;
		LF lf = null;
		try { 
			// LogicalForm to xml-File
			// Make it a proper XML document, with a
			// document type and an indication of the root
			// element.
			DocType doctype = new DocType("xml");
			Element root1 = new Element("xml");
			doc = new Document(root1,doctype);
			// basic Element "lf"
			Element lf1 = new Element("lf");
			// First, deal with the root of the LF
			LFNominal inputNom = (LFNominal)input.root;
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
			// Now the root has a nominal, type, and proposition. 
			// if there are features create and add elements in 'editFeatures()'
			// still dealing with the root at this point.
			//---------------------------------------------- 

			if (LFUtils.lfNominalGetFeatures(inputNom).hasNext()) {
				Iterator iter = LFUtils.lfNominalGetFeatures(inputNom); 
				while (iter.hasNext()) {
					de.dfki.lt.tr.dialogue.slice.lf.Feature feature = (de.dfki.lt.tr.dialogue.slice.lf.Feature) iter.next();
					sat = editFeatures(input,inputNom,feature.feat,sat);
				}
			} else { 
				//System.out.println("Nominal ["+inputNom.nomVar+"] has no features");
			} 
			// now we start some recursion down the relations of the nominal ...  
			// if there are Relations create and add elements in 'editRelations()' 
			if (LFUtils.lfNominalHasRelations(inputNom)) { 
				Iterator it = LFUtils.lfNominalGetRelations(inputNom);
				while (it.hasNext()) {
					sat = editRelations(input,(LFRelation)it.next(),sat);
				} // end while over relations
			} // end if.. check for relations
			// now we supposedly have a complete structure 
			lf1 = lf1.addContent(sat);
			root1 = doc.getRootElement().addContent(lf1);			
			Element targetElt = new Element("target");
			targetElt = targetElt.setText("*** dummy target***");
			root1 = root1.addContent(targetElt);
			// call ccg.Realizer with XML-document which extracts the contained LF and returns it  
			lf = realizer.getLfFromDoc(doc);
		} catch (Exception e) {
			log("error in LF: "+e.getMessage()); 
		} 
		// return resulting LF
		return lf;
	} 

	/** This method creates a new diamond-element representing a feature of
	 *  the given LFNominal. It appends this new Element to the given one and
	 *  returns the combined Element. It is a help-method for creating an XML-doument
	 *  that represents a given LogicalForm
	 *  
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
		
		for(Iterator<de.dfki.lt.tr.dialogue.slice.lf.Feature> featsIter = LFUtils.lfNominalGetFeatures(nom); featsIter.hasNext(); ) { 
			de.dfki.lt.tr.dialogue.slice.lf.Feature nomFeature = featsIter.next();
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
					de.dfki.lt.tr.dialogue.slice.lf.Feature feature = 
						(de.dfki.lt.tr.dialogue.slice.lf.Feature) iter.next();
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
	 * Return a string representation of the logical form. Nominals not
	 * reachable from the root are ignored.
	 *
	 * @param lf  the logical form
	 * @return string representation
	 */
	public static String lfToString(LogicalForm lf) {
		if (lf == null || lf.root == null) {
			return null;
		}
		else {
			return lfNominalToString(lf.root, lf, new Vector<String>(), true);
		}
	}

	private static String lfNominalToString(LFNominal nom, LogicalForm lf, Vector<String> occur, boolean isRoot) {
		String result = "";
		boolean needDelim = false;

		if (isRoot) {
			result += "@{" + nom.nomVar + ":" + nom.sort + "}(";
			if (occur.contains(nom.nomVar)) {
				return result;
			}
		}
		else {
			result += nom.nomVar + ":" + nom.sort;
			if (occur.contains(nom.nomVar)) {
				return result;
			}
			needDelim = true;
		}
		occur.add(nom.nomVar);

		if (!nom.prop.prop.equals("")) {
			if (needDelim) {
				result += " ^ ";
			}
			needDelim = true;
			result += nom.prop.prop;
		}

		Iterator<Feature> fit = lfNominalGetFeatures(nom);
		while (fit.hasNext()) {
			Feature f = (Feature)fit.next();
			if (needDelim) {
				result += " ^ ";
			}
			needDelim = true;
			result += "<" + f.feat + ">" + f.value;
		}

		Iterator<LFRelation> rit = lfNominalGetRelations(nom);
		while (rit.hasNext()) {
			LFRelation r = (LFRelation)rit.next();
			if (needDelim) {
				result += " ^ ";
			}
			needDelim = true;
			result += "<" + r.mode + ">(" + lfNominalToString(lfGetNominal(lf, r.dep), lf, occur, false) + ")";
		}

		if (isRoot) {
			result += ")";
		}

		return result;
	}
	
} // end class
