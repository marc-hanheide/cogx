//=================================================================
//Copyright (C) 2007-2010 Pierre Lison (plison@dfki.de)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION
package de.dfki.lt.tr.dialogue.util;


//=================================================================
//IMPORTS

// Java
import java.util.Hashtable ;
import java.util.ListIterator ;
import java.util.Vector ;
import java.util.Iterator ;
import java.util.LinkedList ;
import java.util.List ;
import java.util.Stack ;
import java.util.Set ;
import java.util.HashSet ;
import java.util.ArrayList ;
import java.util.Enumeration ;
import java.util.Arrays ;
import java.util.Iterator; 

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.asr.*;
import de.dfki.lt.tr.dialogue.slice.lf.*;
import de.dfki.lt.tr.dialogue.slice.parse.*;

// Dialogue API util
import de.dfki.lt.tr.dialogue.util.LFUtils;


/**
 * Class for building and updating *packed* logical forms, ie. compacted
 *  representation of a set of logical forms based on shared 
 *  partial representations.
 *   
 * <p>The class provides methods for <br><ul>
 * <li> packing: constructing packed logical forms out of an array of LF ;
 * <li> unpacking: retrieving a set of LF out of a packed logical form ;
 * <li> + other functionalities not yet implemented
 * </ul>
 * 
 * @author plison
 * @version 20.08.07
 *
 */
public class LFPacking {

	// loggin behavior
	public boolean logging = false; 


	// hashtable of the packing nodes
	Hashtable<String,PackingNode> PNs = 
		new Hashtable<String,PackingNode>() ;

	// hashtable of the propositions
	Hashtable <String,PackingNode> Prop2PNs = 
		new Hashtable<String,PackingNode>() ;

	// increment for the packing edge identifiers
	int peIncr ;

	// Packing Edges 
	Vector<PackingEdge> PackingEdges = new Vector<PackingEdge>();

	// Packed nominals
	Hashtable<String,PackedNominal> PackedNominals =
		new Hashtable<String,PackedNominal>();

	// Whether to merge nominals with same propositional content
	// but different ontological sort
	boolean mergeSorts = false ;

	// whether to merge when there is no proposition in the nominal
	boolean mergeSortsForEmptyProps = false ;

	// whether to rename the nominal identifiers
	boolean renameNominalIds = true ;
	
	int increment = 0 ;
	
	// counter
	static int packedLfsCounter = 0;
	
	int incr = 0;

	/**
	 * Pack an array of LF into a single packed logical form, ie.
	 * a compacted representation of the array  based on shared
	 * partial representations.
	 * 
	 * @param Lfs the array of logical forms
	 * @return the packed logical form
	 */
	public  PackedLogicalForm packLogicalForms(LogicalForm[] Lfs) {
		logging = false;
	/**
	 	for (int i=0; i < Lfs.length ; i++) {
			LFUtils.lfToGraph(Lfs[i], "graphs/testLF" +incr);
			System.out.println("LF" + incr + ": " + LFUtils.lfToString(Lfs[i]));
			incr++;
		}
	*/	
		
		PNs = new Hashtable<String,PackingNode>() ;
		Prop2PNs = new Hashtable<String,PackingNode>() ;
		PackingEdges = new Vector<PackingEdge>();
		PackedNominals = new Hashtable<String,PackedNominal>();
		
		// add logical form identifiers in case they aren't specified
		for (int i=0; i < Lfs.length ; i++) {	
			if (Lfs[i].logicalFormId == null || Lfs[i].logicalFormId.length() == 0) {
				Lfs[i].logicalFormId = "id"+increment + "-" + i;
			}
		}
		increment++;
		
		log("STEP 1: rename identifiers");

		// STEP 1 (optional): We start by renaming the identifiers
	//	if(renameNominalIds)
	//		renameNominalIds(Lfs) ;
			
		log("STEP 2: create packed logical form");

		// STEP 2: we create the packed logical form
		PackedLogicalForm plf = createPackedLogicalForm(Lfs) ;

		log("STEP 3: merge packing nodes");

		// STEP 3: We merge the packing nodes
		mergePackingNodes(plf);

		// STEP 4: We compute the preference scores
	//	computePreferenceScores(Lfs,plf);
		
		return plf ;
	}


	/**
	 * Create a packed logical form, provided the identifiers used in the
	 * array of LF are properly renamed
	 * @param Lfs the array of logical forms
	 * @return the packed LF
	 */
	private  PackedLogicalForm createPackedLogicalForm(LogicalForm[] Lfs) {

		// the packed logical form
		PackedLogicalForm plf = new PackedLogicalForm();

		log("Creating packing nodes");
		// we populate the packing nodes of the representation
		// by finding the partial shared representations
		// in each logical form of the array.
		for (int i=0; i < Lfs.length ; i++) {
			LogicalForm lf = Lfs[i];  
			createPackingNodes(lf, plf);
		}

		log("Creating packing root node");
		// We create the dummy root node and create connections to all the 
		// possible roots in the logical forms
		PackingNode root = createRootNode(Lfs);
		addPackingNode(root, plf);
		plf.root = root.pnId ;


		log("Merging sorts");
		// If the mergeSorts option is activated, perform some additional 
		// minor changes to the packed logical form
		if (mergeSorts) {
			plf = mergeSorts(plf);
		}

		// give an identifier
		plf.packedLFId = "plf" + packedLfsCounter;
		packedLfsCounter++;
		
		// and return the packed logical form
		return plf ;
	}

	/**
	 * Small method to rename packing node identifiers after a merge
	 * @param pnId
	 */
	private String changeId(String Id, Set<String> s) {
		if (!s.contains(Id)) {
			for (Iterator<String> i = s.iterator(); i.hasNext() ;) {
				String key = i.next() ;
				if (key.contains(Id.replace("_PN", ""))) {
					return key;
				}
			}
		}
		return Id;
	}

	/**
	 * Add a new packing node to the packed logical form
	 * @param pn packing node
	 * @param plf packed logical form (will hence be modified by the method!)
	 */
	private  void addPackingNode(PackingNode pn, PackedLogicalForm plf) {
		if (plf.pNodes==null) {
			plf.pNodes = new PackingNode[1];
			plf.pNodes[0] = pn ;
		}
		else {
			ArrayList<PackingNode> PNs = new ArrayList<PackingNode> 
			(Arrays.asList(plf.pNodes));
			PNs.add(pn);
			plf.pNodes = PNs.toArray(plf.pNodes) ;
		}
	}

	/**
	 * Create a new dummy packing node which will serve as the root
	 * of our packed logical form and will be linked to all the possible
	 * roots of the logical form
	 * @param Lfs the array of logical forms
	 * @return
	 */
	private  PackingNode createRootNode(LogicalForm[] Lfs)  {

		// (we store the identifier of each LF)
		Vector<String> lfIds = new Vector<String>();
		for (int i=0; i < Lfs.length ; i++) {
			lfIds.add(Lfs[i].logicalFormId) ;
		}
		

		// We create a new (dummy) packing node which will
		// be used as the root of the packed logical form
		PackingNode root = new PackingNode();
		root.pnId = "root_PN";
		root.lfIds = new String[0];
		root.lfIds = lfIds.toArray(root.lfIds) ;
		PackedNominal rootNominal = new PackedNominal() ;
		rootNominal.nomVar = "rootNom" + utteranceIncrement;
		rootNominal.prop = new Proposition("", ConnectiveType.NONE);

		rootNominal.packedSorts = new PackedOntologicalSort[1];
		rootNominal.packedSorts[0] = new PackedOntologicalSort();
		rootNominal.packedSorts[0].sort = "";
		rootNominal.packedSorts[0].lfIds = root.lfIds ;

		rootNominal.feats = new PackedFeature[0];
		rootNominal.rels = new LFRelation[0];
		root.packedNoms = new PackedNominal[1];
		root.packedNoms[0] = rootNominal ;
		root.nomsPePairs = new NominalPackingEdgePair[0];
		root.root = rootNominal.nomVar ;
		PackedNominals.put(rootNominal.nomVar, rootNominal);
		root.nomsPePairs = new NominalPackingEdgePair[1];
		root.nomsPePairs[0] = new NominalPackingEdgePair();
		root.nomsPePairs[0].head = rootNominal.nomVar ;
		PackingEdge rootPe = new PackingEdge();
		rootPe.mode = "root";
		rootPe.peId = "pe"+peIncr++;
		rootPe.coIndexedDep = false ;
		rootPe.head = rootNominal.nomVar ;
		root.nomsPePairs[0].pe = rootPe;

		// We create an packing edge from the root of the packed
		// logical form to the packing node containing the root
		// of each logical form
		// (it seems complicated, but the basic idea is pretty simple:
		// we simply create an edge between the root and each packing node
		// where there is a nominal which is a root in at leat one logical form)
		ArrayList<PackingNodeTarget> pnTargets = new ArrayList<PackingNodeTarget>();
		for (int i=0; i < Lfs.length ; i++) {
		//	if (Lfs[i].root == null || Lfs[i].root.nomVar == null)
		//		LFUtils.lfToGraph(Lfs[i], "ahaha", true);
			LogicalForm lf = Lfs[i];    
			String pnRootId = lf.root.nomVar+"_PN";
			boolean foundPackingNodeTarget = false ;
			for (Iterator<PackingNodeTarget> e= pnTargets.iterator() ;
			e.hasNext() & !foundPackingNodeTarget; ) {
				PackingNodeTarget pnt = e.next() ;
				if (pnt.pnId.equals(pnRootId)) {
					pnt.lfIds = addIdToList(pnt.lfIds, lf.logicalFormId);
					foundPackingNodeTarget = true ;
				}
			}
			if (!foundPackingNodeTarget) {
				PackingNodeTarget pnt = new PackingNodeTarget() ;
				pnt.lfIds = new String[1];
				pnt.lfIds[0] = lf.logicalFormId ;
				pnt.pnId = pnRootId ;
				pnTargets.add(pnt);
			}
		}
		rootPe.targets = new PackingNodeTarget[0];
		rootPe.targets = pnTargets.toArray(rootPe.targets) ;
		
		PackingEdge[] edges = new PackingEdge[1];
		edges[0] = rootPe;
		rootNominal.pEdges = edges;
		return root ;
	}


	/**
	 * Create/modify the packing nodes of the packed logical form by adding
	 * the information contained in the logical form
	 * 
	 * @param lf the logical form
	 * @param plf the packed logical form (which will be modified!)
	 */
	private  void createPackingNodes(LogicalForm lf, PackedLogicalForm plf) {

		// We loop on each nominal of the logical form
		for (int i= 0 ; i < lf.noms.length ; i++ ) {

			LFNominal nom = lf.noms[i];

			// If a packing node for this nominal already exists
			if (PNs.containsKey(nom.nomVar +"_PN")) {
				PackingNode pn =  PNs.get(nom.nomVar +"_PN") ;

				// We add the logical form identifier to the packing node
				pn.lfIds = addIdToList(pn.lfIds, lf.logicalFormId) ;

				boolean foundSort = false;
				for (int j = 0; j < pn.packedNoms[0].packedSorts.length; j++) {
					if (pn.packedNoms[0].packedSorts[j].sort.equals(nom.sort)) {
						// And to the ontological sorts
						pn.packedNoms[0].packedSorts[j].lfIds = 
							addIdToList(pn.packedNoms[0].packedSorts[j].lfIds, lf.logicalFormId) ;
						foundSort = true;
					}
				}
				if (!foundSort) {
						PackedOntologicalSort newpos = new PackedOntologicalSort();
						newpos.sort = nom.sort;
						newpos.lfIds = new String[1];
						newpos.lfIds[0] = lf.logicalFormId;
						ArrayList<PackedOntologicalSort> pos = new ArrayList<PackedOntologicalSort> 
						(Arrays.asList(pn.packedNoms[0].packedSorts));
						pos.add(newpos);
						pn.packedNoms[0].packedSorts = pos.toArray(pn.packedNoms[0].packedSorts);		
				}

				// NB: at this time, we know the packing node contains one 
				// and only one packed nominal

				// We add the features of the nominal to the packed nominal
				// inside the packing node
				addFeatsToPackedNominal(pn.packedNoms[0], 
						nom, lf.logicalFormId) ;	       	 

				// With the relations attached to the nominal, we create
				// a new packing edge, or modify an existing one
				addPackingEdgetoPN(pn, nom, lf.logicalFormId) ;
			}	

			else {

				// If the mergeSorts option is activated, we search
				// for a node in which to merge the nominal
				PackingNode pn = null  ;
				for (Enumeration<PackingNode> t = PNs.elements(); t.hasMoreElements() && mergeSorts ;) {
					PackingNode node = t.nextElement() ;
					if (node.pnId.contains(nom.nomVar)) {
						pn = node ;
					}
				}	
				// If we found a node in which to merge the nominal
				if (pn != null  && mergeSorts && (mergeSortsForEmptyProps | 
						nom.prop.prop.length() > 0)) {

					PNs.remove(pn.pnId);
					PackedNominals.remove(pn.packedNoms[0].nomVar);

					if (!pn.pnId.contains(nom.nomVar)) {
						pn.pnId = pn.pnId.substring(0, pn.pnId.length()-3) 
						+ "_" + nom.nomVar + "_PN";
						pn.packedNoms[0].nomVar = pn.pnId.substring(0, pn.pnId.length()-3) ;
						pn.root = pn.packedNoms[0].nomVar ;
					}

					// we add one ontological sort
					ArrayList<PackedOntologicalSort> sorts = new ArrayList<PackedOntologicalSort>(Arrays.asList(pn.packedNoms[0].packedSorts));

					boolean foundSort = false ;
					for (Iterator<PackedOntologicalSort> it=sorts.iterator(); it.hasNext() ;) {
						PackedOntologicalSort sort = it.next() ;
						if (sort.sort.equals(nom.sort)) {
							sort.lfIds = addIdToList(sort.lfIds, lf.logicalFormId);
							foundSort = true ;
						}
					}
					if (!foundSort) {
						PackedOntologicalSort newSort = new PackedOntologicalSort();
						newSort.sort = nom.sort ;
						newSort.lfIds = new String[1];
						newSort.lfIds[0] = lf.logicalFormId ;
						sorts.add(newSort);
					}

					pn.packedNoms[0].packedSorts = sorts.toArray(pn.packedNoms[0].packedSorts);

					pn.lfIds = addIdToList(pn.lfIds, lf.logicalFormId) ;

					addFeatsToPackedNominal(pn.packedNoms[0], 
							nom, lf.logicalFormId) ;	       	 

					addPackingEdgetoPN(pn, nom, lf.logicalFormId) ;

					PNs.put(pn.pnId, pn);
					PackedNominals.put(pn.packedNoms[0].nomVar, pn.packedNoms[0]) ;
				} 

				// we create a new packing node
				else {
					PackingNode newPN = createNewPN (nom, lf.logicalFormId) ;
					addPackingNode(newPN, plf) ;
					PNs.put(newPN.pnId, newPN);
				}			
			}
		}
	}


	/**
	 * Add a new logical form identifier to the identifier list
	 * @param LfIds a string array of identifier (will hence be modified!)
	 * @param newId the new Id to add
	 */
	private  String[] addIdToList(String[] LfIds, String newId) {
		if (LfIds == null) {
			LfIds = new String[1];
			LfIds[0] = newId ;
		}
		else {
			ArrayList<String> LfIdsList = new ArrayList
			<String>(Arrays.asList(LfIds));
			LfIdsList.add(newId);
			LfIds = LfIdsList.toArray(LfIds);
		}
		return LfIds ;
	}


	/**
	 * Modify the packed nominal by adding a new set of 
	 * features value pairs.
	 * 
	 * <p>For each feature-value pair to be added, two cases are possible<br><ul>:
	 * <li> if the feature AND the value are already existing in the packed
	 *   features of the nominal, then just add its LF identifier 
	 *   to the packed feature structure
	 * <li> else, create a new packed feature with the feature, its value
	 *   and the LF identifier, and add it to the packed nominal
	 * </ul>   
	 * @param packedNom the packedNominal
	 * @param features the new set of feature value pairs
	 * @param lfId logical form identifier
	 */
	private  void addFeatsToPackedNominal(PackedNominal packedNom, 
			LFNominal nom, String lfId) {

		// loop on the features
		for (int i=0; i< nom.feats.length ; i++) {
			boolean foundFeat = false ;
			for (int j=0; j<packedNom.feats.length ; j++) {
				// we found an existing feature-value pair
				if (packedNom.feats[j].feat.equals(nom.feats[i].feat) &&
						packedNom.feats[j].value.equals(nom.feats[i].value)) {
					packedNom.feats[j].lfIds = addIdToList(packedNom.feats[j].lfIds, lfId);
					foundFeat = true ;
				}
			}

			// if not, we add one
			if (!foundFeat) {
				PackedFeature pf = new PackedFeature();
				pf.feat = nom.feats[i].feat ;
				pf.value = nom.feats[i].value ;
				pf.lfIds = new String[1];
				pf.lfIds[0] = lfId ;
				ArrayList<PackedFeature> pfs = new ArrayList<PackedFeature> 
				(Arrays.asList(packedNom.feats));
				pfs.add(pf);
				packedNom.feats = pfs.toArray(packedNom.feats);

			}
		}
	}



	/**
	 * Add a new packing edge attached to the packing node, given
	 * the information provided by the LF Nominal
	 * 
	 * @param PN the packing node to which the edge is attached
	 * @param nom the LF nominal
	 * @param lfId the logical form identifier
	 */
	private  void addPackingEdgetoPN(PackingNode PN, 
			LFNominal nom, String lfId ) {

		LFRelation[] newRels = nom.rels ;

		// We loop on the relations
		for (int i=0; i< newRels.length ; i++) {
			boolean foundEdge1 = false ;
			for (int z=0; z < PN.nomsPePairs.length ; z++) {

				PackingEdge pe = PN.nomsPePairs[z].pe ;
				// if there is already a packing edge with the same head and mode
				if (pe.head.equals(newRels[i].head) &&
						pe.mode.equals(newRels[i].mode)) {

					// We now check if there is one of the edge targets (ie a packing
					// node) which refers to the same object as the dependent of the
					// relation
					boolean foundEdge = false ;
					for (int j=0 ; j < pe.targets.length ; j++) {
						if (!foundEdge && pe.targets[j].pnId.equals(newRels[i].dep+"_PN")) { 
							foundEdge = true ;
							// Cool, we now just have to add the identifier
							pe.targets[j].lfIds = addIdToList(pe.targets[j].lfIds, lfId);
						}
					}
					// else, we have to add a new target
					if (!foundEdge) {
						PackingNodeTarget newPnt = new PackingNodeTarget();
						newPnt.pnId = newRels[i].dep+"_PN" ;
						newPnt.lfIds = new String[1];
						newPnt.lfIds[0] = lfId ;
						if (pe.targets == null) {
							pe.targets = new PackingNodeTarget[1];
							pe.targets[0] = newPnt ;
						}
						else {
							ArrayList<PackingNodeTarget> pnts = new ArrayList
							<PackingNodeTarget>(Arrays.asList(pe.targets));
							pnts.add(newPnt);
							pe.targets = pnts.toArray(pe.targets);
						}
					}
					foundEdge1 = true;
				}
			}

			// If no packing edge is found, we create a new one
			if (!foundEdge1)  {
				PackingEdge pe = new PackingEdge();
				pe.head = newRels[i].head ;
				pe.mode = newRels[i].mode ;
				pe.coIndexedDep = newRels[i].coIndexedDep ;
				pe.targets = new PackingNodeTarget[1];
				pe.targets[0] = new PackingNodeTarget();
				
				pe.targets[0].pnId = newRels[i].dep+"_PN";
				pe.targets[0].lfIds = new String[1];
				pe.targets[0].lfIds[0] = lfId ;
				pe.peId = "pe"+peIncr++;
				NominalPackingEdgePair newNpep = new NominalPackingEdgePair();
				newNpep.head = pe.head ;
				newNpep.pe = pe ;
				if (PN.nomsPePairs == null) {
					PN.nomsPePairs = new NominalPackingEdgePair[1];
					PN.nomsPePairs[0] = newNpep ;
				}
				else {
					ArrayList<NominalPackingEdgePair> npeps = new ArrayList
					<NominalPackingEdgePair> (Arrays.asList(PN.nomsPePairs));
					npeps.add(newNpep);
					PN.nomsPePairs = npeps.toArray(PN.nomsPePairs) ;
				}
				PackingEdges.add(pe);
				
				// we also add a reference to the packing edge in the packed nominal
				PackedNominal pnom = LFUtils.plfGetPackedNominal(PN, newRels[i].head);
				if (pnom != null) {
					if (pnom.pEdges == null) {
						pnom.pEdges = new PackingEdge[1];
						pnom.pEdges[0] = pe ;
					}
					else {
						int oldLength = pnom.pEdges.length;
						pnom.pEdges= (PackingEdge[]) LFUtils.resizeArray(pnom.pEdges, oldLength + 1);
						pnom.pEdges[oldLength] = pe;
					}
				}
				else {
					log("WARNING: problem adding a packing edge referent to the packed nominal");
				}
				
			}
		}
	}

	/**
	 * Create a new Packing node based on the LF nominal
	 * @param nom the nominal
	 * @param lfId the logical form identifier
	 * @return the packing node
	 */
	private  PackingNode createNewPN(LFNominal nom, String lfId) {
		PackingNode pn = new PackingNode();

		// We add basic information 
		
		pn.pnId = nom.nomVar + "_PN";
		pn.lfIds = addIdToList(pn.lfIds, lfId);
		pn.root = nom.nomVar ;
		pn.nomsPePairs = new NominalPackingEdgePair[0];

		// and the nominal, of course 
		pn.packedNoms = new PackedNominal[1];
		pn.packedNoms[0] = new PackedNominal();
		pn.packedNoms[0].feats = new PackedFeature[0];
		pn.packedNoms[0].nomVar = nom.nomVar ;
		addFeatsToPackedNominal(pn.packedNoms[0], nom, lfId);
		pn.packedNoms[0].prop = nom.prop ;

		// ontological sorts
		pn.packedNoms[0].packedSorts = new PackedOntologicalSort[1];
		pn.packedNoms[0].packedSorts[0] = new PackedOntologicalSort();
		pn.packedNoms[0].packedSorts[0].sort = nom.sort ;
		pn.packedNoms[0].packedSorts[0].lfIds = new String[1];
		pn.packedNoms[0].packedSorts[0].lfIds[0] = lfId ;

		pn.packedNoms[0].rels = new LFRelation[0];
		PackedNominals.put(pn.packedNoms[0].nomVar, pn.packedNoms[0]);

		// add the packing edges 
		addPackingEdgetoPN(pn, nom, lfId);

		return pn ;
	}

	/**
	 * Rename all nominal variable identifiers so as to guarantee the unicity
	 * of the nominal variable identifiers accross all the logical forms of the
	 * array of LF.
	 * 
	 * <p>(We indeed noticed that an identifier could in some cases refer to 
	 * different propositions in distinct logical forms.)
	 * 
	 * <p>Practically, the algorithm determines a new identifier for every 
	 * <proposition,ontological sort> pair, and subsequently changes all the
	 * identifiers used in the logical forms to use these new Ids.
	 * 
	 */
	public  void renameNominalIds(LogicalForm[] Lfs) {

		Hashtable<Integer,String> prop2nomId = new Hashtable<Integer,String>() ;

		Hashtable<String,String> mapOld2NewId ;

		// Iterate on the array of LF
		for (int i=0; i < Lfs.length ; i++) {
			LogicalForm lf = Lfs[i];    	   

			// First step: find the Ids than need to be changed, and store
			// these changes in the hashtable mapOld2NewId
			mapOld2NewId  = new Hashtable<String,String>() ;

			// Iterate on the nominals

			for (int j=0 ; j < lf.noms.length ; j++ ) {

				String[] propAndSort = new String[2] ;
				if (!lf.noms[j].prop.prop.equals("") && 
						!lf.noms[j].prop.prop.equals("elem")) {
					propAndSort[0] = lf.noms[j].prop.prop;
				}
				else {
					propAndSort[0] = lf.noms[j].nomVar ;
				}
				propAndSort[1] = lf.noms[j].sort ;

				// We create an hashCode for the pair <prop,sort>
				Integer hash;
				hash = new Integer(Arrays.hashCode(propAndSort)) ;

				String oldId =  lf.noms[j].nomVar ;
				String newId ;
				if (!prop2nomId.containsKey(hash)) {
					if (prop2nomId.containsValue(oldId)) {
						// we change the name
						newId = getNewName(prop2nomId, oldId) ; 
					}
					else {
						// or we keep the old one
						newId = oldId ;
					}
					// and we store the value
					prop2nomId.put(hash, newId) ;
				}
				else {
					newId = (String) prop2nomId.get(hash) ;
				}
				mapOld2NewId.put(oldId, newId) ;
				//	log("avant: " + LFUtils.lfToString(lf));
				lf.noms[j].nomVar = newId ;

			}	
			// Second step: change the Ids in the head and dep variables in 
			// all the relations
			for (int l=0 ; l < lf.noms.length ; l++ ) { 
				for (int k=0 ; k < lf.noms[l].rels.length ; k++ ) {			
					lf.noms[l].rels[k].head = mapOld2NewId.get(lf.noms[l].rels[k].head) ;
					lf.noms[l].rels[k].dep = mapOld2NewId.get(lf.noms[l].rels[k].dep) ;
				}
			}
		}
	}


/**
 * Based on the hashtable containing the mapping between <prop,sort> pairs
 * and nominal identifiers, return a new name of the identifier
 */
private  String getNewName (Hashtable prop2nomId, String oldName) {
	int incr=1;
	String newName = oldName.substring(0,oldName.length()-1)+(new Integer(incr)).toString() ;
	while (prop2nomId.containsValue(newName)) {
		incr++ ;
		newName = oldName.substring(0,oldName.length()-1)+(new Integer(incr)).toString() ;
	}
	return newName ;
}



/**
 * Perform some minor modifications to the packed logical form
 * to accomate node fusion in case of the option mergeSorts 
 * (ie the merge of nodes with same propositional content but 
 * different ontological sorts) is activated;
 * 
 * @param plf packed logical form
 * @return the packed logical form
 */
private PackedLogicalForm mergeSorts(PackedLogicalForm plf) {
	log("Merging of ontological sorts");
	for (int z=0 ; z < plf.pNodes.length ; z++) {
		for (int y=0; y < plf.pNodes[z].nomsPePairs.length ; y++) {
			plf.pNodes[z].nomsPePairs[y].head =
				changeId(plf.pNodes[z].nomsPePairs[y].head, PackedNominals.keySet()) ;
			plf.pNodes[z].nomsPePairs[y].pe.head =
				changeId(plf.pNodes[z].nomsPePairs[y].pe.head, PackedNominals.keySet()) ;
			for (int x=0 ; x < plf.pNodes[z].nomsPePairs[y].pe.targets.length ; x++) {
				plf.pNodes[z].nomsPePairs[y].pe.targets[x].pnId =
					changeId(plf.pNodes[z].nomsPePairs[y].pe.targets[x].pnId, PNs.keySet());
			}
		}

		boolean mergeOutEdges = true ;
		while (mergeOutEdges) {
			mergeOutEdges = false ;
			for (int u=0 ; u < plf.pNodes[z].nomsPePairs.length ; u++) {
				for (int v=u+1 ; v < plf.pNodes[z].nomsPePairs.length ; v++) {
					if (!mergeOutEdges && plf.pNodes[z].nomsPePairs[u].pe.mode.equals(plf.pNodes[z].nomsPePairs[v].pe.mode)) {

						ArrayList<PackingNodeTarget> targets = 
							new ArrayList<PackingNodeTarget>
						(Arrays.asList(plf.pNodes[z].nomsPePairs[v].pe.targets));

						for (int t=0 ; t < plf.pNodes[z].nomsPePairs[u].pe.targets.length ; t++) {
							PackingNodeTarget pnt= plf.pNodes[z].nomsPePairs[u].pe.targets[t];
							boolean foundPNT = false ;
							for (int s=0 ; !foundPNT && s < plf.pNodes[z].nomsPePairs[v].pe.targets.length ; s++) {
								PackingNodeTarget pnt2 = plf.pNodes[z].nomsPePairs[v].pe.targets[s];
								if (pnt.pnId.equals(pnt2.pnId)) {
									for (int r=0 ; r < pnt.lfIds.length ; r++) {
										pnt2.lfIds = addIdToList(pnt2.lfIds,pnt.lfIds[r]);
									}
									foundPNT = true ;
								}
							}
							if (!foundPNT) {
								targets.add(pnt);
							}
						}

						plf.pNodes[z].nomsPePairs[v].pe.targets
						= targets.toArray(plf.pNodes[z].nomsPePairs[v].pe.targets) ;
						ArrayList<NominalPackingEdgePair> npeps = 
							new ArrayList<NominalPackingEdgePair>
						(Arrays.asList(plf.pNodes[z].nomsPePairs));
						npeps.remove(plf.pNodes[z].nomsPePairs[u]);
						plf.pNodes[z].nomsPePairs = 
							npeps.toArray(new NominalPackingEdgePair[npeps.size()]);

						mergeOutEdges = true ;
					}
				}
				for (int p=0; p<plf.pNodes[z].nomsPePairs[u].pe.targets.length ; p++) {
					for (int o=p+1; o<plf.pNodes[z].nomsPePairs[u].pe.targets.length ; o++) {
						PackingNodeTarget pnt1 = plf.pNodes[z].nomsPePairs[u].pe.targets[p];
						PackingNodeTarget pnt2 = plf.pNodes[z].nomsPePairs[u].pe.targets[o];
						if (pnt1.pnId.equals(pnt2.pnId)) {
							for (int n = 0 ; n < pnt1.lfIds.length ; n++) {
								pnt2.lfIds = addIdToList(pnt2.lfIds, pnt1.lfIds[n]);
							}
							ArrayList<PackingNodeTarget> targets = 
								new ArrayList<PackingNodeTarget>
							(Arrays.asList(plf.pNodes[z].nomsPePairs[u].pe.targets));
							targets.remove(pnt1);
							plf.pNodes[z].nomsPePairs[u].pe.targets = targets.toArray(new PackingNodeTarget[targets.size()]);
							mergeOutEdges = true ;
						}	
					}
				}
			}
		}
	}
	return plf ;
}


/**
 * Merge the packing nodes of the representation, ie. if we find
 * that two packing nodes have exactly the same set of LF identifiers,
 * we merge them into a single packing form
 */
private  void mergePackingNodes (PackedLogicalForm plf) {

	PackedLogicalForm plfInit = LFUtils.plfClone(plf);
	// We iterate several times till to apply all 
	// possible node mergings
	int lastNbrPNs = 0 ;
	while (lastNbrPNs != plf.pNodes.length) {

		lastNbrPNs = plf.pNodes.length ;

		// Loop on the packing nodes
		for (int i=0 ; i < plf.pNodes.length ; i++) {
			PackingNode PN1 = plf.pNodes[i] ;
			Set<String> LfIds1 = new HashSet<String> (Arrays.asList(PN1.lfIds)) ; 
			// loop on the packing edges attached to the PN
			for (int j=0; PN1.nomsPePairs.length > 0 && j < PN1.nomsPePairs.length ; j++) {
				
				PackingEdge pe = PN1.nomsPePairs[j].pe ;
				// loop on the targets for the packing edges
				
				log("Looping on the "+PN1.nomsPePairs.length+" targets for the packing edges from "+PN1.pnId);
				
				Iterator<PackingNodeTarget> it = Arrays.asList(pe.targets).iterator();
				while (PN1.nomsPePairs.length > 0 && it.hasNext()) {
					PackingNodeTarget target = it.next();
					String PN2Id = target.pnId ;
					Set<String> LfIds2 = new HashSet<String> (Arrays.asList(target.lfIds)) ;

					PackingNode PN2 = PNs.get(PN2Id);
					
					if (PN2==null) {
						log("PROBLEM: Node "+ PN2Id + " not found in the packing nodes hashtable");
						log(PNs.keySet().toString());
					}
					try { 
					Set<String> LfIds3 = new HashSet<String> (Arrays.asList(PN2.lfIds)) ;
					// If PN1 always dominates PN2
					if (LfIds1.equals(LfIds2) && (LfIds1.equals(LfIds3))) {
						// we must verify the absence of cycle 
						// between the two nodes
						boolean hasCycle = false ;
						for (int o=0 ; o < PN2.nomsPePairs.length ; o++) {
							for (int p=0 ; p < PN2.nomsPePairs[o].pe.targets.length ; p++ ) {
								if (PN2.nomsPePairs[o].pe.targets[p].pnId.equals(PN1.pnId)) {
									log("Cycle found between two packing nodes");
									hasCycle = true ;
								}
							}
						}

						log("Verifying packing edges");
						boolean additionalVerificationOK = true;
						// If our design is correct, we shouldn't have any other packing
						// edge pointing to PN2.  Nevertheless, we verify whether this
						// is indeed the case
						for (Enumeration<PackingEdge> e = PackingEdges.elements(); e.hasMoreElements() ;) {
							PackingEdge peTest = e.nextElement() ;
							if (!peTest.equals(pe)) {
								for (int m = 0; m < peTest.targets.length ; m++) {
									if (peTest.targets[m].pnId.equals(PN2.pnId)) {
									//	Set<String> LfIds4 = new HashSet<String> (Arrays.asList(peTest.targets[m].lfIds)) ;
									//	if (!LfIds4.equals(LfIds1)) {
											log ("Warning: there is another edge pointing to the packing node we want to delete");
											additionalVerificationOK = false;
									//	}
									}
								}
							}
						}
						
						
						if (!hasCycle && additionalVerificationOK && pe.targets.length == 1) {
							log ("Found 2 packing nodes to merge: " + PN1.pnId + " and " + PN2.pnId); 

							log("Creating internal relation");
							// First, we create an internal relation between the 
							// two packed nominals
							PackedNominal Nominal1 = PackedNominals.get(PN1.nomsPePairs[j].head);	
							PackedNominal Nominal2 = PackedNominals.get(PN2.root) ;
							log("Create relation");
							LFRelation newRelation = new LFRelation();
							newRelation.head = Nominal1.nomVar ;
							newRelation.dep = Nominal2.nomVar ;
							newRelation.mode = pe.mode ;
							newRelation.coIndexedDep = pe.coIndexedDep ; 
	
							if (Nominal1.rels==null) {
								Nominal1.rels = new LFRelation[1];
								Nominal1.rels[0]= newRelation ;
							}
							else {
								ArrayList<LFRelation> rels = new ArrayList<LFRelation>(Arrays.asList(Nominal1.rels));
								rels.add(newRelation);
								Nominal1.rels = rels.toArray(Nominal1.rels);
							}

							log("Adding nominal to the list of nominals");
							// We add the nominal to the list of nominals
							ArrayList<PackedNominal> packedNoms = new ArrayList<PackedNominal>(Arrays.asList(PN1.packedNoms));
							packedNoms.addAll(Arrays.asList(PN2.packedNoms));
							PN1.packedNoms = packedNoms.toArray(PN1.packedNoms) ;

							log("Adding packing edges");
							// We also have to add all the packing edges connected to PN2
							ArrayList<NominalPackingEdgePair> pairs = new ArrayList<NominalPackingEdgePair>(Arrays.asList(PN1.nomsPePairs));
							pairs.addAll(Arrays.asList(PN2.nomsPePairs));
							//	PN1.nomsPePairs = new NominalPackingEdgePair[PN1.nomsPePairs.length + PN2.nomsPePairs.length];
							PN1.nomsPePairs = pairs.toArray(PN1.nomsPePairs);

							log("Suppressing packing edge");
						
							// suppression of the reference to the packing edge in the nominal
							if (Nominal1.pEdges != null && Nominal1.pEdges.length > 0) {
								ArrayList<PackingEdge> edges = new ArrayList<PackingEdge>(Arrays.asList(Nominal1.pEdges));
								edges.remove(PN1.nomsPePairs[j].pe);
								Nominal1.pEdges = (PackingEdge[]) LFUtils.resizeArray(Nominal1.pEdges, Nominal1.pEdges.length - 1);
								Nominal1.pEdges = edges.toArray(Nominal1.pEdges);
							}

							// Suppression of the packing edge itself
							if (pe.targets.length == 1) {
							ArrayList<NominalPackingEdgePair> pairs2 =
								new ArrayList<NominalPackingEdgePair>(Arrays.asList(PN1.nomsPePairs));
							pairs2.remove(PN1.nomsPePairs[j]);
							PN1.nomsPePairs = new NominalPackingEdgePair[PN1.nomsPePairs.length -1] ;
							PN1.nomsPePairs = pairs2.toArray(PN1.nomsPePairs);
							}
							else if (pe.targets.length > 1){
								ArrayList<PackingNodeTarget> pnt = new ArrayList<PackingNodeTarget>(Arrays.asList(pe.targets));
								pnt.remove(target);
								int newSize = pe.targets.length - 1;
								pe.targets = (PackingNodeTarget[]) LFUtils.resizeArray(pe.targets, newSize);
								pe.targets = pnt.toArray(pe.targets);								
							}
							
					
							log("Suppressing packing node");
							// Suppression of the packing node
							Vector<PackingNode> pns =
								new Vector<PackingNode>(Arrays.asList(plf.pNodes));
							pns.remove(PN2);
							plf.pNodes = new PackingNode[plf.pNodes.length-1];
							plf.pNodes = pns.toArray(plf.pNodes);

						}
					}
				} catch (NullPointerException npe) { 
					System.err.println("[LFPacking] temporary problem in packing (merging nodes): "+npe.getMessage());
				} 
				}
			}
		}
	}
}


/**
* Unpack a packed logical form using the accessibility paths
* specified in the representation
* 
* @param plf the packed logical form
* @return a set of logical forms
*/
public LogicalForm[] unpackPackedLogicalForm(PackedLFs plf) {
	LogicalForm[] Lfs = unpackPackedLogicalForm(plf.packedLF);
	for (int i=0; i < Lfs.length ; i++) {
		Lfs[i].stringPos = plf.stringPos;
	}
	return Lfs;
}


/**
 * Unpack a packed logical form using the accessibility paths
 * specified in the representation
 * 
 * @param plf the packed logical form
 * @return a set of logical forms
 */
public LogicalForm[] unpackPackedLogicalForm(PackedLogicalForm plf) {
	
	log ("UNPACKING: unpacking of logical forms...");
	Vector<LogicalForm> LogicalForms = new Vector<LogicalForm>();

	PNs = new Hashtable<String, PackingNode>();
	for (int i=0; plf != null && i< plf.pNodes.length ; i++) {
		PNs.put(plf.pNodes[i].pnId, plf.pNodes[i]);
	}
	
	PackingNode RootPN = PNs.get(plf.root);

	// the set of LF id at the root node characterizes the full
	// set of possible Ids
	String[] LfIds = RootPN.lfIds ;

	// iterate on the LF identifiers
	for (int i=0; i< LfIds.length ; i++) {
		LogicalForm lf = extractLogicalForm(plf, LfIds[i]);
		LogicalForms.add(lf);
	}

	LogicalForm[] logicalFormsArray = new LogicalForm[LogicalForms.size()]; 
	logicalFormsArray = LogicalForms.toArray(logicalFormsArray);
	
	return logicalFormsArray;
}


/**
 * Extract a specific logical form from the PLF
 * @param lf
 * @return
 */
public LogicalForm extractLogicalForm(PackedLogicalForm plf, String lfId) {
	
	log("handling LF identifier " + lfId);
	
	PNs = new Hashtable<String, PackingNode>();
	for (int i=0; i< plf.pNodes.length ; i++) {
		PNs.put(plf.pNodes[i].pnId, plf.pNodes[i]);
	}
	
	// create a new logical form
	LogicalForm lf = new LogicalForm();
	lf.logicalFormId = lfId;

	// hashtable containing all the nominals, 
	// using their identifiers as keys
	Hashtable<String,LFNominal> AccessibleNoms = 
		new Hashtable<String,LFNominal> ();

	// pref score
	Vector<Float> prefScore = new Vector<Float>();
	prefScore.add((PNs.get(plf.root).preferenceScore)) ;

	// get all accessible nominals from the root, 
	// given a specific LF identifier
	AccessibleNoms = getAccessibleNominals 
	(plf.root, lfId, AccessibleNoms, prefScore);

	String rootNom = "";
	for (Enumeration<String> e = AccessibleNoms.keys(); e.hasMoreElements();) {
		String nomvar = e.nextElement();
		if (nomvar.contains("rootNom")) {
			rootNom = nomvar;
		}
	}
	
	log("rootNom: " + rootNom);
	AccessibleNoms.remove(rootNom);
	
	log("number of accessible nominals found:" + AccessibleNoms.size());

	LFNominal[] nominals = new LFNominal[AccessibleNoms.size()] ;
	int incr = 0 ;
	for (Enumeration<LFNominal> e = AccessibleNoms.elements() ; 
	e.hasMoreElements() ;) {
		nominals[incr] = e.nextElement() ;
		log("nominal " + incr + ": " + nominals[incr].nomVar);
		incr++ ;
	}

	// search for the root node
	PackingNode root = PNs.get(plf.root) ;
	boolean foundRoot = false ;
	
	for (int p=0 ; !foundRoot && p < root.nomsPePairs.length ; p++) {
		for (int q =0 ; !foundRoot && root.nomsPePairs[p].head.equals(rootNom) && q < root.nomsPePairs[p].pe.targets.length ; q++) {
			for (int r=0 ;!foundRoot &&  r < root.nomsPePairs[p].pe.targets[q].lfIds.length ; r++) {
				if (root.nomsPePairs[p].pe.targets[q].lfIds[r].equals(lf.logicalFormId)) {
					String pnId = root.nomsPePairs[p].pe.targets[q].pnId;
					lf.root = AccessibleNoms.get(pnId.substring(0,pnId.length()-3)) ;
					log("found root for LF - algo 1");
					foundRoot = true ;
				}
			}
		}
	} 
	for (int p=0 ; !foundRoot && p < root.packedNoms.length ; p++) {
		if (!foundRoot && root.packedNoms[p].nomVar.equals(rootNom)) {
			for (int q = 0 ; !foundRoot && q < root.packedNoms[p].rels.length ; q++ ) {
				if (!foundRoot && root.packedNoms[p].rels[q].mode.equals("root")) {
					lf.root = AccessibleNoms.get(root.packedNoms[p].rels[q].dep) ;
					log("found root for LF - algo 2");
					foundRoot = true ;
				}
			}
		}
	}
	if (!foundRoot) {
		System.out.println(">>>>>>>WARNING: no root found!");
	}

	lf.preferenceScore = prefScore.elementAt(0).floatValue() ;
	lf.noms = nominals;
	
	return lf;
}


/**
 * Return a set of LFNominals accessible from a given packing node, using
 * a specific logical form identifier
 * 
 * @param PNId the packing node identifier
 * @param LfId the LF identifier
 * @param AccessibleNoms hashtable containing all the accessible nominals 
 *                       already found
 * @return an array of LFNominals
 */
private  Hashtable<String,LFNominal> getAccessibleNominals (String PNId, String LfId, 
		Hashtable<String,LFNominal> AccessibleNoms, 
		Vector<Float> prefScore) {
	try {
		PackingNode pn = PNs.get(PNId);
		for (int i=0; i <pn.packedNoms.length ; i++) {

			// if no LFNominal related to specific packed nominal exists in 
			// the hashtable, we create a new one and insert it in the hashtable
			if (!AccessibleNoms.containsKey(pn.packedNoms[i].nomVar) ) {
				LFNominal LfNom = new LFNominal() ;
				LfNom.prop = pn.packedNoms[i].prop ;
				LfNom.sort = getUnpackedSort(pn.packedNoms[i].packedSorts, LfId);
				LfNom.nomVar = pn.packedNoms[i].nomVar ;
				LfNom.feats = getUnpackedFeatures(pn.packedNoms[i].feats, LfId);
				LfNom.rels = new LFRelation[pn.packedNoms[i].rels.length];
				
				for (int j = 0; j < pn.packedNoms[i].rels.length ; j++) {
					LfNom.rels[j] = new LFRelation();
					LfNom.rels[j].coIndexedDep = pn.packedNoms[i].rels[j].coIndexedDep;
					LfNom.rels[j].dep = new String(pn.packedNoms[i].rels[j].dep);
					LfNom.rels[j].head = new String(pn.packedNoms[i].rels[j].head);
					LfNom.rels[j].mode = new String(pn.packedNoms[i].rels[j].mode);
				}
				
				AccessibleNoms.put(LfNom.nomVar, LfNom);
			}
		}

		// We iterate on the <nominal, packing edge> pairs
		for (int i=0; i< pn.nomsPePairs.length ; i++) {
			PackingEdge pe = pn.nomsPePairs[i].pe ;

			// We iterate on the packing edge targets
			for (int j=0 ; j< pe.targets.length ; j++) {
				Set<String> lfIds1 = new HashSet<String>(Arrays.asList(pe.targets[j].lfIds));
				Set<String> lfIds2 = new HashSet<String>(Arrays.asList(pn.lfIds));
				if (PNs.get(pe.targets[j].pnId) != null) {
					Set<String> lfIds3 = new HashSet<String>(Arrays.asList(PNs.get(pe.targets[j].pnId).lfIds));
					if (lfIds1.contains(LfId) && lfIds2.contains(LfId) && lfIds3.contains(LfId)) {
						PackingNode pn2 = PNs.get(pe.targets[j].pnId) ;
						if (!AccessibleNoms.containsKey(pn2.root)) {

							// update the preference score of the logical form by multiplying
							// the current preference score by the one of the new packing node
							// and the packing edge related to it
							// (in other words, the preference score in a LF is defined as the product
							// of the preference scores of all the packing nodes and packing edges)
							Float oldScore = prefScore.firstElement() ;
							Float newScore = oldScore * pe.preferenceScore * pn2.preferenceScore ;
							prefScore.insertElementAt(newScore,0);

							Hashtable<String,LFNominal> AccessibleNoms2 = getAccessibleNominals (pe.targets[j].pnId, LfId, AccessibleNoms, prefScore);
							for (Enumeration<LFNominal> e = AccessibleNoms2.elements() ; e.hasMoreElements() ;) {
								LFNominal nom = e.nextElement() ;
								if (!AccessibleNoms.containsKey(nom.nomVar)) {
									AccessibleNoms.put(nom.nomVar, nom);
								}
							}
						}		
						// We also have to add a relation instead of the packing edge
						LFNominal NomToExtend = AccessibleNoms.get(pn.nomsPePairs[i].head) ;

						LFRelation rel = new LFRelation();
						rel.head = NomToExtend.nomVar ;
						rel.mode = pe.mode ;
						rel.coIndexedDep = pe.coIndexedDep ;
						rel.dep = (PNs.get(pe.targets[j].pnId)).root ;
						if (NomToExtend.rels == null) {
							NomToExtend.rels = new LFRelation[1];
							NomToExtend.rels[0] = rel ;
						}
						else {
							ArrayList<LFRelation> Rels = new ArrayList<LFRelation>(Arrays.asList(NomToExtend.rels));
							Rels.add(rel);
							NomToExtend.rels = Rels.toArray(NomToExtend.rels);
						}
					}
				}		}
		}

		return AccessibleNoms ;
	}
	catch (NullPointerException e) {
		e.printStackTrace();
		return null ;
	}
}


/**
 * Return an array with all the features accessible 
 * from a specific LF identifier
 * 
 * @param packedFeats the packed features
 * @param LfId the LF identifier
 * @return the array of features
 */
private Feature[] getUnpackedFeatures (PackedFeature[] packedFeats, String LfId) {
	Vector<Feature> Features = new Vector<Feature>();
	for (int i=0; i < packedFeats.length ; i++) {
		Set<String> lfIds = new HashSet<String>(Arrays.asList(packedFeats[i].lfIds));
		if (lfIds.contains(LfId)) {
			Feature feat = new Feature();
			feat.feat = packedFeats[i].feat ;
			feat.value = packedFeats[i].value ;
			Features.add(feat);
		}
	}
	Feature[] Features2 = new Feature[Features.size()];
	Features2 = Features.toArray(Features2);
	return Features2 ;
}

/**
 * Return the ontological sort, given an array of packed ontological sorts,
 * and a LF identifier
 * @param packedSorts the packed ontological sorts
 * @param LfId the LF identifier
 * @return a string with the ontological sort
 */
private String getUnpackedSort(PackedOntologicalSort[] packedSorts, String LfId) {

	for (int i=0; i < packedSorts.length ; i++) {
		ArrayList<String> lfIds = new ArrayList<String>
		(Arrays.asList(packedSorts[i].lfIds));
		if (lfIds.contains(LfId)) {
			return packedSorts[i].sort ;
		}
	}

	// If we haven't found any sort, there is a problem
	log("ERROR: ontological sort not found");
	return null;
}
/**
 *  Get a string version of a string array
 */
private  String getStringOfArray(String[] strArray) {
	Vector<String> v = new Vector<String>(Arrays.asList(strArray));
	return v.toString() ;
}

/**
 * Compute the preference scores for each packing node and each packing edge,
 * based on the scores given in the array of LF.
 * 
 * @param lfcoll the array of LF
 * @param plf the packed logical form (will be modified!)
 * @return the packed logical form with preference scores
 */
private PackedLogicalForm computePreferenceScores (LogicalForm[] Lfs, PackedLogicalForm plf) {

	// build an hashtable with the preference score for each logical form
	Hashtable<String,Float> Prefs = new Hashtable<String,Float>();
	for (int i=0 ; i< Lfs.length ; i++) {
		String lfId = Lfs[i].logicalFormId ;
		Float pref = new Float(Lfs[i].preferenceScore);
		Prefs.put(lfId, pref);
	}

	// Iterate on the packing nodes
	for (int i=0; i < plf.pNodes.length ; i++) {
		PackingNode pn = plf.pNodes[i] ;
		float pref = 0.0f ;
		for (int j=0; j< pn.lfIds.length ; j++) {
			pref += Prefs.get(pn.lfIds[j]).floatValue() ;
		}
		pn.preferenceScore = (pref/Lfs.length);

		// and on the outgoing edges
		for (int j=0 ; j< pn.nomsPePairs.length ; j++) {
			PackingEdge pe = pn.nomsPePairs[j].pe ;
			pref = 0.0f ;
			for (int k = 0 ; k < pe.targets.length ; k++) {
				for (int l=0 ; l < pe.targets[k].lfIds.length ; l++) {
					pref += Prefs.get(pe.targets[k].lfIds[l]) ;
				}
			}
			pe.preferenceScore = (pref/Lfs.length);
		}
	}
	return plf ;
}


/**
 * Prune an array of logical forms according to a threshold on the
 * preference scores
 * @param Lfs
 * @param threshold
 */
static public LogicalForm[] pruneLogicalForm(LogicalForm[] Lfs, float threshold) {
	ArrayList<LogicalForm> newLfs = new ArrayList<LogicalForm>();
	for (int i=0; i< Lfs.length ; i++) {
		if (Lfs[i].preferenceScore >= threshold) {
			newLfs.add(Lfs[i]);
		}
	}
	LogicalForm[] result = new LogicalForm[newLfs.size()];
	result = newLfs.toArray(result) ;
	return result ;
}
/**
 * Logging function
 * @param str string to be shown
 */
 private  void log (String str) {
	if (logging) { 
		System.out.println("[LOG \"packing\"] "+str) ;
	}
}
 
 
 
 // utterance increment (used for labelling the nominal variable identifiers)
 private static int utteranceIncrement = 0;
 
 public void setUtteranceIncrement(int utteranceIncr) {
 	utteranceIncrement = utteranceIncr ;
 }
 
 public void incrementUttNumber() {
	 utteranceIncrement++;
 }
 
 
} // end class
