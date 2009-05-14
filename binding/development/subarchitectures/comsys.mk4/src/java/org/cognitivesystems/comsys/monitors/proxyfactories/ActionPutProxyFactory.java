//=================================================================
// Copyright (C) 2008 Geert-Jan M. Kruijff (gj@dfki.de)
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

package org.cognitivesystems.comsys.monitors.proxyfactories;

//=================================================================
// IMPORTS
//=================================================================

// ----------------------------------------------------------------
// BINDING imports
// ----------------------------------------------------------------
import binding.common.BindingComponentException;

// ----------------------------------------------------------------
// CAST imports
// ----------------------------------------------------------------
import cast.architecture.subarchitecture.SubarchitectureProcessException;

// ----------------------------------------------------------------
// COMSYS imports
// ----------------------------------------------------------------
import org.cognitivesystems.comsys.monitors.AbstractProxyFactory;
import org.cognitivesystems.comsys.monitors.PendingProxyRelation;
import org.cognitivesystems.comsys.monitors.ProxyFactoryResults;
import org.cognitivesystems.comsys.monitors.LocalProxyStructure;

// ----------------------------------------------------------------
// JAVA imports
// ----------------------------------------------------------------
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.Vector;


// ----------------------------------------------------------------
// LF imports
// ----------------------------------------------------------------
import org.cognitivesystems.repr.lf.autogen.LFEssentials.*;
import org.cognitivesystems.repr.lf.autogen.LFPacking.*;

public class ActionPutProxyFactory 
	extends AbstractProxyFactory
{

	
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

	boolean logging; 
	
	TemporalModifierFactory timeMF;

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public ActionPutProxyFactory () { 
		init();
	} // end

	private void init() { 
		logging = true; 
		timeMF  = new TemporalModifierFactory();
	} 
	

    //=================================================================
    // COMPUTATION METHODS
    //=================================================================
	
	protected void addFeatures (Iterator featsIter) 
	{ 
		while (featsIter.hasNext()) { 
			PackedFeature pFeat = (PackedFeature) featsIter.next();
			String feature	= pFeat.feat;
			String value	= pFeat.value; 
			proxyStructure.setFeature(feature,value);
		} // end while over features 
	} // end features


	/** Produces a proxy, or collection of proxies, starting from the given 
		nominal, using the packed logical form and the treemap-index into the nominals. 
		The method returns a result structure including variables of nominals for which we should no 
		longer generate proxies, and the relations that should be introduced.
		
		@param nom The packed nominal from which proxy production should start
		@param plf The packed logical form in which the nominal appears
		@param packedNoms The map with nominal variables indexing into the packed logical form
		@return ProxyFactoryResults A set with nominal variables of packed nominals for which proxies have been produced
		@see org.cognitivesystems.comsys.monitors.AbstractProxyFactory#getRelProxiesTable		 
		
	*/
	public ProxyFactoryResults produceProxies (PackedNominal nom, PackedLogicalForm plf, TreeMap packedNoms) 
	{  
		proxyStructure = new LocalProxyStructure();
		// Initialize the result structures
		ProxyFactoryResults result = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		// Get the verbal proposition
		String verb = nom.prop.prop;
		// Add the basic content to the proxy structure
		addBasicContent(nom);
		// Create the relations
		ProxyFactoryResults relResults = createRelations(nom,plf,packedNoms);
		Vector<PendingProxyRelation> pendingRels = new Vector<PendingProxyRelation>();
		pendingRels.addAll(relResults.getPendingProxyRelations());
		excludes.addAll(relResults.getExcludes());
		// Return the relations as part of the result
		result.setPendingProxyRelations(pendingRels);
		result.setExcludes(excludes);
		result.setProxyStructure(proxyStructure);
		// Return the results
		return result;
	} // end produceProxies	
		
	/**	
		Cycles over the LF and PLF relations, and creates the appropriate pending proxy relations where necessary. 
	*/ 	
		
	private ProxyFactoryResults createRelations (PackedNominal nom, PackedLogicalForm plf, TreeMap packedNoms) { 	
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		// =======================================================================
		// HANDLE THE LF-BASED CONTENT
		// =======================================================================		
		// cycle over the LF relations
		Vector<PendingProxyRelation> pendingRels = new Vector<PendingProxyRelation>();
		ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(nom.rels));
		Iterator relsIter = relations.iterator();
		while (relsIter.hasNext()) { 
			LFRelation rel = (LFRelation) relsIter.next();
			if (rel.mode.equals("Actor")) { 
				// Get the dependent nominal
				PendingProxyRelation actorPPR = new PendingProxyRelation();
				actorPPR.headNomVar = nom.nomVar;
				actorPPR.relMode = "Actor";
				actorPPR.depNomVar = rel.dep;
				actorPPR.addContentStatus("intentional");								
				pendingRels.add(actorPPR); 
				log("Added an Actor for [put], based on LF-structure");
				
			} else if (rel.mode.equals("Patient")) { 
				PendingProxyRelation patientPPR = new PendingProxyRelation();
				patientPPR.headNomVar = nom.nomVar;
				patientPPR.relMode = "Patient";
				patientPPR.depNomVar = rel.dep;
				patientPPR.addContentStatus("intentional");				
				pendingRels.add(patientPPR); 
			} else if (rel.mode.equals("Subject")) { 
				// do nothing
			} else if (rel.mode.equals("Modifier")) { 
				// Get the dependent nominal
				PackedNominal modNom = (PackedNominal) packedNoms.get(rel.dep);
				String modSort = null; 
				// Get the sort of the modifier
				if (modNom.packedSorts.length > 0) { modSort = modNom.packedSorts[0].sort; } 
				// Ensure we have a sort, 
				if (modSort != null) { 				
					if (modSort.startsWith("m-time")) { 
						ProxyFactoryResults timeResults = timeMF.produceProxies(nom,plf,packedNoms);
						excludes.addAll(timeResults.getExcludes());
						pendingRels.addAll(timeResults.getPendingProxyRelations());
						proxyStructure.addRelations(timeResults.getPendingProxyRelations().iterator());	
					} else if (modSort.startsWith("m-manner")) { 	
						PendingProxyRelation mannerPPR = new PendingProxyRelation();
						mannerPPR.headNomVar = nom.nomVar;
						mannerPPR.relMode = "Manner";
						mannerPPR.depNomVar = rel.dep;
						mannerPPR.addContentStatus("intentional");				
						pendingRels.add(mannerPPR);
					} else if (modSort.startsWith("m-location")) { 	
						PackedNominal anchor = getDependentNominal(modNom,"Anchor",packedNoms);	
						PendingProxyRelation destPPR = new PendingProxyRelation();
						destPPR.headNomVar = nom.nomVar;
						String modProp = modNom.prop.prop;
						if (modProp.equals("in")) { 
							destPPR.relMode = "Location";	 // take the mug in the kitchen
						} else {
							destPPR.relMode = "Destination"; // take the mug to the kitchen
						} // end if..else
						destPPR.depNomVar = anchor.nomVar;
						destPPR.addContentStatus("intentional");				
						pendingRels.add(destPPR);
						excludes.add(modNom.nomVar);
					} else if (modSort.startsWith("m-whereto")) { 	
						PackedNominal anchor = getDependentNominal(modNom,"Anchor",packedNoms);	
						String anchorSort = null; 
						// Get the sort of the modifier
						if (anchor.packedSorts.length > 0) { anchorSort = anchor.packedSorts[0].sort; } 
						// Ensure we have a sort, 						
						if (anchorSort != null) { 
							if (anchorSort.startsWith("physical")) { 
								PackedNominal anchorMod = getDependentNominal(anchor,"Modifier",packedNoms);
								if (anchorMod != null) { 
									PendingProxyRelation destPPR = new PendingProxyRelation();
									destPPR.headNomVar = nom.nomVar;
									destPPR.relMode = "Destination"; 
									destPPR.depNomVar = anchorMod.nomVar;
									destPPR.addContentStatus("intentional");				
									pendingRels.add(destPPR);
									excludes.add(modNom.nomVar);								
									excludes.add(anchor.nomVar);
								} else { 
									log("Empty modifier under entity of type physical, in m-whereto context");
								} // end if..ensure non-empty modifier
							} else if (anchorSort.equals("e-region")) { 
								if (hasDependent(anchor,"Owner")) { 
									log("Connecting destination to owner of anchor");
									PackedNominal owner = getDependentNominal(anchor,"Owner", packedNoms);
									PendingProxyRelation destPPR = new PendingProxyRelation();
									destPPR.headNomVar = nom.nomVar;
									destPPR.relMode = "Destination:"+anchor.prop.prop; 
									destPPR.depNomVar = owner.nomVar;
									destPPR.addContentStatus("intentional");				
									pendingRels.add(destPPR);
									excludes.add(modNom.nomVar);								
									excludes.add(anchor.nomVar);							
								} else { 	
									log("Cannot handle e-region embedded under m-whereto without Owner");
								} // end if.. 
								
								
							} else { 
								PendingProxyRelation destPPR = new PendingProxyRelation();
								destPPR.headNomVar = nom.nomVar;
								destPPR.relMode = "Destination"; 
								destPPR.depNomVar = anchor.nomVar;
								destPPR.addContentStatus("intentional");				
								pendingRels.add(destPPR);
								excludes.add(modNom.nomVar);						
							} // end if..else differentiate anchor types
						} // end if..check for anchorsort
					} else {
						log("Under ["+nom.prop.prop+"] unknown modifier of sort ["+modSort+"]");
					} // end if..else select over modifier types
				} // end if... ensure we have a sort
			} else if (rel.mode.equals("Result")) {	
					PackedNominal modNom = (PackedNominal) packedNoms.get(rel.dep);
					// take the mug into the kitchen
					PackedNominal anchor = getDependentNominal(modNom,"Anchor",packedNoms);	
					// Check whether the anchor is a "physical" with an additional modifier
					String anchorSort = null; 
					// Get the sort of the modifier
					if (anchor.packedSorts.length > 0) { anchorSort = anchor.packedSorts[0].sort; } 
					// Ensure we have a sort, 						
					if (anchorSort != null) { 
						if (anchorSort.startsWith("physical")) { 
							PackedNominal anchorMod = getDependentNominal(anchor,"Modifier",packedNoms);
							if (anchorMod != null) { 
								PendingProxyRelation destPPR = new PendingProxyRelation();
								destPPR.headNomVar = nom.nomVar;
								destPPR.relMode = "Destination"; 
								destPPR.depNomVar = anchorMod.nomVar;
								destPPR.addContentStatus("intentional");				
								pendingRels.add(destPPR);
								excludes.add(modNom.nomVar);								
								excludes.add(anchor.nomVar);
							} else { 
								log("Empty modifier under entity of type physical, in Result context");
							} // end if.. ensure non-empty modifier
						} else if (anchorSort.startsWith("e-location")) { 
							if (hasDependent(anchor,"Owner")) { 
								PackedNominal owner = getDependentNominal(anchor,"Owner",packedNoms);
								PendingProxyRelation destPPR = new PendingProxyRelation();
								destPPR.headNomVar = nom.nomVar;
								destPPR.relMode = "Destination:"+anchor.prop.prop; 
								destPPR.depNomVar = owner.nomVar;
								destPPR.addContentStatus("intentional");				
								pendingRels.add(destPPR);
								excludes.add(modNom.nomVar);								
								excludes.add(anchor.nomVar);																	
							} else { 
								PendingProxyRelation destPPR = new PendingProxyRelation();
								destPPR.headNomVar = nom.nomVar;
								destPPR.relMode = "Destination"; 
								destPPR.depNomVar = anchor.nomVar;
								destPPR.addContentStatus("intentional");				
								pendingRels.add(destPPR);
								excludes.add(modNom.nomVar);								
							} // end if..else check for owners of destinations
						} else if (anchorSort.equals("e-region")) { 
							if (hasDependent(anchor,"Owner")) { 
								log("Connecting destination to owner of anchor");
								PackedNominal owner = getDependentNominal(anchor,"Owner", packedNoms);
								PendingProxyRelation destPPR = new PendingProxyRelation();
								destPPR.headNomVar = nom.nomVar;
								destPPR.relMode = "Destination:"+anchor.prop.prop; 
								destPPR.depNomVar = owner.nomVar;
								destPPR.addContentStatus("intentional");				
								pendingRels.add(destPPR);
								excludes.add(modNom.nomVar);								
								excludes.add(anchor.nomVar);							
							} else { 	
								log("Cannot handle e-region embedded under m-whereto without Owner");
							} // end if.. 
							
							
						} else { 
							PendingProxyRelation destPPR = new PendingProxyRelation();
							destPPR.headNomVar = nom.nomVar;
							destPPR.relMode = "Destination:"+modNom.prop.prop; 
							destPPR.depNomVar = anchor.nomVar;
							pendingRels.add(destPPR);
							excludes.add(modNom.nomVar);						
						} // end if..else differentiate anchor types
					} // end if..check for anchorsort					
			} else { 
				PendingProxyRelation ppr = new PendingProxyRelation();
				ppr.headNomVar = nom.nomVar; 
				ppr.relMode = rel.mode;
				ppr.depNomVar = rel.dep;
				ppr.addContentStatus("intentional");				
				pendingRels.addElement(ppr);
			} // end if..else check for property relation
		} // end while over relations
		// =======================================================================
		// HANDLE THE PLF-BASED CONTENT
		// =======================================================================		

		// create the relations based on packing edges, if there are any
		if (nom.pEdges != null) { 
			log("Cycling over packing edges under ["+nom.nomVar+"]"); 
			ArrayList<PackingEdge> peEdges = new ArrayList<PackingEdge>(Arrays.asList(nom.pEdges));
			Iterator peIter = peEdges.iterator();
			while (peIter.hasNext()) { 
				PackingEdge packingEdge = (PackingEdge) peIter.next();
				if (packingEdge.mode.equals("Modifier")) { 
					ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
					if (targets.size() > 0) { 
						for(Iterator targetIter = targets.iterator(); targetIter.hasNext(); ) { 
							PackingNodeTarget target = (PackingNodeTarget) targetIter.next();
							String targetNV = target.pnId.substring(0,target.pnId.indexOf("_PN"));
							// Check for the sort of the target to see what modifier it is
							PackedNominal depNom = (PackedNominal) packedNoms.get(targetNV);
							String propertySort = null; 
							// Check whether it is a property, or a location
							if (depNom.packedSorts.length > 0) { 
								propertySort = depNom.packedSorts[0].sort;
							} 							
							if (propertySort.startsWith("m-time")) { 	
								ProxyFactoryResults timeResults = timeMF.produceProxies(nom,plf,packedNoms);
								excludes.addAll(timeResults.getExcludes());
								pendingRels.addAll(timeResults.getPendingProxyRelations());
								proxyStructure.addRelations(timeResults.getPendingProxyRelations().iterator());				
							} else if (propertySort.startsWith("m-manner")) { 	
								PendingProxyRelation mannerPPR = new PendingProxyRelation();
								mannerPPR.headNomVar = nom.nomVar;
								mannerPPR.relMode = "Manner";
								mannerPPR.depNomVar = targetNV;
								mannerPPR.addContentStatus("intentional");				
								pendingRels.add(mannerPPR);
							} else if (propertySort.startsWith("m-location")) { 	
								PackedNominal anchor = getDependentNominal(depNom,"Anchor",packedNoms);	
								PendingProxyRelation destPPR = new PendingProxyRelation();
								destPPR.headNomVar = nom.nomVar;
								String modProp = depNom.prop.prop;
								if (modProp.equals("in")) { 
									destPPR.relMode = "Location";	 // take the mug in the kitchen
								} else {
									destPPR.relMode = "Destination"; // take the mug to the kitchen
								} // end if..else
								destPPR.depNomVar = anchor.nomVar;
								destPPR.addContentStatus("intentional");				
								pendingRels.add(destPPR);
								excludes.add(depNom.nomVar);
							} else if (propertySort.startsWith("m-whereto")) { 	
								PackedNominal anchor = getDependentNominal(depNom,"Anchor",packedNoms);	
								String anchorSort = null; 
								// Get the sort of the modifier
								if (anchor.packedSorts.length > 0) { anchorSort = anchor.packedSorts[0].sort; } 
								// Ensure we have a sort, 						
								if (anchorSort != null) { 
									if (anchorSort.startsWith("physical")) { 
										PackedNominal anchorMod = getDependentNominal(anchor,"Modifier",packedNoms);
										if (anchorMod != null) { 
											PendingProxyRelation destPPR = new PendingProxyRelation();
											destPPR.headNomVar = nom.nomVar;
											destPPR.relMode = "Destination"; 
											destPPR.depNomVar = anchorMod.nomVar;
											destPPR.addContentStatus("intentional");				
											pendingRels.add(destPPR);
											excludes.add(depNom.nomVar);								
											excludes.add(anchor.nomVar);
										} else { 
											log("Empty modifier under entity of type physical, in m-whereto context");
										} // end if..ensure non-empty modifier
									} else if (anchorSort.equals("e-region")) { 
										if (hasDependent(anchor,"Owner")) { 
											log("Connecting destination to owner of anchor");
											PackedNominal owner = getDependentNominal(anchor,"Owner", packedNoms);
											PendingProxyRelation destPPR = new PendingProxyRelation();
											destPPR.headNomVar = nom.nomVar;
											destPPR.relMode = "Destination:"+anchor.prop.prop; 
											destPPR.depNomVar = owner.nomVar;
											destPPR.addContentStatus("intentional");				
											pendingRels.add(destPPR);
											excludes.add(depNom.nomVar);								
											excludes.add(anchor.nomVar);							
										} else { 	
											log("Cannot handle e-region embedded under m-whereto without Owner");
										} // end if.. 										
									} else { 
										PendingProxyRelation destPPR = new PendingProxyRelation();
										destPPR.headNomVar = nom.nomVar;
										destPPR.relMode = "Destination"; 
										destPPR.depNomVar = anchor.nomVar;
										destPPR.addContentStatus("intentional");				
										pendingRels.add(destPPR);
										excludes.add(depNom.nomVar);						
									} // end if..else differentiate anchor types
								} // end if..check for anchorsort
							} else {
								log("Under ["+nom.prop.prop+"] unknown modifier of sort ["+propertySort+"]");
							} // end if..else select over modifier types
						} // end for over targets
					} else { 
						log("No targets for a modifier");
					} // end if.. check for available targets
				} else if (packingEdge.mode.equals("Result")) {	
					ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
					for (Iterator targetIter = targets.iterator(); targetIter.hasNext(); ) { 			
						PackingNodeTarget target = (PackingNodeTarget) targetIter.next();
						String targetNV = target.pnId.substring(0,target.pnId.indexOf("_PN"));
						PackedNominal modNom = (PackedNominal) packedNoms.get(targetNV);
						PackedNominal anchor = getDependentNominal(modNom,"Anchor",packedNoms);	
						// Check whether the anchor is a "physical" with an additional modifier
						String anchorSort = null; 
						// Get the sort of the modifier
						if (anchor != null && anchor.packedSorts.length > 0) { anchorSort = anchor.packedSorts[0].sort; } 
						// Ensure we have a sort, 						
						if (anchorSort != null) { 
							if (anchorSort.startsWith("physical")) { 
								PackedNominal anchorMod = getDependentNominal(anchor,"Modifier",packedNoms);
								if (anchorMod != null) { 
									PendingProxyRelation destPPR = new PendingProxyRelation();
									destPPR.headNomVar = nom.nomVar;
									destPPR.relMode = "Destination"; 
									destPPR.depNomVar = anchorMod.nomVar;
									destPPR.addContentStatus("intentional");				
									pendingRels.add(destPPR);
									excludes.add(modNom.nomVar);								
									excludes.add(anchor.nomVar);
								} else { 
									log("Empty modifier under entity of type physical, in Result context");
								} // end if.. ensure non-empty modifier
							} else if (anchorSort.equals("e-region")) { 
								if (hasDependent(anchor,"Owner")) { 
									log("Connecting destination to owner of anchor");
									PackedNominal owner = getDependentNominal(anchor,"Owner", packedNoms);
									PendingProxyRelation destPPR = new PendingProxyRelation();
									destPPR.headNomVar = nom.nomVar;
									destPPR.relMode = "Destination:"+anchor.prop.prop; 
									destPPR.depNomVar = owner.nomVar;
									destPPR.addContentStatus("intentional");				
									pendingRels.add(destPPR);
									excludes.add(modNom.nomVar);								
									excludes.add(anchor.nomVar);							
								} else { 	
									log("Cannot handle e-region embedded under m-whereto without Owner");
								} // end if.. 										
							} else { 
								PendingProxyRelation destPPR = new PendingProxyRelation();
								destPPR.headNomVar = nom.nomVar;
								destPPR.relMode = "Destination:"+modNom.prop.prop; 
								destPPR.depNomVar = anchor.nomVar;
								destPPR.addContentStatus("intentional");				
								pendingRels.add(destPPR);
								excludes.add(modNom.nomVar);						
							} // end if..else differentiate anchor types
						} else { 	
							PendingProxyRelation destPPR = new PendingProxyRelation();							
							destPPR.headNomVar = nom.nomVar;
							destPPR.relMode = "Destination"; 
							destPPR.depNomVar = modNom.nomVar;
							destPPR.addContentStatus("intentional");				
							pendingRels.add(destPPR);
						} // end if..check for anchorsort		
					} // end for over targets										
				} else if (packingEdge.mode.equals("Actor")) { 
					ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
					for (Iterator targetIter = targets.iterator(); targetIter.hasNext(); ) { 
						PackingNodeTarget target = (PackingNodeTarget) targetIter.next();
						String targetNV = target.pnId.substring(0,target.pnId.indexOf("_PN"));
						PendingProxyRelation actorPPR = new PendingProxyRelation();
						actorPPR.headNomVar = nom.nomVar;
						actorPPR.relMode = "Actor";
						actorPPR.depNomVar = targetNV;
						actorPPR.addContentStatus("intentional");				
						pendingRels.add(actorPPR); 
						log("Added an Actor for [put], based on PLF-structure");

					}
				} else if (packingEdge.mode.equals("Patient")) { 
					ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
					for (Iterator targetIter = targets.iterator(); targetIter.hasNext(); ) { 
						PackingNodeTarget target = (PackingNodeTarget) targetIter.next();
						String targetNV = target.pnId.substring(0,target.pnId.indexOf("_PN"));				
						PendingProxyRelation patientPPR = new PendingProxyRelation();
						patientPPR.headNomVar = nom.nomVar;
						patientPPR.relMode = "Patient";
						patientPPR.depNomVar = targetNV;
						patientPPR.addContentStatus("intentional");				
						pendingRels.add(patientPPR); 
					}
				} else if (packingEdge.mode.equals("Subject")) { 
					// do nothing
				} else { 
					ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
					for (Iterator targetIter = targets.iterator(); targetIter.hasNext(); ) { 
							PackingNodeTarget target = (PackingNodeTarget) targetIter.next();
							String targetNV = target.pnId.substring(0,target.pnId.indexOf("_PN"));
							// Create the result
							log(" Putting in a new packing-edge based relation of type ["+packingEdge.mode+"] under Thing ["+nom.nomVar+"]");  
							PendingProxyRelation ppr = new PendingProxyRelation();
							ppr.headNomVar = nom.nomVar; 
							ppr.relMode = packingEdge.mode; 
							ppr.depNomVar = targetNV;
							ppr.addContentStatus("intentional");				
							pendingRels.addElement(ppr);
							proxyStructure.addRelation(ppr);
						} // end for over targets
				} // end if..else check for relation types
			} // end while over relations
		} // end if ... check for packing edges
		// -------------------------------------
		// RETURN THE RESULTS
		// -------------------------------------		
		// Add the relations to the result
		results.setPendingProxyRelations(pendingRels);
		results.setExcludes(excludes);
		results.setProxyStructure(proxyStructure);
		// Return the results
		return results;
	} // end createRelations	


			
} // end class		
		
		
		
		
		
