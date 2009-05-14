//=================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
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
// COMSYS imports
// ----------------------------------------------------------------
import org.cognitivesystems.comsys.monitors.AbstractProxyFactory;
import org.cognitivesystems.comsys.monitors.LocalProxyStructure; 
import org.cognitivesystems.comsys.monitors.PendingProxyRelation;
import org.cognitivesystems.comsys.monitors.ProxyFactoryResults;



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
import org.cognitivesystems.repr.lf.utils.*;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
The class <b>AscriptionProxyFactory</b> implements the  mapping 
for producing a proxy, or a collection of proxies and proxy relations, 
for ASCRIPTION type nominals. The factory covers the following types of constructions: 

<ul>
<li> 


</ul>

The proxy factory first determines the "argument", and then the "predicate" over that argument. It then applies the predicate to the argument, to yield a proposition. 
This proposition forms the basis for the proxy structures written to a working memory. 


@started 080709 (reimplementation of old version)
@version 080820
@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class AscriptionProxyFactory 
	extends AbstractProxyFactory
	
{

    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

	// inherited from the Abstract PF: 
	
	// pendingRels: pending proxy relations
	// proxyStructure: a proxy structure being built

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public AscriptionProxyFactory () { 
		super();
		rootSort = "ascription";
	} // end



    //=================================================================
    // COMPUTATION METHODS
    //=================================================================




	/** 
	The method <i>retrieveArgument</i> processes the ascription construction under the 
	provided root <tt>nom</tt>, to retrieve the argument of the predicate. This
	argument is returned as a nominal object (available in the <tt>packedNoms</tt> map). 
<p>
	<ul>
	<li> DEICTIC is ARG: DEICTIC is "entity:context"
	<li> ARG is PROPERTY: PROPERTY is a "q-" type
	<li> ABSTRACT-TYPE is ARG: identical types of ABSTRACT-TYPE and ARG
	<li> ARG is  LOCATION is an "m-location" with an Anchor
	<li> LOC is ARG: same as before, only now inverted ("fronted")
	<li> ABSTRACT-PROP is ARG: ABSTRACT-PROP is an "entity" and then a property proposition
	<li> ABSTRACT-LOCATION is ARG: 
	</ul>


	@param nom				The root of the ascription construction
	@param packedNoms		The nominals in the packed logical form 
	@returns PackedNominal	The packed nominal representation of the argument
	*/ 

	public PackedNominal retrieveArgument (PackedNominal nom, TreeMap packedNoms) { 
		// Initialize the argument result
		PackedNominal argument = new PackedNominal();
		
		
		// first, check whether we have a BE or a HAVE ascription
		
		if (nom.prop.prop.equals("be")) { 
			// Retrieve the restrictor and the scope
			PackedNominal restrictor = getDependentNominal(nom,"Cop-Restr",packedNoms); 
			PackedNominal scope = getDependentNominal(nom,"Cop-Scope",packedNoms);
			// Get the propositions, sorts of the restrictor and the scope
			String restrSort = getSort(restrictor); 
			String restrProp = restrictor.prop.prop;
			String scopeSort = getSort(scope);
			String scopeProp = scope.prop.prop;
			// Check for different constructions
			if (restrSort.equals("entity")) { 
				// if deictic, the argument is the scope; else abstract property or contextual reference ("this one"), use restrictor as argument
				if (restrProp.equals("context")) { 
					if (hasDependent(restrictor,"Modifier")) { 
						argument = restrictor;	
					} else if(scopeProp.startsWith("wh")) { 
						argument = restrictor;
					} else {
						argument = scope;				
					} 
				} else { 
					argument = restrictor;
				} // end if..else check for type of entity
			} else if (scopeSort.startsWith("q-")) {
				// argument is restrictor, scope provides predicated property
				argument = restrictor;
			} else if (restrSort.equals(scopeSort)) { 
				if (scopeSort.equals("e-location")) {
					if (scopeProp.equals("room") || scopeProp.equals("location")) { 
						argument = restrictor;					
					} else { 
						argument = scope;				
					} 
				} else if (scopeSort.equals("thing")) { 
					if (scopeProp.equals("thing") || scopeProp.equals("object")) { 
						argument = restrictor;					
					} else { 
						argument = scope;				
					} 	
				} else { 
					// abstract type description, argument is the scope
					argument = scope;
				}
			} else if (scopeSort.equals("m-location")) { 
				argument = restrictor;
			} else if (scopeSort.equals("e-measure")) { 
				argument = scope;
			} else if (restrSort.equals("person") || restrSort.equals("animate")) { 
				argument = restrictor;
			} else if (restrSort.equals("thing")) { 
				argument = restrictor;			
			} else { 
				log("Unable to handle ascription construction, cannot establish argument ");
			} // end if..else over cases
			return argument;
		} else { 
			return getDependentNominal(nom,"Actor",packedNoms);
		} // end if..else 
	} // end retrieveArgument


	/** 
	The method <i>retrievePredicate</i> processes the ascription construction under the 
	provided root <tt>nom</tt>, to retrieve the predicate of the predicate. This
	predicate is returned as a nominal object (available in the <tt>packedNoms</tt> map). 

	@param nom				The root of the ascription construction
	@param packedNoms		The nominals in the packed logical form 
	@returns Vector			A vector of packed nominal representations of predicates
	*/ 

	private Vector retrievePredicate (PackedNominal nom, TreeMap packedNoms) { 
		// Initialize the predicate result
		Vector results = new Vector();
		// Retrieve the restrictor and the scope
		
		if (nom.prop.prop.equals("be")) { 
		PackedNominal restrictor = getDependentNominal(nom,"Cop-Restr",packedNoms); 
		for (Iterator<PackedNominal> scopeIter = getDependentNominals(nom,"Cop-Scope",packedNoms).iterator(); scopeIter.hasNext(); ) { 
			PackedNominal predicate = new PackedNominal();
			PackedNominal scope = scopeIter.next();
			// Get the propositions, sorts of the restrictor and the scope
			String restrSort = getSort(restrictor); 
			String restrProp = restrictor.prop.prop;
			String scopeSort = getSort(scope);
			String scopeProp = scope.prop.prop;
			
			log("Establishing predicate given restr ["+restrSort+"]:["+restrProp+"] and scope ["+scopeSort+"]:["+scopeProp+"]");
			
			// Check for different constructions
			if (restrSort.equals("entity")) { 
			// if deictic, the argument is the scope; else abstract property or contextual reference ("this one"), use restrictor as argument
			if (restrProp.equals("context")) { 
				if (hasDependent(restrictor,"Modifier")) { 
					predicate = scope;				
				} else if(scopeProp.startsWith("wh")) { 
					predicate = scope;
				} else {
					predicate = restrictor;				
				} 
			} else { 
				predicate = scope;
			} // end if..else check for type of entity		
			} else if (scopeSort.startsWith("q-")) {
				// scope provides predicated property
				predicate = scope;
			} else if (restrSort.equals(scopeSort)) { 
				if (scopeSort.equals("e-location")) {
					// the kitchen is the room on the left
					if (scopeProp.equals("room") || scopeProp.equals("location")) { 
						predicate = scope;					
					} else { 
						predicate = restrictor;				
					} 
				} else if (scopeSort.equals("thing")) { 
					// the ball is that red thing
					if (scopeProp.equals("thing") || scopeProp.equals("object")) { 
						predicate = scope;					
					} else { 
						predicate = restrictor;				
					} 	
				} else { 
					// abstract type description, argument is the scope
					predicate = restrictor;
				}
			} else if (scopeSort.equals("m-location")) { 
				predicate = scope;
			} else if (scopeSort.equals("e-measure")) { 
				predicate = restrictor;				
			} else if (restrSort.equals("person") || restrSort.equals("animate")) { 	
				predicate = scope;
			} else if (scopeSort.equals("quality") || scopeSort.equals("entity")) { 	
				predicate = scope;				
			} else { 
				log("Unable to handle ascription construction, cannot establish predicate");
			} // end if..else over cases
			results.add(predicate);
		} // end for.. over Scope
		} else { 
			results.add(getDependentNominal(nom,"Patient",packedNoms));
		} 
		return results;
	} // end retrievePredicate



	private void addProperty (PackedNominal property) {
		String propSort = getSort(property);
		String predProp = property.prop.prop;
		if (propSort.equals("q-age")) { 
			proxyStructure.setFeature("Age",predProp);				
		} else if (propSort.equals("q-attitude")) { 
			proxyStructure.setFeature("Attitude",predProp);	
		} else if (propSort.equals("q-color")) { 		
			proxyStructure.setFeature("Colour",predProp);	
		} else if (propSort.equals("q-location")) { 
			String degree = getFeatureValue(property,"Degree");
			if (degree != null) { 
				proxyStructure.setFeature("Proximity",predProp+"-"+degree);						
			} else { 
				proxyStructure.setFeature("Proximity",predProp);			
			} 
		} else if (propSort.equals("q-number")) { 					
			proxyStructure.setFeature("Number",predProp);			
		} else if (propSort.equals("q-physical")) { 			
			proxyStructure.setFeature("State",predProp);			
		} else if (propSort.equals("q-position")) { 					
			proxyStructure.setFeature("Position",predProp);					
		} else if (propSort.equals("q-shape")) { 	
			proxyStructure.setFeature("Shape",predProp);							
		} else if (propSort.equals("q-size")) { 				
			proxyStructure.setFeature("Size",predProp);											
		} else if (propSort.equals("q-state")) { 
			proxyStructure.setFeature("State",predProp);														
		} else if (propSort.equals("number-cardinal")) { 
			proxyStructure.setFeature("Cardinality",predProp);	
		} else { 
			log("Unknown property sort in constructing proposition");
			System.exit(0);					
		} // end if..else check for property
	} // end addProperty


	private ProxyFactoryResults apply(PackedNominal argument, String abstractArgument, Vector predicates, PackedLogicalForm plf, TreeMap packedNoms) { 
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();

		log("Before applying any predicate, the proxy rels are "+pendingRels);
		Vector filteredRels = new Vector();
		TreeSet argDeps = getDependentNomVars(argument,packedNoms);
		for (Iterator<PendingProxyRelation> pendingIter = pendingRels.iterator(); pendingIter.hasNext(); ) { 
			PendingProxyRelation pendingRel = pendingIter.next();
			if (argDeps.contains(pendingRel.depNomVar)) { filteredRels.add(pendingRel); }
		} 
		pendingRels = filteredRels;
		log("After filtering, the proxy rels are "+pendingRels);		
		
		
		for (Iterator<PackedNominal> predsIter = predicates.iterator(); predsIter.hasNext(); ) { 
			PackedNominal predicate = predsIter.next();
			String predSort = getSort(predicate);
			String predProp = predicate.prop.prop;		
			
			
			
			log("Predicate: ["+predSort+"]:["+predProp+"]");
			
			// Next apply the predicate
			// Check for context or abstract property
			if (predSort.equals("entity")) { 
				if (predProp.equals("context")) { 
					proxyStructure.setFeature("Salience",predicate.prop.prop);
					// now establish the appropriate proximity, based on the Proximity feature 
					ArrayList<PackedFeature> packedFeats = new ArrayList<PackedFeature>(Arrays.asList(predicate.feats));
					Iterator featsIter = packedFeats.iterator();
					while (featsIter.hasNext()) { 
						PackedFeature pFeat = (PackedFeature) featsIter.next();
						String feature = pFeat.feat;
						if (feature.equals("Proximity")) { 
							proxyStructure.setFeature("Proximity",pFeat.value);
						} // end if.. check for proximity feature
					} // end while over features 
					// exclude the predicate
					excludes.add(predicate.nomVar);
					if (hasDependent(predicate, "Modifier")) { 
						PackedNominal mod = getDependentNominal(predicate,"Modifier",packedNoms);
						// Add the property
						addProperty(mod);
						// handle the excludes
						excludes.add(predicate.nomVar);
						excludes.addAll(getDependentNomVars(predicate,packedNoms));
					}
				} else { 
					log("Unknown entity construction in constructing proposition");
					System.exit(0);			
				} // end if..else check for entity type
			} else if (predSort.startsWith("q-")) { 
				// Add the property
				addProperty(predicate);
				// handle the excludes
				excludes.add(predicate.nomVar);
				excludes.addAll(getDependentNomVars(predicate,packedNoms));
			} else if (predSort.equals("quality")) { 
				// conjunction
				boolean contCoord = true;
				PackedNominal coordHead = predicate;
				excludes.add(coordHead.nomVar);
				while (contCoord) { 
					PackedNominal first = getDependentNominal(coordHead,"First",packedNoms);
					PackedNominal next  = getDependentNominal(coordHead,"Next",packedNoms);
					log("Conjunction, looking at first ["+first.nomVar+"]:["+getSort(first)+"]");
					log("Conjunction, looking at next ["+next.nomVar+"]:["+getSort(first)+"]");					
					addProperty(first);
					log("Applied property to first");
					excludes.add(first.nomVar);
					excludes.add(next.nomVar);
					if (getSort(next).equals("quality")) { 
						coordHead = next;
					} else { 
						contCoord = false;
						addProperty(next);
						log("Applied property to next");						
					} // end if..else check for embedded list
				} // end while over embedded conjuncts
			} else if (predSort.equals("person") || predSort.equals("animate")) { 
				log("Putting in a new naming relation of type [ISA] to ["+predicate.nomVar+"] from ["+argument.nomVar+"]");
				PendingProxyRelation ppr = new PendingProxyRelation();
				ppr.headNomVar = argument.nomVar; 
				ppr.relMode = "ISA";
				ppr.depNomVar = predicate.nomVar;
				pendingRels.addElement(ppr);
				proxyStructure.addRelation(ppr);				
			} else if (predSort.equals("m-location")) { 
				log("Applying a spatial predicate, LF-style");
				if (hasDependent(predicate,"Anchor")) { 
					// Get the anchor
					PackedNominal anchor = getDependentNominal(predicate,"Anchor",packedNoms);
					// Exclude the predicate and add a relation directly to the anchor
					excludes.add(predicate.nomVar);
					String anchorSort = getSort(anchor);
					// Initialize the spatial relation
					PendingProxyRelation ppr = new PendingProxyRelation();
					ppr.headNomVar = argument.nomVar;					
					// let the relation by default point to the anchor
					ppr.depNomVar = anchor.nomVar;	
					// Now initialize the spatial relation, collapsing things where necessary
					String spatrel = "";
					if (anchorSort.equals("thing")) {
						if (hasDependent(predicate,"Manner")) { 
							// the ball is right next to the star
							PackedNominal manner = getDependentNominal(predicate,"Manner",packedNoms);
							spatrel = "Position:"+manner.prop.prop;
						} else { 
							// the ball is right of the star
							spatrel = "Position:"+predProp;
						} // end if..else
					} else if (anchorSort.equals("e-region")) { 
						spatrel = "Position:"+anchor.prop.prop;
						if (hasDependent(anchor,"Owner")) { 
							excludes.add(anchor.nomVar);
							PackedNominal owner = getDependentNominal(anchor,"Owner",packedNoms);
							ppr.depNomVar = owner.nomVar;
						} // end if.. check whether "to the X of Y" construction
					} else { 
						// spatrel = anchor.prop.prop;
						spatrel = "Position:"+predicate.prop.prop;
					} // end if..else check for anchor type
					if (hasDependent(predicate,"Modifier")) {
						PackedNominal modifier = getDependentNominal(predicate,"Modifier",packedNoms);
						String modSort = getSort(modifier);
						if (modSort.equals("manner")) { 
							spatrel = spatrel+":"+modifier.prop.prop;
						} 
						excludes.add(modifier.nomVar);
					} // end if.. check for manner
					

					if (abstractArgument != null) { 
						ppr.headNomVar = abstractArgument;
					} 
					
					log("Putting in a new spatial relation of type ["+spatrel+"] to ["+ppr.depNomVar+"] from ["+ppr.headNomVar+"]");

					ppr.relMode = spatrel;
					pendingRels.addElement(ppr);
					proxyStructure.addRelation(ppr);
					
					log("Pending rels are now "+pendingRels);
					
				} else { 
					proxyStructure.setFeature("Proximity",predProp);					
					excludes.add(predicate.nomVar);
				} 
			} else if (predSort.equals("e-location") || predSort.equals("thing")) { 
				ArrayList<PackedFeature> packedFeats = new ArrayList<PackedFeature>(Arrays.asList(predicate.feats));
				Iterator featsIter = packedFeats.iterator();
				while (featsIter.hasNext()) { 
					PackedFeature pFeat = (PackedFeature) featsIter.next();
					String feature = pFeat.feat;
					if (feature.equals("Proximity")) { 
						proxyStructure.setFeature("Proximity",pFeat.value);
					} // end if.. check for proximity feature
				} // end while over features
				if (hasDependent(predicate,"Modifier")) { 
					for (Iterator<PackedNominal> modsIter = getDependentNominals(predicate,"Modifier",packedNoms).iterator(); modsIter.hasNext(); ) { 
						PackedNominal mod = modsIter.next();
						String modSort = getSort(mod);
						if (modSort.startsWith("q-")) { 
							addProperty(mod);
							excludes.add(mod.nomVar);
						} else if (modSort.equals("m-location")) { 
							if (hasDependent(mod,"Anchor")) {
								PackedNominal anchor = getDependentNominal(mod,"Anchor",packedNoms);
								PendingProxyRelation ppr = new PendingProxyRelation();
								ppr.headNomVar = argument.nomVar; 
								ppr.relMode = "Position:"+mod.prop.prop;
								// used to be: ppr.relMode = anchor.prop.prop
								ppr.depNomVar = anchor.nomVar;
								pendingRels.addElement(ppr);
								proxyStructure.addRelation(ppr);							
								excludes.add(mod.nomVar);
							} else { 
								PendingProxyRelation ppr = new PendingProxyRelation();
								ppr.headNomVar = argument.nomVar; 
								ppr.relMode = mod.prop.prop;
								ppr.depNomVar = mod.nomVar;
								pendingRels.addElement(ppr);
								proxyStructure.addRelation(ppr);							
							} 
						} // end if..else
					} // end for.. over modifiers
					excludes.add(predicate.nomVar);
				} else { 
					// create the relations based on packing edges, if there are any
					if (predicate.pEdges != null) { 
						ArrayList<PackingEdge> peEdges = new ArrayList<PackingEdge>(Arrays.asList(predicate.pEdges));
						Iterator peIter = peEdges.iterator();
						while (peIter.hasNext()) { 
							PackingEdge packingEdge = (PackingEdge) peIter.next();
							// if (packingEdge.mode.equals("Property")) { 
							if (packingEdge.mode.equals("Modifier")) { 
								log("Looking at packing edge Modifier");
								ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
								if (targets.size() > 0) { 
									for(Iterator targetIter = targets.iterator(); targetIter.hasNext(); ) { 
										PackingNodeTarget target = (PackingNodeTarget) targetIter.next();
										String targetNV = target.pnId.substring(0,target.pnId.indexOf("_PN"));
										// check for the sort of the target to see what modifier it is
										PackedNominal mod = (PackedNominal) packedNoms.get(targetNV);
										String modSort = getSort(mod); 				
										if (modSort.startsWith("q-")) { 
											addProperty(mod);
											excludes.add(mod.nomVar);
										} else if (modSort.equals("m-location")) { 
											if (hasDependent(mod,"Anchor")) {
												PackedNominal anchor = getDependentNominal(mod,"Anchor",packedNoms);
												if (hasDependent(anchor,"Owner")) { 
													PackedNominal owner = getDependentNominal(anchor,"Owner",packedNoms);
													PendingProxyRelation ppr = new PendingProxyRelation();
													ppr.headNomVar = argument.nomVar; 
													ppr.relMode = "Position:"+anchor.prop.prop;
													ppr.depNomVar = owner.nomVar;
													pendingRels.addElement(ppr);
													proxyStructure.addRelation(ppr);							
													excludes.add(mod.nomVar);
													excludes.add(anchor.nomVar);																										
												} else { 	
													PendingProxyRelation ppr = new PendingProxyRelation();
													ppr.headNomVar = argument.nomVar; 
													ppr.relMode = "Position:"+anchor.prop.prop;
													ppr.depNomVar = anchor.nomVar;
													pendingRels.addElement(ppr);
													proxyStructure.addRelation(ppr);							
													excludes.add(mod.nomVar);
												} 
													
											} else { 
												PendingProxyRelation ppr = new PendingProxyRelation();
												ppr.headNomVar = argument.nomVar; 
												ppr.relMode = mod.prop.prop;
												ppr.depNomVar = mod.nomVar;
												pendingRels.addElement(ppr);
												proxyStructure.addRelation(ppr);							
											} 
										} // end if..else
									} // end for over targets
								} // end if.. targets
								log("pending relations are now: "+pendingRels);
							} // end if.. modifier
						} // end while over packing edges
						excludes.add(predicate.nomVar);
					} else {		
						log("Trying to apply an unknown type of e-location / thing predication: ["+predSort+"]:["+predProp+"]");
					}
				} // end if..else
				
			} else { 
				log("Trying to apply an unknown type of predication: ["+predSort+"]:["+predProp+"]");
			} 
		} // end for over predicates	
		
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRels);
		results.setProxyStructure(proxyStructure);		
	
		return results; 
		
	} 

	/**
	The method <i>applyPredicate</i> constructs the core proxy structure of the proposition 
	resulting from applying the predicate to the argument. Relational arguments of this core
	structure may be left for further generation by other proxy factories. 
	
	@param argument					The argument, as packed nominal
	@param predicate				The predicate, as packed nominal
	@param packedNoms				The nominals in the packed logical form
	@returns ProxyFactoryResults	The results, including the proxy structure, excludes, and pending relations
	*/ 

	private ProxyFactoryResults applyPredicate (PackedNominal argument, Vector predicates, PackedLogicalForm plf, TreeMap packedNoms) { 
		// If non-null, pointer to the packed nominal for which an abstract property was given, e.g. "the location of..."
		String abstractArgument = null;
		// Initialize the result and the excludes. 
		TreeSet excludes = new TreeSet();
		ProxyFactoryResults results = new ProxyFactoryResults();
		// Initialize helpers
		String argSort = getSort(argument);
		String argProp = argument.prop.prop;
		// Establish the kind of argument, predicate
		log("Argument ["+argSort+"]:["+argProp+"]");
		// Create the proxy structure for the argument; this is the one we will extend
		ProxyFactoryResults argPRX = new ProxyFactoryResults();
		if (argSort.equals("thing")) { 
			ThingProxyFactory thingPF = new ThingProxyFactory();
			thingPF.setDiscRefs(_discRefsAccess);
			argPRX = thingPF.produce(argument,plf,packedNoms);
		} else if (argSort.equals("e-location") || argSort.equals("e-measure") || argSort.equals("e-place")) { 
			if (hasDependent(argument,"Owner")) { 
				// the location of the E-LOCATION
				PackedNominal owner = getDependentNominal(argument,"Owner",packedNoms);	
				if (getSort(owner).equals("e-location")) {
					ELocationProxyFactory elocPF = new ELocationProxyFactory();
					elocPF.setDiscRefs(_discRefsAccess);
					argPRX = elocPF.produce(owner,plf,packedNoms);
					excludes.add(argument.nomVar);
					abstractArgument = owner.nomVar;
				} else if (getSort(owner).equals("thing")) {
					ThingProxyFactory thingPF = new ThingProxyFactory();
					thingPF.setDiscRefs(_discRefsAccess);
					argPRX = thingPF.produce(owner,plf,packedNoms);	
					excludes.add(argument.nomVar);
					abstractArgument = owner.nomVar;					
				} else if (getSort(owner).equals("person")) { 
					// the X of PERSON Y
					ELocationProxyFactory elocPF = new ELocationProxyFactory();
					elocPF.setDiscRefs(_discRefsAccess);
					argPRX = elocPF.produce(argument,plf,packedNoms);
					excludes.add(argument.nomVar);
				} else { 
					log("Unknown type of argument under abstract e-location");
					System.exit(0);
				} 
				
			} else {
				ELocationProxyFactory elocPF = new ELocationProxyFactory();
				elocPF.setDiscRefs(_discRefsAccess);
				argPRX = elocPF.produce(argument,plf,packedNoms);		
			} // end if..else
		} else if (argSort.equals("person")) { 
			PersonProxyFactory personPF = new PersonProxyFactory();
			personPF.setDiscRefs(_discRefsAccess);
			argPRX = personPF.produce(argument,plf,packedNoms);
		} else if (argSort.equals("animate")) { 
			AnimateProxyFactory animatePF = new AnimateProxyFactory();
			animatePF.setDiscRefs(_discRefsAccess);
			argPRX = animatePF.produce(argument,plf,packedNoms);
		} else if (argSort.equals("entity")) { 	
			log("Argument with sort ENTITY has proposition ["+argProp+"]");
			// abstract contextual reference
			if (argProp.equals("context") || argProp.startsWith("wh")) { 
				EntityProxyFactory entityPF = new EntityProxyFactory();
				entityPF.setDiscRefs(_discRefsAccess);
				argPRX = entityPF.produce(argument,plf,packedNoms);
				// abstract descriptions: the X of Y
			} else if (hasDependent(argument,"Owner")) { 
				log("Dealing with restrictor type the X of Y");
				PackedNominal owner = getDependentNominal(argument,"Owner",packedNoms);
				String ownerSort = getSort(owner);
				if (ownerSort.equals("thing")) { 
					ThingProxyFactory thingPF = new ThingProxyFactory();
					thingPF.setDiscRefs(_discRefsAccess);
					argPRX = thingPF.produce(owner,plf,packedNoms);
					excludes.add(argument.nomVar);					
				} else if (ownerSort.equals("e-location")) { 
					ELocationProxyFactory elocPF = new ELocationProxyFactory();
					elocPF.setDiscRefs(_discRefsAccess);
					argPRX = elocPF.produce(owner,plf,packedNoms);					
					excludes.add(argument.nomVar);					
				} else if (ownerSort.equals("person")) { 
					PersonProxyFactory personPF = new PersonProxyFactory();
					personPF.setDiscRefs(_discRefsAccess);
					argPRX = personPF.produce(owner,plf,packedNoms);
					excludes.add(argument.nomVar);					
				} else if (ownerSort.equals("animate")) { 
					AnimateProxyFactory animatePF = new AnimateProxyFactory();
					animatePF.setDiscRefs(_discRefsAccess);
					argPRX = animatePF.produce(owner,plf,packedNoms);				
					excludes.add(argument.nomVar);					
				} else { 
					log("Unknown entity Owner construction in constructing argument proxy");
					System.exit(0);				
				} 
				abstractArgument = owner.nomVar;
			} else { 
				log("Unknown entity construction in constructing argument proxy");
				System.exit(0);
			} // end if..else
		} else { 
			log("Unknown type of argument for applying predicate to");
			System.exit(0);
		} // end if..else
		// Retrieve the proxy structure, excludes, pending relations
		proxyStructure = argPRX.getProxyStructure();
		proxyStructure.addContentStatus("indexical");
		proxyStructure.addContentStatus("intentional");		
		pendingRels = argPRX.getPendingProxyRelations();
		excludes.addAll(argPRX.getExcludes());
		// Add the already covered nominals to the excludes
		excludes.addAll(argPRX.getAddedNominalsSet());		
		
		// check for context predication, apply properties
		for (Iterator<PackedNominal> predsIter = predicates.iterator(); predsIter.hasNext(); ) { 
			PackedNominal predicate = predsIter.next();
			String predSort = getSort(predicate);
			String predProp = predicate.prop.prop;
			if (predProp.equals("context")) { 
				excludes.addAll(getDependentNomVars(predicate,packedNoms));			
				// now establish the appropriate proximity, based on the Proximity feature 
				ArrayList<PackedFeature> packedFeats = new ArrayList<PackedFeature>(Arrays.asList(predicate.feats));
				Iterator featsIter = packedFeats.iterator();
				while (featsIter.hasNext()) { 
					PackedFeature pFeat = (PackedFeature) featsIter.next();
					String feature = pFeat.feat;
					if (feature.equals("Proximity")) { 
						proxyStructure.setFeature("Proximity",pFeat.value);
					} // end if.. check for proximity feature
				} // end while over features 	
				if (hasDependent(predicate,"Modifier")) { 
					for (Iterator<PackedNominal> modsIter = getDependentNominals(predicate,"Modifier",packedNoms).iterator(); modsIter.hasNext(); ) { 
						addProperty(modsIter.next());
					} 
				}
			} else if (predSort.startsWith("q-")) { 
				log("Don't know what to do with q- style predications ... guess nothing");
				
			} else if (predSort.equals("m-location")) { 
				log("Applying a spatial predicate to a question argument, LF-style");
				if (hasDependent(predicate,"Anchor")) { 
					// Get the anchor
					PackedNominal anchor = getDependentNominal(predicate,"Anchor",packedNoms);
					// Exclude the predicate and add a relation directly to the anchor
					excludes.add(predicate.nomVar);
					String anchorSort = getSort(anchor);
					// Initialize the spatial relation
					PendingProxyRelation ppr = new PendingProxyRelation();
					ppr.headNomVar = argument.nomVar;					
					// let the relation by default point to the anchor
					ppr.depNomVar = anchor.nomVar;	
					// Now initialize the spatial relation, collapsing things where necessary
					String spatrel = "";
					if (anchorSort.equals("thing")) {
						if (hasDependent(predicate,"Manner")) { 
							// the ball is right next to the star
							PackedNominal manner = getDependentNominal(predicate,"Manner",packedNoms);
							spatrel = "Position:"+manner.prop.prop;
						} else { 
							// the ball is right of the star
							spatrel = "Position:"+predProp;
						} // end if..else
					} else if (anchorSort.equals("e-region")) { 
						spatrel = "Position:"+anchor.prop.prop;
						if (hasDependent(anchor,"Owner")) { 
							excludes.add(anchor.nomVar);
							PackedNominal owner = getDependentNominal(anchor,"Owner",packedNoms);
							ppr.depNomVar = owner.nomVar;
						} // end if.. check whether "to the X of Y" construction
					} else { 
						// spatrel = anchor.prop.prop;
						spatrel = "Position:"+predicate.prop.prop;
					} // end if..else check for anchor type
					if (hasDependent(predicate,"Modifier")) {
						PackedNominal modifier = getDependentNominal(predicate,"Modifier",packedNoms);
						String modSort = getSort(modifier);
						if (modSort.equals("manner")) { 
							spatrel = spatrel+":"+modifier.prop.prop;
						} 
						excludes.add(modifier.nomVar);
					} // end if.. check for manner
					
					
					if (abstractArgument != null) { 
						ppr.headNomVar = abstractArgument;
					} 
					
					log("Putting in a new spatial relation of type ["+spatrel+"] to ["+ppr.depNomVar+"] from ["+ppr.headNomVar+"]");
					
					ppr.relMode = spatrel;
					pendingRels.addElement(ppr);
					proxyStructure.addRelation(ppr);
					
					log("Pending rels are now "+pendingRels);
					
				} else { 
					proxyStructure.setFeature("Proximity",predProp);					
					excludes.add(predicate.nomVar);
				} 
			}
		} // end for over predicates
		
		// Now handle the assertion structure
		// Initialize the TYPE-Q and HEARER-KNOWS relations
		PendingProxyRelation typeA = new PendingProxyRelation();
		PendingProxyRelation hearerA = new PendingProxyRelation();
		
		typeA.depNomVar = argument.nomVar; 
		typeA.relMode = "Assertion";
		typeA.addContentStatus("intentional");
		hearerA.depNomVar = argument.nomVar; 
		hearerA.addContentStatus("intentional");		
		
		if (abstractArgument != null) { 
			typeA.headNomVar = abstractArgument;
			hearerA.headNomVar = abstractArgument;
		} else { 
			hearerA.headNomVar = argument.nomVar;
			typeA.headNomVar = argument.nomVar;
		}
		
		// we are dealing with a polar question 
		// first, apply the predication
		
		// check whether we are dealing with identical argument and predicate sorts, 
		// without it being an abstractArgument; if so, do not apply! 
		PackedNominal prd = (PackedNominal) predicates.firstElement();
		String prdsort = getSort(prd);
		String argsort = getSort(argument);
		if (prdsort.equals(argsort) && abstractArgument == null) { 
			log("Do not apply the predicate to the argument!");
			hearerA.relMode = "HEARER-KNOWS:Concept";
			hearerA.headNomVar = prd.nomVar; // used to be dep
			typeA.headNomVar = prd.nomVar;
		} else { 
			log("Pred sort ["+prdsort+"] and argument sort ["+argsort+"] different under polar, so apply");
			ProxyFactoryResults predication = apply(argument, abstractArgument, predicates,  plf,  packedNoms); 
			excludes.addAll(predication.getExcludes());
			pendingRels.addAll(predication.getPendingProxyRelations());	
		}
		
		// we need to update the Hearer-Knows relation, depending on the predicate / scope
		
		for (Iterator<PackedNominal> predsIter = predicates.iterator(); predsIter.hasNext(); ) { 
			PackedNominal predicate = predsIter.next();
			String scopeProp =	getSort(predicate);
			
			log("Looking at scope / predicate proposition: ["+scopeProp+"]");
			
			if (scopeProp.equals("m-location")) { 
				hearerA.relMode="HEARER-KNOWS:Position";
			} else if (scopeProp.equals("q-color")) { 
				hearerA.relMode="HEARER-KNOWS:Colour";
			} else if (scopeProp.equals("q-shape")) { 
				hearerA.relMode="HEARER-KNOWS:Shape";
			} else if (scopeProp.equals("q-size")) { 
				hearerA.relMode="HEARER-KNOWS:Size";
			} else if (scopeProp.equals("q-state")) { 
				hearerA.relMode="HEARER-KNOWS:State";
			} else if (scopeProp.equals("entity")) {
				String delimitation = LFUtils.plfGetFeatureValue(argument,"Delimitation"); 
				if (delimitation != null) { 
					if (delimitation.equals("unique")) { 
						hearerA.relMode="HEARER-KNOWS:Identity";					
					} else { 
						hearerA.relMode="HEARER-KNOWS:Concept?"+argument.prop.prop;												
					} 
				}
			} 
		} // end for over predicates
			
		
		pendingRels.addElement(hearerA);
		proxyStructure.addRelation(hearerA);	
		pendingRels.addElement(typeA);
		proxyStructure.addRelation(typeA);	
		
		// Set the results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRels);
		results.setProxyStructure(proxyStructure);
		results.setAddedNominals(argPRX.getAddedNominalsSet());
		
		log("Proxy structure after applying assertion:\n"+results.getProxyStructure().toString());
		log("Pending proxy relations:\n"+results.getPendingProxyRelations().toString());
		log("Excludes: "+results.getExcludes().toString());		
		log("Added nominals: "+results.getAddedNominalsSet().toString());
		
		return results; 
	}// end applyPredicate





	/**
	The method <i>applyQuestion</i> constructs the core proxy structure of the proposition 
	resulting from applying the question to the argument. Relational arguments of this core
	structure may be left for further generation by other proxy factories. Creating the question
	means putting in place the relations between the WH proxy and the argument, and appropriately 
	initializing questioned features (where necessary). 
	
	@param argument					The argument, as packed nominal
	@param predicate				The predicate, as packed nominal
	@param packedNoms				The nominals in the packed logical form
	@returns ProxyFactoryResults	The results, including the proxy structure, excludes, and pending relations
	*/ 

	private ProxyFactoryResults applyQuestion (PackedNominal argument, PackedNominal WHRestr, Vector predicates, PackedLogicalForm plf, TreeMap packedNoms) { 
		// If non-null, pointer to the packed nominal for which an abstract property was given, e.g. "the location of..."
		String abstractArgument = null;

		TreeSet excludes = new TreeSet();
		ProxyFactoryResults results = new ProxyFactoryResults();
		//
		
		boolean factualQuestion;
		String whSort = null;
		
		if (WHRestr == null) { 
			factualQuestion = false; 
		} else { 
			factualQuestion = true; 
			whSort = getSort(WHRestr);		
		}
		String argSort = getSort(argument);
		String argProp = argument.prop.prop;

		// Establish the kind of argument, predicate

		
		log("Argument ["+argSort+"]:["+argProp+"]");
		
		// Create the proxy structure for the argument; this is the one we will extend
		ProxyFactoryResults argPRX = new ProxyFactoryResults();
		if (argSort.equals("thing")) { 
			ThingProxyFactory thingPF = new ThingProxyFactory();
			thingPF.setDiscRefs(_discRefsAccess);
			argPRX = thingPF.produce(argument,plf,packedNoms);
		} else if (argSort.equals("e-location") || argSort.equals("e-measure") || argSort.equals("e-place")) { 
			if (hasDependent(argument,"Owner")) { 
				// the location of the E-LOCATION
				PackedNominal owner = getDependentNominal(argument,"Owner",packedNoms);	
				if (getSort(owner).equals("e-location")) {
					ELocationProxyFactory elocPF = new ELocationProxyFactory();
					elocPF.setDiscRefs(_discRefsAccess);
					argPRX = elocPF.produce(owner,plf,packedNoms);
					excludes.add(argument.nomVar);
					abstractArgument = owner.nomVar;
				} else if (getSort(owner).equals("thing")) {
					ThingProxyFactory thingPF = new ThingProxyFactory();
					thingPF.setDiscRefs(_discRefsAccess);
					argPRX = thingPF.produce(owner,plf,packedNoms);	
					excludes.add(argument.nomVar);
					abstractArgument = owner.nomVar;					
				} else if (getSort(owner).equals("person")) { 
					// the X of PERSON Y
					ELocationProxyFactory elocPF = new ELocationProxyFactory();
					elocPF.setDiscRefs(_discRefsAccess);
					argPRX = elocPF.produce(argument,plf,packedNoms);
					excludes.add(argument.nomVar);
				} else { 
					log("Unknown type of argument under abstract e-location");
					System.exit(0);
				} 

			} else {
				ELocationProxyFactory elocPF = new ELocationProxyFactory();
				elocPF.setDiscRefs(_discRefsAccess);
				argPRX = elocPF.produce(argument,plf,packedNoms);		
			} // end if..else
		} else if (argSort.equals("person")) { 
			PersonProxyFactory personPF = new PersonProxyFactory();
			personPF.setDiscRefs(_discRefsAccess);
			argPRX = personPF.produce(argument,plf,packedNoms);
		} else if (argSort.equals("animate")) { 
			AnimateProxyFactory animatePF = new AnimateProxyFactory();
			animatePF.setDiscRefs(_discRefsAccess);
			argPRX = animatePF.produce(argument,plf,packedNoms);
		} else if (argSort.equals("entity")) { 	
			log("Argument with sort ENTITY has proposition ["+argProp+"]");
			// abstract contextual reference
			if (argProp.equals("context") || argProp.startsWith("wh")) { 
				EntityProxyFactory entityPF = new EntityProxyFactory();
				entityPF.setDiscRefs(_discRefsAccess);
				argPRX = entityPF.produce(argument,plf,packedNoms);
			// abstract descriptions: the X of Y
			} else if (hasDependent(argument,"Owner")) { 
				log("Dealing with restrictor type the X of Y");
				PackedNominal owner = getDependentNominal(argument,"Owner",packedNoms);
				String ownerSort = getSort(owner);
				if (ownerSort.equals("thing")) { 
					ThingProxyFactory thingPF = new ThingProxyFactory();
					thingPF.setDiscRefs(_discRefsAccess);
					argPRX = thingPF.produce(owner,plf,packedNoms);
					excludes.add(argument.nomVar);					
				} else if (ownerSort.equals("e-location")) { 
					ELocationProxyFactory elocPF = new ELocationProxyFactory();
					elocPF.setDiscRefs(_discRefsAccess);
					argPRX = elocPF.produce(owner,plf,packedNoms);					
					excludes.add(argument.nomVar);					
				} else if (ownerSort.equals("person")) { 
					PersonProxyFactory personPF = new PersonProxyFactory();
					personPF.setDiscRefs(_discRefsAccess);
					argPRX = personPF.produce(owner,plf,packedNoms);
					excludes.add(argument.nomVar);					
				} else if (ownerSort.equals("animate")) { 
					AnimateProxyFactory animatePF = new AnimateProxyFactory();
					animatePF.setDiscRefs(_discRefsAccess);
					argPRX = animatePF.produce(owner,plf,packedNoms);				
					excludes.add(argument.nomVar);					
				} else { 
					log("Unknown entity Owner construction in constructing argument proxy");
					System.exit(0);				
				} 
				abstractArgument = owner.nomVar;
			} else { 
				log("Unknown entity construction in constructing argument proxy");
				System.exit(0);
			} // end if..else
		} else { 
			log("Unknown type of argument for applying predicate to");
			System.exit(0);
		} // end if..else
		// Retrieve the proxy structure, excludes, pending relations
		proxyStructure = argPRX.getProxyStructure();
		proxyStructure.addContentStatus("indexical");
		pendingRels = argPRX.getPendingProxyRelations();
		excludes.addAll(argPRX.getExcludes());
		// Add the already covered nominals to the excludes
		excludes.addAll(argPRX.getAddedNominalsSet());		

		// check for context predication, apply properties
		for (Iterator<PackedNominal> predsIter = predicates.iterator(); predsIter.hasNext(); ) { 
			PackedNominal predicate = predsIter.next();
			String predSort = getSort(predicate);
			String predProp = predicate.prop.prop;
			if (predProp.equals("context")) { 
				excludes.addAll(getDependentNomVars(predicate,packedNoms));			
				// now establish the appropriate proximity, based on the Proximity feature 
				ArrayList<PackedFeature> packedFeats = new ArrayList<PackedFeature>(Arrays.asList(predicate.feats));
				Iterator featsIter = packedFeats.iterator();
				while (featsIter.hasNext()) { 
					PackedFeature pFeat = (PackedFeature) featsIter.next();
					String feature = pFeat.feat;
					if (feature.equals("Proximity")) { 
						proxyStructure.setFeature("Proximity",pFeat.value);
					} // end if.. check for proximity feature
				} // end while over features 	
				if (hasDependent(predicate,"Modifier")) { 
					for (Iterator<PackedNominal> modsIter = getDependentNominals(predicate,"Modifier",packedNoms).iterator(); modsIter.hasNext(); ) { 
						addProperty(modsIter.next());
					} 
				}
			} else if (predSort.startsWith("q-")) { 
				
				
			} else if (factualQuestion && predSort.equals("m-location")) { 
				log("Applying a spatial predicate to a question argument, LF-style");
				if (hasDependent(predicate,"Anchor")) { 
					// Get the anchor
					PackedNominal anchor = getDependentNominal(predicate,"Anchor",packedNoms);
					// Exclude the predicate and add a relation directly to the anchor
					excludes.add(predicate.nomVar);
					String anchorSort = getSort(anchor);
					// Initialize the spatial relation
					PendingProxyRelation ppr = new PendingProxyRelation();
					ppr.headNomVar = argument.nomVar;					
					// let the relation by default point to the anchor
					ppr.depNomVar = anchor.nomVar;	
					// Now initialize the spatial relation, collapsing things where necessary
					String spatrel = "";
					if (anchorSort.equals("thing")) {
						if (hasDependent(predicate,"Manner")) { 
							// the ball is right next to the star
							PackedNominal manner = getDependentNominal(predicate,"Manner",packedNoms);
							spatrel = "Position:"+manner.prop.prop;
						} else { 
							// the ball is right of the star
							spatrel = "Position:"+predProp;
						} // end if..else
					} else if (anchorSort.equals("e-region")) { 
						spatrel = "Position:"+anchor.prop.prop;
						if (hasDependent(anchor,"Owner")) { 
							excludes.add(anchor.nomVar);
							PackedNominal owner = getDependentNominal(anchor,"Owner",packedNoms);
							ppr.depNomVar = owner.nomVar;
						} // end if.. check whether "to the X of Y" construction
					} else { 
						// spatrel = anchor.prop.prop;
						spatrel = "Position:"+predicate.prop.prop;
					} // end if..else check for anchor type
					if (hasDependent(predicate,"Modifier")) {
						PackedNominal modifier = getDependentNominal(predicate,"Modifier",packedNoms);
						String modSort = getSort(modifier);
						if (modSort.equals("manner")) { 
							spatrel = spatrel+":"+modifier.prop.prop;
						} 
						excludes.add(modifier.nomVar);
					} // end if.. check for manner
					
					
					if (abstractArgument != null) { 
						ppr.headNomVar = abstractArgument;
					} 
					
					log("Putting in a new spatial relation of type ["+spatrel+"] to ["+ppr.depNomVar+"] from ["+ppr.headNomVar+"]");
					
					ppr.relMode = spatrel;
					pendingRels.addElement(ppr);
					proxyStructure.addRelation(ppr);
					
					log("Pending rels are now "+pendingRels);
					
				} else { 
					proxyStructure.setFeature("Proximity",predProp);					
					excludes.add(predicate.nomVar);
				} 
			}
		} // end for over predicates

		// now handle the question structure, 
		// make sure to differentiate between polar and factual questions!
		
		
		

		// Initialize the TYPE-Q and SPEAKER-KNOWS relations
		PendingProxyRelation typeQ = new PendingProxyRelation();
		PendingProxyRelation speakerQ = new PendingProxyRelation();
		
		if (factualQuestion) { 
			typeQ.headNomVar = WHRestr.nomVar; 
			typeQ.relMode = "Fact-Q";
			typeQ.addContentStatus("intentional");
			speakerQ.headNomVar = WHRestr.nomVar; 
			speakerQ.addContentStatus("intentional");
		} else { 	
			typeQ.headNomVar = argument.nomVar; 
			typeQ.relMode = "Polar-Q";
			typeQ.addContentStatus("intentional");
			speakerQ.headNomVar = argument.nomVar; 
			speakerQ.addContentStatus("intentional");		
		}

		if (abstractArgument != null) { 
			typeQ.depNomVar = abstractArgument;
			speakerQ.depNomVar = abstractArgument;
		} else { 
			speakerQ.depNomVar = argument.nomVar;
			typeQ.depNomVar = argument.nomVar;
		}

		
		if (factualQuestion) { 
			// Establish question type
			if (whSort.equals("specifier")) {
				// Specifiers have another entity in the scope, identifying the property -- "WH PROPERTY ..." over which is scopes
				PackedNominal whscope = getDependentNominal(WHRestr,"Scope",packedNoms);
				// EXCLUDE the predicate and the scope of the WH specifier
				if (WHRestr.pEdges != null) { 
					ArrayList<PackingEdge> peEdges = new ArrayList<PackingEdge>(Arrays.asList(WHRestr.pEdges));
					Iterator peIter = peEdges.iterator();
					while (peIter.hasNext()) { 
						PackingEdge packingEdge = (PackingEdge) peIter.next();
						if (packingEdge.mode.equals("Scope")) { 
							ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
							if (targets.size() > 0) { 
								for(Iterator targetIter = targets.iterator(); targetIter.hasNext(); ) { 
									PackingNodeTarget target = (PackingNodeTarget) targetIter.next();
									String targetNV = target.pnId.substring(0,target.pnId.indexOf("_PN"));
									excludes.add(targetNV);
								}
							}	
						}	
					}
				} else { 
					excludes.add(whscope.nomVar);
				} 
				// Get the relevant info
				String whProp = WHRestr.prop.prop;
				String scopeProp = whscope.prop.prop;
				String scopeSort = getSort(whscope);
				if (scopeProp.equals("color")) { 
					// what COLOR is this SOMETHING				
					proxyStructure.updateFeature("Colour","?colour");
					speakerQ.relMode="SPEAKER-KNOWS:Colour";
				} else if (scopeProp.equals("size")) { 
					// what SIZE is this SOMETHING			
					proxyStructure.updateFeature("Size","?size");
					speakerQ.relMode="SPEAKER-KNOWS:Size";
				} else if (scopeProp.equals("shape")) { 
					// what SHAPE is this SOMETHING
					proxyStructure.updateFeature("Shape","?shape");
					speakerQ.relMode="SPEAKER-KNOWS:Shape";
				} else if (scopeSort.startsWith("q-")) { 
					// what is PROPERTY?
					speakerQ.depNomVar = whscope.nomVar;
					typeQ.depNomVar = whscope.nomVar;
					
				} else if (scopeSort.equals("e-measure") || scopeSort.equals("thing") || scopeSort.equals("e-place")) { 
					if (whProp.equals("what")) { 
						// what kind of thing is this, what kind of ball is this
						if (abstractArgument != null) { 
							log("Abstract argument, with proposition ["+argProp+"] and sort ["+argSort+"]");
							PackedNominal owner = getDependentNominal(argument,"Owner",packedNoms);	
							String ownerProp = owner.prop.prop;	
							speakerQ.relMode="SPEAKER-KNOWS:Identity";
							proxyStructure.updateFeature("Concept","?"+ownerProp);
						} else { 
							speakerQ.relMode="SPEAKER-KNOWS:Identity";
							proxyStructure.updateFeature("Concept","?"+argProp);
						}	
						speakerQ.relMode="SPEAKER-KNOWS:Concept";						
					} else if (whProp.equals("which") && scopeSort.equals("thing")) { 
						proxyStructure.updateFeature("Delimitation","?identification");
						speakerQ.relMode="SPEAKER-KNOWS:Identity";
					} else if (whProp.equals("which") && scopeSort.equals("e-place")) { 
						proxyStructure.updateFeature("Delimitation","?identification");
						speakerQ.relMode="SPEAKER-KNOWS:Identity";
					} 
				} else {
					System.out.println("Unknown type of WH specifier: "+scopeProp);
					System.exit(0);
				} // end if..else
			} else if (whSort.equals("entity")) { 
				if (abstractArgument != null) { 
					log("Abstract argument, with proposition ["+argProp+"] and sort ["+argSort+"]");
					PackedNominal owner = getDependentNominal(argument,"Owner",packedNoms);	
					String ownerProp = owner.prop.prop;
					// what is the PROPERTY of the SOMETHING
					if (argProp.equals("color")) { 
						// what COLOR is this SOMETHING				
						proxyStructure.updateFeature("Colour","?colour");
						speakerQ.relMode="SPEAKER-KNOWS:Colour";
					} else if (argProp.equals("size")) { 
						// what SIZE is this SOMETHING			
						proxyStructure.updateFeature("Size","?size");
						speakerQ.relMode="SPEAKER-KNOWS:Size";
					} else if (argProp.equals("shape")) { 
						// what SHAPE is this SOMETHING
						proxyStructure.updateFeature("Shape","?shape");
						speakerQ.relMode="SPEAKER-KNOWS:Shape";
					} else if (argSort.equals("thing")) { 
						// what kind of thing is this, what kind of ball is this
						proxyStructure.updateFeature("Concept","?"+ownerProp);
						speakerQ.relMode="SPEAKER-KNOWS:Concept";
					} else {
						System.out.println("Unknown type of abstract PROPERTY: "+argProp);
						System.exit(0);
					} // end if..else
				} else { 
					log("Standard argument, with proposition ["+argProp+"] and sort ["+argSort+"]");
					if (argSort.equals("entity")) { 
						// what is this
						proxyStructure.updateFeature("Concept","?"+argProp);
						speakerQ.relMode="SPEAKER-KNOWS:Concept";			
					}
					PackedNominal predicate = (PackedNominal) predicates.firstElement();
					String scopeSort = getSort(predicate);
					if (scopeSort.startsWith("q-")) { 
						// what is PROPERTY?
						speakerQ.depNomVar = predicate.nomVar;
						typeQ.depNomVar = predicate.nomVar;
					}
				} // end if..else check for abstract or not
			} else if (whSort.equals("m-location")) { 
				speakerQ.relMode="SPEAKER-KNOWS:Position";			
			} else {
				System.out.println("Unknown type of question");
				System.exit(0);
			} // end if..else check for WH type
		} else { 
			// we are dealing with a polar question 
			// first, apply the predication
			
			// check whether we are dealing with identical argument and predicate sorts, 
			// without it being an abstractArgument; if so, do not apply! 
			PackedNominal prd = (PackedNominal) predicates.firstElement();
			String prdsort = getSort(prd);
			String argsort = getSort(argument);
			if (prdsort.equals(argsort) && abstractArgument == null) { 
				log("Do not apply the predicate to the argument!");
				speakerQ.relMode = "SPEAKER-KNOWS:Concept";
				speakerQ.depNomVar = prd.nomVar;
				typeQ.depNomVar = prd.nomVar;
			} else { 
				log("Pred sort ["+prdsort+"] and argument sort ["+argsort+"] different under polar, so apply");
				ProxyFactoryResults predication = apply(argument, abstractArgument, predicates,  plf,  packedNoms); 
				excludes.addAll(predication.getExcludes());
				pendingRels.addAll(predication.getPendingProxyRelations());	
			}
		
			// we need to update the Speaker-Knows relation, depending on the predicate / scope
			
			for (Iterator<PackedNominal> predsIter = predicates.iterator(); predsIter.hasNext(); ) { 
				PackedNominal predicate = predsIter.next();
				String scopeProp =	getSort(predicate);
				
				log("Looking at scope / predicate proposition: ["+scopeProp+"]");
				
				if (scopeProp.equals("m-location")) { 
					speakerQ.relMode="SPEAKER-KNOWS:Position";
				} else if (scopeProp.equals("q-color")) { 
					speakerQ.relMode="SPEAKER-KNOWS:Colour";
				} else if (scopeProp.equals("q-shape")) { 
					speakerQ.relMode="SPEAKER-KNOWS:Shape";
				} else if (scopeProp.equals("q-size")) { 
					speakerQ.relMode="SPEAKER-KNOWS:Size";
				} else if (scopeProp.equals("q-state")) { 
					speakerQ.relMode="SPEAKER-KNOWS:State";
				} else if (scopeProp.equals("entity")) {
					String delimitation = LFUtils.plfGetFeatureValue(argument,"Delimitation"); 
					if (delimitation != null) { 
						if (delimitation.equals("unique")) { 
							speakerQ.relMode="SPEAKER-KNOWS:Identity";					
						} else { 
							speakerQ.relMode="SPEAKER-KNOWS:Concept?"+argument.prop.prop;												
						} 
					}
				} 
			} // end for over predicates
			
		} // end if..else checking for 
			
		pendingRels.addElement(speakerQ);
		proxyStructure.addRelation(speakerQ);	
		pendingRels.addElement(typeQ);
		proxyStructure.addRelation(typeQ);	

		// Set the results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRels);
		results.setProxyStructure(proxyStructure);
		results.setAddedNominals(argPRX.getAddedNominalsSet());
		
		log("Proxy structure after applying question:\n"+results.getProxyStructure().toString());
		log("Pending proxy relations:\n"+results.getPendingProxyRelations().toString());
		log("Excludes: "+results.getExcludes().toString());		
		log("Added nominals: "+results.getAddedNominalsSet().toString());

		return results; 

	}






    //=================================================================
    // MAIN METHOD
    //=================================================================

	/** 
	The method <i>produceProxies</i> is the main factory method for producing local proxy structures based on the given nominal. 
	*/ 

	public ProxyFactoryResults produceProxies (PackedNominal nom, PackedLogicalForm plf, TreeMap packedNoms) 
	
	{  
		log("starting produceProxies");
		// Initialize the variables inherited from the Abstract factory
		proxyStructure = new LocalProxyStructure();
		pendingRels = new Vector<PendingProxyRelation>();
		// Initialize the local excludes set
		TreeSet excludes = new TreeSet();
		// Initialize the return result
		ProxyFactoryResults result = new ProxyFactoryResults();
		String mood = LFUtils.plfNominalGetFeature(nom,"Mood");
		if (mood.equals("int")) { 
			log("Interrogative ascription");
			// Establish the argument of the ascription construction 
			PackedNominal whrestr = getDependentNominal(nom,"Wh-Restr",packedNoms);
			if (whrestr == null) { 
				log("Polar question");
			} else {
				log("Factual question");
			}
			// Establish the argument of the ascription construction 
			PackedNominal argument = retrieveArgument(nom,packedNoms);
			// Establish the predicate of the ascription construction
			Vector predicates = retrievePredicate(nom,packedNoms);
			// Apply the predicate, and construct proxy structures etc. returned as factory results
			ProxyFactoryResults proposition = applyQuestion(argument,whrestr, predicates,plf,packedNoms);
			// Transfer the factory results to the (global) variables
			excludes = proposition.getExcludes();
			proxyStructure = proposition.getProxyStructure();
			proxyStructure.addContentStatus("intentional");
			pendingRels = proposition.getPendingProxyRelations();
			// Exclude further modifiers
			log("Checking for modifiers under the ascription root node ["+nom.nomVar+"]");
			if (hasDependent(nom,"Modifier")) { 
				for (Iterator<PackedNominal> modsIter = getDependentNominals(nom,"Modifier",packedNoms).iterator(); modsIter.hasNext(); ) { 
					PackedNominal modifier = modsIter.next();
					log("Excluding modifier ["+modifier.nomVar+"]:["+getSort(modifier)+"]:["+modifier.prop.prop+"]");
					excludes.add(modifier.nomVar);
				} // end for
			} else { 
				log("No modifiers found under the root node");
			} // end if.. check for modifiers
			// Set the results to be returned
			result.setExcludes(excludes);
			result.setPendingProxyRelations(pendingRels);
			result.setProxyStructure(proxyStructure);
			result.setAddedNominals(proposition.getAddedNominalsSet());
		} else if (mood.equals("ind")) {
			log("Indicative ascription");
		
			
			// Establish the argument of the ascription construction 
			PackedNominal argument = retrieveArgument(nom,packedNoms);
			// Establish the predicate of the ascription construction
			Vector predicates = retrievePredicate(nom,packedNoms);

			String argSort = getSort(argument);
			if (nom.prop.prop.equals("have") && argSort.equals("person")) {  
				PossessiveHaveProxyFactory sillyPF = new PossessiveHaveProxyFactory();
				sillyPF.setDiscRefs(_discRefsAccess);
				ProxyFactoryResults sillyResults = sillyPF.produceProxies(nom,plf,packedNoms);
				// Fuse with the resulting proxy structure features
				proxyStructure.fuse(sillyResults.getProxyStructure());
				proxyStructure.addContentStatus("indexical");
				proxyStructure.addContentStatus("intentional");				
				// Update the excludes
				excludes.addAll(sillyResults.getExcludes());
				// Update the relations
				pendingRels.addAll(sillyResults.getPendingProxyRelations());
				proxyStructure.addRelations(sillyResults.getPendingProxyRelations().iterator());
				result.setAddedNominals(sillyResults.getAddedNominalsSet());
			} else { 
				// Apply the predicate, and construct proxy structures etc. returned as factory results
				ProxyFactoryResults proposition = applyPredicate(argument,predicates,plf,packedNoms);
				// Transfer the factory results to the (global) variables
				excludes = proposition.getExcludes();
				proxyStructure = proposition.getProxyStructure();
				pendingRels = proposition.getPendingProxyRelations();
				// Exclude further modifiers
				log("Checking for modifiers under the ascription root node ["+nom.nomVar+"]");
				if (hasDependent(nom,"Modifier")) { 
					for (Iterator<PackedNominal> modsIter = getDependentNominals(nom,"Modifier",packedNoms).iterator(); modsIter.hasNext(); ) { 
						PackedNominal modifier = modsIter.next();
						log("Excluding modifier ["+modifier.nomVar+"]:["+getSort(modifier)+"]:["+modifier.prop.prop+"]");
						excludes.add(modifier.nomVar);
					} // end for
				} else { 
					log("No modifiers found under the root node");
				} // end if.. check for modifiers
				result.setAddedNominals(proposition.getAddedNominalsSet());
			} // end if.. check for possessive use of ascription
				
			// Set the results to be returned
			result.setExcludes(excludes);
			result.setPendingProxyRelations(pendingRels);
			result.setProxyStructure(proxyStructure);
		} // end if..else check for mood

		// Return the result
		return result;
	} // end produceProxies







} // end class 