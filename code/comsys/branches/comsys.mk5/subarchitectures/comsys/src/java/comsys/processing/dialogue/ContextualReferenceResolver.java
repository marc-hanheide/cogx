//=================================================================
//Copyright (C) 2005,2006 Geert-Jan M. Kruijff (gj@acm.org)

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

package comsys.processing.dialogue;

//=================================================================
//IMPORTS
//=================================================================

//JAVA
import java.util.Iterator;
import java.util.Vector;

import comsys.arch.*;
import comsys.datastructs.CacheWrapper;
import comsys.processing.uttplan.GREAttributeTemplateFiller;
import comsys.datastructs.comsysEssentials.*;
import comsys.datastructs.lf.*;
import comsys.lf.utils.LFUtils;
import comsys.utils.SDRSUtils;


//=================================================================
//JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
    The class <b>ContextualReferenceResolver</b> implements algorithms
    for resolving various types of contextual references: 

    <ul>
    <li> Pronominal references: "it" gets a dependent <i>Ctxt</i>
         pointing to the antecedent</li>
    <li> Definite descriptions: "the X" </li> 
    </ul>

    The algorithms work off a model of the dialogue context, against
    which a bound LF (with all variables fresh in the given dialogue
    context model) needs to be resolved. 

    @version 071111 (started 051003)
    @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
//CLASS DEFINITION 
//=================================================================

public class ContextualReferenceResolver  {


	//=================================================================
	// CLASS-INTERNAL GLOBAL VARIABLES
	//=================================================================

	boolean logging = true; 

	SDRS ctxtmodel; 

	static int counter = 0;

	// Counter for DiscRefBindings identifiers
	private int bindingsIdCounter;

	// cache with the nominals already resolved
	Cache uncommittedCache ;

	// identifiers of the nominals already resolved
	Vector<String> resolvedNominals  ;

	CacheWrapper bindingsWrapper;

	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================

	/** 
	 *  The basic constructor initializes the internal variables
	 */

	public ContextualReferenceResolver () {
		init();
	} // end constructor

	/** 
	The unary constructor sets the dialogue model that resolution
	works on, and initializes the remaining variables.
	 */

	public ContextualReferenceResolver (SDRS md) { 
		init();
		ctxtmodel = md; 
	} // end constructor

	public void init() { 
		ctxtmodel = new SDRS();
		bindingsWrapper = null;
	} // end init

	//=================================================================
	// ACCESSOR METHODS
	//=================================================================

	/**
        The method <i>setContextModel</i> sets the context model
        against that resolution works on. 
	 */

	public void setContextModel (SDRS md) { ctxtmodel = md; }

	public void setUncommittedCache (Cache uncommittedCache) { 
		this.uncommittedCache = uncommittedCache;
		resolvedNominals = new Vector<String>();
		if (uncommittedCache != null && uncommittedCache.content2 != null) {
			for (int i=0; i < uncommittedCache.content2.length; i++) {
				String el = uncommittedCache.content2[i];
				resolvedNominals.add(el);
			}
		}
	}


	public void setLogging (boolean l) { logging = l; }


	//=================================================================
	// COMPUTATION METHODS
	//=================================================================

	/**
       The method <i>resolve</i> takes a BoundLF object, and resolves
       any contextual references in the logical form stored in the
       object. The method returns a new BoundLF object in which the 
       bindings (if any) are reflected. 
       <p>

       The method cycles over the nominals in the logical form,
       currently of type "thing", and checks for each of them whether
       we should perform resolution. Resolution yields the following:

       <ul> 
       <li> 

       </ul> 

	 */ 

	int increment = 0;

	public Cache resolve (PackedLFs packedLf) { 

		increment++;

		// create the entry for working memory
		Cache bindings = new Cache() ;
		bindings.CacheId = "discRef";
		bindings.plf = packedLf;

		bindingsWrapper = new CacheWrapper(bindings);

		PackedLogicalForm plf = packedLf.packedLF ;    	
		for (int i=0; i < plf.pNodes.length ; i++) {
			for (int j=0; j < plf.pNodes[i].packedNoms.length ; j++) {
				PackedNominal thingnom = plf.pNodes[i].packedNoms[j];
				Proposition nomProp = thingnom.prop;
				String sort = LFUtils.getPackedNominalSort(thingnom);
				String nomPropVal = nomProp.prop; 
				log("Looking at nomVar ["+thingnom.nomVar+"] with sort ["+sort+"] and proposition ["+nomPropVal+"]");
				if (!resolvedNominals.contains(thingnom.nomVar)) {
					if (sort.equals("thing")) { 
						if (LFUtils.plfNominalHasFeature(thingnom, "Delimitation")) { 
							String fValue = LFUtils.plfNominalGetFeature(thingnom, "Delimitation");
							if (fValue.equals("unique") && !nomPropVal.equals("ant")) { 
								log("Unique determination: "+ thingnom.nomVar ) ;
								String anchorNV = this.resolveUniqueNominalGroup(thingnom,plf.packedLFId);
								if (anchorNV.equals("unknown")) { 
									anchorNV = thingnom.nomVar; 
									log("Reference for definite noun resolved to NEW discourse referent ["+anchorNV+"] ");				
								} else { 
									log("Reference for definite noun resolved to OLD discourse referent ["+anchorNV+"] ");
								} 

								bindingsWrapper.addCacheAssociation(anchorNV, thingnom.nomVar);
							} else {
								// indefinite determiner
								log("Existential determination: "+ thingnom.nomVar) ;
								bindingsWrapper.addCacheAssociation(thingnom.nomVar, thingnom.nomVar);							
							} // end if..else check for determination
						} 


						else if (nomPropVal.equals("it")) {
							log("Pronominal reference: " + thingnom.nomVar);
								String anchorNV = this.resolvePronominalRef(plf);

							if (!anchorNV.equals("unknown")) {
								bindingsWrapper.addCacheAssociation(anchorNV, thingnom.nomVar);
								log("Reference for 'it' resolved to ["+anchorNV+"] ");
							} else { 
								bindingsWrapper.addCacheAssociation(thingnom.nomVar, thingnom.nomVar);
								log("Unresolvable reference for 'it', resolved to itself ["+thingnom.nomVar+"] ");						
							} // end if.. check for anchor found
						} 
						else {
							bindingsWrapper.addCacheAssociation(thingnom.nomVar, thingnom.nomVar);
						}
					} // end if check for things 
					else if (sort.equals("deictic-pronoun")) { 
						log("Pronominal reference: " + thingnom.nomVar+" ["+nomPropVal+"]");
						String anchorNV = this.resolvePronominalRef(plf);
						if (!anchorNV.equals("unknown")) {
							bindingsWrapper.addCacheAssociation(anchorNV, thingnom.nomVar);
							log("Reference for pronoun resolved to ["+anchorNV+"] ");
						} else { 
							bindingsWrapper.addCacheAssociation(thingnom.nomVar, thingnom.nomVar);
							log("Unresolvable reference for 'this', resolved to itself ["+thingnom.nomVar+"] ");								
						} // end if.. check for anchor found
					} // end if check for deictic pronouns
					else {
						if (!thingnom.nomVar.contains("rootNom")) {
							log("Adding an association using the same nomvar: ["+thingnom.nomVar+"]");
							bindingsWrapper.addCacheAssociation(thingnom.nomVar, thingnom.nomVar);
						} // end if.. check for whatever is left
					} // end if..else checking for sorts
				} else {
					log("Adding an association with an already resolved reference");
					bindingsWrapper.addCacheAssociation((new CacheWrapper(uncommittedCache)).getDiscRef(thingnom.nomVar), thingnom.nomVar);
				} // end if.. else check whether already resolved
			} // end for over packing nominals at a packing node
		} // end while over packing nodes
		log("Discourse referents resolution done!");
		return bindingsWrapper.getCache();
	} // end resolve

	/** 
	The method <i>resolveAgainstAssertionLF</i> tries to resolve a
	reference <tt>ref</tt> against the utterance logical form
	<tt>lf</tt>. If succesful, an identifier is returned;
	otherwise, "unknown".
	 */ 

	public String resolveAgainstAssertionLF (PackedLogicalForm ref, PackedLogicalForm lf) { 
		String result = "unknown";
		// HACK - plison
		for (int i=0; i < lf.pNodes.length ; i++) {
			for (int j=0; j < lf.pNodes[i].packedNoms.length ; j++) {
				for (int k=0 ; k < lf.pNodes[i].packedNoms[j].packedSorts.length ; k++) {
					if (lf.pNodes[i].packedNoms[j].packedSorts[k].sort.equals("thing")) {
						if (LFUtils.plfNominalHasFeature(lf.pNodes[i].packedNoms[j], "Delimitation")) { 
							String fValue = LFUtils.plfNominalGetFeature(lf.pNodes[i].packedNoms[j], "Delimitation");
							if (fValue.equals("existential") || fValue.equals("unique")) { 
								log("Found an appropriate reference for the contextual resolution!");
								return lf.pNodes[i].packedNoms[j].nomVar ;
							}
						}
					}
				}
			}
		}
		return result;
	} // end 


	/**
       The method <i>resolvePronominalRef</i> tries to find an
       antecedent for a pronominal reference, given a dialogue
       context. It will return the nominal variable of the nearest
       available anchor.
	 */

	public String resolvePronominalRef (PackedLogicalForm ref) { 
		String result = "unknown"; 
		// Start with the LAST utterance
		String LOCUS = SDRSUtils.getLAST(ctxtmodel);
		log("resolution of pronominal reference");
		while (!LOCUS.equals("none")) { 
			SDRSFormula LOCUSFormula = SDRSUtils.getFormula(ctxtmodel, LOCUS);
			log("locus label: " + LOCUSFormula.label);
			// In case the formula in locus is precisely the one we're analysing,
			// we take the previous one
			if (SDRSUtils.getFormulaType(LOCUSFormula).type.equals(SDRSUtils.PLF_TYPE) &&
					LOCUSFormula.type.plf.packedLF.packedLFId.equals(ref.packedLFId)) {
				LOCUS = LOCUSFormula.tprec ;
				LOCUSFormula = SDRSUtils.getFormula(ctxtmodel, LOCUS);
			}
			else if (LOCUSFormula != null) {
				if (SDRSUtils.getFormulaType(LOCUSFormula).type.equals(SDRSUtils.PLF_TYPE)) {
					PackedLogicalForm plf = LOCUSFormula.type.plf.packedLF ;
					if (plf != null) { 
						String resolution = this.resolveAgainstAssertionLF(ref,plf); 
						if (!resolution.equals("unknown")) { 
							log("reference found: " + resolution);
							result = SDRSUtils.getRefNomvar(LOCUSFormula, resolution);	
							LOCUS = "none";
						} // end if
					} // end if.. check for lf
				}
				else {
					result = "unknown";
				}
				// If not LAST, go back over ELABORATE to PARALLEL; then over PARALLEL
				// Currently, we just take the LAST of the previous utterance .. 
				if (result.equals("unknown")) {
					LOCUS = SDRSUtils.getFormula(ctxtmodel, LOCUS).tprec ;
					log("no reference found, go one utterance backwards");
				}
				else {
					LOCUS = "none";
				}
			}
			else {
				log("LOCUS formula not found");
				LOCUS = "none";
			}
		} // end while over locus 	
		log("Returning as referent for pronominal reference: "+result);
		return result;
	} // end resolvePronominalRef



	/**
    The method <i>resolvePronominalRef</i> tries to find an
    antecedent for a pronominal reference, given a dialogue
    context. It will return the nominal variable of the nearest
    available anchor.
	 */

	public String resolveReferenceByProfile (PackedLogicalForm ref, 
			GREAttributeTemplateFiller template) { 
		String result = "unknown"; 
		// Start with the LAST utterance
		String LOCUS = SDRSUtils.getLAST(ctxtmodel);
		log("resolution of pronominal reference");
		while (!LOCUS.equals("none")) { 
			SDRSFormula LOCUSFormula = SDRSUtils.getFormula(ctxtmodel, LOCUS);
			log("locus label: " + LOCUSFormula.label);
			// In case the formula in locus is precisely the one we're analysing,
			// we take the previous one
			if (SDRSUtils.getFormulaType(LOCUSFormula).type.equals(SDRSUtils.PLF_TYPE) &&
					LOCUSFormula.type.plf.packedLF.packedLFId.equals(ref.packedLFId)) {
				LOCUS = LOCUSFormula.tprec ;
				LOCUSFormula = SDRSUtils.getFormula(ctxtmodel, LOCUS);
			}
			else if (LOCUSFormula != null) {
				if (SDRSUtils.getFormulaType(LOCUSFormula).type.equals(SDRSUtils.PLF_TYPE)) {
					PackedLogicalForm plf = LOCUSFormula.type.plf.packedLF ;
					if (plf != null) { 
						String resolution = this.resolveRefAgainstPLFByProfile(ref,plf, template); 
						if (!resolution.equals("unknown")) { 
							log("reference found: " + resolution);
							result = SDRSUtils.getRefNomvar(LOCUSFormula, resolution);	
							LOCUS = "none";
						} // end if
					} // end if.. check for lf
				}
				else {
					result = "unknown";
				}
				// If not LAST, go back over ELABORATE to PARALLEL; then over PARALLEL
				// Currently, we just take the LAST of the previous utterance .. 
				if (result.equals("unknown")) {
					LOCUS = SDRSUtils.getFormula(ctxtmodel, LOCUS).tprec ;
					log("no reference found, go one utterance backwards");
				}
				else {
					LOCUS = "none";
				}
			}
			else {
				log("LOCUS formula not found");
				LOCUS = "none";
			}
		} // end while over locus 	
		log("Returning as referent for pronominal reference: "+result);
		return result;
	} // end resolvePronominalRef


	public String resolveRefAgainstPLFByProfile (PackedLogicalForm ref, PackedLogicalForm lf, 
			GREAttributeTemplateFiller template) { 
		String result = "unknown";
		// HACK - plison
		for (int i=0; i < lf.pNodes.length ; i++) {
			for (int j=0; j < lf.pNodes[i].packedNoms.length ; j++) {
				PackedNominal pNom = lf.pNodes[i].packedNoms[j];
				for (int k=0 ; k < pNom.packedSorts.length ; k++) {
					if (pNom.packedSorts[k].sort.equals("thing")) {
						if (LFUtils.plfNominalHasFeature(pNom, "Delimitation")) { 
							String fValue = LFUtils.plfNominalGetFeature(pNom, "Delimitation");
							if (fValue.equals("existential") || fValue.equals("unique")) { 
								log("Found an appropriate reference for the contextual resolution!");
								log("now checking the profile...");

								boolean attributesOK = true;
								
								for (Iterator<GREAttributeTemplateFiller.GREAttribute> it = 
									
									template.getAttributes().iterator() ; it.hasNext(); ) {
									GREAttributeTemplateFiller.GREAttribute attr = it.next();
									log("attribute currently analysed: " + attr);
									
									LFRelation modifier = LFUtils.plfGetRelation(pNom, "Modifier");
									if (modifier != null) {
										PackedNominal depNom = LFUtils.plfGetPackedNominal(lf, modifier.dep);
										
										if (!template.getAttributeValue(attr).equals(depNom.prop.prop)) {
											attributesOK = false;
										}
									}
									else {
										attributesOK = false;
									}
								}
								if (attributesOK) {
									String nomvar = pNom.nomVar ;
									log("nomvar returned: " + nomvar);
									return nomvar;
								}
							}
						}
					}
				}
			}
		}
		return result;
	} // end 



	public String resolveAnaphorAgainstLF (PackedNominal ref, PackedLogicalForm lf) { 
		String result = "unknown";
		String refSort = LFUtils.getPackedNominalSort(ref);
		String refProp = ref.prop.prop;
		// HACK - plison
		for (int i=0; i < lf.pNodes.length ; i++) {
			for (int j=0; j < lf.pNodes[i].packedNoms.length ; j++) {
				for (int k=0 ; k < lf.pNodes[i].packedNoms[j].packedSorts.length ; k++) {
					if (lf.pNodes[i].packedNoms[j].packedSorts[k].sort.equals(refSort)) {
						if (lf.pNodes[i].packedNoms[j].prop.prop.equals(refProp)) { 
							log("Found an appropriate reference for the contextual resolution!");
							return lf.pNodes[i].packedNoms[j].nomVar ;
						} // end if.. check for matching proposition
					} // end if .. check for matching sort
				} // end for.. over packed sorts on a packing nominal
			} // end for.. over packed nominals on a packing node
		} // end for.. over packingnodes in a packed logical form
		return result;
	} // end resolveAnaphorAgainstLF




	/** 
		The method <i>resolveUniqueNominalGroup</i> takes a packed nominal, 
		which has a unique determination, and tries to match it with a 
		nominal in a previous utterance of the same type and proposition. 
	 */ 

	public String resolveUniqueNominalGroup (PackedNominal ref, String packedLFId) { 
		String result = "unknown"; 
		// Start with the LAST utterance
		String LOCUS = SDRSUtils.getLAST(ctxtmodel);
		while (!LOCUS.equals("none")) { 
			SDRSFormula LOCUSFormula = SDRSUtils.getFormula(ctxtmodel, LOCUS);
			// In case the formula in locus is precisely the one we're analysing,
			// we take the previous one
			if (SDRSUtils.getFormulaType(LOCUSFormula).type.equals(SDRSUtils.PLF_TYPE) &&
					LOCUSFormula.type.plf.packedLF.packedLFId.equals(packedLFId)) {
				LOCUS = LOCUSFormula.tprec ;
				LOCUSFormula = SDRSUtils.getFormula(ctxtmodel, LOCUS);
			}
			else if (LOCUSFormula != null)  {
				if (SDRSUtils.getFormulaType(LOCUSFormula).type.equals(SDRSUtils.PLF_TYPE)) {
					PackedLogicalForm plf = LOCUSFormula.type.plf.packedLF ;
					if (plf != null) { 
						String resolution = this.resolveAnaphorAgainstLF(ref,plf); 
						if (!resolution.equals("unknown")) { 
							result = SDRSUtils.getRefNomvar(LOCUSFormula, resolution);						
						} // end if
					} // end if.. check for lf
				}
				// If not LAST, go back over ELABORATE to PARALLEL; then over PARALLEL
				// Currently, we just take the LAST of the previous utterance .. 
				if (result.equals("unknown")) {
					LOCUS = SDRSUtils.getFormula(ctxtmodel, LOCUS).tprec ;
				}
				else {
					LOCUS = "none";
				}
			}
			else {
				log("Problem: LOCUS formula not found");
				LOCUS = "none";
			}
		} // end while over locus 	
		log("Returning as referent for unique nominal reference: "+result);
		return result;
	} // end resolveUniqueNominalGroup



	/**
       The method <i>isContentEligible</i> checks whether a given
       logical form <tt>anc</tt> provides an eligible anchor for the
       reference <tt>ref</tt>, given the content. The method returns a
       boolean.

       <h4>Note</h4>

       Spatial demonstratives do not count as antecedents; in
       e.g. "This is a box", only "box" would be a proper antecedent
       for "it". 

       <h4>Warning</h4>

       Currently, we only check for type. We will add checking against
       features added to the pronominal reference to establish whether
       an anchor is appropriate.
	 */ 

	public boolean isContentEligible (PackedLogicalForm ref, PackedLogicalForm anc) { 
		boolean result = false;
		PackedNominal refRoot = LFUtils.plfGetRoot(ref);
		PackedNominal ancRoot = LFUtils.plfGetRoot(anc);
		String refType = refRoot.packedSorts[0].sort ;
		String ancType = ancRoot.packedSorts[0].sort ;
		String refProp = refRoot.prop.prop;
		String ancProp = ancRoot.prop.prop;
		if (refType.equals(ancType) || ancType.equals("location")) { 
			if (LFUtils.plfNominalHasFeature(ancRoot,"VisCtxt") || LFUtils.plfNominalHasFeature(ancRoot,"Ctxt")) { 

			} else { 
				if (!LFUtils.plfNominalHasFeature(refRoot,"Delimitation")) { 
					result = true;
				} else if (refProp.equals(ancProp)) { 
					result = true;
				} // end if.. check whether np ref or pron ref
			} // end if..else do checks
		} // end if
		return result;
	} // end isContentEligible




	//=================================================================
	// I/O METHODS
	//=================================================================


	private String newDRBindingsId() {
		String result = "" + bindingsIdCounter + "";
		bindingsIdCounter++;
		return result;
	} // end newDRBindingsId



	//=================================================================
	// MISCELLANEOUS METHODS
	//=================================================================

	private void log (String m) {
		if (logging) { System.out.println("[RefRes] "+m); }
	}


	//=================================================================
	// MAIN METHOD
	//=================================================================


} // end class definition 
