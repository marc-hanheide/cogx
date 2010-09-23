
// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
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


package org.cognitivesystems.binder;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;

import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.data.formulas.DoubleFormula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * Simple CAST component listening to the local working memory for pairs of beliefs
 * which are (1) respectively private and attributed, and (2) contain identical features.  
 * If such pair of belief is found, a new shared belief is created with the subsumed 
 * feature(s), pointing to the private belief.
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 23/08/2010
 * 
 */
public class SharedBeliefCreator extends ManagedComponent {

  
	 
	/*
	 * Starting up the CAST component by registering filters on the insertion or modification
	 * of attributed beliefs
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {

			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(dBelief.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								dBelief belief = getMemoryEntry(_wmc.address, dBelief.class);
								if (belief.estatus instanceof AttributedEpistemicStatus) {
									checkMatchWithPrivateBelief(belief);
								}
							} catch (DoesNotExistOnWMException e) {
								e.printStackTrace();
							} catch (UnknownSubarchitectureException e) {
								e.printStackTrace();
							}
						}
					});
			 
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(dBelief.class,  WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								dBelief belief = getMemoryEntry(_wmc.address, dBelief.class);
								if (belief.estatus instanceof AttributedEpistemicStatus) {
									checkMatchWithPrivateBelief(belief);
								}
							} catch (DoesNotExistOnWMException e) {
								e.printStackTrace();
							} catch (UnknownSubarchitectureException e) {
								e.printStackTrace();
							}
						}
					});	

	}

	
	/**
	 * Fires when a new attributed belief has been detected on the local
	 * working memory
	 * 
	 * @param belief the attributed belief
	 */
	protected void checkMatchWithPrivateBelief(dBelief attrBelief) {
		
		if (!(attrBelief.content instanceof CondIndependentDistribs)) {
			return ;
		}
		
		try {
			CASTData<dBelief>[] all = getWorkingMemoryEntries (dBelief.class);
			
			for (CASTData<dBelief> d : all) {
				dBelief belief2 = d.getData();
				
				Vector<String> shared = compareBeliefs (attrBelief, belief2);
				
				if (shared.size() > 0) {
					dBelief sharedBelief = createSharedBelief (belief2, shared);
					addToWorkingMemory(newDataID(), sharedBelief);
				}
				
			}
			
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
		
	}
	
	
	private Vector<String> compareBeliefs (dBelief attrBelief, dBelief privateBelief) {
		
		Vector<String> sharedFeatures = new Vector<String>();
		
		CondIndependentDistribs attrContent = (CondIndependentDistribs) attrBelief.content;

		if (privateBelief.estatus instanceof PrivateEpistemicStatus && 
				privateBelief.content instanceof CondIndependentDistribs) {
			
			CondIndependentDistribs privateContent = (CondIndependentDistribs) privateBelief.content;
			
			for (String feature : attrContent.distribs.keySet()) {
				
				if (attrContent.distribs.get(feature) instanceof BasicProbDistribution) {
					
					DistributionValues attrVals = ((BasicProbDistribution)attrContent.distribs.get(feature)).values;
											dFormula attrValue = getHighestProbValue (attrVals);
						
						if (privateContent.distribs.containsKey(feature) && 
								privateContent.distribs.get(feature) instanceof BasicProbDistribution) {
							
							DistributionValues privateVals = ((BasicProbDistribution)privateContent.distribs.get(feature)).values;
							
								dFormula privateValue = getHighestProbValue (privateVals);
								
								if (isEqualTo(attrValue, privateValue)) {
									sharedFeatures.add(feature);
								}						
						}
				}
			}
		}
		
		return sharedFeatures;
	}
	
		
	
	
	private dBelief createSharedBelief (dBelief privateBelief, Vector<String> sharedFeatures) {
		
		dBelief sharedBelief = new dBelief();
		LinkedList<String> agents = new LinkedList<String>();
		agents.add("human");
		agents.add("robot");
		sharedBelief.estatus = new SharedEpistemicStatus(agents);
			
		sharedBelief.content = new CondIndependentDistribs();
		((CondIndependentDistribs)sharedBelief.content).distribs = new HashMap<String,ProbDistribution>();	
		
		for (String feat : sharedFeatures) {
			
			DistributionValues values = ((BasicProbDistribution)((CondIndependentDistribs)privateBelief.content).distribs.get(feat)).values;
			BasicProbDistribution distrib = new BasicProbDistribution (feat, values);
			((CondIndependentDistribs)sharedBelief.content).distribs.put(feat, distrib);

		}
		
		return sharedBelief;
	}
	
	
	private dFormula getHighestProbValue (DistributionValues values) {
		
		if (! (values instanceof FormulaValues)) {
			return null;
		}
		
		float curMaxProb = 0.0f;
		dFormula bestForm = null;
		for (FormulaProbPair pair : ((FormulaValues)values).values) {
			if (pair.prob > curMaxProb) {
				bestForm = pair.val;
			}
		}
		
		return bestForm;
	}
	
	
	private boolean isEqualTo (dFormula form1, dFormula form2) {
		
		if (!form1.getClass().equals(form2.getClass())) {
			return false;
		}
		
		if (form1 instanceof ElementaryFormula &&
			((ElementaryFormula)form1).prop.equals(((ElementaryFormula)form2).prop)) {
				return true;
		}
		
		if (form1 instanceof BooleanFormula &&
				((BooleanFormula)form1).val == ((BooleanFormula)form2).val) {
					return true;
			}
		
		if (form1 instanceof FloatFormula &&
				((FloatFormula)form1).val == ((FloatFormula)form2).val ) {
					return true;
			}
		
		if (form1 instanceof IntegerFormula &&
				((IntegerFormula)form1).val == ((IntegerFormula)form2).val ) {
					return true;
			}
		
		return false;
	}

}
