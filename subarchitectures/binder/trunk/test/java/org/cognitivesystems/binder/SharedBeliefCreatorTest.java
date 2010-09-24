

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

import java.util.LinkedList;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.GenericPointerFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;


/**
 * Test to see if the SharedBeliefCreator component works correctly
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 23/10/2010
 *
 */
public class SharedBeliefCreatorTest extends ManagedComponent{

	
	boolean detectedSharedBelief = false;
	String sharedBeliefId = null;
	boolean detectedOverwrittenBelief = false;
	 
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
								if (belief.estatus instanceof SharedEpistemicStatus) {
									detectedSharedBelief = true;
									sharedBeliefId = _wmc.address.id;
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
								if (belief.estatus instanceof SharedEpistemicStatus) {
									detectedOverwrittenBelief = true;
								}
							} catch (DoesNotExistOnWMException e) {
								e.printStackTrace();
							} catch (UnknownSubarchitectureException e) {
								e.printStackTrace();
							}
						}
					});
	
	

	}
	
	@Override
	public void run() {
		
		test1();
		test2();
		test3();
		test4();
		test5();
		test6();
		test7();
		test8();
	}
	
	
	
	public void test1() {
		IndependentFormulaDistributionsBelief<dBelief> privateBelief = createPrivateBelief();
		addFeatureToBelief(privateBelief, "colour", "red");
		String privBeliefID = addBeliefToWM (privateBelief.get());
		
		IndependentFormulaDistributionsBelief<dBelief> attributedBelief = createAttributedBelief(privBeliefID);
		addFeatureToBelief(attributedBelief, "colour", "red");
		addBeliefToWM (attributedBelief.get());
	
		sleepComponent(100);
		
		if (detectedSharedBelief) {
			log("Test 1: OK");
		}
		else {
			log("Test 1: FAIL");
		}
		detectedSharedBelief = false;
	}
	
	

	public void test2() {
		IndependentFormulaDistributionsBelief<dBelief> privateBelief = createPrivateBelief();
		addFeatureToBelief(privateBelief, "colour", "red");
		String privBeliefID = addBeliefToWM (privateBelief.get());
		
		IndependentFormulaDistributionsBelief<dBelief> attributedBelief = createAttributedBelief("unrelatedID");
		addFeatureToBelief(attributedBelief, "colour", "red");
		addBeliefToWM (attributedBelief.get());
	
		sleepComponent(100);
		
		if (detectedSharedBelief) {
			log("Test 2: FAIL");
		}
		else {
			log("Test 2: OK");
		}
		detectedSharedBelief = false;
	}
	

	
	public void test3() {
		IndependentFormulaDistributionsBelief<dBelief> privateBelief = createPrivateBelief();
		addFeatureToBelief(privateBelief, "colour", "red");
		String privBeliefID = addBeliefToWM (privateBelief.get());
		
		IndependentFormulaDistributionsBelief<dBelief> attributedBelief = createAttributedBelief(privBeliefID);
		addFeatureToBelief(attributedBelief, "colour", "blue");
		addBeliefToWM (attributedBelief.get());
	
		sleepComponent(100);
		
		if (detectedSharedBelief) {
			log("Test 3: FAIL");
		}
		else {
			log("Test 3: OK");
		}
		detectedSharedBelief = false;
	}
	

	
	public void test4() {
		String privBeliefID = newDataID();
		
		IndependentFormulaDistributionsBelief<dBelief> attributedBelief = createAttributedBelief(privBeliefID);
		addFeatureToBelief(attributedBelief, "colour", "red");
		addBeliefToWM (attributedBelief.get());
	
		sleepComponent(100);
		
		if (detectedSharedBelief) {
			log("Test 4: FAIL");
		}
		else {
			log("Test 4: OK");
		}
		detectedSharedBelief = false;
	}
	
	

	public void test5() {
		IndependentFormulaDistributionsBelief<dBelief> privateBelief = createPrivateBelief();
		addFeatureToBelief(privateBelief, "colour", "red");
		String privBeliefID = addBeliefToWM (privateBelief.get());
		
		IndependentFormulaDistributionsBelief<dBelief> attributedBelief = createAttributedBelief(privBeliefID);
		addFeatureToBelief(attributedBelief, "colour", "red");
		String attrBeliefID = addBeliefToWM (attributedBelief.get());
	
		sleepComponent(100);
		
		detectedSharedBelief = false;
		
		try {
			overwriteWorkingMemory(attrBeliefID, attributedBelief.get());
		} catch (Exception e) {
			e.printStackTrace();
		} 
		
		sleepComponent(100);
		
		if (!detectedSharedBelief && detectedOverwrittenBelief) {
			log("Test 5: OK");
		}
		else {
			log("Test 5: FAIL");
		}
		detectedSharedBelief = false;
		detectedOverwrittenBelief = false;
	}
	



	public void test6() {
		sharedBeliefId = null;
		
		IndependentFormulaDistributionsBelief<dBelief> privateBelief = createPrivateBelief();
		addFeatureToBelief(privateBelief, "colour", "red");
		String privBeliefID = addBeliefToWM (privateBelief.get());
		
		IndependentFormulaDistributionsBelief<dBelief> attributedBelief = createAttributedBelief(privBeliefID);
		addFeatureToBelief(attributedBelief, "colour", "red");
		addBeliefToWM (attributedBelief.get());

		sleepComponent(100);

		try {
			if (sharedBeliefId != null) {
				dBelief sharedBelief = getMemoryEntry (sharedBeliefId, dBelief.class);
				if (sharedBelief.content instanceof CondIndependentDistribs) {

					if (((CondIndependentDistribs)sharedBelief.content).distribs.
							containsKey(POINTERLABEL.value)) {

						if (((CondIndependentDistribs)sharedBelief.content).distribs.
								get(POINTERLABEL.value) instanceof BasicProbDistribution) {
							
							BasicProbDistribution pointerDistrib = (BasicProbDistribution)
								((CondIndependentDistribs)sharedBelief.content).distribs.get(POINTERLABEL.value);
							
							if (pointerDistrib.values instanceof FormulaValues) {
								
								if (((FormulaValues)pointerDistrib.values).values.size() > 0) {
									
									if (((FormulaValues)pointerDistrib.values).values.get(0).val instanceof GenericPointerFormula) {
										
										if (((GenericPointerFormula)((FormulaValues)pointerDistrib.values).
												values.get(0).val).pointer.equals(privBeliefID)) {
											log("Test 6: OK");
											detectedSharedBelief = false;
											return;
										}
									}
									
									else {
										debug("shared belief contains a pointer, but is not a generic pointer formula");
									}
									
								}
						}
							
						}
					}
					else {
						debug("shared belief does not contain a pointer");
					}
				}
				else {
					debug("sharedBelief content is not a CondIndependentDistribs");
				}
			}
			else {
				debug("sharedBelief is null");
			}
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		log("Test 6: FAIL");

		detectedSharedBelief = false;
	}


	
	


	public void test7() {
		sharedBeliefId = null;
		
		IndependentFormulaDistributionsBelief<dBelief> privateBelief = createPrivateBelief();
		addFeatureToBelief(privateBelief, "colour", "red");
		String privBeliefID = addBeliefToWM (privateBelief.get());
		
		IndependentFormulaDistributionsBelief<dBelief> attributedBelief = createAttributedBelief(privBeliefID);
		addFeatureToBelief(attributedBelief, "colour", "red");
		addBeliefToWM (attributedBelief.get());

		sleepComponent(100);

		try {
			if (sharedBeliefId != null) {
				dBelief sharedBelief = getMemoryEntry (sharedBeliefId, dBelief.class);
				if (sharedBelief.content instanceof CondIndependentDistribs) {

					if (((CondIndependentDistribs)sharedBelief.content).distribs.
							containsKey("colour")) {

						if (((CondIndependentDistribs)sharedBelief.content).distribs.
								get("colour") instanceof BasicProbDistribution) {
							
							BasicProbDistribution colourDistrib = (BasicProbDistribution)
								((CondIndependentDistribs)sharedBelief.content).distribs.get("colour");
							
							if (colourDistrib.values instanceof FormulaValues) {
								
								if (((FormulaValues)colourDistrib.values).values.size() > 0) {
									
									if (((FormulaValues)colourDistrib.values).values.get(0).val instanceof ElementaryFormula) {
										
										if (((ElementaryFormula)((FormulaValues)colourDistrib.values).
												values.get(0).val).prop.equals("red")) {
											log("Test 7: OK");
											detectedSharedBelief = false;
											return;
										}
									}
									
									else {
										debug("shared belief contains a colour, but is not an elementary formula");
									}
									
								}
						}
							
						}
					}
					else {
						debug("shared belief does not contain a colour");
					}
				}
				else {
					debug("sharedBelief content is not a CondIndependentDistribs");
				}
			}
			else {
				debug("sharedBelief is null");
			}
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		}
		log("Test 7: FAIL");

		detectedSharedBelief = false;
	}





	public void test8() {
		sharedBeliefId = null;
		
		IndependentFormulaDistributionsBelief<dBelief> privateBelief = createPrivateBelief();
		addFeatureToBelief(privateBelief, "colour", "red");
		addFeatureToBelief(privateBelief, "shape", "cubic");
		String privBeliefID = addBeliefToWM (privateBelief.get());
		
		IndependentFormulaDistributionsBelief<dBelief> attributedBelief = createAttributedBelief(privBeliefID);
		addFeatureToBelief(attributedBelief, "colour", "red");
		addFeatureToBelief(attributedBelief, "shape", "cubic");
		addBeliefToWM (attributedBelief.get());

		sleepComponent(100);

		try {
			if (sharedBeliefId != null) {
				dBelief sharedBelief = getMemoryEntry (sharedBeliefId, dBelief.class);
				if (sharedBelief.content instanceof CondIndependentDistribs) {

					if (((CondIndependentDistribs)sharedBelief.content).distribs.keySet().size() == 3) {
						log("Test 8: OK");
						detectedSharedBelief = false;
						return;
					}
					else {
						debug("number of features in distribution: " + ((CondIndependentDistribs)sharedBelief.content).distribs.keySet().size());
					}
				}
				else {
					debug("sharedBelief content is not a CondIndependentDistribs");
				}
			}
			else {
				debug("sharedBelief is null");
			}
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		}
		log("Test 8: FAIL");

		detectedSharedBelief = false;
	}


	
	private IndependentFormulaDistributionsBelief<dBelief> createPrivateBelief () {

		IndependentFormulaDistributionsBelief<dBelief> beliefTest = 
			IndependentFormulaDistributionsBelief.create(dBelief.class);
							
		return beliefTest;
	}
	
	private IndependentFormulaDistributionsBelief<dBelief> addFeatureToBelief
		(IndependentFormulaDistributionsBelief<dBelief> belief, String label, String value) {
		
		FormulaDistribution colours = FormulaDistribution.create();	
		colours.addAll(new Object[][] { { value, 0.8 } });

		belief.getContent().put(label, colours);
		return belief;
	}
	
	
	private IndependentFormulaDistributionsBelief<dBelief> createAttributedBelief  (String privBeliefID) {

		IndependentFormulaDistributionsBelief<dBelief> beliefTest = 
			IndependentFormulaDistributionsBelief.create(dBelief.class);
	
		
		FormulaDistribution pointerVals = FormulaDistribution.create();	
		pointerVals.add(new GenericPointerFormula(0,privBeliefID), 1.0);
		beliefTest.getContent().put(POINTERLABEL.value, pointerVals);
		
		String attributingAgent = "self";
		LinkedList<String> attributedAgents = new LinkedList<String>();
		attributedAgents.add("human");
		beliefTest.setAttributed(attributingAgent, attributedAgents);
		
		return beliefTest;
	}
	
	
	
	private String addBeliefToWM (dBelief b) {
		
		try {
			String id = newDataID();
			addToWorkingMemory(id, b);
			return id;
		} 
		catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
		return "";
	}
}
