

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
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.data.Belief;
import de.dfki.lt.tr.beliefs.data.IndependentDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentDistributionBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
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
	}
	
	
	
	public void test1() {
		dBelief privateBelief = createPrivateBelief("red");
		String privBeliefID = addBeliefToWM (privateBelief);
		
		dBelief attributedBelief = createAttributedBelief("red", privBeliefID);
		addBeliefToWM (attributedBelief);
	
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
		dBelief privateBelief = createPrivateBelief("red");
		
		dBelief attributedBelief = createAttributedBelief("red", "another ID");
		addBeliefToWM (attributedBelief);
	
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
		dBelief privateBelief = createPrivateBelief("red");
		String privBeliefID = addBeliefToWM (privateBelief);
		
		dBelief attributedBelief = createAttributedBelief("blue", privBeliefID);
		addBeliefToWM (attributedBelief);
	
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
		
		dBelief attributedBelief = createAttributedBelief("blue", privBeliefID);
		addBeliefToWM (attributedBelief);
	
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
		dBelief privateBelief = createPrivateBelief("red");
		String privBeliefID = addBeliefToWM (privateBelief);
		
		dBelief attributedBelief = createAttributedBelief("red", privBeliefID);
		String attrBeliefID = addBeliefToWM (attributedBelief);
	
		sleepComponent(100);
		
		detectedSharedBelief = false;
		
		try {
			overwriteWorkingMemory(attrBeliefID, attributedBelief);
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
		dBelief privateBelief = createPrivateBelief("red");
		String privBeliefID = addBeliefToWM (privateBelief);
		
		dBelief attributedBelief = createAttributedBelief("red", privBeliefID);
		addBeliefToWM (attributedBelief);
	
		sleepComponent(100);
		
		if (detectedSharedBelief) {
			
			log("Test 6: OK");
		}
		else {
			log("Test 6: FAIL");
		}
		detectedSharedBelief = false;
	}
	
	
	
	
	private dBelief createPrivateBelief (String colourValue) {

		IndependentFormulaDistributionsBelief<dBelief> beliefTest = 
			IndependentFormulaDistributionsBelief.create(dBelief.class);
					
		FormulaDistribution colours = FormulaDistribution.create();	
		colours.addAll(new Object[][] { { colourValue, 0.8 } });

		beliefTest.getContent().put("colour", colours);
		
		return beliefTest.get();
	}
	
	
	private dBelief createAttributedBelief  (String colourValue, String privBeliefID) {

		IndependentFormulaDistributionsBelief<dBelief> beliefTest = 
			IndependentFormulaDistributionsBelief.create(dBelief.class);
					
		FormulaDistribution colours = FormulaDistribution.create();	
		colours.add(colourValue, 0.8);
		beliefTest.getContent().put("colour", colours);
		
		FormulaDistribution pointerVals = FormulaDistribution.create();	
		pointerVals.add(new GenericPointerFormula(0,privBeliefID), 1.0);
		beliefTest.getContent().put(SharedBeliefCreator.POINTER_LABEL, pointerVals);
		
		String attributingAgent = "self";
		LinkedList<String> attributedAgents = new LinkedList<String>();
		attributedAgents.add("human");
		beliefTest.setAttributed(attributingAgent, attributedAgents);
		
		return beliefTest.get();
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
