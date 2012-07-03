// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@ifi.uio.no)                                                                
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


package de.dfki.lt.tr.dialmanagement.utils;

import static org.junit.Assert.*;

import java.util.List;
import java.util.Vector;

import org.junit.Test;

import de.dfki.lt.tr.dialmanagement.data.policies.PolicyEdge;
import de.dfki.lt.tr.dialmanagement.data.conditions.IntentionCondition;

public class PartialOrderTest {

	
	
	@Test
	public void testPartialOrder1() {
		
		Vector<PolicyEdge> edges = new Vector<PolicyEdge>();
		
		PolicyEdge edge1 = new PolicyEdge("", new IntentionCondition("", "*"));
		PolicyEdge edge2 = new PolicyEdge("", new IntentionCondition("", "blabla"));
		edges.add(edge1);
		edges.add(edge2);
		
		PolicyUtils.sortEdges(edges);
		
		assertTrue(edges.indexOf(edge1) == 1);
	}
	
	@Test
	public void testPartialOrder2() {
		
		Vector<PolicyEdge> edges = new Vector<PolicyEdge>();
		
		PolicyEdge edge1 = new PolicyEdge("", new IntentionCondition("", "<bla>(*)"));
		PolicyEdge edge2 = new PolicyEdge("", new IntentionCondition("", "<bla>(blo)"));
		edges.add(edge1);
		edges.add(edge2);
		
		PolicyUtils.sortEdges(edges);
		
		assertTrue(edges.indexOf(edge1) == 1);
	}
	
	
	@Test
	public void testPartialOrder3() {
		
		Vector<PolicyEdge> edges = new Vector<PolicyEdge>();
		
		PolicyEdge edge1 = new PolicyEdge("", new IntentionCondition("", "<bla>(*)"));
		PolicyEdge edge2 = new PolicyEdge("", new IntentionCondition("", "<bla>(hehe ^ <hoho>(*))"));
		PolicyEdge edge3 = new PolicyEdge("", new IntentionCondition("", "<bla>(hehe ^ <hoho>(tadam ^ boumboum))"));
		edges.add(edge1);
		edges.add(edge2);
		edges.add(edge3);
		
		PolicyUtils.sortEdges(edges);
		
		assertTrue(edges.indexOf(edge1) == 2);
		assertTrue(edges.indexOf(edge2) == 1);

	}
	
}
