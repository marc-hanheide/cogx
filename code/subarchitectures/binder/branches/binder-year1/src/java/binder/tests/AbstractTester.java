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

package binder.tests;

import binder.abstr.ProxyWriter;

/**
 * Abstract class for testing
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 15/09/2009)
 */
public abstract class AbstractTester extends ProxyWriter {

	// test number
	public int testNumber;
	
	// task description
	public String task;

	
	/**
	 * Initialise abstract tester
	 * 
	 * @param testNumber test nu,ber
	 * @param task description of the task
	 */
	public AbstractTester (int testNumber, String task) {
		this.testNumber = testNumber;
		this.task = task;
	}
	
	
	/**
	 * Run the test
	 */
	@Override
	public void run() {
		try {
			System.out.println("==========");
			System.out.println("Test number " + testNumber + ": ");
			System.out.println("TASK: " + task);
			System.out.print("Running test ...");
		long startTime = System.currentTimeMillis();

		boolean result = performTest();
		
		long stoptime = System.currentTimeMillis();
		if (result) {
			System.out.println( "\t\033[32mSUCCESS\033[0m\t"); 
		} 
		else {
			System.out.println("\t\033[31mFAILURE\033[0m\t");
		}
		System.out.println(" --> Total execution time: " +  (stoptime - startTime)/1000.0 + " seconds");
		System.out.println("==========");

		}
		
		catch (Exception e) {
			log("Test 1 FAILED, exception thrown");
			e.printStackTrace();
		}

	}
	
	/**
	 * Perform the specific test (must be overwritten by the concrete class), and
	 * return the outcome
	 * 
	 * @return true is the test passed successfully, false otherwise
	 */
	public abstract boolean performTest();

}
