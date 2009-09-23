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

import binder.abstr.BindingWorkingMemoryWriter;

public abstract class AbstractTester extends BindingWorkingMemoryWriter {


	public int testNumber;
	public String task;
	
	
	public AbstractTester (int testNumber, String task) {
		this.testNumber = testNumber;
		this.task = task;
	}
	
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
	
	public abstract boolean performTest();

}
