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
