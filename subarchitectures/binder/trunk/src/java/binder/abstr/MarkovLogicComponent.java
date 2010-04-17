package binder.abstr;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Vector;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import binder.utils.MLNGenerator;

public abstract class MarkovLogicComponent extends BeliefWriter {

	String inferCmd = "tools/alchemy/bin/infer";
	protected String markovlogicDir = "subarchitectures/binder/markovlogic/";
	
	String emptyFile = markovlogicDir + "empty.db";
	
	String query = "Outcome";
	 
	
	public HashMap<String,Float> runAlchemyInference(String mlnFile, String resultsFile) throws BeliefException {
		
		Runtime run = Runtime.getRuntime(); 
		log("Now running Alchemy...");
		try {
			String[] args = {inferCmd, "-i", mlnFile, "-e", emptyFile, "-r", resultsFile, "-q", query};
			Process p = run.exec(args);

			BufferedReader stdInput = new BufferedReader(new InputStreamReader(p.getInputStream()));

			boolean inferenceSuccessful = true;
			
			String s;
			String output = "";
			while ((s = stdInput.readLine()) != null) {
				output += s + "\n";
				if (s.contains("ERROR")) {
					inferenceSuccessful = false;
				}
			}
			
			if (inferenceSuccessful) {
				log("Alchemy inference was successful, now retrieving the results");
				
				return readResultsFile (resultsFile);
			}
			
			else {
				log("ERROR: Alchemy inference failed");
				System.out.println(output);
				throw new BeliefException("ERROR: Alchemy inference failed");
			}

		}
		catch (IOException e) {
			e.printStackTrace();
		}
		return new HashMap<String,Float>();
	}
	
	  
	
	public HashMap<String,Float> readResultsFile (String resultsFile) {
	    
		HashMap<String,Float> inferenceResults = new HashMap<String,Float>();

	    try {
		FileInputStream fstream = new FileInputStream(resultsFile);
	    DataInputStream in = new DataInputStream(fstream);
	    BufferedReader br = new BufferedReader(new InputStreamReader(in));
	    String strLine;
	    while ((strLine = br.readLine()) != null)   {
	      log (strLine);
	      String line2 = strLine.replace(query+"(", "");
	      String markovlogicunion = line2.substring(0, line2.indexOf(")"));
	      float prob = Float.parseFloat(line2.substring(line2.indexOf(" ")));
	      
	      String union = MLNGenerator.getIDFromMarkovLogicConstant(markovlogicunion);
	      
	      inferenceResults.put(union, prob);
	      
	    } 

		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
		return inferenceResults;
	}
	
}
