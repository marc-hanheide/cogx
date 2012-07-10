//=================================================================
//Copyright (C) 2010 Pierre Lison (plison@dfki.de)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION
package de.dfki.lt.tr.dialogue.parse.examplegeneration;

//=================================================================
//IMPORTS

// Java
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;

import java.util.Properties;
import java.util.Random;
import java.util.Vector;

// Dialogue API 
import de.dfki.lt.tr.dialogue.util.CFG;


/**
 * 
 * @author Pierre Lison (plison@dfki.de)
 *
 */

public class SemanticCorpusGenerator {

	String inputGrammarFile;
	String inputRawCorpusFile;
	String inputClassFile;
	
	String parseErrorsFile;
	String genCorpusFile;
	
	boolean lineFactorisation = true;
	public int genCorpusSize = 0;
	boolean classSmoothing = true;
	String outputClassFile;




	public SemanticCorpusGenerator (Properties properties) {

		inputGrammarFile = (String) properties.get("--inputGrammarFile");
		inputRawCorpusFile = (String) properties.get("--inputRawCorpusFile");

		parseErrorsFile = (String) properties.get("--parseErrorsFile");
		genCorpusFile = (String) properties.get("--genCorpusFile");

		if ((String)properties.get("--genCorpusSize") != null) {
			genCorpusSize = (int) new Integer((String)properties.get("--genCorpusSize")).intValue();
		}
		inputClassFile = (String) properties.get("--inputClassFile");

		lineFactorisation = (boolean) new Boolean((String) properties.get("--lineFactorisation")).booleanValue();
		classSmoothing = (boolean) new Boolean((String) properties.get("--classSmoothing")).booleanValue();

	}


	public CFG retrieveCFG() {

		CFG cfg = null;
		try {
			if (inputGrammarFile != null) {
				log("Start converting the grammar into CFG format ...") ;

				// Open the file
				File f = new File (inputGrammarFile) ;
				log("Open grammar file:\t\tOK") ;
				SemCorpusParser parser = new SemCorpusParser((new FileInputStream(f)));
				log ("Load grammar parser:\tOK") ;

				cfg = SemCorpusParser.Input();
				
		/**		File f2 = new File (inputClassFile) ;
				log("Open class file:\t\t\tOK") ;
				SemCorpusParser.ReInit((new FileInputStream(f2)));
				log ("Load grammar parser:\t\t\tOK") ;
				
				// Start the parse
				CFG cfg2 = SemCorpusParser.Input();
				
				cfg.addCFG(cfg2);
				*/
				
				log("Parse grammar file:\t\tOK");

				//	fst.modifyNuanceSpecific();

			}
			else {
				log("ERROR, you must specify either an input grammar " +
				"file or a CFG persistent file");
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return cfg;
	}


	public CFG assignWeights (CFG cfg) {
		if (inputRawCorpusFile != null) {
			log("Weight assignment in the CFG using the input corpus...");
			Vector<String> recogUtterances = retrieveRecognizedCorpus(cfg);
			cfg.setWeights(recogUtterances);
		}
		else {
			log("No input corpus specified, assigning uniform weights...");
			//	cfg.addUniformWeights();
			log("Add Uniform weights:\t\tOK") ;
		}
		return cfg;
	}


	/**
	 * Parse the corpora
	 */  
	public Vector<String> retrieveRecognizedCorpus(CFG cfg) {

		String inputRawCorpusString = GenerationUtils.readFile(inputRawCorpusFile);

		Vector<String> result = GenerationUtils.retrieveRecognizedCorpus
		(cfg, inputRawCorpusString);

		if (parseErrorsFile != null)
			GenerationUtils.writeFile(GenerationUtils.getErrors(), parseErrorsFile);

		return result;
	}

	
	public String retrieveRandomSentence(CFG cfg) {
		GenerationUtils.setLineFactorisation(lineFactorisation);
	
		Vector<String> result = GenerationUtils.retrieveRandomSentences(cfg, genCorpusSize);
		if (result.size() == 1) {
			return result.elementAt(0);
		}
		else {
			return "";
		}
	}

	public Vector<String> retrieveRandomSentences(CFG cfg) {
		GenerationUtils.genWithSemantics = true;
		GenerationUtils.setLineFactorisation(lineFactorisation);
		return GenerationUtils.retrieveRandomSentences(cfg, genCorpusSize);
	}

	public Vector<String> retrieveUniqueSentences(CFG cfg) {
		GenerationUtils.setLineFactorisation(lineFactorisation);
		return GenerationUtils.retrieveUniqueSentences(cfg, genCorpusSize);
	}



	public String generateCorpus(Vector<String> utterances) {
		StringBuffer result = new StringBuffer();
		Random gen = new Random();

		log("Rerandomize corpus utterances...");
		for (int i= 0 ; i < utterances.size()*1 ; i++) {
			int random = gen.nextInt(utterances.size());
			String utterance = utterances.elementAt(random);
			utterance = utterance.replace("  ", " ");
			if (utterance.charAt(0) == ' ') {
				utterance = utterance.substring(1);
			}
			result.append(utterance + "\n");
		}
		log("Rerandomization:\t\tOK");

		result.trimToSize();
		return result.toString();
	}


	public void writeCorpus(String corpus, String fileName) {
		GenerationUtils.writeFile(corpus, fileName);
	}

	private void weightClasses() {

		String inputClasses = GenerationUtils.readFile(inputClassFile);
		String inputRawCorpus = GenerationUtils.readFile(inputRawCorpusFile);

		if (classSmoothing) {
			ClassWeighter.setSmoothing(classSmoothing);
		}
		String sWeightedClasses = ClassWeighter.weightClasses(inputClasses, inputRawCorpus);
		GenerationUtils.writeFile(sWeightedClasses, outputClassFile);
	}

	private void writeFSTPersistentFile(CFG cfg) {
		// Print version the the FST is written to a file
		//	GenUtils.writeFile(cfg.getPersistentCopy(), 
		//			outputFSTPersistentFile, "outputFSTPersistentFile");
	}


	public static void main(String args[]) throws ParseException {

		String params = "";
		for (int i= 0 ; i < args.length ; i++) {
			params += args[i] + " ";
		}

		try {
			InputStream stream = new ByteArrayInputStream(params.getBytes());
			Properties properties = new Properties();
			properties.load(stream);		

			String configFile = (String) properties.get("--configFile");

			if (configFile != null) {
				String config = GenerationUtils.readFile(configFile.trim());
				stream = new ByteArrayInputStream(config.getBytes());
				properties = new Properties();
				properties.load(stream);	
			}

			log("Initializing the speech corpus generator...");
			SemanticCorpusGenerator generator = new SemanticCorpusGenerator(properties);
			log("Initialization:\t\tOK\n");

			log("STEP 1: Retrieving the context-free grammar...");
			CFG cfg = generator.retrieveCFG();
			cfg.isClassBased = false;
			
			if (cfg != null) {

				log("STEP 2: Weights assignment");
				generator.assignWeights(cfg);
				GenerationUtils.weightHeuristics(cfg);
				log("Weights assignment:\t\tOK\n");

				log("STEP 3: Generation of random sentences using CFG...");
				Vector<String> utterances = generator.retrieveUniqueSentences(cfg);

				String genCorpus = generator.generateCorpus(utterances);

				generator.writeCorpus(genCorpus, generator.genCorpusFile);
				log("Generation of random sentences:\t\tOK\n");
			}
		}
		catch (Exception e) {e.printStackTrace();} 

	}

	private static void log(String s) {
		System.out.println("\033[32m[Example generator] " +s + "\033[0m") ;
	}

}
