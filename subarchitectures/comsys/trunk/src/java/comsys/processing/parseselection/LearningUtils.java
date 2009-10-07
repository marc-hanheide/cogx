package comsys.processing.parseselection;

import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.FileReader;
import java.io.InputStream;
import java.util.Properties;
import java.util.Vector;
import java.util.Enumeration;
import java.util.Hashtable;

import comsys.datastructs.comsysEssentials.NonStandardRule;
import comsys.datastructs.comsysEssentials.NonStandardRulesAppliedForLF;
import comsys.datastructs.comsysEssentials.PackedLFs;
import comsys.processing.parse.examplegeneration.GenerationUtils;

public class LearningUtils {

	public static boolean logging = true;	
	
	static public Vector<TrainingExample> readCorpus (String filename) {
		
		Vector<TrainingExample> examples = new Vector<TrainingExample>();
		
		Vector<String> lines = readFile(filename);
		
		for (Enumeration<String> e = lines.elements() ; e.hasMoreElements(); ) {
			String line = e.nextElement();
			TrainingExample newExample = new TrainingExample(line);
			if (newExample.getExpectedUtterance() != null 
					&& newExample.getSemantics() != null) {			
				examples.add(newExample);
			}
		//	log("added semantics:" + LFUtils.lfToString(newExample.getSemantics()));		
		}
		
		log("Training examples successfully extracted from file" + filename);
		return examples;
	}
	

	
	static public Vector<String> readFile(String fileName) {
		
		Vector<String> result = new Vector<String>();

		try {
			if (fileName != null) {
				BufferedReader inFile;
				inFile = new BufferedReader(new FileReader(fileName));

				String line = inFile.readLine();
				while (line != null) {
					result.add(line);
					line = inFile.readLine();
				}
				log ("File " + fileName + " successfully read");
			}
			else {
				log("WARNING: file " + fileName + " does not exist, unable to read it");
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return result;
	}

	
	static public ParameterVector incrInitPV(TrainingExample example,
			ParameterVector params, PackedLFs plf) {
		
		Vector<String> newfeaturesToConsider = new Vector<String>();
		
		FeatureExtractionFromLF extraction = new FeatureExtractionFromLF(example.getSemantics());
		FeatureVector feats = extraction.extractSemanticFeatures();
		for (Enumeration<String> f = feats.getFeatures(); f.hasMoreElements();) {
			String feature = f.nextElement();
			if (!newfeaturesToConsider.contains(feature)) {
				newfeaturesToConsider.add(feature);
			//		log("feature to consider: " + feature);
			}
		}
		
		for (int i= 0; i < plf.nonStandardRulesForLF.length; i++) {
			NonStandardRulesAppliedForLF nonStandardRulesForLF = plf.nonStandardRulesForLF[i];
			String lfId = nonStandardRulesForLF.logicalFormId;
			for (int j=0; j < nonStandardRulesForLF.nonStandardRules.length; j++) {
				NonStandardRule rule = nonStandardRulesForLF.nonStandardRules[j];
				String rulename = rule.rulename;
				String featName = FeatureExtraction.getNonStandardRuleFeatName(rulename);
				if (!newfeaturesToConsider.contains(featName)) {
					newfeaturesToConsider.add(featName);
				}
			}
		}
		
		params.initialise(newfeaturesToConsider);
		return params;
	}
	
	
	public static ParameterVector buildParameterVector(Vector<String> lines) {
		ParameterVector params = new ParameterVector();
		
		for (Enumeration<String> e = lines.elements(); e.hasMoreElements(); ) {
			String line = e.nextElement();
			line = line.replace("w_{", "");
			String[] split = line.split("} <- ");
			if (split.length == 2) {
				String feature = split[0];
				double value = (new Double(split[1])).doubleValue();
				params.setFeatureValue(feature, value);
			}
		}
		
		return params;
	}
	
	public static String statisticsToString(Vector<Integer> statistics) {
		String result = "";
		for (Enumeration<Integer> e = statistics.elements(); e.hasMoreElements();) {
			Integer value = e.nextElement();
			result += value +"\n";
		}
		return result;
	}
	
	public static Properties readProperties(String[] args) {
		Properties properties = new Properties();
		
		String params = "";
		for (int i= 0 ; i < args.length ; i++) {
			params += args[i] + " ";
		}

		try {
			InputStream stream = new ByteArrayInputStream(params.getBytes());
			
			properties.load(stream);		

			String configFile = (String) properties.get("--configFile");

			// online 
			if (configFile != null) {
				String config = GenerationUtils.readFile(configFile.trim());
				stream = new ByteArrayInputStream(config.getBytes());
				properties = new Properties();
				properties.load(stream);
			}
			else {
				log("WARNING: no configuration file set!");
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
		return properties;
	}
	
	
			
	private static void log(String str) {
		if (logging)
		System.out.println("\033[36m[Learning utils] " + str+ "\033[0m");
	}
}
