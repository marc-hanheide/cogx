package comsys.processing.parseselection;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Vector;
import java.util.Enumeration;

public abstract class FeatureExtraction {

	public static boolean logging = true;
		

	public static String getLexicalFeatName (String sort, String proposition) {
		String name = "lexf=<" + sort + "," + proposition + ">";
		return name;
	}
	
	public static String getLexical2FeatName (String sort) {
		String name = "lex2f=<" + sort + ">";
		return name;
	}
	
	public static String getDependencyFeatName (String headSort, String depSort, String mode) {
		String name = "depf=<" + headSort + "," + depSort + "," + mode + ">";
		return name;
	}
	
	public static String getDependency2FeatName (String headSort, String headProp, String mode) {
		String name = "dep2f=<" + headSort + "," + headProp + "," + mode + ">";
		return name;
	}
	
	
	public static String getBigramDependencyFeatName (String mode1, String mode2) {
		String name = "bigramf=<" + mode1 + "," + mode2 + ">";
		return name;
	}
	
	public static String getBigramDependency2FeatName (String headSort, String mode1, String mode2) {
		String name = "bigram2f=<" + headSort + "," + mode1 + "," + mode2 + ">";
		return name;
	}

	
	public static String getNominalFeatureFeatName (String sort, String feature, String value) {
		String name = "ff=<" + sort + "," + feature + "," + value + ">";
		return name;
	}
	
	public static String getNbrNominalsFeatName () {
		String name = "nbrNominals";
		return name;
	}
	
	public static String getNbrEmptyNominalsFeatName () {
		String name = "nbrEmptyNominals";
		return name;
	}
	
	public static String getNonStandardRuleFeatName(String rulename) {
		String name = "nonstandardRule<" + rulename + ">";
		return name;
	}
	
	public static String getConfidenceValueFeatName() {
		String name = "confidenceValue";
		return name;
	}
	
	
	public static String getNoParseFeatName() {
		String name = "noparse";
		return name;
	}
	
	public static String getNLConfidenceValueFeatName() {
		String name = "NLconfidenceValue";
		return name;
	}
	
	public static String getConfidenceValueHigherThan50FeatName() {
		String name = "confidenceValueHigherThan50";
		return name;
	}
	
	public static String getConfidenceValueSignificantlyHigherFeatName() {
		String name = "confidenceValueSignificantlyHigher";
		return name;
	}
	
	public static String getRankFeatName() {
		String name = "rank";
		return name;
	}
	
	public static String getWordActivationFeatName(String word) {
		String name = "wordActivation=<" + word + ">";
		return name;
	}
	
	public static String getWordActivationNonContextualFeatName() {
		String name = "wordActivation=<OTHER>";
		return name;
	}
	
	public String[] intersection(String[] lfIds1, String[] lfIds2) {
		Vector<String> inter = new Vector<String>();
		
		for (int i=0; i < lfIds1.length ; i++) {
			for (int j=0; j < lfIds2.length; j++) {
				if (lfIds1[i].equals(lfIds2[j])) {
					inter.add(lfIds1[i]);
				}
			}
		}
		String[] result = new String[inter.size()];
		for (int i=0; i < inter.size(); i++) {
			result[i] = inter.elementAt(i);
		}
		return result;
	}
	
	public Vector<String> extractWords(String filename) {
		Vector<String> entities = new Vector<String>();
		String text = readFile(filename);
		String[] split = text.split("\n");
		for (int i = 0; i < split.length ; i++) {
			String line = split[i];
		//	log ("word added: " + line);
			entities.add(line);
		}
		return entities;
	}
	
	
	public String readFile(String fileName) {
		String result = "";

		try {
			if (fileName != null) {
				BufferedReader inFile;
				inFile = new BufferedReader(new FileReader(fileName));

				String line = inFile.readLine();
				while (line != null) {
					result += line + "\n";
					line = inFile.readLine();
				}
		//		log ("File " + fileName + " successfully read");
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
	
	protected static void log(String str) {
		if (logging) {
			System.out.println("[feature extraction] " + str);
		}
	}
}
