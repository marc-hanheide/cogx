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
import java.io.BufferedReader;
import java.util.Iterator;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;
import java.util.Random;
import java.util.Vector;
import java.util.StringTokenizer;
import java.io.StringReader;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.lf.*;

// Dialogue API 
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.dialogue.util.CFG;
import de.dfki.lt.tr.dialogue.util.Rule;

/**
 * <b>Warning</b>: contains several methods that apply resource-specific edits on the input
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 100608
 */


public class GenerationUtils {

	static String errors ;

	static public boolean logging = false;
	
	static boolean lineFactorisation;
	
	static boolean genWithSemantics = true;

	/**
	 * Parse the corpora
	 */  
	public static Vector<String> retrieveRecognizedCorpus(CFG cfg, String inputCorpus) {

		try {
			errors = "";

			int nbrTruePositives = 0;
			int totalCount = 0;

			Hashtable<String,Integer> hashtable = new Hashtable<String,Integer>();

			try {
				BufferedReader in = new BufferedReader(new StringReader(inputCorpus));
				String line = in.readLine() ;

				while (line != null) {
			//		line = removeDisfluencies(line);
					line = line.replace("  ", " ");
					line = line.trim();

					if (cfg.recognize(line)) {
						//		log(line);
						nbrTruePositives++ ;
						//	Vector<String> lines = factorizeLine(line);
						Vector<String> lines = new Vector<String>();
						lines.add(line);
						for (Enumeration<String> e = lines.elements() ; e.hasMoreElements() ; ) {
							String line2 = e.nextElement() ;
							if (hashtable.containsKey(line2)) {
								int integ = hashtable.get(line2).intValue() ;
								hashtable.put(line2, new Integer(integ+1));
							}
							else {
								hashtable.put(line2, new Integer(1));
							}
						}
					}
					else {
						errors += line+"\n";
					}
					totalCount++;

					line = in.readLine();
				}
				in.close();
			} 
			catch (IOException e) {log("Problem reading corpora"); }

			log ("Successful utterance recognitions: \t" + 
					(new Integer(nbrTruePositives)).toString() + "/" + 
					(new Integer(totalCount)).toString()) ;


			Vector<String> utterances = new Vector<String>();
			for (Enumeration<String> e = hashtable.keys() ; e.hasMoreElements() ; ) {
				String key = e.nextElement();
				for (int i=0; i< hashtable.get(key).intValue() ; i++) {
					utterances.add(key);
				}
			}
			return utterances;
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}

	public static String getErrors() {return errors; }


	static private String trim (String str) {
		str = str.replace("   ", " ");
		str = str.replace("  ", " ");
		//	if (str.startsWith("i ")) 
		//		str = str.replaceFirst("i ", "I ");  
		if (str.startsWith(" ")) 	
			str = str.replaceFirst(" ", ""); 
		if (str.endsWith(" ")) {	
			str = str + "#";
			str = str.replace(" #", "");
		}

		if (genWithSemantics) {
		str = str.replace("that's", "that is");
		str = str.replace("it's", "it is");
		str = str.replace("there's", "there is");
		str = str.replace("what's", "what is");
		str = str.replace("here's", "here is");
		str = str.replace("where's", "where is");
		str = str.replace("they're", "they are");

		str = str.replace(" i ", " I ");
		str = "#"+str;
		str = str.replace("#i ", "I ");
		str = str.replace("#",""); 
		}

		return str;
	}


	public static void weightHeuristics (CFG cfg) {
		Hashtable<String,Vector<Rule>> cfgRules = cfg.getRules();
		for (Iterator<String> e = cfgRules.keySet().iterator() ; e.hasNext() ; ) {
			String key = e.next();
			for (Iterator<Rule> f = cfgRules.get(key).iterator() ; f.hasNext() ;) {
				Rule rule = f.next();
				
				if (key.contains("DISC_PARTICLE") && 
						rule.weights.containsKey(".SENTENCE") && 
						rule.RHS.size() > 0 && 
						rule.RHS.elementAt(0).equals("TEMPORAL_LC")) {
					rule.weights.put(".SENTENCE", rule.weights.get(".SENTENCE") / 2.0f);
				}
				
				if (key.contains("DISC_PARTICLE") && 
						rule.weights.containsKey(".SENTENCE") && 
						rule.RHS.size() > 0 && 
						rule.RHS.elementAt(0).equals("PERSON_LC")) {
					rule.weights.put(".SENTENCE", rule.weights.get(".SENTENCE") * 10.0f);
				}
				
				if (key.equals("CONTENT")  && 
						rule.weights.containsKey(".SENTENCE") && 
						rule.RHS.size() > 0 && 
						rule.RHS.elementAt(0).equals("EVALUATION")) {
					rule.weights.put(".SENTENCE", rule.weights.get(".SENTENCE") / 1.5f);
				}			
				
				if (key.equals("CONTENT")  && 
						rule.weights.containsKey(".SENTENCE") && 
						rule.RHS.size() > 0 && 
						rule.RHS.elementAt(0).contains("DISC_PARTICLE")) {
					rule.weights.put(".SENTENCE", rule.weights.get(".SENTENCE") / 3.0f);
				}	
				
				if (key.equals(".SENTENCE") && 
						rule.RHS != null &&
						rule.weights.containsKey("earleyStart") && 
						rule.RHS.size() > 0 && 
						(rule.RHS.elementAt(0).contains("DISC_PARTICLE")  || 
						rule.RHS.elementAt(0).contains("DISFL_LC"))) {
					rule.weights.put("earleyStart", rule.weights.get("earleyStart") / 5.0f);
				}
				
				if (key.equals(".SENTENCE") && 
						rule.weights.containsKey("earleyStart") && 
						rule.RHS.size() > 0 && 
						rule.RHS.elementAt(0).equals("CONTENT")) {
					rule.weights.put("earleyStart", rule.weights.get("earleyStart") * 3.0f);
				}
				
				if (key.equals("CONTENT")  && 
						rule.weights.containsKey(".SENTENCE") && 
						rule.RHS.size() > 0 && 
						rule.RHS.elementAt(0).equals("COMMAND")) {
					rule.weights.put(".SENTENCE", rule.weights.get(".SENTENCE") * 3.0f);
				}
				
				if (key.equals("CONTENT")  && 
						rule.weights.containsKey(".SENTENCE") && 
						rule.RHS.size() > 0 && 
						rule.RHS.elementAt(0).equals("QUESTION")) {
					rule.weights.put(".SENTENCE", rule.weights.get(".SENTENCE") * 5.0f);
				}
				
				if (key.equals("CONTENT")  && 
						rule.weights.containsKey(".SENTENCE") && 
						rule.RHS.size() > 0 && 
						rule.RHS.elementAt(0).equals("ASSERTION")) {
					rule.weights.put(".SENTENCE", rule.weights.get(".SENTENCE") * 5.0f);
				}
				
				if (key.equals(".SENTENCE") && 
						rule.weights.containsKey("earleyStart") && 
						rule.RHS.size() > 1 &&
						(rule.RHS.elementAt(1).contains("DISC_PARTICLE") || 
						rule.RHS.elementAt(1).contains("DISFL_LC"))) {
					rule.weights.put("earleyStart", rule.weights.get("earleyStart") / 5.0f);
				}	
				
				if (key.equals(".SENTENCE") && 
						rule.RHS.size() > 2 &&
						rule.weights.containsKey("earleyStart") && 
						rule.RHS.elementAt(2).contains("DISC_PARTICLE")) {
					rule.weights.put("earleyStart", rule.weights.get("earleyStart") / 5.0f);
				}
			}
		}
	}
	static private Vector<String> factorizeLine(String line) {

		Vector<String> result = new Vector<String>();
		String init = line;
		result.add(trim(init));

		String initCopy = new String(init);

		while (initCopy.contains("FILLER*")) {
			String initCopy2 = initCopy.replace("FILLER*", "");
			initCopy2 = initCopy2.replace("  ", " ");
			if (!initCopy2.equals(" ") && !initCopy2.equals("")) {
				result.add(trim(initCopy2));
			}
			initCopy = new String(initCopy2);
		}

		return result;
	}

	public static void setLineFactorisation(boolean factorisation) {
		lineFactorisation = factorisation;
	}

	public static String produceASRErrors (String utterance) {
		String newUtterance = "";
		
		Random rand = new Random();
		
		StringTokenizer tokenizer = new StringTokenizer(utterance);
		while (tokenizer.hasMoreTokens()) {
		
			String word = tokenizer.nextToken();
		if (word.equals("and")) {
			int random = rand.nextInt(2);
			if (random < 1) {
				word = "then";
			}
		}
		if (word.equals("this")) {
			int random = rand.nextInt(5);
			if (random < 1) {
				word = "the";
			}
		}
		
		if (word.equals("the")) {
			int random = rand.nextInt(5);
			if (random < 1) {
				word = "this";
			}
		}
		if (word.equals("a")) {
			int random = rand.nextInt(3);
			if (random < 1) {
				word = "the";
			}
		}
		if (word.equals("up")) {
			int random = rand.nextInt(2);
			if (random < 1) {
				word = "cup";
			}
		}
		if (word.equals("now")) {
			int random = rand.nextInt(3);
			if (random < 1) {
				word = "no";
			}
		}
		if (word.equals("now")) {
			int random = rand.nextInt(3);
			if (random < 1) {
				word = "not";
			}
		}
		if (word.equals("not")) {
			int random = rand.nextInt(3);
			if (random < 1) {
				word = "no";
			}
		}
		if (word.equals("round")) {
			int random = rand.nextInt(2);
			if (random < 1) {
				word = "wrong";
			}
		}

		if (word.equals("on")) {
			int random = rand.nextInt(5);
			if (random < 1) {
				word = "in";
			}
		}
		newUtterance += word;
		if (tokenizer.hasMoreTokens())
			newUtterance += " ";
		}
		return newUtterance;
	}

	private static Vector<String> retrieveSentences(CFG cfg, int number, boolean uniqueness) {
		
		
		Vector<String> utterances = new Vector<String>();

		int percentCount = 0;
		for (int i=0; i < number ; i++) {
			String utterance;
			if (genWithSemantics) {
				utterance = cfg.generateWeightedRandomWithSemantics();
			}
			 else {
				 utterance = cfg.generateWeightedRandom();
			 }
			if (lineFactorisation) {
				Vector<String> factUtterance = factorizeLine(utterance);
				for (Enumeration<String> e = factUtterance.elements() ; e.hasMoreElements() ;) {
					String utt = e.nextElement();
					if (!uniqueness || !utterances.contains(utt)) {
						utterances.add(utt);
					}					
				}
			}
			else {
				String utt = trim(utterance);
				if (!uniqueness || !utterances.contains(utt)) {
					utterances.add(utt);
				}
			}
			
			percentCount++;
			if (number > 1 && percentCount > number/20) {
				log("\tPercent complete: " + 
						new Float(((float)i*100.0f)/number).toString().substring(0, 2)
						+ " %");
				percentCount=0;
			}
		}
		return utterances;
	}

	public static Vector<String> retrieveRandomSentences(CFG cfg, int number) {
		return retrieveSentences (cfg, number, false);
	}

	public static Vector<String> retrieveUniqueSentences(CFG cfg, int number) {
		return retrieveSentences (cfg, number, true);
	}



	public static String readFile(String fileName) {
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

	public static void writeFile(String text, String fileName) {
		if (fileName != null) {
			try {
				BufferedWriter out = new BufferedWriter(new FileWriter(fileName));
				out.write(text);
				out.close();
				log ("File " +  fileName + " successfully written");           
			} catch (IOException e) {
				e.printStackTrace();
			} 
		}
		else {
			log("WARNING: " + fileName + " does not exist, unable to write");
		}
	}
	

	public static void appendToFile(String text, String fileName) {
		
		String existingText = readFile(fileName);
		
		if (fileName != null) {
			try {
				BufferedWriter out = new BufferedWriter(new FileWriter(fileName));
				out.write(existingText+text);
				out.close();
				log ("File " +  fileName + " successfully written");           
			} catch (IOException e) {
				e.printStackTrace();
			} 
		}
		else {
			log("WARNING: " + fileName + " does not exist, unable to write");
		}
	}


	static public String modifyCase (String input) {
		String result = "";
		StringTokenizer tokenizer = new StringTokenizer(input);
		while (tokenizer.hasMoreTokens()) {
			String token = tokenizer.nextToken();
			if (token.trim().equals("no"))
				token = token.replace("no", "No");
			if (token.trim().equals("pierre"))
				token = token.replace("pierre", "Pierre");
			if (token.trim().equals("gj"))
				token = token.replace("gj", "GJ");
			if (token.trim().equals("i"))
				token = token.replace("i", "I");
			result = result + token;
			if (tokenizer.hasMoreTokens()) result += " ";
		}
		return result;
	}
	
	static public String modifyCase2 (String input) {
		String result = "";
		StringTokenizer tokenizer = new StringTokenizer(input);
		while (tokenizer.hasMoreTokens()) {
			String token = tokenizer.nextToken();
			if (token.trim().equals("No"))
				token = token.replace("No", "no");
			if (token.trim().equals("Pierre"))
				token = token.replace("Pierre", "pierre");
			if (token.trim().equals("yeah"))
				token = token.replace("yeah", "yes");
			if (token.trim().equals("GJ"))
				token = token.replace("GJ", "gj");
			if (token.trim().equals("I"))
				token = token.replace("I", "i");
			if (token.trim().equals("that's"))
				token = token.replace("that's", "that is");
			if (token.trim().equals("it's"))
				token = token.replace("it's", "it is");
			if (token.trim().equals("where's"))
				token = token.replace("where's", "where is");
			if (token.trim().equals("I"))
				token = token.replace("I", "i");
			if (token.trim().equals("an"))
				token = token.replace("an", "a");
			if (token.trim().equals("isn't"))
				token = token.replace("isn't", "is not");
			result = result + token;
			if (tokenizer.hasMoreTokens()) result += " ";
		}
		return result;
	}


	static public String removeDisfluencies (String input) {
		String result = "";
		StringTokenizer tokenizer = new StringTokenizer(input);
		while (tokenizer.hasMoreTokens()) {

			String token = tokenizer.nextToken();

			if (token.trim().equals("mm"))
				token = token.replace("mm", "");
			//		token = token.replace("yeah", "");
			if (token.trim().equals("uh"))
				token = token.replace("uh", "");
			if (token.trim().equals("er"))
				token = token.replace("er", "");
			if (token.trim().equals("err"))
				token = token.replace("err", "");
			if (token.trim().equals("yeah"))
				token = token.replace("yeah", "yes");
			if (token.trim().equals("that's"))
				token = token.replace("that's", "that is");
			if (token.trim().equals("it's"))
				token = token.replace("it's", "it is");
			if (token.trim().equals("where's"))
				token = token.replace("where's", "where is");
			if (token.trim().equals("I"))
				token = token.replace("I", "i");
			if (token.trim().equals("an"))
				token = token.replace("an", "a");
			if (token.trim().equals("isn't"))
				token = token.replace("isn't", "is not");
			if (token.trim().equals("how many"))
				token = token.replace("how many", "howmany");
			result = result + token;
			if (tokenizer.hasMoreTokens()) result += " ";
		}
		result = result.trim();
		return result;
	}


	static public LogicalForm removeDuplicateNominalsAndFeatures(LogicalForm lf) {

		boolean OneMoreLoop = true;

		while (OneMoreLoop) {
			OneMoreLoop = false;
			Vector<String> nominals = new Vector<String>();
			
			for (int i=0; !OneMoreLoop && i <lf.noms.length ; i++) {

				if (nominals.contains(lf.noms[i].nomVar) 
						&& lf.noms[i].prop != null &&
						lf.noms[i].prop.prop != "") {
					ArrayList<LFNominal> nomsList = new ArrayList<LFNominal>(Arrays.asList(lf.noms));
					nomsList.remove(lf.noms[i]);
					lf.noms = (LFNominal[]) LFUtils.resizeArray(lf.noms, lf.noms.length - 1);
					lf.noms = nomsList.toArray(lf.noms);
					OneMoreLoop = true;
				}
				else {
					nominals.add(lf.noms[i].nomVar);
					Feature[] feats = lf.noms[i].feats;
					Hashtable<String,Feature> processedFeats = new Hashtable<String,Feature>();
				
					for (int j=0; !OneMoreLoop && j < feats.length ; j++) {
						Feature feat = feats[j];
						if (processedFeats.containsKey(feat.feat)) {
							ArrayList<Feature> featsList = new ArrayList<Feature>(Arrays.asList(feats));
							featsList.remove(feat);
							feats = (Feature[]) LFUtils.resizeArray(feats, feats.length - 1);
							feats = featsList.toArray(feats);
							lf.noms[i].feats = feats;
							OneMoreLoop = true;
						}
						else {
							processedFeats.put(feat.feat, feat);
						}
					}
				}
			}
		}

		return lf;
	}




	private static void log(String s) {
		if (logging)
			System.out.println("\033[36m[Generator Utils] " +s + "\033[0m") ;
	}


	
	public static void main (String[] args) {
	//	String semantics = "@q1:ascription(be ^ <Mood>int ^ <Tense>pres ^  <Cop-Restr>(o11:thing ^ machine ^ <Num>sg ^ <Delimitation>unique ^ <Quantification>specific ^  <Compound>(c1:e-substance ^ coffee)) ^  <Cop-Scope>(w1_0:m-location ^ where) ^  <Subject>(o11:thing ^ machine ^ <Num>sg ^ <Delimitation>unique ^ <Quantification>specific ^  <Compound>(c1:e-substance ^ coffee)) ^  <Wh-Restr>w1_0:m-location ^  <Modifier>(t1:m-time-point ^ now))";
	//	GenerationResult gr = new GenerationResult();
	//	gr.setSemantics(semantics);
	//	gr.setUtterance("where is the coffee machine now");
	//	log("final result: " + verifyResult(gr));
		
		String utt = "and put it here";
		log("initial utterance: " + utt);
		String alteredUtt = produceASRErrors(utt);
		log("altered utt: " + alteredUtt);
	}
}
